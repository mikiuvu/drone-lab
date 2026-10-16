"""Verify native micro sensor noise and preservation of imported sensor models."""
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / 'simulation/models/generate_sensor_models.py'

def generate(tmp_path, *arguments):
    output = tmp_path / 'generated'
    subprocess.run([sys.executable, str(GENERATOR), '--output', str(output),
                    '--vehicle-count', '2', *arguments], capture_output=True, text=True,
                   encoding='utf-8', check=True)
    return output

def test_micro_native_sensor_noise_has_all_axes_and_pressure(tmp_path):
    output = generate(tmp_path, '--dynamics', 'micro')
    for vehicle in range(2):
        model = ET.parse(output / f'micro_{vehicle}/model.sdf')
        imu = model.find('.//sensor[@name="imu_sensor"]')
        assert imu.findtext('update_rate') == '250'
        for group, stddev in [('angular_velocity', .00018665), ('linear_acceleration', .00186)]:
            for axis in 'xyz':
                noise = imu.find(f'imu/{group}/{axis}/noise')
                assert noise is not None, f'missing {group}/{axis} noise'
                assert noise.get('type') == 'gaussian'
                assert float(noise.findtext('stddev')) == stddev
        barometer = model.find('.//sensor[@name="air_pressure_sensor"]')
        assert barometer.findtext('update_rate') == '50'
        assert float(barometer.findtext('air_pressure/pressure/noise/stddev')) == .01
    contract = json.loads((output / 'sensor_contract.json').read_text(encoding='utf-8'))
    assert contract['native_sensor_noise']['profile'] == 'px4_gz_x500_base'

def test_native_noise_matches_px4_stock_profile(tmp_path):
    path = os.environ.get('PX4_GZ_X500_BASE_SDF')
    if not path:
        pytest.skip('PX4_GZ_X500_BASE_SDF required for stock profile comparison')
    output = generate(tmp_path, '--dynamics', 'micro')
    actual = ET.parse(output / 'micro_0/model.sdf')
    stock = ET.parse(path)
    for name in ('imu_sensor', 'air_pressure_sensor'):
        observed = actual.find(f'.//sensor[@name="{name}"]')
        reference = stock.find(f'.//sensor[@name="{name}"]')
        expected = [(noise.get('type'), {p.tag:float(p.text) for p in noise})
                    for noise in reference.iter('noise')]
        assert expected
        assert [(noise.get('type'), {p.tag:float(p.text) for p in noise})
                for noise in observed.iter('noise')] == expected

def test_imported_model_keeps_its_native_noise(tmp_path):
    source = tmp_path / 'source.sdf'
    source.write_text('<sdf version="1.9"><model name="source"><link name="base_link">'
        '<sensor name="imu_sensor" type="imu"><imu><linear_acceleration><x>'
        '<noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise>'
        '</x></linear_acceleration></imu></sensor></link></model></sdf>', encoding='utf-8')
    output = generate(tmp_path, '--base-model', str(source))
    model = ET.parse(output / 'micro_0/model.sdf')
    assert float(model.findtext('.//sensor[@name="imu_sensor"]/imu/linear_acceleration/x/noise/stddev')) == .02


@pytest.mark.parametrize('rate', [250, 1000])
def test_explicit_micro_imu_rate_matches_native_integrator_without_changing_physics(tmp_path, rate):
    baseline = generate(tmp_path / 'baseline', '--dynamics', 'micro')
    candidate = generate(tmp_path / 'candidate', '--dynamics', 'micro', '--imu-rate-hz', str(rate))
    for vehicle in range(2):
        original = ET.parse(baseline / f'micro_{vehicle}/model.sdf')
        changed = ET.parse(candidate / f'micro_{vehicle}/model.sdf')
        sensor = changed.find('.//sensor[@name="imu_sensor"]')
        assert sensor.findtext('update_rate') == str(rate)
        sensor.find('update_rate').text = '250'
        assert ET.tostring(original.getroot()) == ET.tostring(changed.getroot())
    before = json.loads((baseline / 'sensor_contract.json').read_text(encoding='utf-8'))
    after = json.loads((candidate / 'sensor_contract.json').read_text(encoding='utf-8'))
    assert after.pop('native_imu_rate_hz') == rate
    assert after['px4_parameters'].pop('IMU_INTEG_RATE') == rate
    assert before == after
    commands = (candidate / 'px4_micro_reference.params').read_text(encoding='utf-8')
    assert f'param set IMU_INTEG_RATE {rate}\n' in commands


@pytest.mark.parametrize('arguments', [('--dynamics', 'fixture', '--imu-rate-hz', '1000'),
                                       ('--dynamics', 'micro', '--imu-rate-hz', '0'),
                                       ('--dynamics', 'micro', '--imu-rate-hz', '500')])
def test_invalid_or_unapplicable_imu_rate_is_rejected_before_output(tmp_path, arguments):
    with pytest.raises(subprocess.CalledProcessError):
        generate(tmp_path, *arguments)
    assert not (tmp_path / 'generated').exists()
