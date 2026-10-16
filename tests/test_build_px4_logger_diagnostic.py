"""独立 PX4 logger 诊断构建的链接参数边界。"""

import importlib.util
from pathlib import Path, PurePosixPath
import subprocess
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('logger_builder', Path(__file__).resolve().parents[1] /
                                             'scripts/build_px4_logger_diagnostic.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class DiagnosticLinkTests(unittest.TestCase):
    def test_control_response_profile_preserves_other_topics_and_uses_five_ms(self):
        source = ('void LoggedTopics::add_default_topics()\n{\n\tadd_topic("trajectory_setpoint", 200);\n}\n'
                  'void LoggedTopics::add_estimator_replay_topics()\n{\n\tadd_topic("sensor_combined");\n}\n')
        changed = builder.configure_control_response_log(source)
        self.assertTrue(changed.startswith(source.split('void LoggedTopics::add_estimator_replay_topics')[0]))
        self.assertIn('add_topic("sensor_combined");', changed)
        for name in builder.CONTROL_RESPONSE_TOPICS:
            self.assertEqual(changed.count(f'add_topic("{name}", 5);'), 1)

    def test_missing_or_ambiguous_control_profile_is_rejected(self):
        for source in ('none', 'void LoggedTopics::add_estimator_replay_topics()\n{\n' * 2):
            with self.assertRaisesRegex(ValueError, 'locate estimator'):
                builder.configure_control_response_log(source)

    def test_imu_retention_requires_base_retention_before_any_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'output'
            with self.assertRaisesRegex(ValueError, 'requires visual'):
                builder.build_diagnostic(Path('missing-sdk'), Path('missing-build'), output,
                                         Path('missing-patch'), imu_replay=True)
            self.assertFalse(output.exists())

    def test_patch_inside_parent_repository_only_changes_copied_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '--quiet', str(root)], check=True)
            original = root / 'input.txt'
            original.write_bytes(b'before\n')
            source = root / 'build/diagnostic/source'
            source.mkdir(parents=True)
            (source / 'input.txt').write_bytes(original.read_bytes())
            patch = root / 'diagnostic.patch'
            patch.write_bytes(b'--- a/input.txt\n+++ b/input.txt\n@@ -1 +1 @@\n-before\n+after\n')
            builder.apply_diagnostic_patch(source, patch)
            self.assertEqual((source / 'input.txt').read_bytes(), b'after\n')
            self.assertEqual(original.read_bytes(), b'before\n')

    def test_compound_ninja_command_preserves_space_arguments(self):
        raw = (': && /usr/bin/c++ "source with space.o" -o bin/px4 '
               'src/modules/logger/libmodules__logger.a "other library.a" '
               '&& cd bin && /usr/bin/cmake -E create_symlink px4 px4-logger')
        result = builder.link_command(raw, PurePosixPath('/tmp/trace build/logger.a'), PurePosixPath('/tmp/trace build/px4'))
        self.assertEqual(result, ['/usr/bin/c++', 'source with space.o', '-o', '/tmp/trace build/px4',
                                  '/tmp/trace build/logger.a', 'other library.a'])

    def test_no_recognizable_link_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '单个可识别'):
            builder.link_command('cmake -E echo none', Path('a'), Path('b'))

    def test_multiple_link_steps_are_rejected(self):
        raw = '/usr/bin/c++ -o a x && /usr/bin/c++ -o b y'
        with self.assertRaisesRegex(ValueError, '单个可识别'):
            builder.link_command(raw, Path('a'), Path('b'))

    def test_missing_or_repeated_logger_archive_is_rejected(self):
        for count in (0, 2):
            raw = '/usr/bin/c++ -o bin/px4 ' + 'src/modules/logger/libmodules__logger.a ' * count
            with self.subTest(count=count), self.assertRaisesRegex(ValueError, '单个 logger'):
                builder.link_command(raw, Path('a'), Path('b'))

    def test_extra_shell_operation_in_link_step_is_rejected(self):
        raw = '/usr/bin/c++ -o bin/px4 src/modules/logger/libmodules__logger.a | other'
        with self.assertRaisesRegex(ValueError, 'shell'):
            builder.link_command(raw, Path('a'), Path('b'))


if __name__ == '__main__':
    unittest.main()
