import json
import socket
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tof_bridge import ClockModel, MavlinkParser, convert_nano_record, make_timesync_request, x25_crc


def record(status=5, vehicle=None):
    value = {
        "schema": 1,
        "type": "tof",
        "source": "NANO",
        "sensor": 1,
        "source_time_us": 123456,
        "time_basis": "nano_data_ready_poll",
        "distance_mm": [1000] * 64,
        "status": [status] * 64,
    }
    if vehicle is not None:
        value["vehicle"] = vehicle
    return value


class BridgeTests(unittest.TestCase):
    def test_conversion_is_meters_and_uses_receive_clock(self):
        clock = ClockModel("monotonic", receive_error_us=2500)
        result = convert_nano_record(record(), 2, 3, {5, 9}, 5_000_000_000, clock)
        self.assertEqual(result["kind"], "tof")
        self.assertEqual(result["vehicle"], 2)
        self.assertEqual(result["distance_m"][0], 1.0)
        self.assertEqual(result["valid"], [1] * 64)
        self.assertEqual(result["sim_time_us"], 5_000_000)
        self.assertEqual(result["source_time_us"], 123456)
        self.assertEqual(result["time_error_limit_us"], 2500)

    def test_status_and_sensor_validation(self):
        clock = ClockModel("monotonic")
        result = convert_nano_record(record(0), 0, 3, {5, 9}, 2_000_000_000, clock)
        self.assertEqual(result["valid"], [0] * 64)
        zero_or_negative = record()
        zero_or_negative["distance_mm"] = [0, -1] + [1000] * 62
        converted = convert_nano_record(zero_or_negative, 0, 3, {5, 9}, 2_000_000_000, clock)
        self.assertEqual(converted["valid"][:2], [0, 0])
        self.assertEqual(converted["valid"][2:], [1] * 62)
        bad = record()
        bad["sensor"] = 3
        self.assertIsNone(convert_nano_record(bad, 0, 3, {5, 9}, 2_000_000_000, clock))

    def test_vehicle_identity_is_checked_when_present(self):
        clock = ClockModel("monotonic")
        self.assertIsNotNone(convert_nano_record(record(vehicle=1), 1, 3, {5, 9}, 2_000_000_000, clock))
        self.assertIsNone(convert_nano_record(record(vehicle=1), 2, 3, {5, 9}, 2_000_000_000, clock))

    def test_timesync_clock_requires_valid_sample_and_reports_bound(self):
        clock = ClockModel("timesync", max_rtt_us=20_000, receive_error_us=2000)
        self.assertIsNone(clock.stamp(2_000_000_000))
        self.assertTrue(clock.add_timesync_response(1_000_000_000, 1_004_000_000, 1_002_500_000))
        stamped = clock.stamp(2_000_000_000)
        self.assertEqual(stamped["time_basis"], "fc_boot_timesync")
        self.assertEqual(stamped["sim_time_us"], 2_000_500)
        self.assertEqual(stamped["time_error_limit_us"], 4000)

    def test_mavlink_timesync_frame_parser(self):
        frame = make_timesync_request(200, 191, 987654321)
        parser = MavlinkParser()
        self.assertEqual(list(parser.feed(frame[:7])), [])
        decoded = list(parser.feed(frame[7:]))
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0][1]["msgid"], 111)
        self.assertEqual(decoded[0][1]["ts1_ns"], 987654321)

    def test_mavlink_parser_keeps_non_timesync_frames_for_forwarding(self):
        payload = bytes([1, 2, 3])
        header = bytes([len(payload), 0, 0, 7, 1, 1]) + (30).to_bytes(3, "little")
        frame = bytes([0xFD]) + header + payload + x25_crc(header + payload + bytes([50])).to_bytes(2, "little")
        parsed = list(MavlinkParser().feed(frame))
        self.assertEqual(parsed, [(frame, {"msgid": 30, "payload": payload})])

    def test_udp_closed_loop_without_serial(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(3)
        port = receiver.getsockname()[1]
        bridge = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).with_name("tof_bridge.py")),
                "--vehicle",
                "1",
                "--tof-udp-port",
                str(port),
                "--time-mode",
                "monotonic",
                "--mavlink-udp-listen-port",
                str(port + 1),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            bridge.stdin.write(json.dumps(record()) + "\n")
            bridge.stdin.flush()
            payload, _ = receiver.recvfrom(65535)
            output = json.loads(payload)
            self.assertEqual(output["kind"], "tof")
            self.assertEqual(output["vehicle"], 1)
            self.assertEqual(output["distance_m"][0], 1.0)
        finally:
            bridge.stdin.close()
            bridge.wait(timeout=3)
            bridge.stderr.close()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
