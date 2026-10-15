"""Exercise flight-feedback expiry through the runner's loopback MAVLink input."""

import json
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


BINARY = Path(os.environ.get(
    "DRONE_COMPANION_BIN", "/home/devcontainers/drone-sim/build-companion/drone_companion_runner"))


def recorded_rows(path):
    data = path.read_text(encoding="utf-8")
    complete = data[:data.rfind("\n") + 1]
    return [json.loads(line) for line in complete.splitlines()]


def mavlink_packet(message_id, payload, crc_extra, system=1, component=1):
    header = bytes((len(payload), 0, system, component, message_id))
    checksum = 0xFFFF
    for byte in header + payload + bytes((crc_extra,)):
        value = byte ^ (checksum & 0xFF)
        value ^= (value << 4) & 0xFF
        checksum = ((checksum >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 0xFFFF
    return b"\xfe" + header + payload + struct.pack("<H", checksum)


@unittest.skipUnless(BINARY.is_file(), "drone_companion_runner binary required")
class FlightFeedbackIOTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.status.close)
        self.status.bind(("127.0.0.1", 18100))
        self.status.settimeout(0.1)
        self.mav = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.mav.close)
        self.mav.bind(("127.0.0.1", 0))
        self.mav.settimeout(0.02)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            self.runner_port = reservation.getsockname()[1]
        (self.root / "map.json").write_text(json.dumps({
            "spawns": [{"id": 240, "x": 0, "y": 0, "z": 0.516}], "walls": []}), encoding="utf-8")
        (self.root / "config.json").write_text(json.dumps({
            "mission": {"mode": "team_contact"}}), encoding="utf-8")
        self.errors = (self.root / "stderr.log").open("w", encoding="utf-8")
        self.addCleanup(self.errors.close)
        self.process = subprocess.Popen([
            str(BINARY), "--offboard", "--arm", "--vehicle", "240", "--map", str(self.root / "map.json"),
            "--config", str(self.root / "config.json"), "--runtime", str(self.root),
            "--mavlink-local-port", str(self.runner_port),
            "--mavlink-remote-port", str(self.mav.getsockname()[1]),
        ], stdout=subprocess.DEVNULL, stderr=self.errors)
        self.addCleanup(self.stop_runner)
        self.receive_status()

    def stop_runner(self):
        self.process.terminate()
        self.process.wait(timeout=5)

    def receive_status(self, predicate=lambda row: True, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.process.poll(), (self.root / "stderr.log").read_text(encoding="utf-8"))
            try:
                row = json.loads(self.status.recv(16384))
            except socket.timeout:
                continue
            if row.get("vehicle") == 240 and predicate(row):
                return row
        self.fail("Expected runner status did not arrive")

    def send_heartbeat(self, armed=False, system=1, component=1, system_status=4, main_mode=6):
        payload = struct.pack("<IBBBBB", main_mode << 16, 2, 12, 128 if armed else 0, system_status, 3)
        self.mav.sendto(mavlink_packet(0, payload, 50, system, component), ("127.0.0.1", self.runner_port))

    def send_landed(self, on_ground=True, system=1, component=1):
        payload = bytes((0, 1 if on_ground else 2))
        self.mav.sendto(mavlink_packet(245, payload, 130, system, component), ("127.0.0.1", self.runner_port))

    def grounded(self):
        self.send_heartbeat()
        self.send_landed()
        return self.receive_status(lambda row: row["landed"] and not row["armed"])

    def send_sensors(self, stamp):
        attitude = struct.pack("<Iffffff", stamp // 1000, 0, 0, 0, 0, 0, 0)
        self.mav.sendto(mavlink_packet(30, attitude, 39), ("127.0.0.1", self.runner_port))
        imu = struct.pack("<Q13fH", stamp, 0, 0, -9.80665, 0, 0, 0, 0, 0, 0, 1013.25, 0, 0, 20, 0xFFFF)
        self.mav.sendto(mavlink_packet(105, imu, 93), ("127.0.0.1", self.runner_port))
        for sample in [
            {"schema": 1, "kind": "height_assist", "source": "SIM_HEIGHT_ASSIST",
             "vehicle": 240, "sim_time_us": stamp, "z_world_m": 0.516},
            {"schema": 1, "kind": "optical_flow", "vehicle": 240, "sim_time_us": stamp,
             "vx_body_mps": 0, "vy_body_mps": 0, "quality": 1, "variance_m2s2": 0.001}]:
            self.mav.sendto(json.dumps(sample).encode("utf-8"), ("127.0.0.1", 16240))

    def test_ground_feedback_expires_while_heartbeats_continue(self):
        self.grounded()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            self.send_heartbeat()
            time.sleep(0.05)
        row = self.receive_status(lambda row: not row["landed"], timeout=0.5)
        self.assertFalse(row["armed"])
        self.assertTrue(row["heartbeat_fresh"])
        self.assertFalse(row["landed_fresh"])
        self.send_landed()
        self.receive_status(lambda row: row["landed"])

    def test_attitude_is_sampled_at_imu_time_and_old_attitude_is_ignored(self):
        self.grounded()
        started = time.monotonic()
        while time.monotonic() - started < 8.3:
            self.send_heartbeat()
            self.send_landed()
            stamp = 1_000_000 + int((time.monotonic() - started) * 1e6)
            self.send_sensors(stamp)
            time.sleep(0.02)
        source = (stamp // 1000 + 100) * 1000

        def send_sample(imu_stamp, attitude_stamp, yaw):
            attitude = struct.pack("<Iffffff", attitude_stamp // 1000, 0, 0, yaw, 0, 0, 1)
            self.mav.sendto(mavlink_packet(30, attitude, 39), ("127.0.0.1", self.runner_port))
            imu = struct.pack("<Q13fH", imu_stamp, 0, 0, -9.80665, 0, 0, 1, 0, 0, 0,
                              1013.25, 0, 0, 20, 0xFFFF)
            self.mav.sendto(mavlink_packet(105, imu, 93), ("127.0.0.1", self.runner_port))
            height = {"schema": 1, "kind": "height_assist", "source": "SIM_HEIGHT_ASSIST",
                      "vehicle": 240, "sim_time_us": imu_stamp, "z_world_m": 0.516}
            self.mav.sendto(json.dumps(height).encode("utf-8"), ("127.0.0.1", 16240))

        def expect_yaw(expected):
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                estimates = [row for row in recorded_rows(self.root / "companion_240.jsonl")
                             if row.get("kind") == "estimate" and row["sim_time_us"] >= source]
                if estimates and abs(estimates[-1]["yaw_rad"] - expected) < 1e-6:
                    return
                time.sleep(0.01)
            self.fail(f"Expected yaw {expected} at IMU exposure, latest: {estimates[-1] if estimates else None}")

        send_sample(source + 20_000, source, 0)
        expect_yaw(-0.02)
        send_sample(source + 30_000, source - 1000, 1)
        expect_yaw(-0.03)

    def test_stale_heartbeat_cannot_be_refreshed_by_ground_packets(self):
        self.grounded()
        deadline = time.monotonic() + 3.2
        while time.monotonic() < deadline:
            self.send_landed()
            time.sleep(0.05)
        row = self.receive_status(lambda row: row["armed"] and not row["landed"], timeout=0.5)
        self.assertFalse(row["heartbeat_fresh"])
        self.send_heartbeat()
        row = self.receive_status(lambda row: row["heartbeat_fresh"] and not row["armed"])
        self.assertFalse(row["landed"])
        self.send_landed()
        self.receive_status(lambda row: row["landed"])

    def test_arming_invalidates_previous_ground_feedback(self):
        self.grounded()
        self.send_heartbeat(armed=True)
        row = self.receive_status(lambda row: row["armed"])
        self.assertFalse(row["landed"])
        self.assertFalse(row["landed_fresh"])
        self.send_landed(on_ground=False)
        self.receive_status(lambda row: row["landed_fresh"] and not row["landed"])

    def test_native_failsafe_stops_task_and_remains_latched_after_recovery(self):
        self.grounded()
        self.send_heartbeat(armed=True, system_status=5, main_mode=2)
        row = self.receive_status(lambda row: row["unrecoverable"], timeout=1)
        self.assertTrue(row["native_failsafe_active"])
        self.assertTrue(row["native_failsafe_latched"])
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            self.send_heartbeat(armed=True, system_status=5, main_mode=2)
            time.sleep(0.05)
        rows = recorded_rows(self.root / "companion_240.jsonl")
        requests = [row["command"] for row in rows if row.get("kind") == "command_request"]
        self.assertTrue(requests)
        self.assertEqual(set(requests), {"LAND"})
        self.send_heartbeat(armed=True, main_mode=2)
        row = self.receive_status(lambda row: row.get("native_failsafe_active") is False)
        self.assertTrue(row["unrecoverable"])
        self.assertTrue(row["native_failsafe_latched"])

    def test_disarmed_preflight_warning_does_not_latch_task_failure(self):
        self.grounded()
        self.send_heartbeat(system_status=5)
        row = self.receive_status(lambda row: row.get("native_failsafe_active") is True)
        self.assertFalse(row["unrecoverable"])
        self.assertFalse(row["native_failsafe_latched"])

    def test_armed_feedback_gap_latches_even_when_queued_heartbeat_is_healthy(self):
        self.grounded()
        self.send_heartbeat(armed=True)
        self.send_landed(on_ground=False)
        self.receive_status(lambda row: row["armed"] and not row["landed"])
        self.process.send_signal(signal.SIGSTOP)
        try:
            deadline = time.monotonic() + 3.2
            while time.monotonic() < deadline:
                self.send_heartbeat(armed=True)
                time.sleep(0.1)
        finally:
            self.process.send_signal(signal.SIGCONT)
        row = self.receive_status(lambda row: row["unrecoverable"], timeout=1)
        self.assertTrue(row["heartbeat_gap_latched"])
        self.assertFalse(row["native_failsafe_latched"])
        self.send_heartbeat(armed=True)
        row = self.receive_status(lambda row: row["heartbeat_fresh"])
        self.assertTrue(row["unrecoverable"])
        self.assertTrue(row["heartbeat_gap_latched"])

    def test_short_setpoint_gap_latches_with_healthy_queued_heartbeats(self):
        self.grounded()
        started = time.monotonic()
        setpoint_seen = False
        while time.monotonic() - started < 11:
            self.send_heartbeat(armed=True)
            self.send_landed(on_ground=False)
            self.send_sensors(1_000_000 + int((time.monotonic() - started) * 1e6))
            try:
                packet = self.mav.recv(16384)
                message_id = int.from_bytes(packet[7:10], "little") if packet[0] == 0xFD else packet[5]
                setpoint_seen = setpoint_seen or message_id in (82, 84)
            except socket.timeout:
                pass
            if setpoint_seen:
                break
            time.sleep(0.01)
        self.assertTrue(setpoint_seen, "Runner never sent an OFFBOARD setpoint before the pause")
        self.process.send_signal(signal.SIGSTOP)
        try:
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                self.send_heartbeat(armed=True)
                time.sleep(0.05)
        finally:
            self.process.send_signal(signal.SIGCONT)
        row = self.receive_status(lambda row: row["unrecoverable"], timeout=1)
        self.assertTrue(row["setpoint_gap_latched"])
        self.assertFalse(row["heartbeat_gap_latched"])
        self.assertFalse(row["native_failsafe_latched"])
        deadline = time.monotonic() + 1.1
        while time.monotonic() < deadline:
            self.send_heartbeat(armed=True)
            self.send_landed(on_ground=False)
            self.send_sensors(1_000_000 + int((time.monotonic() - started) * 1e6))
            time.sleep(0.02)
        rows = recorded_rows(self.root / "companion_240.jsonl")
        event = next(index for index, item in enumerate(rows) if item.get("kind") == "unrecoverable")
        self.assertEqual(rows[event]["reason"], "OFFBOARD_STREAM_GAP")
        requests = [item["command"] for item in rows[event:] if item.get("kind") == "command_request"]
        self.assertTrue(requests)
        self.assertEqual(set(requests), {"LAND"})

    def test_grounded_member_in_abort_return_enters_native_recovery(self):
        for index in range(32):
            stamp = 1000000 + index * 50000
            self.send_heartbeat(armed=True)
            self.send_landed()
            height = {"schema": 1, "kind": "height_assist", "source": "SIM_HEIGHT_ASSIST",
                      "vehicle": 240, "sim_time_us": stamp, "z_world_m": 0.516}
            self.mav.sendto(json.dumps(height).encode("utf-8"), ("127.0.0.1", 16240))
            command = {"kind": "team_command", "vehicle": 240, "sequence": index + 1,
                       "sim_time_us": stamp, "phase": "ABORT_RETURN", "disarm": False,
                       "target_x_m": 2, "target_y_m": 0, "target_z_m": 1.5,
                       "target_yaw_rad": 0, "contact_speed_mps": 0}
            self.mav.sendto(json.dumps(command).encode("utf-8"), ("127.0.0.1", 18240))
            time.sleep(0.04)
        row = self.receive_status(lambda row: row["unrecoverable"], timeout=0.5)
        self.assertFalse(row["native_failsafe_latched"])
        rows = recorded_rows(self.root / "companion_240.jsonl")
        event = next(row for row in rows if row.get("kind") == "unrecoverable")
        self.assertEqual(event["phase"], "ABORT_RETURN")
        self.assertTrue(event["down"])

    def test_other_vehicle_and_component_cannot_replace_feedback(self):
        self.grounded()
        self.send_heartbeat(armed=True)
        self.send_landed(on_ground=False)
        self.receive_status(lambda row: row["armed"] and not row["landed"])
        for _ in range(10):
            self.send_heartbeat(system=2)
            self.send_landed(system=2)
            self.send_heartbeat(component=42)
            self.send_landed(component=42)
            time.sleep(0.01)
        for _ in range(20):
            row = self.receive_status()
            self.assertTrue(row["armed"])
            self.assertFalse(row["landed"])

    def disarm_requests(self, seconds, send_ground=False):
        requests = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.send_heartbeat(armed=True)
            if send_ground:
                self.send_landed()
            try:
                packet = self.mav.recv(16384)
            except socket.timeout:
                continue
            if packet[0] == 0xFD:
                message_id, payload = int.from_bytes(packet[7:10], "little"), packet[10:10 + packet[1]]
            elif packet[0] == 0xFE:
                message_id, payload = packet[5], packet[6:6 + packet[1]]
            else:
                continue
            if message_id == 76 and int.from_bytes(payload[28:30], "little") == 400:
                requests.append(struct.unpack_from("<ff", payload.ljust(33, b"\0")))
        return requests

    def test_team_disarm_requires_current_ground_feedback_without_imu(self):
        self.send_heartbeat(armed=True)
        self.send_landed()
        self.receive_status(lambda row: row["armed"] and row["landed"])
        self.disarm_requests(0.7)
        command = {"kind": "team_command", "vehicle": 240, "sequence": 1,
                   "sim_time_us": 0, "phase": "LAND_ALL", "disarm": True,
                   "target_x_m": 0, "target_y_m": 0, "target_z_m": 0.5,
                   "target_yaw_rad": 0, "contact_speed_mps": 0}
        self.mav.sendto(json.dumps(command).encode("utf-8"), ("127.0.0.1", 18240))
        self.assertEqual(self.disarm_requests(0.6), [])
        command["sequence"] = 2
        self.mav.sendto(json.dumps(command).encode("utf-8"), ("127.0.0.1", 18240))
        self.assertEqual(self.disarm_requests(0.7, send_ground=True), [(0.0, 0.0)])

    def test_abort_mission_while_holding_stops_arm_retries(self):
        started = time.monotonic()
        sequence = 0

        def pump(phase):
            nonlocal sequence
            sequence += 1
            stamp = 1000000 + int((time.monotonic() - started) * 1e6)
            self.send_heartbeat()
            self.send_landed()
            self.send_sensors(stamp)
            command = {"kind": "team_command", "vehicle": 240, "sequence": sequence,
                       "sim_time_us": stamp, "phase": "TAKEOFF" if phase == "TAKEOFF" else "HOLD",
                       "mission_phase": phase, "disarm": False,
                       "target_x_m": 0, "target_y_m": 0, "target_z_m": 1.5,
                       "target_yaw_rad": 0, "contact_speed_mps": 0}
            self.mav.sendto(json.dumps(command).encode("utf-8"), ("127.0.0.1", 18240))
            time.sleep(0.02)

        path = self.root / "companion_240.jsonl"
        while time.monotonic() - started < 11:
            pump("TAKEOFF")
            rows = recorded_rows(path)
            arms = [row for row in rows if row.get("kind") == "command_request" and row.get("command") == "ARM"]
            if arms:
                break
        else:
            self.fail("Mock sensor stream never became ready for an arm request")
        baseline = len(arms)
        deadline = time.monotonic() + 1.4
        while time.monotonic() < deadline:
            pump("ABORT_RETURN")
        rows = recorded_rows(path)
        arms = [row for row in rows if row.get("kind") == "command_request" and row.get("command") == "ARM"]
        self.assertEqual(len(arms), baseline)


if __name__ == "__main__":
    unittest.main()
