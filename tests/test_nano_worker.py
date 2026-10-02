"""SSH transport test doubles validate deployment and stream handling, not hardware."""

import hashlib
import io
import json
from app.workers import NanoWorker
from app.data import demonstration


class Exit:
    def recv_exit_status(self):
        return 0


class Output(io.BytesIO):
    channel = Exit()


class Sftp:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def get_channel(self):
        return self

    def settimeout(self, t):
        pass

    def put(self, local, remote):
        self.path = local

    def chmod(self, path, mode):
        pass


class Stream:
    def __init__(self):
        frame = demonstration(1)[0]
        frame["source"] = "NANO"
        self.payload = (
            b"DRONELAB_PID:1234"
            + bytes([10])
            + json.dumps(frame).encode()
            + bytes([10])
        )

    def exec_command(self, command):
        self.command = command

    def recv_ready(self):
        return bool(self.payload)

    def recv(self, count):
        data = self.payload
        self.payload = b""
        return data

    def recv_stderr_ready(self):
        return False

    def exit_status_ready(self):
        return not self.payload

    def recv_exit_status(self):
        return 0

    def close(self):
        pass


class Client:
    arch = "riscv64"

    def __init__(self):
        self.closed = False
        self.stream = Stream()

    def set_missing_host_key_policy(self, policy):
        pass

    def connect(self, *args, **kwargs):
        pass

    def get_transport(self):
        return self

    def set_keepalive(self, seconds):
        pass

    def is_active(self):
        return True

    def open_session(self, **kwargs):
        return self.stream

    def open_sftp(self):
        return Sftp()

    def exec_command(self, command, **kwargs):
        if command.startswith("uname"):
            data = (
                self.arch + chr(10) + "NAME=Buildroot" + chr(10) + "/dev/i2c-1"
            ).encode()
        elif command.startswith("mv "):
            data = (
                hashlib.sha256(b"test binary").hexdigest()
                + "  /tmp/drone-lab/drone-nano"
            ).encode()
        else:
            data = b""
        return None, Output(data), Output(b"")

    def close(self):
        self.closed = True


def test_nano_upload_and_parse(tmp_path, monkeypatch):
    import app.workers as workers

    binary = tmp_path / "build" / "nano-riscv64" / "drone-nano"
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"test binary")
    client = Client()
    monkeypatch.setattr(workers.paramiko, "SSHClient", lambda: client)
    worker = NanoWorker(
        tmp_path,
        dict(host="test", port=22, user="root"),
        "password",
        "stream",
        dict(sensors=[dict(bus="/dev/i2c-1", address="0x29")], hz=10),
    )
    records = []
    errors = []
    worker.record.connect(records.append)
    worker.failure.connect(errors.append)
    worker.run()
    assert len(records) == 1 and records[0]["source"] == "NANO" and not errors
    assert client.closed and worker.password == ""
    assert "--sensor /dev/i2c-1:0x29" in client.stream.command


def test_nano_reject_wrong_arch(tmp_path, monkeypatch):
    import app.workers as workers

    client = Client()
    client.arch = "aarch64"
    monkeypatch.setattr(workers.paramiko, "SSHClient", lambda: client)
    worker = NanoWorker(
        tmp_path, dict(host="test", port=22, user="root"), "secret", "deploy"
    )
    errors = []
    worker.failure.connect(errors.append)
    worker.run()
    assert len(errors) == 1 and "aarch64" in errors[0] and client.closed
