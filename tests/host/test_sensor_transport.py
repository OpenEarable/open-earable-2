import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class SensorTransportTest(unittest.TestCase):
    def test_packet_boundaries_timestamps_and_legacy_format(self):
        with tempfile.TemporaryDirectory() as temp:
            executable = str(pathlib.Path(temp) / "sensor_transport")
            sources = ROOT / "src/bluetooth/gatt_services"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-I", str(sources),
                str(sources / "sensor_transport.c"),
                str(ROOT / "tests/host/sensor_transport.c"), "-o", executable,
            ], check=True)
            subprocess.run([executable], check=True)
