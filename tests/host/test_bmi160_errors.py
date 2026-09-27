import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class Bmi160ErrorTests(unittest.TestCase):
    def test_failed_reads_do_not_use_invalid_register_data(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = str(pathlib.Path(directory) / 'bmi160-errors')
            subprocess.run(['cc', '-std=c11',
                            '-I' + str(ROOT / 'src/SensorManager/BMX160/bosch'),
                            str(ROOT / 'tests/host/bmi160_errors.c'), '-o', binary], check=True)
            subprocess.run([binary], check=True)
