"""Exercise image selection and flash entry points without connecting hardware."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from intelhex import IntelHex, AddressOverlapError
import yaml

from prepare_images import prepare_images


class FlashTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="flash-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.build = self.root / "build with spaces"
        self.output = self.root / "images"
        self.tools = Path(__file__).resolve().parent
        self.stub_dir = self.root / "bin"
        self.stub_dir.mkdir()
        self.log = self.root / "commands.jsonl"
        stub = self.stub_dir / "nrfjprog"
        stub.write_text(f"#!{sys.executable}\n" + '''
import json, os, pathlib, sys
args = sys.argv[1:]
with open(os.environ['FLASH_TEST_LOG'], 'a') as log:
    log.write(json.dumps(args) + '\\n')
if '--readuicr' in args:
    pathlib.Path(args[args.index('--readuicr') + 1]).write_text(':00000001FF\\n')
if os.environ.get('FLASH_TEST_FAIL') in args:
    sys.exit(42)
''')
        stub.chmod(0o755)
        sleep = self.stub_dir / "sleep"
        sleep.write_text("#!/bin/sh\nexit 0\n")
        sleep.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.stub_dir) + os.pathsep + os.environ["PATH"],
                        PYTHON=sys.executable, FLASH_TEST_LOG=str(self.log),
                        TMPDIR=str(self.root), TMP=str(self.root), TEMP=str(self.root))

    def fixture(self, fota):
        # Non-default application name and paths with spaces are intentional.
        images = [("custom-app", "APP", 0x10000 if fota else 0, "zephyr.signed.hex" if fota else "zephyr.hex", 11),
                  ("ipc_radio", "NET", 0x1008800 if fota else 0x1000000,
                   "../../signed_by_b0_ipc_radio.hex" if fota else "zephyr.hex", 22)]
        if fota:
            images += [("mcuboot", "APP", 0, "zephyr.hex", 33),
                       ("b0n", "NET", 0x1000000, "../../b0n_provision_merged.hex", 44)]
        for name, core, address, filename, value in images:
            z = self.build / name / "zephyr"
            z.mkdir(parents=True)
            config = f"CONFIG_SOC_NRF5340_CPU{core}=y\n"
            if name == "custom-app" and fota:
                config += "CONFIG_BOOTLOADER_MCUBOOT=y\nCONFIG_AUDIO_BT_MGMT_DFU=y\n"
            (z / ".config").write_text(config)
            (z / "runners.yaml").write_text(yaml.safe_dump({"config": {"hex_file": filename}}))
            IntelHex({address: value}).write_hex_file(str(z / filename))
        (self.build / "domains.yaml").write_text(yaml.safe_dump({
            "default": "custom-app", "domains": [{"name": row[0]} for row in images]}))

    def run_script(self, fota, extra=(), powershell=False):
        if powershell:
            pwsh = os.environ.get("PWSH") or shutil.which("pwsh")
            if not pwsh:
                self.skipTest("PowerShell not installed")
            script = str(self.tools / "flash_fota.ps1").replace("'", "''")
            cmd = [pwsh, "-NoProfile", "-Command", "function Start-Sleep {} ; & '" + script +
                   "' -Snr 123 -BuildDir '" + str(self.build).replace("'", "''") +
                   "' -Python '" + sys.executable.replace("'", "''") + "' " + " ".join(extra) +
                   "; exit $LASTEXITCODE"]
        else:
            cmd = ["bash", str(self.tools / ("flash_fota.sh" if fota else "flash.sh")),
                   "--snr", "123", "--build-dir", str(self.build), *extra]
        result = subprocess.run(cmd, env=self.env, cwd=self.root, text=True, capture_output=True)
        calls = [json.loads(s) for s in self.log.read_text().splitlines()] if self.log.exists() else []
        return result, calls

    def check_sequence(self, fota, powershell=False):
        self.fixture(fota)
        result, calls = self.run_script(fota, powershell=powershell)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("--readuicr", calls[0])
        self.assertIn("CP_APPLICATION", calls[0])
        self.assertIn("CP_NETWORK", calls[1])
        self.assertIn("CP_APPLICATION", calls[2])
        self.assertEqual(sum("--chiperase" in c for c in calls), 2)
        for call in calls[1:4]:
            self.assertIn("--verify", call)
        self.assertIn("uicr_backup.hex", calls[3][calls[3].index("--program") + 1])
        self.assertIn("--pinreset", calls[4])
        self.assertIn("--reset", calls[5])
        self.assertIn("CP_APPLICATION", calls[5])
        self.assertEqual(len(calls), 6)

    def test_signed_images_and_boot_provisioning(self):
        self.fixture(True)
        prepare_images(self.build, self.output, True)
        self.assertEqual(IntelHex(str(self.output / "merged.hex")).todict(), {0: 33, 0x10000: 11})
        self.assertEqual(IntelHex(str(self.output / "merged_CPUNET.hex")).todict(),
                         {0x1000000: 44, 0x1008800: 22})

    def test_missing_input_prevents_device_access(self):
        self.fixture(True)
        (self.build / "signed_by_b0_ipc_radio.hex").unlink()
        result, calls = self.run_script(True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(calls, [])

    def test_wrong_build_type_prevents_device_access(self):
        self.fixture(False)
        result, calls = self.run_script(True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(calls, [])

    def test_overlap_rejected(self):
        self.fixture(True)
        IntelHex({0: 99}).write_hex_file(str(self.build / "custom-app/zephyr/zephyr.signed.hex"))
        with self.assertRaises(AddressOverlapError):
            prepare_images(self.build, self.output, True)
        self.assertFalse(self.output.exists())

    def test_uicr_in_input_rejected(self):
        self.fixture(False)
        IntelHex({0xff80f4: 1}).write_hex_file(str(self.build / "custom-app/zephyr/zephyr.hex"))
        with self.assertRaises(ValueError):
            prepare_images(self.build, self.output)

    def test_standard_shell(self):
        self.check_sequence(False)

    def test_fota_shell(self):
        self.check_sequence(True)

    def test_fota_powershell(self):
        self.check_sequence(True, powershell=True)

    def test_failed_program_stops_before_identity_or_reset(self):
        self.fixture(True)
        self.env["FLASH_TEST_FAIL"] = "--program"
        for powershell in (False, True):
            with self.subTest(powershell=powershell):
                self.log.unlink(missing_ok=True)
                result, calls = self.run_script(True, powershell=powershell)
                self.assertEqual(result.returncode, 42, result.stdout + result.stderr)
                self.assertEqual(len(calls), 2)
                backup = Path(calls[0][calls[0].index("--readuicr") + 1])
                self.assertTrue(backup.exists())

    def test_side_and_hardware_options(self):
        self.fixture(False)
        result, calls = self.run_script(False, ("--right", "--standalone", "--hw", "2.0.1"))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(any("--readuicr" in c for c in calls))
        values = {c[c.index("--memwr") + 1]: c[c.index("--val") + 1] for c in calls if "--memwr" in c}
        self.assertEqual(values, {"0x00FF80F4": "1", "0x00FF80FC": "0", "0x00FF8100": "0x02000100"})


if __name__ == "__main__":
    unittest.main()
