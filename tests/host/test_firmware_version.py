"""Run with python3 -m unittest discover -s tests/host."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class FirmwareVersionTests(unittest.TestCase):
    def test_release_version_does_not_follow_an_old_ancestor_tag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('version.cmake', 'version.h.in'):
                shutil.copy(ROOT / name, root / name)
            (root / 'VERSION').write_text(
                'VERSION_MAJOR = 2\nVERSION_MINOR = 2\nPATCHLEVEL = 9\n'
                'VERSION_TWEAK = 0\nEXTRAVERSION =\n')
            (root / 'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.20)\n'
                'project(version_test NONE)\ninclude(version.cmake)\n')
            (root / '.gitignore').write_text('build/\n')

            def git(*args):
                return subprocess.check_output(['git', '-C', str(root), *args], text=True)

            git('init', '-q')
            git('config', 'user.name', 'Version Test')
            git('config', 'user.email', 'version-test@example.invalid')
            git('add', '.')
            git('commit', '-qm', 'ancestor')
            git('tag', 'v2.2.7')
            git('commit', '--allow-empty', '-qm', 'release candidate')

            def version():
                env = {k: v for k, v in os.environ.items() if k not in (
                    'GITHUB_EVENT_NAME', 'GITHUB_REF', 'GITHUB_HEAD_REF',
                    'GITHUB_REF_NAME', 'SYSTEM_PULLREQUEST_PULLREQUESTNUMBER', 'CHANGE_ID')}
                subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')],
                               env=env, check=True, capture_output=True)
                return (root / 'build/include/generated/version.h').read_text()

            self.assertIn('"2.2.9-dev.1+g', version())
            git('tag', 'v2.2.9')
            self.assertIn('"2.2.9"', version())
            with (root / 'CMakeLists.txt').open('a') as f:
                f.write('# dirty test\n')
            self.assertIn('.dirty"', version())


if __name__ == '__main__':
    unittest.main()
