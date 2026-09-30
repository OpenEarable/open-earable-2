"""Exercise the firmware's timestamp resolver with a host C compiler."""

from pathlib import Path
import subprocess
import tempfile
import unittest


class TimestampTests(unittest.TestCase):
    def test_actual_phase_regulator_across_controller_wrap(self):
        root = Path(__file__).resolve().parents[2]
        datapath = (root / "src/audio/audio_datapath.c").read_text()
        function = datapath[datapath.index("static int32_t err_us_calculate"):
                            datapath.index("static void hfclkaudio_set")]
        source = '''#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define BLK_PERIOD_US 1000
#define AUDIO_SYNC_FIX_ENABLED(name) true
''' + function + '''
int main(void) {
    assert(err_us_calculate(200, 0xffffff00U) == 456);
    assert(err_us_calculate(0xffffff00U, 200) == -456);
    assert(err_us_calculate(12100, 1000) == 100);
    assert(err_us_calculate(1000, 12100) == -100);
    assert(err_us_calculate(1700, 1000) == -300);
    assert(err_us_calculate(1000, 1700) == 300);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.c").write_text(source)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)

    def test_missing_timestamps_and_wraparound(self):
        source = r'''
#include <assert.h>
#include "audio_sdu_timing.h"
int main(void) {
    struct audio_sdu_timing a = {0};
    uint32_t value = 123;
    assert(!audio_sdu_timing_resolve(&a, false, 1, 0, 100, 10000, &value));
    assert(value == 123);
    assert(audio_sdu_timing_resolve(&a, true, 65535, UINT32_MAX - 4999,
                                  UINT32_MAX - 999, 10000, &value));
    assert(audio_sdu_timing_resolve(&a, false, 0, 0, 9000, 10000, &value));
    assert(value == 5000);
    assert(audio_sdu_timing_resolve(&a, false, 9, 0, 99000, 10000, &value));
    assert(value == 95000);
    assert(!audio_sdu_timing_resolve(&a, false, 10, 0, 109000, 10000, &value));
    assert(!audio_sdu_timing_resolve(&a, false, 65535, 0, 1000, 10000, &value));
    assert(!audio_sdu_timing_resolve(&a, false, 65534, 0, 1000, 10000, &value));
    assert(!audio_sdu_timing_resolve(&a, false, 0, 0, 200001, 10000, &value));
    /* Zero is a legitimate controller timestamp if the valid bit is set. */
    assert(audio_sdu_timing_resolve(&a, true, 2, 0, 1000, 10000, &value));
    assert(value == 0);
    assert(audio_sdu_timing_resolve(&a, false, 3, 0, 11000, 10000, &value));
    assert(value == 10000);
    assert(a.sequence == 2); /* Estimates never extend the concealment limit. */
}
'''
        include = Path(__file__).resolve().parents[2] / "src/audio"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.c").write_text(source)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(include), str(path / "test.c"),
                            "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
