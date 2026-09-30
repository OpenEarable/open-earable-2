"""Check the same startup gate and sample scaling used by the I2S callback."""

from pathlib import Path
import subprocess
import tempfile
import unittest


class StartupTests(unittest.TestCase):
    def test_settling_restart_and_full_scale_fade(self):
        source = r'''
#include <assert.h>
#include <limits.h>
#include "audio_startup.h"
int main(void) {
    struct audio_startup gate = {0};
    for (int i = 0; i < 19; ++i) assert(!audio_startup_ready(&gate, true));
    /* One missing block or unlocked measurement restarts the settling time. */
    assert(!audio_startup_ready(&gate, false));
    for (int i = 0; i < 19; ++i) assert(!audio_startup_ready(&gate, true));
    assert(audio_startup_ready(&gate, true));
    /* Ongoing presentation measurements must not repeatedly mute music. */
    assert(audio_startup_ready(&gate, false));
    for (uint32_t n = 1; n <= 240; ++n) {
        uint32_t gain = audio_startup_gain(&gate, 240);
        assert(gain == n);
        assert(audio_startup_scale(INT32_MAX, gain, 240) ==
               (int64_t)INT32_MAX * n / 240);
        assert(audio_startup_scale(INT32_MIN, gain, 240) ==
               (int64_t)INT32_MIN * n / 240);
        assert(audio_startup_scale(INT16_MIN, gain, 240) >= INT16_MIN);
        assert(audio_startup_scale(0, gain, 240) == 0);
    }
    assert(audio_startup_gain(&gate, 240) == 240);
    assert(audio_startup_scale(INT16_MIN, 240, 240) == INT16_MIN);
    assert(audio_startup_scale(INT32_MIN, 240, 240) == INT32_MIN);
    gate = (struct audio_startup){0}; /* New stream, even if I2S stayed on. */
    assert(!audio_startup_ready(&gate, false));
    assert(!gate.open && gate.fade_frames == 0);
}
'''
        include = Path(__file__).resolve().parents[2] / "src/audio"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.c").write_text(source)
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=undefined", "-I", str(include),
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
