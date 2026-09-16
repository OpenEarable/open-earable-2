"""Exercise production SD flush/close code at the battery shutdown boundary.

FatFs f_write may return FR_OK with zero bytes written on a full volume;
Zephyr fatfs_write and SDCardManager::write forward that zero. The test copies
the real method bodies, substitutes only storage/RTOS services, and imposes an
external timeout so a regression cannot hang the test runner.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


def method(source, name):
    match = re.search(r"^int SDLogger::" + name + r"\(\)\s*\{", source, re.M)
    if not match:
        raise AssertionError(f"Production SDLogger::{name} missing")
    depth = 1
    end = source.index("{", match.start()) + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


PRELUDE = r'''
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <string>
#define K_FOREVER -1
#define LOG_ERR(...)
#define LOG_INF(...)
#define LOG_DBG(...)
int g_stop_writing=0, g_sd_removed=0, logger_sig=0, ring_mutex=0, file_mutex=0;
int count_max_buffer_fill=0, signal_resets=0, faults=0;
uint8_t bytes[512]{};
struct Ring { uint32_t size=512, claimed=0; } ring_buffer;
void atomic_set(int *p,int v) { *p=v; }
void atomic_clear(int *p) { *p=0; }
void k_poll_signal_raise(int *,int) {}
void k_mutex_lock(int *,int) {}
void k_mutex_unlock(int *) {}
void k_yield() {}
void reset_logger_signal() { ++signal_resets; }
uint32_t ring_buf_size_get(Ring *r) { return r->size; }
uint32_t ring_buf_get_claim(Ring *r,uint8_t **p,uint32_t n) {
    assert(r->claimed == 0);
    *p=bytes;
    r->claimed=n < r->size ? n : r->size;
    return r->claimed;
}
void ring_buf_get_finish(Ring *r,uint32_t n) {
    assert(n <= r->claimed); r->size-=n; r->claimed=0;
}
constexpr int SD_FAULT=1;
struct Indicator { void set_sd_state(int state) { assert(state==SD_FAULT); ++faults; } } state_indicator;
struct Card {
    std::string scenario;
    int writes=0, closes=0;
    bool is_mounted() { return true; }
    int write(char *,size_t *requested,bool sync) {
        assert(!sync);
        ++writes;
        if (scenario == "full") return 0;
        if (scenario == "partial_full") return writes == 1 ? 128 : 0;
        if (scenario == "error") return -EIO;
        return *requested < 128 ? *requested : 128;
    }
    int close_file() { ++closes; return scenario == "close_error" ? -EIO : 0; }
} card;
struct SDLogger {
    Card *sd_card=&card;
    bool is_open=true;
    int flush();
    int end();
};
'''

TESTS = r'''
int main(int argc,char **argv) {
    assert(argc == 2);
    card.scenario=argv[1];
    SDLogger logger;
    if (card.scenario == "empty") ring_buffer.size=0;
    int result=logger.end();
    assert(card.closes == 1);
    assert(ring_buffer.claimed == 0);
    if (card.scenario == "full" || card.scenario == "partial_full") {
        assert(result == -ENOSPC);
        assert(faults == 1);
        assert(card.writes == (card.scenario == "full" ? 1 : 2));
        assert(ring_buffer.size == (card.scenario == "full" ? 512 : 384));
    } else if (card.scenario == "error") {
        assert(result == -EIO && card.writes == 1 && faults == 1);
        assert(ring_buffer.size == 512);
    } else if (card.scenario == "close_error") {
        assert(result == -EIO && logger.is_open && ring_buffer.size == 0);
        return 0;
    } else {
        assert(result == 0 && ring_buffer.size == 0 && faults == 0);
    }
    assert(!logger.is_open && !g_stop_writing && signal_resets == 1);
}
'''


class SDShutdownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which(os.environ.get("CXX", "c++"))
        if compiler is None:
            raise unittest.SkipTest("A C++17 compiler is required")
        cls.temp = tempfile.TemporaryDirectory(prefix="sd-shutdown-test-")
        cls.addClassCleanup(cls.temp.cleanup)
        base = Path(cls.temp.name)
        source = (ROOT / "src/SD_Card/SDLogger/SDLogger.cpp").read_text()
        harness = base / "test.cpp"
        harness.write_text(PRELUDE + method(source, "flush") + method(source, "end") + TESTS)
        cls.binary = base / "test"
        subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        str(harness), "-o", str(cls.binary)], check=True)

    def run_scenario(self, scenario):
        try:
            result = subprocess.run([str(self.binary), scenario], timeout=1,
                                    capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            self.fail(f"Shutdown never finished after {scenario} storage write")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_full_card_shutdown(self):
        self.run_scenario("full")

    def test_full_card_after_partial_write(self):
        self.run_scenario("partial_full")

    def test_storage_error_still_closes(self):
        self.run_scenario("error")

    def test_partial_writes_make_progress(self):
        self.run_scenario("partial")

    def test_empty_buffer(self):
        self.run_scenario("empty")

    def test_close_error_is_preserved(self):
        self.run_scenario("close_error")


if __name__ == "__main__":
    unittest.main()
