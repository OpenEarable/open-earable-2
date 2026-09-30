"""Exercise clock reconstruction at real RTC, HF, and controller boundaries."""
from pathlib import Path
import subprocess
import tempfile
import unittest

class ClockTests(unittest.TestCase):
    def test_edges_and_counter_wraps(self):
        source = r'''
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "audio_sync_clock.h"
int main(void) {
    /* Just before/after a one-second RTC anchor. */
    assert(audio_sync_clock_from_anchor(32768, 0, 100, 99) == 999999);
    assert(audio_sync_clock_from_anchor(32768, 0, 100, 101) == 1000001);
    /* HF wraps independently of the 512-second RTC epoch. */
    assert(audio_sync_clock_from_anchor(32768, 0, 5, UINT32_MAX-4) == 999990);
    assert(audio_sync_clock_from_anchor(32768, 0, UINT32_MAX-4, 5) == 1000010);
    assert(audio_sync_clock_from_anchor(0, 1, 100, 84) == 511999984);
    assert(audio_sync_clock_from_anchor(0, 1, 100, 116) == 512000016);
    /* Controller timestamps intentionally wrap at 2^32 microseconds. */
    assert(audio_sync_clock_from_anchor(6553600, 8, 123, 123) == 1032704);
    assert(audio_sync_clock_from_anchor(0, 0, 100, 80) == UINT32_MAX-19);
    /* Sweep both sides of RTC edges with independent HF phase, up to 1%
     * HFINT rate error, and a nearby coherent anchor. A delayed RTC capture
     * would add an entire tick here; reconstruction stays within quantization.
     */
    for (unsigned tick=1; tick<10000; ++tick) {
        double anchor_time=tick*1000000.0/32768.0;
        for (int delta=-30; delta<=30; ++delta) {
            for (int ppm=-10000; ppm<=10000; ppm+=10000) {
                double rate=1.0+ppm/1000000.0;
                uint32_t anchor=(uint32_t)floor(anchor_time*rate+123.25);
                uint32_t frame=(uint32_t)floor((anchor_time+delta)*rate+123.25);
                uint32_t actual=audio_sync_clock_from_anchor(tick,0,anchor,frame);
                int64_t expected=(int64_t)floor(anchor_time+delta);
                assert(llabs((int32_t)(actual-(uint32_t)expected))<=1);
            }
        }
    }
}
'''
        include=Path(__file__).resolve().parents[2]/'src/modules'
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'test.c').write_text(source)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(include),str(p/'test.c'),'-lm','-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)

    def test_reference_crossing_and_pending_overflow(self):
        root=Path(__file__).resolve().parents[2]
        firmware=(root/'src/modules/audio_sync_timer.c').read_text()
        actual=firmware[firmware.index(' static uint32_t timestamp_from_anchor_get'):
                        firmware.index(' uint32_t audio_sync_timer_capture(void)')]
        source=r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "audio_sync_clock.h"
#define NRF_TIMER1 1
#define NRF_TIMER_TASK_CAPTURE3 3
#define NRF_RTC_EVENT_OVERFLOW 1
#define LOG_WRN(...) (++warnings)
static struct {int p_reg;} audio_sync_lf_timer_instance;
static uint32_t num_rtc_overflows, ticks, anchor, now, next_edge;
static unsigned waits, warnings, lock_depth;
static bool pending, running, edge_on_capture;
static unsigned irq_lock(void) {return lock_depth++;}
static void irq_unlock(unsigned key) {assert(lock_depth==key+1);lock_depth=key;}
static bool nrf_rtc_event_check(int p,int e) {(void)p;(void)e;return pending;}
static uint32_t nrf_rtc_counter_get(int p) {(void)p;return ticks;}
static uint32_t nrf_timer_cc_get(int p,int c) {(void)p;return c==2?anchor:now;}
static void advance(void) {
    if (running && now>=next_edge) {
        anchor=next_edge;next_edge+=31;ticks=(ticks+1)&0xffffff;
        if (!ticks) pending=true;
    }
}
static void nrf_timer_task_trigger(int p,int t) {
    (void)p;(void)t;
    if (edge_on_capture) {edge_on_capture=false;now=next_edge+2;advance();}
}
static void k_busy_wait(unsigned us) {++waits;if(running)now+=us;advance();}
static void reset(void) {
    ticks=32768;anchor=1000;now=1005;next_edge=1031;
    num_rtc_overflows=0;waits=warnings=lock_depth=0;
    pending=edge_on_capture=false;running=true;
}
"""+actual+r"""
int main(void) {
    reset();assert(timestamp_from_anchor_get(999)==999999);
    assert(!waits && !warnings && !lock_depth);
    /* The counter and its latched tick must come from the same event. */
    reset();now=1030;
    assert(timestamp_from_anchor_get(1029)==1000028);
    assert(waits>=3 && !warnings && !lock_depth);
    reset();edge_on_capture=true;
    assert(timestamp_from_anchor_get(1029)==1000028);
    assert(waits && !warnings && !lock_depth);
    /* Hardware has wrapped; the low-priority epoch ISR is still pending. */
    reset();ticks=0xffffff;now=1030;
    assert(timestamp_from_anchor_get(1029)==511999998);
    assert(pending && !num_rtc_overflows && !warnings && !lock_depth);
    /* Once acknowledged, the same epoch must not be counted twice. */
    num_rtc_overflows=1;pending=false;
    assert(timestamp_from_anchor_get(1029)==511999998);
    /* A controller that has not started the HF timer cannot hang boot. */
    reset();running=false;now=anchor=0;
    assert(timestamp_from_anchor_get(0)==1000000);
    assert(waits==16 && warnings==1 && !lock_depth);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'test.c').write_text(source)
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                            '-I',str(root/'src/modules'),str(p/'test.c'),
                            '-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test')],check=True)

if __name__=='__main__':unittest.main()
