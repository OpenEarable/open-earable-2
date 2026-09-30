"""Positive/negative controls for the measurement analysis, independent of hardware."""

import json
from pathlib import Path
import tempfile
import unittest

from analyze import analyze
from capture import signed_delta


class AnalysisTests(unittest.TestCase):
    def captures(self, offsets):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        paths = []
        for side, offset in enumerate(offsets):
            path = Path(directory.name) / f"{side}.jsonl"
            rows = [dict(time=1000 + i/100, side=side, delay_us=21000+offset,
                         estimated_delay_us=21000+offset, receive_us=-2000,
                         presentation_delay=20000, underruns=0, states=771)
                    for i in range(400)]
            path.write_text("\n".join(json.dumps(row) for row in rows))
            paths.append(path)
        return paths

    def test_detects_known_one_ms_shift_and_sign(self):
        for offset in [-1000, 0, 1000]:
            with self.subTest(offset=offset):
                result = analyze(self.captures([0, offset]), settle=0)
                self.assertEqual(result["right_minus_left_us"]["median"], offset)

    def test_missing_ear_is_not_a_pass(self):
        result = analyze(self.captures([0]), settle=0)
        self.assertIsNone(result["right_minus_left_us"])

    def test_controller_wrap_does_not_create_a_false_delay(self):
        self.assertEqual(signed_delta(500, 0xffffff00), 756)
        self.assertEqual(signed_delta(0xffffff00, 500), -756)

    def test_short_slip_is_reported_even_when_the_median_stays_aligned(self):
        paths = self.captures([0, 0])
        rows = [json.loads(line) for line in paths[1].read_text().splitlines()]
        rows[50]["delay_us"] += 2000
        paths[1].write_text("\n".join(json.dumps(row) for row in rows))
        report = analyze(paths, settle=0)
        self.assertEqual(report["right_minus_left_us"]["median"], 0)
        self.assertEqual(report["sides"]["right"]["off_target_frames_100us"], 1)
        self.assertEqual(report["sides"]["right"]["target_error_us"]["max"], 2000)

    def test_estimated_timestamps_cannot_prove_stereo_alignment(self):
        paths = self.captures([0, 0])
        rows = [json.loads(line) for line in paths[1].read_text().splitlines()]
        for row in rows:
            row.update(timestamp_valid=False, bad_frame=True)
        paths[1].write_text("\n".join(json.dumps(row) for row in rows))
        result = analyze(paths, settle=0)
        self.assertIsNone(result["right_minus_left_us"])
        self.assertIsNone(result["sides"]["right"]["delay_us"])
        self.assertEqual(result["sides"]["right"]["estimated_timestamp_frames"], 400)
        self.assertEqual(result["sides"]["right"]["bad_frames"], 400)

    def test_explicit_restarts_do_not_count_startup_underruns_as_steady(self):
        paths = self.captures([0, 0])
        for path in paths:
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            for row in rows:
                row["underruns"] = 100 if row["time"] >= 1002 else 10
            path.write_text("\n".join(json.dumps(row) for row in rows))
        report = analyze(paths, settle=.25,
                         playback_events=[dict(event="pause", time=1001.8),
                                          dict(event="play", time=1002)])
        self.assertEqual(report["sides"]["right"]["new_underruns"], 0)
        self.assertGreater(report["sides"]["right"]["excluded_transition_frames"], 25)
        self.assertEqual(report["right_minus_left_us"]["median"], 0)


if __name__ == "__main__":
    unittest.main()
