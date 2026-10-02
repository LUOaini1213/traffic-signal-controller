"""Exercise the real replay executable, including files exported on Windows."""
import argparse
import json
import subprocess
import tempfile
import unittest
from pathlib import Path


class ReplayCli(unittest.TestCase):
    def replay(self, contents, end="30"):
        with tempfile.TemporaryDirectory() as tmp:
            events = Path(tmp) / "events.csv"
            events.write_bytes(contents)
            return subprocess.run(
                [str(BINARY), "replay", str(CONFIG), str(events), end],
                capture_output=True, text=True, timeout=20,
            )

    def assert_rejected(self, contents, *, row=2, end="30", message=None):
        result = self.replay(contents, end)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(result.stdout, "", "invalid input must not start a replay")
        self.assertIn("error:", result.stderr)
        if row is not None:
            self.assertIn(f"events.csv:{row}:", result.stderr)
        if message is not None:
            self.assertIn(message, result.stderr)

    def test_lf_crlf_and_bom_preserve_arrivals_and_signal_timeline(self):
        data = b"t_s,detector_id,on\n5,N_2,1\n5.5,N_2,0\n"
        baseline = self.replay(data)
        self.assertEqual(baseline.returncode, 0, baseline.stderr)
        summary = json.loads(baseline.stderr)
        self.assertEqual(summary["phases"]["NS_right"]["served"], 1)
        self.assertEqual(summary["audit_violations"], 0)
        self.assertFalse(summary["failsafe"])
        for variant in (data.replace(b"\n", b"\r\n"), b"\xef\xbb\xbf" + data,
                        data.rstrip(b"\n")):
            with self.subTest(variant=variant):
                result = self.replay(variant)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, baseline.stdout)
                self.assertEqual(json.loads(result.stderr), summary)

    def test_space_padding_and_blank_crlf_lines(self):
        data = b" t_s , detector_id , on \r\n \t\r\n 5 , N_2 , 1 \r\n5.5,N_2,0\r\n"
        result = self.replay(data)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stderr)["phases"]["NS_right"]["served"], 1)

    def test_off_event_does_not_place_a_call(self):
        result = self.replay(b"t_s,detector_id,on\r\n5,N_2,0\r\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stderr)["phases"]["NS_right"]["served"], 0)

    def test_bad_boolean_is_rejected_not_silently_off(self):
        for value in (b"2", b"true", b"-1", b"01", b"", b"1x"):
            with self.subTest(value=value):
                self.assert_rejected(b"t_s,detector_id,on\n5,N_2," + value + b"\n")

    def test_missing_or_extra_columns_are_rejected(self):
        for row in (b"5,N_2", b"5,N_2,1,extra", b"5,N_2,1,", b"5,,1", b",N_2,1"):
            with self.subTest(row=row):
                self.assert_rejected(b"t_s,detector_id,on\n" + row + b"\n")

    def test_header_is_required_and_checked(self):
        for data in (b"", b"5,N_2,1\n", b"time,detector_id,on\n5,N_2,1\n",
                     b"t_s,on,detector_id\n5,1,N_2\n", b"t_s,detector_id,on,\n"):
            with self.subTest(data=data):
                self.assert_rejected(data, row=1)

    def test_invalid_event_times_are_rejected_before_any_output(self):
        for value in (b"5x", b"nan", b"inf", b"-0.0001", b"1e9999", b"9223372036854776"):
            with self.subTest(value=value):
                self.assert_rejected(b"t_s,detector_id,on\n" + value + b",N_2,1\n",
                                     message="t_s")

    def test_invalid_end_time_is_rejected(self):
        for end in ("30x", "nan", "inf", "-1", "1e9999", "9223372036854776"):
            with self.subTest(end=end):
                self.assert_rejected(b"t_s,detector_id,on\n", row=None, end=end,
                                     message="end_s")

    def test_unknown_detector_reports_the_csv_line(self):
        self.assert_rejected(b"t_s,detector_id,on\n\n5,missing,1\n", row=3,
                             message="missing")

    def test_header_only_zero_duration_is_valid(self):
        result = self.replay(b"t_s,detector_id,on\r\n", "0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.splitlines(), ["t_s,signals,failsafe", "0,rrrrrrrr,0"])

    def test_unordered_rows_are_sorted_and_seconds_are_converted(self):
        result = self.replay(b"t_s,detector_id,on\n5.5,N_2,0\n5,N_2,1\n", "3e1")
        baseline = self.replay(b"t_s,detector_id,on\n5,N_2,1\n5.5,N_2,0\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, baseline.stdout)
        self.assertEqual(json.loads(result.stderr), json.loads(baseline.stderr))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args, rest = parser.parse_known_args()
    BINARY, CONFIG = args.binary.resolve(), args.config.resolve()
    unittest.main(argv=[__file__, *rest])
