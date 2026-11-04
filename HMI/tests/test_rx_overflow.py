"""Política de desborde del cliente TCP: no parsear una línea cortada."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.tcp_link import take_rx_lines


class RxOverflowTest(unittest.TestCase):
    def test_complete_lines(self) -> None:
        rest, overflow, lines = take_rx_lines('{"a":1}\n{"b":2}\n', False, 100)
        self.assertEqual(rest, "")
        self.assertFalse(overflow)
        self.assertEqual(lines, ['{"a":1}', '{"b":2}'])

    def test_partial_kept(self) -> None:
        rest, overflow, lines = take_rx_lines('{"a":', False, 100)
        self.assertEqual(rest, '{"a":')
        self.assertFalse(overflow)
        self.assertEqual(lines, [])

    def test_oversize_frame_dropped_and_next_kept(self) -> None:
        huge = "x" * 20
        rest, overflow, lines = take_rx_lines(huge + "\n{\"ok\":1}\n", False, 8)
        self.assertEqual(rest, "")
        self.assertFalse(overflow)
        self.assertEqual(lines, ['{"ok":1}'])

    def test_split_oversize_does_not_parse_tail(self) -> None:
        rest, overflow, lines = take_rx_lines("abcdefghij", False, 8)
        self.assertEqual(rest, "")
        self.assertTrue(overflow)
        self.assertEqual(lines, [])
        rest, overflow, lines = take_rx_lines(rest + "KLM\n{\"ok\":1}\n", overflow, 8)
        self.assertFalse(overflow)
        self.assertEqual(lines, ['{"ok":1}'])


if __name__ == "__main__":
    unittest.main()
