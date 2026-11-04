"""El ACK del PLC cierra la orden con su id, no la siguiente del mismo byte."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cycle import _PlcAckPending, plc_ack_match_index


def _order(order_id: int, since_seq: int, expired: bool = False) -> _PlcAckPending:
    item = _PlcAckPending(order_id, 0x1A, since_seq, "Cortador", None)
    item.expired = expired
    return item


class PlcAckMatchTest(unittest.TestCase):
    def test_id_closes_that_order_not_the_newer_one(self) -> None:
        ghost = _order(1, 10, expired=True)
        live = _order(2, 20)
        self.assertEqual(plc_ack_match_index([ghost, live], seq=30, req_id=1), 0)
        self.assertEqual(plc_ack_match_index([ghost, live], seq=30, req_id=2), 1)

    def test_unknown_id_does_not_pick_another_order(self) -> None:
        live = _order(2, 20)
        self.assertIsNone(plc_ack_match_index([live], seq=30, req_id=9))

    def test_legacy_without_id_still_takes_the_oldest(self) -> None:
        ghost = _order(1, 10, expired=True)
        live = _order(2, 20)
        self.assertEqual(plc_ack_match_index([ghost, live], seq=30, req_id=0), 0)

    def test_legacy_ignores_ack_already_seen(self) -> None:
        live = _order(2, 20)
        self.assertIsNone(plc_ack_match_index([live], seq=20, req_id=0))


if __name__ == "__main__":
    unittest.main()
