from __future__ import annotations

import unittest

from litearm_vr_bridge.events import EventLog


class EventLogTest(unittest.TestCase):
    def test_add_and_drain(self) -> None:
        log = EventLog(ttl_ms=10_000.0)
        log.add("warn", "frame_stall", "无帧", now_wall_ms=1000, now_mono=0.0)
        log.add("info", "latch_done", "已持位", now_wall_ms=1001, now_mono=0.1)
        drained = log.drain(now_wall_ms=1002)
        self.assertEqual([e["code"] for e in drained], ["frame_stall", "latch_done"])
        self.assertEqual(drained[0]["id"], 1)
        self.assertEqual(drained[0]["level"], "warn")

    def test_ttl_expiry(self) -> None:
        log = EventLog(ttl_ms=1000.0)
        log.add("info", "a", "第一", now_wall_ms=0, now_mono=0.0)
        self.assertEqual(len(log.drain(now_wall_ms=500)), 1)
        self.assertEqual(log.drain(now_wall_ms=1500), [])

    def test_max_items_bounded(self) -> None:
        log = EventLog(ttl_ms=10_000.0, max_items=3, dedupe_window_s=0.0)
        for i in range(10):
            log.add("info", f"code{i}", f"第{i}条", now_wall_ms=i, now_mono=float(i))
        drained = log.drain(now_wall_ms=10)
        self.assertEqual(len(drained), 3)
        self.assertEqual(drained[-1]["code"], "code9")

    def test_dedupe_merges_rapid_repeats(self) -> None:
        log = EventLog(ttl_ms=10_000.0, dedupe_window_s=1.0)
        for i in range(4):
            log.add("warn", "frame_stall", "无帧", now_wall_ms=1000 + i, now_mono=i * 0.1)
        drained = log.drain(now_wall_ms=1005)
        self.assertEqual(len(drained), 1)
        self.assertIn("×4", drained[0]["message"])

    def test_clear_all_and_one(self) -> None:
        log = EventLog(ttl_ms=10_000.0)
        log.add("info", "a", "A", now_wall_ms=0, now_mono=0.0)
        log.add("info", "b", "B", now_wall_ms=1, now_mono=1.0)
        self.assertEqual(log.clear(1), 1)
        remaining = log.drain(now_wall_ms=2)
        self.assertEqual([e["code"] for e in remaining], ["b"])
        self.assertEqual(log.clear(), 1)
        self.assertEqual(log.drain(now_wall_ms=3), [])


if __name__ == "__main__":
    unittest.main()
