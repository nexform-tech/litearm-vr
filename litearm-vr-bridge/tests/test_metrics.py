from __future__ import annotations

import unittest

from litearm_vr_bridge.frames import parse_frame
from litearm_vr_bridge.metrics import HeadsetMetrics, LatencyEstimator, RollingStats


class RollingStatsTest(unittest.TestCase):
    def test_last_mean_percentile(self) -> None:
        stats = RollingStats(size=100)
        for value in (1.0, 2.0, 3.0, 4.0):
            stats.add(value)
        self.assertEqual(stats.last, 4.0)
        self.assertEqual(stats.mean, 2.5)
        self.assertEqual(stats.max, 4.0)
        self.assertEqual(stats.percentile(1.0), 4.0)

    def test_empty(self) -> None:
        stats = RollingStats()
        self.assertIsNone(stats.last)
        self.assertIsNone(stats.mean)


class LatencyEstimatorTest(unittest.TestCase):
    def test_direct_subtraction_when_clocks_comparable(self) -> None:
        est = LatencyEstimator()
        value = est.update(100.0, 1_787_641_468_200.0, 1_787_641_468_180.0)
        self.assertAlmostEqual(value or 0.0, 20.0, places=3)
        self.assertFalse(est.estimated)

    def test_negative_direct_latency_clamped(self) -> None:
        est = LatencyEstimator()
        value = est.update(100.0, 1_787_641_468_000.0, 1_787_641_468_050.0)
        self.assertEqual(value, 0.0)

    def test_baseline_estimate_when_clock_skewed(self) -> None:
        # 头显时钟慢了 1 小时（+3600s 偏差）：raw 恒为大值，改用窗口最小值做基线
        est = LatencyEstimator()
        skew = 3_600_000.0
        first = est.update(100.0, 1_787_641_468_000.0, 1_787_641_468_000.0 - skew)
        self.assertTrue(est.estimated)
        self.assertEqual(first, 0.0)
        second = est.update(100.1, 1_787_641_468_030.0, 1_787_641_468_010.0 - skew)
        self.assertAlmostEqual(second or 0.0, 20.0, places=3)

    def test_no_timestamp(self) -> None:
        est = LatencyEstimator()
        self.assertIsNone(est.update(1.0, 1_787_641_468_000.0, None))


class HeadsetMetricsTest(unittest.TestCase):
    def _info(self, grip: bool = True):
        return parse_frame(
            '{"timestamp": 1787641468000, "rightController": '
            '{"tracked": true, "gripActive": %s}}' % ("true" if grip else "false")
        )

    def test_fps_from_receive_times(self) -> None:
        metrics = HeadsetMetrics(fps_window_s=2.0)
        for i in range(11):  # 10 帧 / 1s → 10 Hz
            metrics.on_frame(i * 0.1, 1_787_641_468_000.0 + i * 100, self._info())
        self.assertAlmostEqual(metrics.fps(), 10.0, places=3)
        self.assertEqual(metrics.frames, 11)

    def test_age_and_forward_cost(self) -> None:
        metrics = HeadsetMetrics()
        metrics.on_frame(10.0, 1_787_641_468_000.0, self._info())
        self.assertAlmostEqual(metrics.age_ms(10.25) or 0.0, 250.0, places=3)
        metrics.on_forwarded(0.4)
        metrics.on_forwarded(0.6)
        self.assertEqual(metrics.frames_forwarded, 2)
        self.assertAlmostEqual(metrics.forward_cost.mean or 0.0, 0.5, places=3)

    def test_bad_frame_counted_but_kept(self) -> None:
        metrics = HeadsetMetrics()
        metrics.on_frame(1.0, 1_787_641_468_000.0, parse_frame("garbage"))
        self.assertEqual(metrics.frames, 1)
        self.assertEqual(metrics.bad_frames, 1)

    def test_age_none_before_first_frame(self) -> None:
        metrics = HeadsetMetrics()
        self.assertIsNone(metrics.age_ms(5.0))
        self.assertEqual(metrics.fps(), 0.0)


if __name__ == "__main__":
    unittest.main()
