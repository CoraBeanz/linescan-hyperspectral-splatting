"""Map the ESP32's microsecond clock onto ROS time.

The bridge sends PING about once a second and notes the ROS time it sent it and the ROS time
the PONG came back; the PONG carries the ESP32's clock when it answered. The ESP32 answered
somewhere inside that round trip, so the middle of the round trip is the best guess for the
ROS time that matches its clock. USB and Linux scheduling sometimes delay a reply by a few
milliseconds; round trips that took longer than most are left out, because those are the ones
with the most uncertain middle. Over a minute or more, a straight-line fit also follows the
small rate difference between the two crystals (tens of microseconds per second).
"""

from collections import deque

import numpy as np

US = 1000  # ns per us


class ClockSync:
    def __init__(self, window=60, min_samples=3, keep_fraction=0.5, min_fit_span_s=20.0):
        self.samples = deque(maxlen=window)  # (t_esp_us, t_ros_mid_ns, rtt_ns)
        self.min_samples = min_samples
        self.keep_fraction = keep_fraction
        self.min_fit_span_us = min_fit_span_s * 1e6
        self._fit = None

    def add(self, t_send_ns, t_recv_ns, t_esp_us):
        """One PING/PONG round trip. Returns the round trip time in ns."""
        if self.samples and t_esp_us < self.samples[-1][0]:
            self.reset()  # the ESP32 clock went backwards: it rebooted
        rtt = t_recv_ns - t_send_ns
        self.samples.append((t_esp_us, (t_send_ns + t_recv_ns) // 2, rtt))
        self._fit = None
        return rtt

    def reset(self):
        self.samples.clear()
        self._fit = None

    @property
    def ready(self):
        return len(self.samples) >= self.min_samples

    def _solve(self):
        best = sorted(self.samples, key=lambda s: s[2])
        best = best[:max(self.min_samples, int(len(best) * self.keep_fraction))]
        esp = np.array([s[0] for s in best], dtype=np.int64)
        ros = np.array([s[1] for s in best], dtype=np.int64)
        e0, r0 = int(esp[0]), int(ros[0])
        de = (esp - e0).astype(np.float64)
        dr = (ros - r0).astype(np.float64)
        if de.max() - de.min() >= self.min_fit_span_us:
            slope, intercept = np.polyfit(de, dr, 1)
        else:
            slope = float(US)  # too short to see the drift: assume the clocks run at the same rate
            intercept = float(np.median(dr - US * de))
        self._fit = (e0, r0, slope, intercept)
        return self._fit

    def to_ros_ns(self, t_esp_us):
        """ROS time (ns) for an ESP32 clock reading. Needs `ready`."""
        if not self.ready:
            raise RuntimeError("no clock samples yet")
        e0, r0, slope, intercept = self._fit or self._solve()
        return r0 + int(round(intercept + slope * (t_esp_us - e0)))

    def uncertainty_ns(self):
        """Half the shortest round trip: how far off the mapping can be, at best."""
        return min(s[2] for s in self.samples) // 2 if self.samples else None
