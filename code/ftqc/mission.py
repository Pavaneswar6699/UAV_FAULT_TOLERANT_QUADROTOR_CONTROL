"""Waypoint mission with the post-fault plan (settle, then controlled landing).

Used by both the simulator and the real-flight script, so they behave the same.
"""
import numpy as np

DEFAULT_WAYPOINTS = [(0.5, 0.0, 0.6), (0.5, 0.5, 0.6), (0.0, 0.5, 0.6), (0.0, 0.0, 0.6)]


class Mission:
    def __init__(self, start, waypoints=DEFAULT_WAYPOINTS, takeoff_alt=0.6,
                 speed=0.3, hold=1.0, settle=1.0, land_speed=0.15):
        self.speed, self.hold, self.settle, self.land_speed = speed, hold, settle, land_speed
        start = np.asarray(start, dtype=float)
        pts = [start, np.array([start[0], start[1], takeoff_alt])]
        pts += [np.asarray(w, dtype=float) for w in waypoints]
        # schedule of (t_start, t_end, from, to); a hold is a segment with from == to
        self.segments, t = [], 0.0
        self.waypoint_times = []
        for a, b in zip(pts[:-1], pts[1:]):
            dur = max(np.linalg.norm(b - a) / speed, 0.5)
            self.segments.append((t, t + dur, a, b))
            t += dur
            self.waypoint_times.append(t)
            self.segments.append((t, t + hold, b, b))
            t += hold
        self.t_end = t
        self.end_point = pts[-1]
        self.fault_t = None
        self.fault_p = None
        self.done = False

    def notify_fault(self, t, pos):
        if self.fault_t is None:
            self.fault_t = t
            self.fault_p = np.array([pos[0], pos[1], max(pos[2], 0.15)])

    def _nominal(self, t):
        for t0, t1, a, b in self.segments:
            if t <= t1:
                s = 0.0 if t1 == t0 else np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
                return a + s * (b - a)
        return None

    def point(self, t):
        """Reference position and phase name at time t."""
        if self.fault_t is not None:
            if t < self.fault_t + self.settle:
                return self.fault_p.copy(), "fault_settle"
            z = self.fault_p[2] - self.land_speed * (t - self.fault_t - self.settle)
            return np.array([self.fault_p[0], self.fault_p[1], max(z, -0.1)]), "fault_land"
        nominal = self._nominal(t)
        if nominal is not None:
            return nominal, "mission"
        z = self.end_point[2] - self.land_speed * (t - self.t_end)
        return np.array([self.end_point[0], self.end_point[1], max(z, -0.1)]), "land"

    def ref_traj(self, t, horizon, dt):
        pts = [self.point(t + i * dt)[0] for i in range(horizon + 1)]
        return np.array(pts), self.point(t)[1]
