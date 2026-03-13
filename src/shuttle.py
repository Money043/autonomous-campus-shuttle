"""
shuttle.py
==========
AutonomousShuttle: procedural 3D model + CONTROL logic.

CONTROL MODULE
--------------
The shuttle follows a list of waypoints using a pure-pursuit style
heading correction.  Speed is fed in from the planning module each frame.
"""

import math
from panda3d.core import Vec3, NodePath, GeomNode

from src.environment import _make_box, _make_quad


class AutonomousShuttle:
    """
    Represents the ego vehicle.

    Public interface
    ----------------
    .node            – NodePath (attach camera here)
    .current_speed   – float m/s
    .waypoint_idx    – int
    update(dt, target_speed)
    """

    # Shuttle body dimensions (real minibus scale)
    BODY_W = 2.2
    BODY_H = 1.8
    BODY_D = 4.5

    # Control constants
    ACCEL     =  4.0   # m/s² acceleration
    DECEL     =  6.0   # m/s² braking
    STEER_K   =  3.0   # heading correction gain
    WP_RADIUS =  2.5   # distance to switch to next waypoint

    def __init__(self, render, waypoints):
        self.render    = render
        self.waypoints = waypoints
        self.waypoint_idx   = 0
        self.current_speed  = 0.0

        self.node = render.attachNewNode("shuttle_root")
        wp0 = waypoints[0]
        self.node.setPos(wp0.x, wp0.y, wp0.z)

        # Orient toward first waypoint
        wp1 = waypoints[1]
        dx, dy = wp1.x - wp0.x, wp1.y - wp0.y
        heading = math.degrees(math.atan2(-dx, dy))
        self.node.setH(heading)

        self.node.setScale(0.55)  # scale to realistic minibus size
        self._build_model()

    # ── 3-D model ─────────────────────────────────────────────────────────

    def _build_model(self):
        bw, bh, bd = self.BODY_W, self.BODY_H, self.BODY_D

        # Main body
        body = _make_box(bw, bh, bd, (0.20, 0.55, 0.85, 1.0))
        body.reparentTo(self.node)
        body.setPos(0, 0, bh / 2)

        # Cabin / roof dome (slightly narrower, lighter)
        roof = _make_box(bw - 0.3, bh * 0.5, bd * 0.65, (0.70, 0.82, 0.95, 1.0))
        roof.reparentTo(self.node)
        roof.setPos(0, 0, bh + (bh * 0.25))

        # Front windshield strip
        ws = _make_box(bw - 0.4, 0.12, 1.0, (0.50, 0.70, 0.90, 0.55))
        ws.reparentTo(self.node)
        ws.setPos(0, bd / 2 - 0.05, bh + 0.1)
        ws.setTransparency(1)

        # Wheels (4 cylinders approximated as flat boxes)
        wheel_color = (0.12, 0.12, 0.12, 1.0)
        for sx, sy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            wx = sx * (bw / 2 + 0.1)
            wy = sy * (bd / 2 - 1.0)
            wheel = _make_box(0.3, 0.7, 0.7, wheel_color)
            wheel.reparentTo(self.node)
            wheel.setPos(wx, wy, 0.35)

        # Headlights
        for sx in (-1, 1):
            hl = _make_box(0.25, 0.15, 0.25, (1.0, 0.98, 0.80, 1.0))
            hl.reparentTo(self.node)
            hl.setPos(sx * (bw / 2 - 0.2), bd / 2 + 0.01, bh * 0.45)

        # University logo plate (coloured rectangle on front)
        plate = _make_quad(1.2, 0.3, (0.90, 0.75, 0.10, 1.0))
        plate.reparentTo(self.node)
        plate.setPos(0, bd / 2 + 0.02, bh * 0.6)
        plate.setP(-90)

    # ── CONTROL ────────────────────────────────────────────────────────────

    def update(self, dt, target_speed):
        """
        CONTROL MODULE
        --------------
        1. Blend current_speed toward target_speed (smooth accel/decel).
        2. Compute heading error to next waypoint (pure-pursuit lite).
        3. Move shuttle forward; advance waypoint when close enough.
        """

        # 1. Speed control – ramp toward target
        if self.current_speed < target_speed:
            self.current_speed = min(target_speed,
                                     self.current_speed + self.ACCEL * dt)
        else:
            self.current_speed = max(target_speed,
                                     self.current_speed - self.DECEL * dt)

        if self.current_speed < 0.01:
            return   # parked – nothing to do

        # 2. Heading toward current waypoint
        pos = self.node.getPos(self.render)
        wp  = self.waypoints[self.waypoint_idx]

        dx  = wp.x - pos.x
        dy  = wp.y - pos.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Advance waypoint
        if dist < self.WP_RADIUS:
            self.waypoint_idx = (self.waypoint_idx + 1) % len(self.waypoints)
            wp = self.waypoints[self.waypoint_idx]
            dx = wp.x - pos.x
            dy = wp.y - pos.y

        desired_heading = math.degrees(math.atan2(-dx, dy))
        current_heading = self.node.getH()

        # Shortest-angle difference
        diff = desired_heading - current_heading
        while diff >  180: diff -= 360
        while diff < -180: diff += 360

        # Apply smooth steering
        steer = min(max(diff * self.STEER_K * dt, -90 * dt), 90 * dt)
        self.node.setH(current_heading + steer)

        # 3. Move forward
        fwd   = self.render.getRelativeVector(self.node, Vec3(0, 1, 0))
        new_pos = pos + fwd * self.current_speed * dt
        new_pos.z = pos.z   # keep on ground
        self.node.setPos(new_pos)
