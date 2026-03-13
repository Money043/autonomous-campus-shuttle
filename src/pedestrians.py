"""
pedestrians.py
==============
PedestrianManager: spawns and animates pedestrians on sidewalks / crosswalks.

Pedestrian movement is constrained to defined path segments so pedestrians
never wander through buildings or onto the road unexpectedly.
"""

import math
import random
from panda3d.core import Vec3, NodePath, LVecBase4f

from src.environment import _make_box


# Colour palette for pedestrian clothing
PED_COLORS = [
    (0.85, 0.20, 0.20, 1.0),
    (0.20, 0.40, 0.80, 1.0),
    (0.90, 0.65, 0.10, 1.0),
    (0.30, 0.70, 0.35, 1.0),
    (0.60, 0.20, 0.70, 1.0),
    (0.85, 0.50, 0.20, 1.0),
    (0.20, 0.70, 0.75, 1.0),
]


class Pedestrian:
    """
    Single pedestrian: procedural 'stick-figure' model + path-following logic.

    Movement states:
      WALKING   – following a sidewalk segment
      CROSSING  – crossing the road at a crosswalk zone
      WAITING   – paused at crosswalk edge before crossing
    """

    WALK_SPEED  = 1.2   # m/s
    CROSS_SPEED = 0.9
    WAIT_TIME   = 2.0   # seconds at kerb before crossing

    def __init__(self, render, start_pos, path_segment, color):
        self.render = render

        self.node = render.attachNewNode(f"ped_{id(self)}")
        self.node.setPos(start_pos)

        self._build_model(color)

        # Path segment: (Vec3 start, Vec3 end)
        self.seg_start, self.seg_end = path_segment
        self.t = 0.0          # parametric position along segment [0, 1]
        self.direction = 1    # +1 forward, -1 reverse
        self.state = "WALKING"
        self.wait_timer = 0.0

        # Bob animation
        self._bob_phase = random.uniform(0, 2 * math.pi)
        self._bob_speed = random.uniform(3.5, 5.0)
        self._bob_amp   = 0.06

    # ── Model ─────────────────────────────────────────────────────────────

    def _build_model(self, color):
        # Torso
        torso = _make_box(0.30, 0.50, 0.25, color)
        torso.reparentTo(self.node)
        torso.setPos(0, 0, 1.05)

        # Head
        head = _make_box(0.22, 0.22, 0.22, (0.87, 0.72, 0.58, 1.0))
        head.reparentTo(self.node)
        head.setPos(0, 0, 1.50)

        # Legs
        leg_color = (0.20, 0.20, 0.35, 1.0)
        for sx in (-0.08, 0.08):
            leg = _make_box(0.12, 0.55, 0.12, leg_color)
            leg.reparentTo(self.node)
            leg.setPos(sx, 0, 0.55)

        # Arms
        arm_color = color
        for sx in (-0.21, 0.21):
            arm = _make_box(0.10, 0.40, 0.10, arm_color)
            arm.reparentTo(self.node)
            arm.setPos(sx, 0, 1.10)

    # ── Update ────────────────────────────────────────────────────────────

    def update(self, dt):
        """Advance pedestrian along their current segment."""
        self._bob_phase += self._bob_speed * dt

        if self.state == "WAITING":
            self.wait_timer -= dt
            if self.wait_timer <= 0:
                self.state = "CROSSING" if random.random() < 0.5 else "WALKING"
            return

        speed = self.CROSS_SPEED if self.state == "CROSSING" else self.WALK_SPEED
        seg_len = (self.seg_end - self.seg_start).length()
        if seg_len < 0.01:
            return

        self.t += self.direction * speed * dt / seg_len

        # Bounce at ends
        if self.t >= 1.0:
            self.t = 1.0
            self.direction = -1
        elif self.t <= 0.0:
            self.t = 0.0
            self.direction = 1

        pos = self.seg_start + (self.seg_end - self.seg_start) * self.t
        pos.z += self._bob_amp * abs(math.sin(self._bob_phase))   # walking bob
        self.node.setPos(pos)

        # Face direction of travel
        dx = (self.seg_end - self.seg_start).x * self.direction
        dy = (self.seg_end - self.seg_start).y * self.direction
        if abs(dx) + abs(dy) > 0.01:
            heading = math.degrees(math.atan2(-dx, dy))
            self.node.setH(heading)


# ─────────────────────────────────────────────────────────────────────────────

class PedestrianManager:
    """
    Spawns pedestrians and manages them.

    pedestrians – list[Pedestrian]  (read by perception module in main.py)
    """

    NUM_SIDEWALK_PEDS  = 18
    NUM_CROSSWALK_PEDS =  6

    def __init__(self, render, sidewalk_paths, crosswalk_zones):
        self.render = render
        self.pedestrians: list[Pedestrian] = []

        random.seed(7)

        # Sidewalk pedestrians
        for _ in range(self.NUM_SIDEWALK_PEDS):
            seg = random.choice(sidewalk_paths)
            t0  = random.uniform(0, 1)
            pos = seg[0] + (seg[1] - seg[0]) * t0
            pos.z = 0
            color = random.choice(PED_COLORS)
            p = Pedestrian(render, pos, seg, color)
            p.t = t0
            self.pedestrians.append(p)

        # Crosswalk pedestrians
        for zone_centre, hw, hd in crosswalk_zones:
            for _ in range(self.NUM_CROSSWALK_PEDS // len(crosswalk_zones)):
                # Create a segment that crosses the road
                perp = Vec3(-math.sin(math.atan2(zone_centre.y, zone_centre.x)),
                             math.cos(math.atan2(zone_centre.y, zone_centre.x)),
                             0)
                start = zone_centre - perp * hd
                end   = zone_centre + perp * hd
                t0    = random.uniform(0, 1)
                pos   = start + (end - start) * t0
                pos.z = 0
                color = random.choice(PED_COLORS)
                p = Pedestrian(render, pos, (start, end), color)
                p.t = t0
                p.state = "CROSSING"
                self.pedestrians.append(p)

    def update(self, dt):
        for ped in self.pedestrians:
            ped.update(dt)

    def show_debug(self, on):
        """Toggle debug bounding boxes (placeholder for future expansion)."""
        pass
