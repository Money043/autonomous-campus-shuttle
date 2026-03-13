"""
3D Autonomous Campus Shuttle Simulation
========================================
A Panda3D-based simulation demonstrating autonomous shuttle operation
on a university campus with perception, planning, and control logic.

Architecture:
  - Perception : Detect pedestrians in forward detection zone
  - Planning   : Decide to drive / slow / stop based on scene state
  - Control    : Smoothly adjust shuttle speed and heading
"""

import sys
import math
import random
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (
    Vec3, Vec4, Point3, LVector3f,
    DirectionalLight, AmbientLight,
    LVecBase4f, LVecBase3f,
    NodePath, GeomNode,
    TransparencyAttrib,
    WindowProperties,
    TextNode,
    AntialiasAttrib,
)
from direct.gui.OnscreenText import OnscreenText

from src.environment import CampusEnvironment
from src.shuttle import AutonomousShuttle
from src.pedestrians import PedestrianManager
from src.hud import HUD


class CampusShuttleSimulation(ShowBase):
    """Main application: wires together environment, shuttle, pedestrians, HUD."""

    def __init__(self):
        ShowBase.__init__(self)

        # ── Window setup ────────────────────────────────────────────────────
        props = WindowProperties()
        props.setTitle("Autonomous Campus Shuttle Simulation")
        props.setSize(1280, 720)
        self.win.requestProperties(props)
        self.setBackgroundColor(0.53, 0.81, 0.98, 1.0)   # sky blue

        # Smooth rendering
        self.render.setAntialias(AntialiasAttrib.MAuto)

        # ── Lighting ────────────────────────────────────────────────────────
        self._setup_lighting()

        # ── Campus environment ───────────────────────────────────────────────
        self.campus = CampusEnvironment(self.render)

        # ── Shuttle ──────────────────────────────────────────────────────────
        self.shuttle = AutonomousShuttle(self.render, self.campus.waypoints)

        # ── Pedestrians ──────────────────────────────────────────────────────
        self.ped_manager = PedestrianManager(
            self.render, self.campus.sidewalk_paths, self.campus.crosswalk_zones
        )

        # ── HUD ──────────────────────────────────────────────────────────────
        self.hud = HUD(self)

        # ── Camera (POV from shuttle cabin) ─────────────────────────────────
        self.disableMouse()
        self._setup_camera()

        # ── Main loop task ───────────────────────────────────────────────────
        self.taskMgr.add(self._update, "MainUpdateTask")

        # ── Key bindings ─────────────────────────────────────────────────────
        self.accept("escape", sys.exit)
        self.accept("f1", self._toggle_debug)
        self._debug_mode = False

    # ────────────────────────────────────────────────────────────────────────
    # Setup helpers
    # ────────────────────────────────────────────────────────────────────────

    def _setup_lighting(self):
        """Ambient fill + directional sun light."""
        ambient = AmbientLight("ambient")
        ambient.setColor(LVecBase4f(0.45, 0.45, 0.50, 1))
        self.render.setLight(self.render.attachNewNode(ambient))

        sun = DirectionalLight("sun")
        sun.setColor(LVecBase4f(1.0, 0.95, 0.85, 1))
        sun_np = self.render.attachNewNode(sun)
        sun_np.setHpr(45, -60, 0)
        self.render.setLight(sun_np)

    def _setup_camera(self):
        """Third-person follow camera: behind and above shuttle."""
        self.camera.reparentTo(self.shuttle.node)
        self.camera.setPos(0, -14, 5.5)   # behind shuttle
        self.camera.setHpr(0, -15, 0)
        # Wide FOV and generous far-clip so the whole campus is visible
        self.camLens.setFov(75)
        self.camLens.setNear(0.5)
        self.camLens.setFar(500)

    # ────────────────────────────────────────────────────────────────────────
    # Main update loop
    # ────────────────────────────────────────────────────────────────────────

    def _update(self, task):
        dt = globalClock.getDt()  # noqa: F821 – Panda3D global

        # 1. Update pedestrian movement
        self.ped_manager.update(dt)

        # 2. ── PERCEPTION ────────────────────────────────────────────────────
        #    Gather pedestrians inside the shuttle's forward detection zone.
        pedestrians_ahead = self._perceive_pedestrians()

        # 3. ── PLANNING ──────────────────────────────────────────────────────
        #    Decide target speed: stop if crossing pedestrian detected,
        #    slow if one is nearby, full speed otherwise.
        target_speed = self._plan(pedestrians_ahead)

        # 4. ── CONTROL ───────────────────────────────────────────────────────
        #    Feed target speed to shuttle; it blends smoothly.
        self.shuttle.update(dt, target_speed)

        # 5. Update HUD
        self.hud.update(
            speed=self.shuttle.current_speed,
            target_speed=target_speed,
            status=self._status_label(pedestrians_ahead),
            waypoint_idx=self.shuttle.waypoint_idx,
            total_waypoints=len(self.campus.waypoints),
        )

        return Task.cont

    # ────────────────────────────────────────────────────────────────────────
    # Perception
    # ────────────────────────────────────────────────────────────────────────

    def _perceive_pedestrians(self):
        """
        PERCEPTION MODULE
        -----------------
        Cast a rectangular detection zone in front of the shuttle.
        Any pedestrian whose world position falls inside that zone is
        considered 'detected'. Returns a list of (distance, pedestrian) tuples.
        """
        shuttle_pos = self.shuttle.node.getPos(self.render)
        shuttle_fwd = self.render.getRelativeVector(
            self.shuttle.node, Vec3(0, 1, 0)
        )
        shuttle_fwd.normalize()

        DETECTION_LENGTH = 12.0   # metres ahead to watch
        DETECTION_WIDTH  =  3.5   # metres either side of centre-line

        detected = []
        for ped in self.ped_manager.pedestrians:
            ped_pos = ped.node.getPos(self.render)
            delta   = ped_pos - shuttle_pos
            delta.z = 0

            fwd_dist  = delta.dot(shuttle_fwd)
            side_dist = abs(delta.dot(Vec3(-shuttle_fwd.y, shuttle_fwd.x, 0)))

            if 0 < fwd_dist < DETECTION_LENGTH and side_dist < DETECTION_WIDTH:
                detected.append((fwd_dist, ped))

        detected.sort(key=lambda x: x[0])
        return detected

    # ────────────────────────────────────────────────────────────────────────
    # Planning
    # ────────────────────────────────────────────────────────────────────────

    def _plan(self, pedestrians_ahead):
        """
        PLANNING MODULE
        ---------------
        Simple rule-based planner:
          - No pedestrians ahead  →  full cruise speed
          - Pedestrian 6-12 m away  →  slow to 30 % speed
          - Pedestrian < 6 m away   →  full stop
        """
        CRUISE_SPEED = 5.0   # m/s  (~18 km/h, appropriate campus speed)
        SLOW_SPEED   = 1.5
        STOP_SPEED   = 0.0

        if not pedestrians_ahead:
            return CRUISE_SPEED

        nearest_dist, _ = pedestrians_ahead[0]

        if nearest_dist < 6.0:
            return STOP_SPEED
        elif nearest_dist < 10.0:
            return SLOW_SPEED
        else:
            return CRUISE_SPEED

    def _status_label(self, pedestrians_ahead):
        if not pedestrians_ahead:
            return "CRUISING"
        d, _ = pedestrians_ahead[0]
        if d < 6.0:
            return "STOPPED – pedestrian"
        return "SLOWING – pedestrian ahead"

    # ────────────────────────────────────────────────────────────────────────
    # Debug toggle
    # ────────────────────────────────────────────────────────────────────────

    def _toggle_debug(self):
        self._debug_mode = not self._debug_mode
        self.ped_manager.show_debug(self._debug_mode)


if __name__ == "__main__":
    app = CampusShuttleSimulation()
    app.run()
