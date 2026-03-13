"""
hud.py
======
Heads-Up Display: overlays telemetry info on screen.
"""

from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode


class HUD:
    """
    Renders four lines of telemetry in the top-left corner:
      • Speed (m/s and km/h)
      • Target speed
      • Autonomous status
      • Waypoint progress
    """

    def __init__(self, base):
        common = dict(
            fg=(1, 1, 1, 1),
            shadow=(0, 0, 0, 0.8),
            scale=0.055,
            align=TextNode.ALeft,
            mayChange=True,
        )

        self._speed  = OnscreenText(text="", pos=(-1.55, 0.90), **common)
        self._target = OnscreenText(text="", pos=(-1.55, 0.83), **common)
        self._status = OnscreenText(text="", pos=(-1.55, 0.76), **common)
        self._wp     = OnscreenText(text="", pos=(-1.55, 0.69), **common)

        # Title
        OnscreenText(
            text="AUTONOMOUS CAMPUS SHUTTLE",
            fg=(1, 0.95, 0.4, 1),
            shadow=(0, 0, 0, 0.9),
            scale=0.055,
            pos=(0, 0.92),
            align=TextNode.ACenter,
            mayChange=False,
        )

        # Controls hint
        OnscreenText(
            text="ESC – quit   F1 – debug",
            fg=(0.8, 0.8, 0.8, 0.8),
            scale=0.040,
            pos=(1.55, -0.92),
            align=TextNode.ARight,
            mayChange=False,
        )

    def update(self, speed, target_speed, status, waypoint_idx, total_waypoints):
        kmh = speed * 3.6
        self._speed.setText(f"Speed:  {speed:4.1f} m/s  ({kmh:4.1f} km/h)")
        self._target.setText(f"Target: {target_speed:4.1f} m/s")
        self._status.setText(f"Status: {status}")
        self._wp.setText(f"Waypoint: {waypoint_idx}/{total_waypoints}")
