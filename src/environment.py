"""
environment.py
==============
Builds the procedural 3D campus scene:
  • Ground plane
  • Road loop with lane markings
  • Sidewalks
  • Crosswalks
  • Buildings (varied sizes)
  • Trees
  • Waypoints for the shuttle path
"""

import math
import random
from panda3d.core import (
    Vec3, Vec4, Point3,
    GeomNode, Geom, GeomTriangles, GeomVertexData,
    GeomVertexFormat, GeomVertexWriter,
    NodePath, LVecBase4f,
)


# ─────────────────────────────────────────────────────────────────────────────
# Low-level geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_box(w, h, d, color):
    """Return a NodePath containing a coloured axis-aligned box centred at origin."""
    fmt    = GeomVertexFormat.getV3n3c4()
    vdata  = GeomVertexData("box", fmt, Geom.UHStatic)
    vdata.setNumRows(24)

    vtx   = GeomVertexWriter(vdata, "vertex")
    nrm   = GeomVertexWriter(vdata, "normal")
    col   = GeomVertexWriter(vdata, "color")

    hw, hh, hd = w / 2, h / 2, d / 2
    r, g, b, a = color

    faces = [
        # (normal,  four corners)
        (( 0,  0,  1), [(-hw, -hd, hh), ( hw, -hd, hh), ( hw,  hd, hh), (-hw,  hd, hh)]),
        (( 0,  0, -1), [(-hw,  hd,-hh), ( hw,  hd,-hh), ( hw, -hd,-hh), (-hw, -hd,-hh)]),
        (( 0,  1,  0), [(-hw,  hd,-hh), ( hw,  hd,-hh), ( hw,  hd, hh), (-hw,  hd, hh)]),
        (( 0, -1,  0), [(-hw, -hd, hh), ( hw, -hd, hh), ( hw, -hd,-hh), (-hw, -hd,-hh)]),
        (( 1,  0,  0), [( hw, -hd,-hh), ( hw,  hd,-hh), ( hw,  hd, hh), ( hw, -hd, hh)]),
        ((-1,  0,  0), [(-hw,  hd,-hh), (-hw, -hd,-hh), (-hw, -hd, hh), (-hw,  hd, hh)]),
    ]

    prim = GeomTriangles(Geom.UHStatic)

    for face_idx, (n, corners) in enumerate(faces):
        base = face_idx * 4
        for cx, cy, cz in corners:
            vtx.addData3(cx, cy, cz)
            nrm.addData3(*n)
            col.addData4(r, g, b, a)
        prim.addVertices(base, base + 1, base + 2)
        prim.addVertices(base, base + 2, base + 3)

    geom = Geom(vdata)
    geom.addPrimitive(prim)
    node = GeomNode("box")
    node.addGeom(geom)
    return NodePath(node)


def _make_quad(w, d, color, normal=(0, 0, 1)):
    """Flat horizontal quad (or tilted if normal differs)."""
    fmt   = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData("quad", fmt, Geom.UHStatic)
    vdata.setNumRows(4)

    vtx = GeomVertexWriter(vdata, "vertex")
    nrm = GeomVertexWriter(vdata, "normal")
    clr = GeomVertexWriter(vdata, "color")

    hw, hd = w / 2, d / 2
    r, g, b, a = color
    corners = [(-hw, -hd, 0), (hw, -hd, 0), (hw, hd, 0), (-hw, hd, 0)]
    for cx, cy, cz in corners:
        vtx.addData3(cx, cy, cz)
        nrm.addData3(*normal)
        clr.addData4(r, g, b, a)

    prim = GeomTriangles(Geom.UHStatic)
    prim.addVertices(0, 1, 2)
    prim.addVertices(0, 2, 3)

    geom = Geom(vdata)
    geom.addPrimitive(prim)
    node = GeomNode("quad")
    node.addGeom(geom)
    return NodePath(node)


# ─────────────────────────────────────────────────────────────────────────────
# CampusEnvironment
# ─────────────────────────────────────────────────────────────────────────────

class CampusEnvironment:
    """
    Assembles the entire campus scene and exposes:
      .waypoints       – ordered list of Vec3 positions for shuttle path
      .sidewalk_paths  – list of (start, end) pairs for pedestrian lanes
      .crosswalk_zones – list of (centre, half_width, half_depth) for crosswalks
    """

    # Road loop parameters
    ROAD_W  = 7.0    # total road width
    TRACK_R = 30.0   # radius of the circular campus loop
    NUM_WP  = 80     # waypoints around the loop

    def __init__(self, render):
        self.render = render
        random.seed(42)

        self._build_ground()
        self._build_road()
        self._build_sidewalks()
        self._build_crosswalks()
        self._build_buildings()
        self._build_trees()

        self.waypoints       = self._compute_waypoints()
        self.sidewalk_paths  = self._compute_sidewalk_paths()
        self.crosswalk_zones = self._compute_crosswalk_zones()

    # ── Ground ────────────────────────────────────────────────────────────

    def _build_ground(self):
        ground = _make_quad(200, 200, (0.30, 0.52, 0.22, 1.0))
        ground.reparentTo(self.render)
        ground.setPos(0, 0, -0.02)

    # ── Road loop ─────────────────────────────────────────────────────────

    def _build_road(self):
        """Approximate the circular road with N quad segments."""
        R  = self.TRACK_R
        N  = 80
        rw = self.ROAD_W

        for i in range(N):
            a0 = 2 * math.pi * i     / N
            a1 = 2 * math.pi * (i+1) / N
            am = (a0 + a1) / 2

            cx = math.cos(am) * R
            cy = math.sin(am) * R
            seg_len = 2 * math.pi * R / N + 0.05

            seg = _make_quad(rw, seg_len, (0.25, 0.25, 0.28, 1.0))
            seg.reparentTo(self.render)
            seg.setPos(cx, cy, 0.001)
            seg.setHpr(math.degrees(-am) + 90, 0, 0)

        # Centre dashed line
        for i in range(N):
            if i % 4 < 2:
                continue
            a0 = 2 * math.pi * i / N
            am = a0 + math.pi / N
            cx = math.cos(am) * R
            cy = math.sin(am) * R
            dash = _make_quad(0.15, 1.0, (0.95, 0.90, 0.30, 1.0))
            dash.reparentTo(self.render)
            dash.setPos(cx, cy, 0.003)
            dash.setHpr(math.degrees(-am) + 90, 0, 0)

    # ── Sidewalks ─────────────────────────────────────────────────────────

    SIDEWALK_R_OUTER = 37.0
    SIDEWALK_R_INNER = 24.5
    SIDEWALK_W       =  2.5

    def _build_sidewalks(self):
        for R in (self.SIDEWALK_R_OUTER, self.SIDEWALK_R_INNER):
            N = 80
            for i in range(N):
                a0 = 2 * math.pi * i / N
                am = a0 + math.pi / N
                cx = math.cos(am) * R
                cy = math.sin(am) * R
                seg_len = 2 * math.pi * R / N + 0.05

                seg = _make_quad(self.SIDEWALK_W, seg_len, (0.75, 0.73, 0.70, 1.0))
                seg.reparentTo(self.render)
                seg.setPos(cx, cy, 0.002)
                seg.setHpr(math.degrees(-am) + 90, 0, 0)

    # ── Crosswalks ────────────────────────────────────────────────────────

    CROSSWALK_ANGLES = [0, 90, 180, 270]   # degrees around the loop

    def _build_crosswalks(self):
        R = self.TRACK_R
        for deg in self.CROSSWALK_ANGLES:
            rad = math.radians(deg)
            cx  = math.cos(rad) * R
            cy  = math.sin(rad) * R
            hpr = deg + 90   # perpendicular to road

            # Five white stripes
            for k in range(-2, 3):
                stripe = _make_quad(0.6, self.ROAD_W, (0.95, 0.95, 0.95, 1.0))
                stripe.reparentTo(self.render)
                stripe.setPos(
                    cx + math.cos(rad) * k * 1.0,
                    cy + math.sin(rad) * k * 1.0,
                    0.004
                )
                stripe.setHpr(hpr, 0, 0)

    # ── Buildings ─────────────────────────────────────────────────────────

    BUILDING_COLORS = [
        (0.75, 0.68, 0.58, 1.0),   # sandstone
        (0.62, 0.55, 0.72, 1.0),   # slate
        (0.55, 0.68, 0.60, 1.0),   # green-grey
        (0.80, 0.62, 0.50, 1.0),   # terracotta
        (0.65, 0.72, 0.78, 1.0),   # concrete blue
    ]

    def _build_buildings(self):
        """Place buildings in two rings – outer campus and inner quad."""
        specs = []

        # Outer ring  (8 buildings)
        for i in range(8):
            ang = math.radians(i * 45)
            R   = 55 + random.uniform(-5, 5)
            w   = random.uniform(6, 12)
            d   = random.uniform(6, 10)
            h   = random.uniform(8, 22)
            specs.append((math.cos(ang)*R, math.sin(ang)*R, w, d, h))

        # Inner quad  (4 buildings)
        for i in range(4):
            ang = math.radians(i * 90 + 45)
            R   = 14 + random.uniform(-2, 2)
            w   = random.uniform(5, 9)
            d   = random.uniform(5, 9)
            h   = random.uniform(6, 15)
            specs.append((math.cos(ang)*R, math.sin(ang)*R, w, d, h))

        for idx, (bx, by, bw, bd, bh) in enumerate(specs):
            color = self.BUILDING_COLORS[idx % len(self.BUILDING_COLORS)]
            bldg = _make_box(bw, bh, bd, color)
            bldg.reparentTo(self.render)
            bldg.setPos(bx, by, bh / 2)

            # Roof cap – slightly darker
            roof_c = tuple(min(1.0, c + 0.08) if i < 3 else c
                           for i, c in enumerate(color))
            roof = _make_box(bw + 0.3, 0.5, bd + 0.3, roof_c)
            roof.reparentTo(self.render)
            roof.setPos(bx, by, bh + 0.25)

    # ── Trees ─────────────────────────────────────────────────────────────

    def _build_trees(self):
        N = 30
        for i in range(N):
            ang = random.uniform(0, 2 * math.pi)
            # Place trees along the outer sidewalk band
            R   = self.SIDEWALK_R_OUTER + random.uniform(2, 8)
            tx  = math.cos(ang) * R
            ty  = math.sin(ang) * R
            h   = random.uniform(3.5, 6.5)

            trunk = _make_box(0.4, h, 0.4, (0.42, 0.28, 0.14, 1.0))
            trunk.reparentTo(self.render)
            trunk.setPos(tx, ty, h / 2)

            cr = random.uniform(2.5, 4.0)
            canopy = _make_box(cr, cr * 0.8, cr, (0.18, 0.52 + random.uniform(-0.1, 0.1), 0.18, 1.0))
            canopy.reparentTo(self.render)
            canopy.setPos(tx, ty, h + cr * 0.4)

    # ── Path / waypoint helpers ───────────────────────────────────────────

    def _compute_waypoints(self):
        """Return ordered Vec3 positions along the road centre-line."""
        R  = self.TRACK_R
        N  = self.NUM_WP
        wps = []
        for i in range(N):
            a = 2 * math.pi * i / N
            wps.append(Vec3(math.cos(a) * R, math.sin(a) * R, 0.5))
        return wps

    def _compute_sidewalk_paths(self):
        """
        Return a list of (start_Vec3, end_Vec3) segments representing
        the pedestrian walkable zone on the outer sidewalk.
        """
        R  = self.SIDEWALK_R_OUTER
        N  = 40
        pts = [Vec3(math.cos(2*math.pi*i/N)*R, math.sin(2*math.pi*i/N)*R, 0)
               for i in range(N)]
        return [(pts[i], pts[(i+1) % N]) for i in range(N)]

    def _compute_crosswalk_zones(self):
        """
        Return list of (centre Vec3, half_width, half_depth) for each crosswalk.
        Used by pedestrian manager to place crossing pedestrians.
        """
        R    = self.TRACK_R
        zones = []
        for deg in self.CROSSWALK_ANGLES:
            rad = math.radians(deg)
            cx  = math.cos(rad) * R
            cy  = math.sin(rad) * R
            zones.append((Vec3(cx, cy, 0), 2.0, self.ROAD_W / 2 + 2))
        return zones
