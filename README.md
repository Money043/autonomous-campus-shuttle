# 🚌 Autonomous Campus Shuttle Simulation

A fully 3-D autonomous shuttle simulation built with **Panda3D** and pure Python.
Demonstrates a complete perception → planning → control pipeline in a procedurally
generated university campus environment.

---

## 📁 Project Structure

```
campus_shuttle/
├── main.py                 # Application entry-point & main loop
├── requirements.txt
├── README.md
└── src/
    ├── __init__.py
    ├── environment.py      # Campus scene: roads, buildings, trees, crosswalks
    ├── shuttle.py          # 3-D model + autonomous CONTROL logic
    ├── pedestrians.py      # Pedestrian spawning & movement
    └── hud.py              # On-screen telemetry display
```

---

## ⚙️ Installation

### 1. Prerequisites

- **macOS** (Intel or Apple Silicon) with **Python 3.9+**
- Pip (comes with Python)

Check your version:
```bash
python3 --version
```

### 2. (Recommended) Create a virtual environment

```bash
cd campus_shuttle
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install Panda3D

```bash
pip install -r requirements.txt
```

> **Apple Silicon note:** Panda3D ships a universal wheel for macOS.
> If you encounter any issues, try:
> ```bash
> pip install --pre panda3d
> ```

---

## ▶️ Running the Simulation

```bash
python3 main.py
```

The window will open at **1280 × 720**. The shuttle starts moving immediately.

### Keyboard controls

| Key | Action |
|-----|--------|
| **ESC** | Quit |
| **F1** | Toggle debug mode |

---

## 🧠 Simulation Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     MAIN LOOP  (60 fps)                 │
│                                                         │
│  ┌──────────────┐   ┌──────────────┐  ┌─────────────┐  │
│  │  PERCEPTION  │──▶│   PLANNING   │─▶│   CONTROL   │  │
│  │              │   │              │  │             │  │
│  │ Scan forward │   │ cruise/slow/ │  │ Ramp speed  │  │
│  │ detection    │   │ stop rules   │  │ Pure-pursuit│  │
│  │ zone for     │   │ based on     │  │ heading     │  │
│  │ pedestrians  │   │ nearest ped  │  │ correction  │  │
│  └──────────────┘   └──────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### Perception
- A rectangular forward detection zone (12 m × 7 m) is cast in front of the shuttle each frame.
- Any pedestrian whose world position falls inside is considered "detected".

### Planning
| Nearest pedestrian distance | Decision |
|-----------------------------|----------|
| > 10 m | Cruise at 5 m/s |
| 6 – 10 m | Slow to 1.5 m/s |
| < 6 m | Full stop |

### Control
- Speed is smoothly ramped (4 m/s² accel, 6 m/s² braking).
- Heading is corrected each frame toward the next waypoint (pure-pursuit style).
- The route is a circular loop of 80 waypoints around the campus.

---

## 🔮 Optional Future Improvements

| Feature | Approach |
|---------|----------|
| **Lidar / raycasting** | Panda3D `CollisionRay` for physics-based detection |
| **Traffic lights** | State machine at each crosswalk; shuttle obeys signals |
| **Multiple shuttles** | Instantiate N `AutonomousShuttle` objects with offset start indices |
| **Textured assets** | Replace procedural boxes with `.egg` / `.gltf` models |
| **Path re-planning** | A* on a road graph for flexible routing |
| **Pedestrian AI** | BehaviorTree or steering behaviours for richer movement |
| **Day/night cycle** | Animated `DirectionalLight` colour and intensity |
| **Sound** | Panda3D `AudioManager` for engine / pedestrian ambience |
