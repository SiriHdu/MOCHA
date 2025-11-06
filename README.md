# MOCHA
**Multi-Agent Trajectory Planning with MOCHA Optimization and Homotopy-Aware Path Generation**

<div align="center">

<table>
<tr>
<td><img src="assets/demo_multi.gif" alt="Multi-robot Demo" width="400"/></td>
<td><img src="assets/avoidance_event.gif" alt="Avoidance Event" width="400"/></td>
</tr>
</table>

*Multi-robot collision avoidance using MOCHA in real-world*

</div>

<video src="https://github.com/SiriHdu/MOCHA/raw/main/assets/multi_demo.mp4" controls width="100%">
  <p>Your browser doesn't support HTML5 video. <a href="assets/multi_demo.mp4">Download the video</a> instead.</p>
</video>

## 📖 Overview

GHOST Planner is a ROS2-based global trajectory planning system designed for **multi-agent** environments. It combines:

- **MOCHA (Motion Camouflage-based Homotopy Aware) Optimization**: Polynomial trajectory optimization using L-BFGS solver
- **CGAL Homotopy Path Generation**: Apollonius graph-based topologically distinct path planning
- **Spatiotemporal Collision Avoidance**: Real-time multi-robot coordination

### Key Features

✅ **Topologically Diverse Paths** - Generates multiple homotopy classes for robust planning  
✅ **Smooth Trajectories** - 5th-order polynomial optimization with continuous acceleration  
✅ **Multi-Agent Coordination** - Distributed planning with collision-free guarantees  
✅ **Real-Time Performance** - Parallel optimization, 50-500ms planning time  
✅ **ROS2 Action Interface** - Easy integration with navigation stacks  

## 🎯 Use Cases

- Multi-robot warehouse logistics
- Drone swarm navigation
- Autonomous vehicle coordination
- Research in motion planning algorithms

## 🔬 Methodology

### 1. Homotopy Path Generation (CGAL)
- Constructs Apollonius graph from circular obstacles
- Extracts Voronoi skeleton for collision-free space
- Generates topologically distinct candidate paths using Dijkstra search

### 2. MOCHA Trajectory Optimization
- **Representation**: Piecewise polynomial trajectories (5th order Bezier curves)
- **Optimization**: L-BFGS minimizes weighted objective:
  ```
  J = w₁·Energy + w₂·Time + w₃·Obstacles + w₄·MultiAgent
  ```
- **Constraints**: Velocity/acceleration limits, continuity, collision avoidance

### 3. Multi-Agent Coordination
- Robots share committed trajectories via ROS2 topics
- Spatiotemporal collision checking during optimization
- Idle robots treated as dynamic obstacles

## 🚀 Deployment

### Prerequisites

**System Requirements:**
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill

**Dependencies:**
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  libcgal-dev \
  libeigen3-dev \
  build-essential \
  cmake \
  python3-colcon-common-extensions
```

See `requirements.txt` for complete dependency list.

### Installation

```bash
# 1. Create workspace
mkdir -p ~/ghost_ws/src
cd ~/ghost_ws/src

# 2. Clone repository
git clone https://github.com/yourusername/ghost.git

# 3. Build
cd ~/ghost_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ghost_planner

# 4. Source workspace
source install/setup.bash
```

### Quick Start

**Launch the verification system:**
```bash
ros2 launch ghost_planner verification_global_system.launch.py
```

This starts:
- 3 robot planners (`robot_1`, `robot_2`, `robot_3`)
- Random obstacle generator
- RViz visualization
- Trajectory tracking nodes

**Set planning goals:**

In RViz, use the **"2D Goal Pose"** tool to click on the map and set goals for each robot.

Or via command line:
```bash
ros2 action send_goal /robot_1/global_plan ghost_planner/action/GlobalPlan \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}}}}"
```

### Configuration

Edit `src/GHOST Planner/config/global_planner.yaml`:

```yaml
mocha:
  v_max: 2.0              # Maximum velocity (m/s)
  a_max: 2.0              # Maximum acceleration (m/s²)
  drone_radius: 0.3       # Robot radius (m)
  w_obstacle: 100.0       # Obstacle avoidance weight
  w_peer: 50.0            # Multi-agent avoidance weight
  peer_safety_margin: 0.4 # Safety distance between robots (m)
```

## 📊 Performance

**Benchmark** (Intel i7 @ 3.5GHz):
- Single robot: **50-150 ms**
- 3 homotopy paths: **150-400 ms**
- Multi-robot (3 agents): **200-500 ms**

**Optimization enabled:** `-O3` compiler flag (Release mode)

## 🗂️ Project Structure

```
ghost/
├── src/GHOST Planner/
│   ├── src/                    # C++ source files
│   │   ├── global_planner_action_server.cpp
│   │   ├── mocha_optimizer.cpp
│   │   ├── cgal_homotopy_planner.cpp
│   │   └── verifation_global_planner.cpp
│   ├── include/ghost planner/  # Header files
│   ├── config/                 # YAML configuration
│   ├── launch/                 # Launch files
│   ├── rviz/                   # RViz configs
│   └── action/                 # ROS2 action definitions
├── assets/                     # Media files (GIFs, videos)
├── README.md
└── requirements.txt
```

## 📡 ROS2 Interface

### Topics

**Subscribed:**
- `/obstacles` (visualization_msgs/MarkerArray) - Dynamic obstacles
- `/team_committed_trajectory` (ghost_planner/CommittedTrajectory) - Peer trajectories

**Published:**
- `/global_path` (nav_msgs/Path) - Optimized path
- `/skeleton_markers` (visualization_msgs/MarkerArray) - Homotopy skeleton
- `/optimized_paths_markers` (visualization_msgs/MarkerArray) - Trajectory visualization

### Action Server

**Action:** `ghost_planner/action/GlobalPlan`

**Goal:** Target pose (geometry_msgs/PoseStamped)  
**Result:** Optimized trajectory with polynomial coefficients  
**Feedback:** Planning status and progress  

## 🛠️ Troubleshooting

**Build fails:**
- Ensure all dependencies installed: `rosdep install --from-paths src --ignore-src -r -y`

**No visualization in RViz:**
- Check topic publishing: `ros2 topic list`
- Verify RViz config loaded from launch file

**Planning fails:**
- Verify start/goal are collision-free
- Check obstacle topic: `ros2 topic echo /obstacles`
- Increase safety margins in config

**CGAL warning (safe to ignore):**
```
CGAL_DATA_DIR cannot be deduced
```

## 📜 License

Apache-2.0

## 🙏 Acknowledgments

Built with:
- [ROS2](https://www.ros.org/) - Robot Operating System
- [CGAL](https://www.cgal.org/) - Computational Geometry Algorithms Library
- [Eigen](https://eigen.tuxfamily.org/) - Linear algebra library

## 📧 Contact

For questions, issues, or collaboration:
- Open an issue on GitHub
- Pull requests welcome!

---

**⭐ Star this repo if you find it useful!**
