# MOCHA
**Multi-Agent Trajectory Planning with MOCHA Optimization and Homotopy-Aware Path Generation**

<div align="center">

<table>
<tr>
<td><img src="assets/demo_multi.gif" alt="Multi-robot Demo" width="400"/></td>
<td><img src="assets/avoidance_event.gif" alt="Avoidance Event" width="400"/></td>
</tr>
<tr>
<td align="center">*Multi-robot collision avoidance using MOCHA in real-world*</td>
<td align="center">*Real-time multi-robot collision avoidance using MOCHA*</td>
</tr>
</table>

</div>

## 📖 Overview

MOCHA is a trajectory planning framework for multi-agent systems that addresses the computational challenges of high-dimensional optimization and local minima in complex environments. Inspired by the motion camouflage phenomenon in nature, MOCHA introduces a novel reparameterization technique that transforms high-dimensional waypoint optimization into a sequence of one-dimensional scalar parameters, significantly reducing decision dimensionality and accelerating computation speed.

The framework integrates **homotopy-aware path planning** using H-signature to distinguish topologically distinct path classes, effectively mitigating local minima through multi-candidate parallel optimization.

### Key Features
✅ **Motion Camouflage Reparameterization** - Reduces trajectory dimensionality for faster optimization  
✅ **Homotopy-Aware Planning** - Multi-topology path search with H-signature classification  
✅ **Real-Time Performance** - Millisecond-level planning speed for rapid replanning  
✅ **Multi-Agent Coordination** - Distributed asynchronous planning with collision-free guarantees   

## 🚀 Deployment

### Prerequisites

Ensure you have **ROS2** installed on your system. Recommended setup:
- **Ubuntu 22.04 LTS**
- **ROS2 Humble**

For ROS2 installation, refer to the [official ROS2 documentation](https://docs.ros.org/en/humble/Installation.html).

**Additional Dependencies:**

Install required libraries:
```bash
sudo apt update && sudo apt install -y libcgal-dev libeigen3-dev
```

### Installation

```bash
# 1. Create workspace
mkdir -p ~/ghost_ws/src
cd ~/ghost_ws/src

# 2. Clone repository
git clone https://github.com/SiriHdu/MOCHA.git

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
