# Autonomous Underwater Gate Navigation System

## Overview

This project implements an autonomous underwater drone navigation system designed to locate, approach, and pass through underwater gates while avoiding obstacles. The solution uses computer vision for gate detection and a state machine approach for robust navigation control.

## Problem Statement

The challenge was to develop a navigation system that can:
1. Search for and locate underwater gates
2. Approach gates with proper alignment and aspect ratio correction
3. Navigate through gates safely
4. Avoid obstacles (flares, buoys, poles) during navigation
5. Handle underwater-specific challenges like buoyancy and drag

## Solution Architecture

### Core Components

1. **ROS2 Node Structure**: Built using `rclpy` with publishers/subscribers for:
   - Velocity commands (`/mavros/setpoint_velocity/cmd_vel_unstamped`)
   - Vision data (`/main_camera/detection/bounding_boxes`)
   - Vehicle state (`/mavros/state`)

2. **Computer Vision Integration**: Uses `vision_msgs/BoundingBoxArray` for object detection:
   - Gate detection (label_id=3)
   - Flare detection (label_id=1) 
   - General obstacles (label_id=0,2,4,5)

3. **State Machine Navigation**: 6-phase sequential approach for reliable navigation

## Navigation Strategy

### Phase-Based Approach

#### **Phase 1: Search**
- **Objective**: Locate the gate in the environment
- **Method**: Slow rotation (`0.22 rad/s`) with periodic forward movement
- **Logic**: Continue rotating until gate becomes visible
- **Transition**: Move to Phase 2 when gate detected

#### **Phase 2: Rough Centering** 
- **Objective**: Initial coarse alignment with the gate
- **Method**: High-gain proportional control for rapid correction
- **Parameters**: 
  - Strong gains (ROUGH_Z_GAIN=4.0, ROUGH_STRAFE_GAIN=2.0)
  - No deadband for continuous correction
  - Tolerance: ±0.05 for both X and Y axes
- **Transition**: Move to Phase 3 when roughly centered

#### **Phase 3: Approach**
- **Objective**: Move closer to gate while maintaining vertical stability
- **Method**: Forward movement with buoyancy compensation
- **Features**:
  - Vertical stabilization to counter buoyancy drift
  - Target Y position offset (0.45 instead of 0.5) to compensate for upward drift
  - Continue until gate reaches sufficient size (w/h > 0.6)
- **Transition**: Move to Phase 4 when gate is large enough

#### **Phase 4: Precise Centering**
- **Objective**: Achieve precise gate alignment before aspect ratio correction
- **Method**: High-precision proportional control
- **Parameters**:
  - Very high gains (z_gain=5.0, yaw_gain=3.0)
  - Tight tolerance (±0.05 for positioning)
  - Strong z-movement capability (clamp=0.8)
- **Transition**: Move to Phase 5 when precisely centered

#### **Phase 5: Aspect Ratio Correction**
- **Objective**: Achieve optimal gate viewing angle for safe passage
- **Method**: Orbital movement with adaptive direction control
- **Key Features**:
  - **Target Aspect Ratio**: 0.58 (optimized for gate passage)
  - **Orbital Motion**: Lateral movement to change viewing angle
  - **Adaptive Direction**: Reverses orbit direction if AR gets worse
  - **Cycle Management**: Alternates between 3s orbit and 1s centering
  - **Stuck Detection**: Forces go-through if minimal movement for 10+ seconds
  - **No Timeout**: Continues until AR tolerance (±0.03) is achieved

#### **Phase 6: Go Through**
- **Objective**: Pass through the gate safely
- **Method**: Direct forward movement at final speed (0.8 m/s)
- **Safety**: Continues obstacle avoidance during passage

### Underwater-Specific Adaptations

#### **Buoyancy Compensation**
- **Problem**: Positive buoyancy causes upward drift
- **Solution**: 
  - Offset target Y position to 0.45 (instead of 0.5)
  - Continuous vertical stabilization during all phases
  - Proportional control: `z_correction = -gain * (current_y - target_y)`

#### **High-Gain Control System**
- **Problem**: Water drag requires stronger control inputs
- **Solution**:
  - Dramatically increased gains (5x normal for Z-axis)
  - Higher velocity clamps to overcome drag
  - No deadband zones for continuous correction

#### **Hybrid Control Approach**
- **Horizontal Control**: Uses yaw (angular.z) for left/right movement
- **Vertical Control**: Uses direct Z velocity (linear.z) for up/down
- **Forward Control**: Uses X velocity (linear.x) for approach/retreat

## Obstacle Avoidance System

### Detection Strategy
- **Trigger**: Any non-gate object with width OR height > 0.6
- **Coverage**: Active during all navigation phases
- **Types**: Flares, buoys, poles, and general obstacles

### Avoidance Logic
1. **Direction Selection**: Move to opposite side of screen from obstacle
   - If obstacle at x < 0.5 (left side) → move right
   - If obstacle at x > 0.5 (right side) → move left

2. **Two-Phase Execution**:
   - **Phase 1**: Strafe until obstacle is completely off-screen
   - **Phase 2**: Brief forward movement (1.0s) to clear obstacle area

3. **Light Touch Approach**:
   - Reduced strafe speed (0.3 m/s) for gentle movement
   - Minimal forward movement (0.2 m/s for 1.0s)
   - Quick return to normal navigation

## Advanced Features

### **Aspect Ratio Correction Algorithm**
```python
# Core AR correction logic
ar_error = current_ar - TARGET_AR
strafe_y = orbit_gain * ar_error * direction_multiplier

# Orbital movement principle:
# If AR too low (gate too tall) → strafe LEFT to see gate from side
# If AR too high (gate too wide) → strafe RIGHT for straight-on view
```

### **Adaptive Direction Control**
- Monitors AR improvement over 3-second cycles
- Reverses orbit direction if AR gets worse by >0.02
- Maintains direction if AR improves by >0.01
- Provides robust correction even with noisy vision data

### **Movement Monitoring**
- Tracks gate position changes over time
- Detects "stuck" conditions (movement < 0.02 for 10+ seconds)
- Forces go-through to prevent infinite loops in AR correction

### **Gentle Backup System**
- Activates when gate becomes too large (w/h > 0.85)
- Performs conservative backward pulses until gate size reduces
- Prevents collision with gate structure

## Technical Implementation

### **Control Gains Tuning**
```python
# Rough centering (Phase 2)
ROUGH_STRAFE_GAIN = 2.0    # Horizontal correction
ROUGH_Z_GAIN = 4.0         # Vertical correction (4x for underwater)

# Precise control (Phase 4+)
yaw_gain = 3.0             # Horizontal precision
z_gain = 5.0               # Vertical precision (5x for drag)
```

### **Target Parameters**
```python
TARGET_AR = 0.58           # Optimal aspect ratio for passage
TARGET_X = 0.5             # Horizontal center
TARGET_Y = 0.45            # Vertical center (offset for buoyancy)
```

### **Speed Configuration**
```python
SEARCH_YAW = 0.22          # Search rotation speed
APPROACH_SPEED = 0.4       # Approach velocity
FINAL_SPEED = 0.8          # Go-through velocity
```

## Key Innovations

1. **Phase-Based State Machine**: Ensures systematic progression through navigation stages
2. **Buoyancy-Aware Control**: Compensates for underwater vehicle dynamics
3. **Adaptive AR Correction**: Self-correcting orbital motion for optimal gate alignment
4. **Stuck Detection**: Prevents infinite loops in complex scenarios
5. **Universal Obstacle Avoidance**: Works across all navigation phases
6. **High-Gain Underwater Control**: Overcomes water drag and disturbances

## Quick Start Commands

### Prerequisites
- ROS2 (Humble)
- MAVROS for vehicle control
- vision_msgs for object detection
- Underwater vehicle with camera and detection system

### Build and Run
```bash
# Terminal 1: Start ROS TCP endpoint (Unity-ROS bridge)
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Build the workspace (if needed)
cd /home/nhs172003/probation_ws
colcon build --packages-select gate_navigator

# Terminal 3: Run the navigator
ros2 run gate_navigator navigator
```

### Configuration
Key parameters can be adjusted in the navigator.py `__init__` method:
- Navigation speeds and gains
- Tolerance values for centering
- Aspect ratio targets
- Obstacle detection thresholds

## Performance Characteristics

- **Robustness**: Handles vision noise, vehicle drift, and environmental disturbances
- **Efficiency**: Direct phase progression without unnecessary iterations
- **Safety**: Comprehensive obstacle avoidance and collision prevention
- **Adaptability**: Self-correcting algorithms for varying gate orientations
- **Reliability**: Stuck detection and timeout mechanisms prevent system lock-up

## Original Probation Task Information

This solution was developed for the Mecatron probation task involving autonomous gate navigation using Unity simulation and ROS 2. The original task requirements included:

- Navigate an autonomous underwater vehicle through a gate
- Handle random initial positions and imperfect object detection
- Demonstrate robust control algorithms and system integration
- Achieve successful gate passage from 3 different starting positions

For complete original documentation and setup instructions, refer to the git history or contact the development team.

---

*This navigation system represents a comprehensive solution to autonomous underwater gate navigation, combining robust control theory with practical underwater vehicle considerations.*


