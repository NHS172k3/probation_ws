import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from vision_msgs.msg import BoundingBoxArray
import subprocess
import time

class GateNavigator(Node):
    # Rough centering parameters (for approach phase)
    ROUGH_STRAFE_GAIN = 2.0  # Increased for stronger horizontal correction
    ROUGH_Z_GAIN = 4.0  # DRAMATICALLY increased for stronger vertical correction
    ROUGH_STRAFE_CLAMP = 0.25
    ROUGH_Z_CLAMP = 0.5  # Much higher clamp to allow stronger z movement
    ROUGH_DEADBAND = 0.0  # NO deadband - continuous correction even at small errors
    def get_aspect_ratio(self):
        # Avoid division by zero
        if self.gate_h == 0:
            return 0.0
        return self.gate_w / self.gate_h

    def is_gate_visible(self):
        # Robust: gate is visible if seen in last 2 seconds
        now = time.time()
        if self.last_gate_detection_time is not None and (now - self.last_gate_detection_time) < 2.0:
            return True
        return False
    def state_callback(self, msg):
        pass

    def vision_callback(self, msg):
        # Support both BoundingBoxArray fields: boxes (3D) and bounding_boxes (2D)
        found_gate = False
        found_flare = False
        obstacles_found = []
        
        box_list = msg.bounding_boxes
        for box in box_list:
            # Support both label_id and label_name
            is_gate = False
            is_flare = False
            is_obstacle = False
            
            if hasattr(box, 'label_id'):
                if box.label_id == 3:
                    is_gate = True
                elif box.label_id == 1:  # Flare (orange stick)
                    is_flare = True
                elif box.label_id in [0, 2, 4, 5]:  # Other obstacle IDs
                    is_obstacle = True
            elif hasattr(box, 'label_name'):
                if box.label_name.lower() in ['gate']:
                    is_gate = True
                elif box.label_name.lower() in ['flare', 'orange', 'stick']:
                    is_flare = True
                elif box.label_name.lower() in ['obstacle', 'buoy', 'pole']:
                    is_obstacle = True
            
            # Process gate detection
            if is_gate:
                # Support both (x, y, w, h) and (center, size)
                if hasattr(box, 'x') and hasattr(box, 'y') and hasattr(box, 'w') and hasattr(box, 'h'):
                    self.gate_x = box.x
                    self.gate_y = box.y
                    self.gate_w = box.w
                    self.gate_h = box.h
                elif hasattr(box, 'center') and hasattr(box, 'size'):
                    self.gate_x = (box.center.position.x if hasattr(box.center, 'position') else box.center.x)
                    self.gate_y = (box.center.position.y if hasattr(box.center, 'position') else box.center.y)
                    self.gate_w = box.size.x if hasattr(box.size, 'x') else box.size[0] if hasattr(box.size, '__getitem__') else 0.0
                    self.gate_h = box.size.y if hasattr(box.size, 'y') else box.size[1] if hasattr(box.size, '__getitem__') else 0.0
                else:
                    # Fallback: set to 0 if unknown
                    self.gate_x = 0.5
                    self.gate_y = 0.5
                    self.gate_w = 0.0
                    self.gate_h = 0.0
                
                # Check for unstable detection (large position jump)
                x_jump = abs(self.gate_x - self.prev_gate_x)
                y_jump = abs(self.gate_y - self.prev_gate_y)
                if (x_jump > 0.3 or y_jump > 0.3) and self.gate_detection_count > 5:
                    self.get_logger().warn(f'⚠️  UNSTABLE DETECTION! Gate jumped: x={self.prev_gate_x:.2f}→{self.gate_x:.2f} y={self.prev_gate_y:.2f}→{self.gate_y:.2f}')
                
                self.prev_gate_x = self.gate_x
                self.prev_gate_y = self.gate_y
                found_gate = True
                self.last_seen = time.time()
                self.gate_detection_count += 1
                self.last_gate_detection_time = time.time()
            
            # Process flare detection
            elif is_flare:
                if hasattr(box, 'x') and hasattr(box, 'y') and hasattr(box, 'w') and hasattr(box, 'h'):
                    self.flare_x = box.x
                    self.flare_y = box.y
                    self.flare_w = box.w
                    self.flare_h = box.h
                elif hasattr(box, 'center') and hasattr(box, 'size'):
                    self.flare_x = (box.center.position.x if hasattr(box.center, 'position') else box.center.x)
                    self.flare_y = (box.center.position.y if hasattr(box.center, 'position') else box.center.y)
                    self.flare_w = box.size.x if hasattr(box.size, 'x') else box.size[0] if hasattr(box.size, '__getitem__') else 0.0
                    self.flare_h = box.size.y if hasattr(box.size, 'y') else box.size[1] if hasattr(box.size, '__getitem__') else 0.0
                
                found_flare = True
                self.flare_detected = True
                self.last_flare_detection_time = time.time()
                
            # Process other obstacles
            elif is_obstacle:
                obstacle_data = {}
                if hasattr(box, 'x') and hasattr(box, 'y') and hasattr(box, 'w') and hasattr(box, 'h'):
                    obstacle_data = {'x': box.x, 'y': box.y, 'w': box.w, 'h': box.h}
                elif hasattr(box, 'center') and hasattr(box, 'size'):
                    obstacle_data = {
                        'x': (box.center.position.x if hasattr(box.center, 'position') else box.center.x),
                        'y': (box.center.position.y if hasattr(box.center, 'position') else box.center.y),
                        'w': box.size.x if hasattr(box.size, 'x') else box.size[0] if hasattr(box.size, '__getitem__') else 0.0,
                        'h': box.size.y if hasattr(box.size, 'y') else box.size[1] if hasattr(box.size, '__getitem__') else 0.0
                    }
                if obstacle_data:
                    obstacles_found.append(obstacle_data)
        
        # Update obstacle lists
        if not found_gate:
            # No gate detected: do not reset values, just rely on timeout in is_gate_visible
            pass
        if not found_flare:
            # No flare detected: clear after timeout
            if self.last_flare_detection_time and (time.time() - self.last_flare_detection_time) > 2.0:
                self.flare_detected = False
        if obstacles_found:
            self.obstacles = obstacles_found
            self.last_obstacle_detection_time = time.time()
        elif self.last_obstacle_detection_time and (time.time() - self.last_obstacle_detection_time) > 2.0:
            self.obstacles = []
    
    def __init__(self):
        super().__init__('gate_navigator')
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.vision_sub = self.create_subscription(BoundingBoxArray, '/main_camera/detection/bounding_boxes', self.vision_callback, 10)

        # Gate detection
        self.gate_x = 0.5
        self.gate_y = 0.5
        self.gate_w = 0.0
        self.gate_h = 0.0
        self.last_seen = None
        self.last_gate_detection_time = None
        self.gate_detection_count = 0
        
        # Track previous position for stability check
        self.prev_gate_x = 0.5
        self.prev_gate_y = 0.5
        
        # Obstacle detection (flare and others)
        self.flare_x = None
        self.flare_y = None
        self.flare_w = 0.0
        self.flare_h = 0.0
        self.flare_detected = False
        self.last_flare_detection_time = None
        
        # General obstacle detection
        self.obstacles = []  # List of detected obstacles
        self.last_obstacle_detection_time = None

        # Simple state machine: 0=init, 1=search, 2=approach_align, 3=go_through, 4=done
        self.mode = 0
        self.guided_set = False

        # Aspect ratio learning
        self.TARGET_AR = 0.58
        
        # Centering targets (compensate for buoyancy)
        self.TARGET_X = 0.5  # Center horizontally
        self.TARGET_Y = 0.45  # Target lower to compensate for upward buoyancy drift

        # Movement speeds
        self.SEARCH_YAW = 0.22  # Reduced from 0.3 for much slower rotation
        self.APPROACH_SPEED = 0.4  # Reduced from 0.7 - prevent overshooting
        self.FINAL_SPEED = 0.8  # Reduced from 1.0 - safer go-through

        # Control gains
        self.YAW_GAIN = 1.0  # Reduced from 1.5
        self.Z_GAIN = 0.6  # Reduced for smoother vertical control

        # Search state
        self.search_start = time.time()

        # Backup state (when too close)
        self.backup_start = None
        self.backup_duration = 2.0  # Back up for 2 seconds
        self.backup_recovery = False  # Flag: just backed up, need AR correction before re-approach
        
        # AR correction timing - ensure we correct long enough before go-through
        self.ar_correction_start_time = None
        self.ar_min_duration = 1.5  # REDUCED from 3.0 to 1.5 seconds - faster transition to Phase 6
        
        # AR orbit cycle management
        self.orbit_cycle_start = None
        self.orbit_cycle_duration = 3.0  # Orbit for 3 seconds, then center for 1 second
        self.centering_cycle_duration = 1.0
        self.in_centering_cycle = False
        
        # AR direction tracking for adaptive orbiting
        self.ar_at_cycle_start = None
        self.orbit_direction_multiplier = 1.0  # 1.0 = normal direction, -1.0 = reversed
        self.orbit_cycles_completed = 0

        # Centering timeout tracking
        self.centering_start_time = None
        self.centering_timeout = 3.0  # Phase 4 timeout - then force Phase 5
        self.close_enough_tolerance = 0.12  # Accept if within ±0.12 after timeout
        self.centering_forced_accept = False  # Flag: lock in 'centered' after timeout accept
        
        # Phase 5 timeout - prevent getting stuck in AR correction
        self.ar_phase_timeout = 60.0  # Max 60 seconds (1 minute) in Phase 5, then force go-through
        
        # Movement monitoring for Phase 5 - detect if stuck
        self.last_gate_position = None
        self.movement_check_start = None
        self.movement_threshold = 0.02  # Consider stuck if moving less than this
        self.stuck_timeout = 10.0  # Force go-through if stuck for 10 seconds
        
        # Obstacle avoidance state
        self.avoiding_obstacle = False
        self.avoidance_phase = 0  # 0=detect, 1=move_aside, 2=move_forward, 3=return
        self.avoidance_direction = 1.0  # 1.0=right, -1.0=left
        self.avoidance_start_time = None
        self.original_x_position = 0.5  # Remember x position before avoidance
        self.current_blocking_obstacle = None  # Track which obstacle we're currently avoiding

        # Start periodic update timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('GateNavigator node started and timer created.')

    def set_guided(self):
        """Set GUIDED mode via external script"""
        try:
            result = subprocess.run(
                ['ros2', 'run', 'gate_navigator', 'mode_setter'],
                capture_output=True, text=True, timeout=5.0
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def cmd(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.z = float(yaw)
        self.vel_pub.publish(msg)
    
    def stop(self):
        """Stop all movement"""
        self.cmd(0.0, 0.0, 0.0, 0.0)
    
    def correct_position_based_on_gate(self, rough=False):
        """Adjust position based on gate location in frame
        Returns: (yaw_correction, z_correction)
            - yaw_correction: goes to angular.z (yaw left/right)
            - z_correction: goes to linear.z (up/down)
        """
        x, y = self.gate_x, self.gate_y

        if rough:
            deadband = self.ROUGH_DEADBAND
            yaw_gain = self.ROUGH_STRAFE_GAIN
            z_gain = self.ROUGH_Z_GAIN
            yaw_clamp = self.ROUGH_STRAFE_CLAMP
            z_clamp = self.ROUGH_Z_CLAMP
        else:
            deadband = 0.015
            yaw_gain = 3.0  # Increased for stronger precise yaw
            z_gain = 5.0  # DRAMATICALLY increased for stronger precise correction
            yaw_clamp = 0.5
            z_clamp = 0.8  # Much higher clamp for precise z

        x_error = x - self.TARGET_X
        y_error = y - self.TARGET_Y

        # Horizontal centering (yaw)
        if abs(x_error) < deadband:
            yaw_correction = 0.0
        else:
            yaw_correction = -yaw_gain * x_error
        yaw_correction = max(min(yaw_correction, yaw_clamp), -yaw_clamp)

        # Vertical centering (z):
        if abs(y_error) < deadband:
            z_correction = 0.0
        else:
            z_correction = -z_gain * y_error
        z_correction = max(min(z_correction, z_clamp), -z_clamp)

        self.get_logger().info(f'  └─ RAW: x_err={x_error:.4f} y_err={y_error:.4f} deadband={deadband:.3f}', throttle_duration_sec=0.5)
        self.get_logger().info(f'  └─ CALC: yaw={yaw_correction:.4f} z={z_correction:.4f}', throttle_duration_sec=0.5)

        return yaw_correction, z_correction
    
    def correct_aspect_ratio(self, gentle_mode=False, direction_multiplier=1.0):
        """
        Correct aspect ratio via actual orbiting around the gate.
        Returns: (strafe_y, orbit_yaw, forward_x) for true orbital movement
        
        Args:
            direction_multiplier: 1.0 for normal direction, -1.0 for reversed direction
        
        Logic:
        - If AR too low (too tall): orbit LEFT (positive strafe_y) to see gate from side
        - If AR too high (too wide): orbit RIGHT (negative strafe_y) to see gate more straight-on
        - Direction can be reversed if AR is getting worse
        - Maintain consistent orbit radius to prevent decentering
        - Add distance compensation to maintain gate size
        """
        current_ar = self.get_aspect_ratio()
        ar_error = current_ar - self.TARGET_AR
        
        # If AR is close enough and NOT in gentle mode, no correction needed
        if abs(ar_error) < 0.05 and not gentle_mode:  # Tighter tolerance as requested
            return 0.0, 0.0, 0.0  # No correction

        # ORBITAL MOVEMENT: Move sideways to change viewing angle
        # If current_ar < target (too tall): need to see gate from side -> strafe LEFT (positive y)
        # If current_ar > target (too wide): need to see gate straight-on -> strafe RIGHT (negative y)
        
        orbit_gain = 0.8  # Reduced gain for more controlled orbit
        strafe_y = orbit_gain * ar_error * direction_multiplier  # Apply direction multiplier
        
        # Clamp strafe speed
        strafe_y = max(min(strafe_y, 0.3), -0.3)  # Reduced max strafe for better control
        
        # Ensure minimum strafe speed to actually move
        if abs(strafe_y) < 0.12:
            if strafe_y > 0:
                strafe_y = 0.12  # Minimum left strafe
            elif strafe_y < 0:
                strafe_y = -0.12  # Minimum right strafe
        
        # RADIUS CONTROL: Compensate for orbital motion to maintain gate centering
        # When strafing, we need to add forward/backward movement to maintain circular orbit
        x_error = self.gate_x - self.TARGET_X
        
        # Forward compensation: when strafing, move slightly forward to maintain orbit radius
        if abs(strafe_y) > 0.1:
            radius_compensation = 0.08 * abs(strafe_y)  # Move forward proportional to strafe speed
        else:
            radius_compensation = 0.0
        
        # Distance control: maintain optimal gate size
        gate_size = max(self.gate_w, self.gate_h)
        if gate_size < 0.6:
            distance_adjust = 0.05  # Gentle approach
        elif gate_size > 0.8:
            distance_adjust = -0.05  # Gentle backup
        else:
            distance_adjust = 0.0
        
        forward_x = radius_compensation + distance_adjust
        forward_x = max(min(forward_x, 0.15), -0.15)  # Clamp forward speed
        
        # Small yaw to keep gate centered (but don't interfere with orbit too much)
        orbit_yaw = -0.2 * x_error  # Small yaw to track gate during orbit
        orbit_yaw = max(min(orbit_yaw, 0.15), -0.15)  # Small yaw limits
        
        return strafe_y, orbit_yaw, forward_x
    
    def should_use_orbital_avoidance(self):
        """Check if gate is large enough to warrant orbital avoidance instead of lateral"""
        return (self.gate_w > 0.5 or self.gate_h > 0.5)
    
    def is_obstacle_blocking(self):
        """Check if any obstacle is detected (any non-gate object with w/h > 0.6 anywhere on screen)"""
        # Check flare (any size > 0.6)
        if self.flare_detected and self.flare_h > 0:
            is_large = self.flare_w > 0.6 or self.flare_h > 0.6
            if is_large:
                return True, 'flare', {'x': self.flare_x, 'y': self.flare_y, 'w': self.flare_w, 'h': self.flare_h}
        
        # Check other obstacles (any size > 0.6)
        for obstacle in self.obstacles:
            is_large = obstacle['w'] > 0.6 or obstacle['h'] > 0.6
            if is_large:
                return True, 'obstacle', obstacle
        
        return False, None, None
    
    def get_avoidance_direction(self, obstacle_data):
        """Determine which direction to avoid the obstacle - move to opposite side"""
        if not obstacle_data:
            return 1.0  # Default right
        
        obstacle_x = obstacle_data['x']
        
        # Move to opposite side of screen from obstacle
        if obstacle_x < 0.5:
            return 1.0  # Obstacle on left side, move right
        else:
            return -1.0  # Obstacle on right side, move left
    
    def get_orbital_avoidance_direction(self, obstacle_data):
        """Determine orbital direction based on obstacle position - move OPPOSITE to obstacle"""
        if not obstacle_data:
            return 1.0  # Default right
        
        obstacle_x = obstacle_data['x']
        
        # Check if obstacle is centered (within ±0.1 of center)
        if abs(obstacle_x - 0.5) < 0.1:
            # Obstacle is centered - choose a side (prefer right)
            return 1.0
        
        # Move in direction OPPOSITE to obstacle position
        if obstacle_x < 0.5:
            return 1.0  # Obstacle on left, orbit right
        else:
            return -1.0  # Obstacle on right, orbit left
    
    def execute_orbital_avoidance(self):
        """Execute 180-degree orbital avoidance to completely avoid obstacle"""
        if not self.avoiding_obstacle:
            return False
        
        current_time = time.time()
        
        # Initialize orbital avoidance state
        if not hasattr(self, 'orbital_start_time'):
            self.orbital_start_time = current_time
            self.orbital_direction = self.get_orbital_avoidance_direction(self.current_blocking_obstacle)
            # Calculate time needed for 180-degree orbit (approximately 6-8 seconds for half orbit)
            self.orbital_duration = 7.0  # seconds for 180-degree orbit
            self.get_logger().info(f'🌀 ORBITAL AVOIDANCE: Starting 180-degree orbit to avoid obstacle')
        
        # Check if obstacle is centered - if so, strafe to side first
        obstacle_x = self.current_blocking_obstacle.get('x', 0.5)
        is_centered = abs(obstacle_x - 0.5) < 0.1
        
        if is_centered and not hasattr(self, 'orbital_side_step_complete'):
            # Phase 1: Move to side first for centered obstacles
            strafe_speed = 0.3  # Move to get off center
            
            # Check if we've moved enough to one side (1.5 seconds)
            if (current_time - self.orbital_start_time) > 1.5:
                self.orbital_side_step_complete = True
                self.orbital_start_time = current_time  # Reset timer for actual orbit
                self.get_logger().info('🌀 ORBITAL AVOIDANCE: Side step complete, starting 180-degree orbital motion')
            
            self.get_logger().info('🌀 ORBITAL AVOIDANCE: Moving to side (centered obstacle)', throttle_duration_sec=1.0)
            self.cmd(x=0.0, y=strafe_speed, z=0.0, yaw=0.0)
            return True
        
        # Phase 2: Execute 180-degree orbital motion
        elapsed_time = current_time - self.orbital_start_time
        
        # Continue orbital motion for the full duration regardless of obstacle visibility
        if elapsed_time < self.orbital_duration:
            # Use orbital motion in the determined direction
            strafe_y, orbit_yaw, forward_x = self.correct_aspect_ratio(direction_multiplier=self.orbital_direction)
            
            # Enhanced orbital motion for complete obstacle avoidance
            strafe_y *= 2.0  # Much stronger strafe for 180-degree orbit
            forward_x *= 1.5  # Enhanced forward movement to maintain orbit
            
            # Vertical stabilization (only if gate is still visible)
            if self.is_gate_visible():
                y_error = self.gate_y - self.TARGET_Y
                z_stabilization = -3.0 * y_error
                z_stabilization = max(min(z_stabilization, 0.6), -0.6)
            else:
                z_stabilization = 0.0  # No vertical correction if no gate
            
            direction_str = "LEFT" if self.orbital_direction < 0 else "RIGHT"
            progress = (elapsed_time / self.orbital_duration) * 180  # Progress in degrees
            self.get_logger().info(f'🌀 ORBITAL AVOIDANCE: 180° orbit {direction_str} - {progress:.0f}°/{180}° complete', throttle_duration_sec=1.0)
            self.cmd(x=forward_x, y=strafe_y, z=z_stabilization, yaw=orbit_yaw)
            return True
        
        # Phase 3: Orbital avoidance complete
        self.avoiding_obstacle = False
        self.avoidance_phase = 0
        
        # Clean up orbital avoidance state
        if hasattr(self, 'orbital_side_step_complete'):
            delattr(self, 'orbital_side_step_complete')
        if hasattr(self, 'orbital_start_time'):
            delattr(self, 'orbital_start_time')
        if hasattr(self, 'orbital_direction'):
            delattr(self, 'orbital_direction')
        if hasattr(self, 'orbital_duration'):
            delattr(self, 'orbital_duration')
        
        self.get_logger().info('✅ ORBITAL AVOIDANCE: 180-degree orbit complete - obstacle fully avoided')
        return False
    
    def execute_simple_avoidance(self):
        """Execute simplified avoidance: strafe until obstacle is off-screen, then move forward lightly"""
        if not self.avoiding_obstacle:
            return False
        
        current_time = time.time()
        
        if self.avoidance_phase == 1:  # STRAFE UNTIL OBSTACLE IS OFF-SCREEN
            strafe_speed = 0.3 * self.avoidance_direction  # Reduced strafe speed
            
            # Check if the SPECIFIC obstacle we're avoiding is off-screen
            obstacle_off_screen = True
            
            # Check if our current blocking obstacle is still on screen
            if self.current_blocking_obstacle:
                # Check flare specifically
                if self.flare_detected:
                    # Flare is still on screen if detected and large
                    if self.flare_w > 0.6 or self.flare_h > 0.6:
                        obstacle_off_screen = False
                
                # Check other obstacles - see if any large obstacle is still on screen
                for obstacle in self.obstacles:
                    if obstacle['w'] > 0.6 or obstacle['h'] > 0.6:
                        obstacle_off_screen = False
                        break
            
            # Move until obstacle is off-screen or timeout after 4 seconds
            if obstacle_off_screen or (current_time - self.avoidance_start_time) > 4.0:
                self.avoidance_phase = 2
                self.avoidance_start_time = current_time
                self.get_logger().info('🚁 LIGHT AVOIDANCE: Phase 2 - Obstacle off-screen, moving forward lightly')
            
            self.get_logger().info(f'🚁 LIGHT AVOIDANCE: Phase 1 - Strafing until obstacle off-screen (off_screen={obstacle_off_screen})', throttle_duration_sec=1.0)
            self.cmd(x=0.0, y=strafe_speed, z=0.0, yaw=0.0)
            return True
            
        elif self.avoidance_phase == 2:  # MOVE FORWARD LIGHTLY
            forward_speed = 0.4  # Very light forward movement
            
            # Move forward for only 1.5 seconds (very brief)
            if (current_time - self.avoidance_start_time) > 1.5:
                # Avoidance complete - just continue with normal navigation
                self.avoiding_obstacle = False
                self.avoidance_phase = 0
                self.get_logger().info('✅ LIGHT AVOIDANCE: Complete - continuing with normal navigation')
                return False
            
            self.get_logger().info(f'🚁 LIGHT AVOIDANCE: Phase 2 - Moving forward lightly', throttle_duration_sec=1.0)
            self.cmd(x=forward_speed, y=0.0, z=0.0, yaw=0.0)
            return True
        
        return False
    
    def is_gate_centered(self, tol=0.08):
        """Check if gate is centered enough in the frame"""
        # Relaxed tolerance for Y centering
        return abs(self.gate_x - self.TARGET_X) < tol and abs(self.gate_y - self.TARGET_Y) < 0.15

    def is_gate_too_close(self, size_thresh=0.7):
        """Check if gate is very close (to avoid knocking over)"""
        return max(self.gate_w, self.gate_h) > size_thresh

    def is_gate_at_edge(self, edge_thresh=0.25):
        """Check if gate is at the edge of the frame (stricter to ensure centering)"""
        return (
            self.gate_x < edge_thresh or self.gate_x > 1.0 - edge_thresh or
            self.gate_y < edge_thresh or self.gate_y > 1.0 - edge_thresh
        )
    
    def is_gate_filling_frame(self):
        """Check if gate is filling the entire frame (danger - too close or collision)"""
        return self.gate_w >= 0.75 or self.gate_h >= 0.75

    def update(self):
        """Main update loop"""
        # MODE 0: INIT - Set GUIDED mode
        if self.mode == 0:
            if not self.guided_set:
                self.get_logger().info('Setting GUIDED mode...')
                if self.set_guided():
                    self.guided_set = True
                    self.mode = 1
                    self.search_start = time.time()
                    self.get_logger().info('Starting gate search...')
                time.sleep(0.5)
            return

        # MODE 1: SEARCH - Rotate until gate found (with obstacle avoidance)
        if self.mode == 1:
            # Check for obstacles during search
            is_blocking, obstacle_type, obstacle_data = self.is_obstacle_blocking()
            if is_blocking and not self.avoiding_obstacle:
                self.avoiding_obstacle = True
                self.current_blocking_obstacle = obstacle_data
                
                # Use simple avoidance for all cases
                self.avoidance_phase = 1  # Simple avoidance
                self.avoidance_direction = self.get_avoidance_direction(obstacle_data)
                self.avoidance_start_time = time.time()
                direction_str = "LEFT" if self.avoidance_direction < 0 else "RIGHT"
                self.get_logger().warn(f'🚁 SEARCH: {obstacle_type.upper()} OBSTACLE DETECTED! Moving to {direction_str} side')
            
            # Execute obstacle avoidance if active
            if self.avoiding_obstacle:
                if self.execute_simple_avoidance():
                    return  # Continue avoidance, don't do normal search
            
            if self.is_gate_visible():
                self.cmd(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info('✓ Gate found! Velocity set to 0. Starting approach with alignment...')
                self.mode = 2
                return

            self.cmd(yaw=self.SEARCH_YAW)

            if time.time() - self.search_start > 10.0:
                if int(time.time()) % 5 == 0:
                    self.cmd(x=0.3)
                self.search_start = time.time()
            return

        # MODE 2: APPROACH, ROUGH CENTER, STRAIGHT APPROACH, CENTER, AR CORRECT, GO THROUGH
        if self.mode == 2:
            # Check for actual obstacles (not the gate itself)
            is_blocking, obstacle_type, obstacle_data = self.is_obstacle_blocking()
            if is_blocking and not self.avoiding_obstacle:
                self.avoiding_obstacle = True
                self.current_blocking_obstacle = obstacle_data
                self.original_x_position = self.gate_x  # Remember where we were
                
                # Use simple avoidance for all cases
                self.avoidance_phase = 1
                self.avoidance_direction = self.get_avoidance_direction(obstacle_data)
                self.avoidance_start_time = time.time()
                direction_str = "LEFT" if self.avoidance_direction < 0 else "RIGHT"
                self.get_logger().warn(f'🚁 APPROACH: {obstacle_type.upper()} OBSTACLE DETECTED! Moving to {direction_str} side')
            
            # Execute obstacle avoidance if active
            if self.avoiding_obstacle:
                if self.execute_simple_avoidance():
                    return  # Continue avoidance, pause normal navigation
            
            if not self.is_gate_visible():
                self.get_logger().warn('Lost gate! Back to search')
                self.mode = 1
                self.search_start = time.time()
                self.centering_forced_accept = False
                self.ar_correction_start_time = None
                self.rough_centered = False  # Reset rough centering flag
                return

            gate_w = self.gate_w
            gate_h = self.gate_h
            current_ar = self.get_aspect_ratio()
            err_x = self.gate_x - 0.5
            err_y = self.gate_y - 0.5
            x_centered = abs(self.gate_x - self.TARGET_X) < 0.05
            y_centered = abs(self.gate_y - self.TARGET_Y) < 0.12
            centered = (x_centered and y_centered) or self.centering_forced_accept

            # --- ROUGH CENTERING AT START ---
            if not hasattr(self, 'rough_centered'):
                self.rough_centered = False
            if not self.rough_centered and (gate_w < 0.6 and gate_h < 0.6):
                rough_tol = 0.05
                x_error = self.gate_x - self.TARGET_X
                y_error = self.gate_y - self.TARGET_Y
                x_in_tol = abs(x_error) < rough_tol
                y_in_tol = abs(y_error) < rough_tol
                if x_in_tol and y_in_tol:
                    self.rough_centered = True
                else:
                    yaw_corr, z_corr = 0.0, 0.0
                    if not x_in_tol or not y_in_tol:
                        yaw_c, z_c = self.correct_position_based_on_gate(rough=True)
                        if not x_in_tol:
                            yaw_corr = yaw_c
                        if not y_in_tol:
                            z_corr = z_c
                    self.get_logger().info(f'STEP 2 - ROUGH CENTERING: x={self.gate_x:.4f} y={self.gate_y:.4f} w={gate_w:.2f} h={gate_h:.2f}', throttle_duration_sec=0.5)
                    self.cmd(x=0.0, y=0.0, z=z_corr, yaw=yaw_corr)
                    return
            '''
            # Gate backup logic - re-enabled
            if gate_w > 0.85 or gate_h > 0.85:
                self.get_logger().warn(f'⚠️  TOO CLOSE! w={gate_w:.2f}, h={gate_h:.2f} - GENTLE BACKUP')
                # Conservative backup - just a brief pulse
                backup_count = 0
                while self.is_gate_visible() and (self.gate_w > 0.75 or self.gate_h > 0.75) and backup_count < 3:
                    self.cmd(x=-0.15, y=0.0, z=0.0, yaw=0.0)  # Very gentle backup speed
                    time.sleep(0.1)
                    backup_count += 1
                self.cmd(x=0.0, y=0.0, z=0.0, yaw=0.0)
                self.get_logger().info(f'✓ Gentle backup complete: w={self.gate_w:.2f}, h={self.gate_h:.2f} (backed {backup_count} steps)')
                time.sleep(0.1)  # Very short pause
                return
            '''

            if gate_w < 0.6 and gate_h < 0.6:
                forward_speed = 0.3
                
                # Vertical stabilization to maintain y position during approach
                y_error = self.gate_y - self.TARGET_Y
                z_stabilization = -2.0 * y_error  # Gentler during approach
                z_stabilization = max(min(z_stabilization, 0.4), -0.4)
                
                self.get_logger().info(f'STEP 3 - APPROACH: x={self.gate_x:.4f} y={self.gate_y:.4f} w={gate_w:.2f} h={gate_h:.2f} z_stab={z_stabilization:.3f}', throttle_duration_sec=0.5)
                self.cmd(x=forward_speed, y=0.0, z=z_stabilization, yaw=0.0)
                return

            # AR CORRECTION PHASE - CONTINUOUS ORBIT (once started, NEVER go back to centering)
            if self.ar_correction_start_time is None:
                # First time entering AR phase - check if we need centering first
                precise_tol = 0.05
                precisely_centered = abs(self.gate_x - self.TARGET_X) < precise_tol and abs(self.gate_y - self.TARGET_Y) < precise_tol
                
                if not precisely_centered:
                    if self.centering_start_time is None:
                        self.centering_start_time = time.time()
                    centering_duration = time.time() - self.centering_start_time
                    yaw_corr, z_corr = self.correct_position_based_on_gate()
                    self.get_logger().info(f'STEP 4 - PRECISE CENTERING: x={self.gate_x:.4f} y={self.gate_y:.4f} w={gate_w:.2f} h={gate_h:.2f} ({centering_duration:.1f}s)', throttle_duration_sec=0.5)
                    self.cmd(x=0.0, y=0.0, z=z_corr, yaw=yaw_corr)
                    return
                else:
                    # Centered! Start AR correction phase
                    self.ar_correction_start_time = time.time()
                    self.centering_start_time = None
                    self.centering_forced_accept = False
                    self.get_logger().info('🔄 Starting AR correction phase - CONTINUOUS ORBIT to correct aspect ratio...')
            
            # INSIDE AR CORRECTION PHASE - CYCLE between orbit and centering
            ar_correction_duration = time.time() - self.ar_correction_start_time
            
            # Movement monitoring - check if stuck (not moving much)
            current_position = (self.gate_x, self.gate_y)
            if self.last_gate_position is None:
                self.last_gate_position = current_position
                self.movement_check_start = time.time()
            else:
                # Calculate movement since last check
                movement = abs(current_position[0] - self.last_gate_position[0]) + abs(current_position[1] - self.last_gate_position[1])
                
                if movement < self.movement_threshold:
                    # Very little movement - check if stuck for too long
                    if self.movement_check_start is not None and (time.time() - self.movement_check_start) > self.stuck_timeout:
                        self.get_logger().warn(f'⏱️  STUCK DETECTED! Very little movement for {self.stuck_timeout}s - FORCING GO-THROUGH!')
                        self.get_logger().info(f'🎯 STUCK GO-THROUGH! x={self.gate_x:.2f} y={self.gate_y:.2f} AR={current_ar:.2f}')
                        self.ar_correction_start_time = None
                        self.orbit_cycle_start = None
                        self.in_centering_cycle = False
                        self.ar_at_cycle_start = None
                        self.orbit_direction_multiplier = 1.0
                        self.orbit_cycles_completed = 0
                        self.last_gate_position = None
                        self.movement_check_start = None
                        self.cmd(x=self.FINAL_SPEED, y=0.0, z=0.0, yaw=0.0)
                        self.mode = 3
                        return
                else:
                    # Significant movement detected - reset timer
                    self.movement_check_start = time.time()
                
                self.last_gate_position = current_position
            
            # Initialize orbit cycle timing
            if self.orbit_cycle_start is None:
                self.orbit_cycle_start = time.time()
                self.in_centering_cycle = False
            
            cycle_time = time.time() - self.orbit_cycle_start
            
            # CYCLE LOGIC: Alternate between orbiting and centering
            if not self.in_centering_cycle:
                # ORBIT PHASE: Focus on AR correction
                
                # Track AR at start of orbit cycle for direction evaluation
                if self.ar_at_cycle_start is None:
                    self.ar_at_cycle_start = current_ar
                
                if cycle_time > self.orbit_cycle_duration:
                    # End of orbit cycle - evaluate AR progress
                    ar_improvement = abs(current_ar - self.TARGET_AR) - abs(self.ar_at_cycle_start - self.TARGET_AR)
                    
                    if ar_improvement > 0.02:  # AR got worse by significant amount
                        self.orbit_direction_multiplier *= -1.0  # Reverse direction
                        self.get_logger().info(f'🔄 AR got worse ({self.ar_at_cycle_start:.3f}→{current_ar:.3f}), REVERSING orbit direction (mult: {self.orbit_direction_multiplier:.1f})')
                    elif ar_improvement < -0.01:  # AR improved
                        self.get_logger().info(f'✅ AR improved ({self.ar_at_cycle_start:.3f}→{current_ar:.3f}), keeping orbit direction (mult: {self.orbit_direction_multiplier:.1f})')
                    
                    self.orbit_cycles_completed += 1
                    self.in_centering_cycle = True
                    self.orbit_cycle_start = time.time()
                    self.ar_at_cycle_start = None  # Reset for next cycle
                    self.get_logger().info('🔄 Switching to CENTERING cycle...')
                
                # Get orbital movement commands with current direction
                strafe_y, orbit_yaw, forward_x = self.correct_aspect_ratio(direction_multiplier=self.orbit_direction_multiplier)
                
                # Vertical stabilization to counteract buoyancy
                y_error = self.gate_y - self.TARGET_Y
                z_stabilization = -3.0 * y_error  # Proportional control to keep y centered
                z_stabilization = max(min(z_stabilization, 0.6), -0.6)  # Clamp
                
                direction_str = "LEFT" if self.orbit_direction_multiplier > 0 else "RIGHT"
                self.get_logger().info(f'STEP 5A - AR ORBIT ({direction_str}): x={self.gate_x:.2f} y={self.gate_y:.2f} AR={current_ar:.2f} strafe={strafe_y:.3f} yaw={orbit_yaw:.3f} fwd={forward_x:.3f} [Cycle: {self.orbit_cycles_completed}]', throttle_duration_sec=1.0)
                
                # Send FULL orbital command
                self.cmd(x=forward_x, y=strafe_y, z=z_stabilization, yaw=orbit_yaw)
                
            else:
                # CENTERING PHASE: Focus on position correction
                if cycle_time > self.centering_cycle_duration:
                    self.in_centering_cycle = False
                    self.orbit_cycle_start = time.time()
                    self.get_logger().info('🔄 Switching to ORBIT cycle...')
                
                # Apply precise centering corrections
                yaw_corr, z_corr = self.correct_position_based_on_gate()
                
                # Additional vertical stabilization for buoyancy
                y_error = self.gate_y - self.TARGET_Y
                z_total = z_corr + (-2.0 * y_error)  # Combine centering + buoyancy compensation
                z_total = max(min(z_total, 0.8), -0.8)  # Clamp
                
                self.get_logger().info(f'STEP 5B - AR CENTER: x={self.gate_x:.2f} y={self.gate_y:.2f} AR={current_ar:.2f} yaw={yaw_corr:.3f} z={z_total:.3f} [Duration: {ar_correction_duration:.1f}s]', throttle_duration_sec=1.0)
                
                # Send centering command (no strafe during centering)
                self.cmd(x=0.0, y=0.0, z=z_total, yaw=yaw_corr)
            
            # Check completion (no timeout - continue until AR is correct)
            if abs(current_ar - self.TARGET_AR) < 0.03 and ar_correction_duration > self.ar_min_duration:  # Tighter tolerance ±0.03
                self.get_logger().info(f'🎯 AR CORRECTION COMPLETE! Proceeding to go-through. x={self.gate_x:.2f} y={self.gate_y:.2f} AR={current_ar:.2f}')
                self.ar_correction_start_time = None
                self.orbit_cycle_start = None
                self.in_centering_cycle = False
                self.ar_at_cycle_start = None
                self.orbit_direction_multiplier = 1.0
                self.orbit_cycles_completed = 0
                # Clean up movement monitoring
                self.last_gate_position = None
                self.movement_check_start = None
                self.cmd(x=self.FINAL_SPEED, y=0.0, z=0.0, yaw=0.0)
                self.mode = 3
                return
            return
        
        # MODE 3: GO THROUGH - Always zoom through, but with obstacle avoidance
        if self.mode == 3:
            # Check for obstacles even during go-through phase
            is_blocking, obstacle_type, obstacle_data = self.is_obstacle_blocking()
            if is_blocking and not self.avoiding_obstacle:
                self.avoiding_obstacle = True
                self.current_blocking_obstacle = obstacle_data
                self.original_x_position = self.gate_x if self.is_gate_visible() else 0.5  # Remember where we were
                
                # Use simple avoidance for all cases
                self.avoidance_phase = 1
                self.avoidance_direction = self.get_avoidance_direction(obstacle_data)
                self.avoidance_start_time = time.time()
                direction_str = "LEFT" if self.avoidance_direction < 0 else "RIGHT"
                self.get_logger().warn(f'GO-THROUGH: {obstacle_type.upper()} OBSTACLE DETECTED! Moving to {direction_str} side')
            
            # Execute obstacle avoidance if active
            if self.avoiding_obstacle:
                if self.execute_simple_avoidance():
                    return  # Continue avoidance, pause go-through
            
            # Normal go-through behavior when no obstacles
            self.cmd(x=self.FINAL_SPEED, y=0.0, z=0.0, yaw=0.0)
            return
        
        # MODE 4: DONE - Stop but still check for obstacles for safety
        if self.mode == 4:
            # Check for obstacles even when supposed to be done
            is_blocking, obstacle_type, obstacle_data = self.is_obstacle_blocking()
            if is_blocking and not self.avoiding_obstacle:
                self.avoiding_obstacle = True
                self.current_blocking_obstacle = obstacle_data
                self.original_x_position = 0.5  # Default center
                
                # Always use lateral avoidance when done (no gate likely visible)
                self.avoidance_phase = 1
                self.avoidance_direction = self.get_avoidance_direction(obstacle_data)
                self.avoidance_start_time = time.time()
                direction_str = "LEFT" if self.avoidance_direction < 0 else "RIGHT"
                self.get_logger().warn(f'🚁 DONE-SAFETY: {obstacle_type.upper()} OBSTACLE DETECTED! Moving {direction_str} for safety')
            
            # Execute obstacle avoidance if active
            if self.avoiding_obstacle:
                if self.execute_simple_avoidance():
                    return  # Continue avoidance, don't stop
            
            # Normal done behavior - stop
            self.stop()
            try:
                self.timer.cancel()
            except Exception:
                pass
            self.get_logger().info('Navigation complete. Node stopped.')
            return

def main(args=None):
    rclpy.init(args=args)
    nav = GateNavigator()
    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        pass
    finally:
        nav.stop()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()