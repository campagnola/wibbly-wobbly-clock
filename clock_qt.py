import sys
import time
import math
import importlib
for qt_lib in ('PyQt5', 'PySide2', 'PyQt6', 'PySide6', None):
    if qt_lib is None:
        raise ImportError("No suitable Qt library found.")
    try:
        QtWidgets = importlib.import_module(qt_lib + '.QtWidgets')
        QtCore = importlib.import_module(qt_lib + '.QtCore')
        QtGui = importlib.import_module(qt_lib + '.QtGui')
        break
    except ImportError:
        pass


class PIDController:
    """Generic PID controller implementation with exponential integral averaging."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None, integral_time_constant=1.0):
        """Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output limits, or None for unlimited
            integral_time_constant: Time constant for exponential integral decay (seconds)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_time_constant = integral_time_constant

        self.integral = 0.0
        self.last_error = 0.0
        self.last_terms = (0.0, 0.0, 0.0)  # (P, I, D)
        self.last_output = 0.0

    def update(self, setpoint, measured_value, dt):
        """Update the PID controller.

        Args:
            setpoint: Desired value
            measured_value: Current measured value
            dt: Time step in seconds

        Returns:
            Control output
        """
        if dt <= 0:
            return 0.0

        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term with exponential decay
        # Decay old integral and add new error contribution
        decay = math.exp(-dt / self.integral_time_constant)
        self.integral = self.integral * decay + error * dt
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error

        # Calculate output
        output = p_term + i_term + d_term
        self.last_terms = (p_term, i_term, d_term)
        self.last_output = output

        # Apply output limits
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))

        return output

    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.last_error = 0.0

    def state_str(self):
        return (
            f"KP={self.kp:0.1f}, KI={self.ki:0.1f}, KD={self.kd:0.1f}, Integral={self.integral:0.1f}, "
            f"error={self.last_error:0.1f}, output={self.last_output:0.1f}, "
            f"terms(P,I,D)={self.last_terms[0]:0.1f},{self.last_terms[1]:0.1f},{self.last_terms[2]:0.1f}"
        )


class Hand:
    """Represents an animated clock hand with PID-based velocity and acceleration."""

    def __init__(self, pixmap_item, max_speed=720.0, max_acceleration=1000.0):
        """Initialize a hand.

        Args:
            pixmap_item: QGraphicsPixmapItem for this hand
            max_speed: Maximum angular velocity in degrees/s (default: 360)
            max_acceleration: Maximum angular acceleration in degrees/s^2 (default: 1000)
        """
        self.item = pixmap_item
        self.angle = 0.0  # Current angle in degrees (0 = 12 o'clock, clockwise)
        self.velocity = 0.0  # Current angular velocity in degrees/s
        self.acceleration = 0.0  # Current angular acceleration in degrees/s^2
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration

        # Target motion tracking
        self.motion_mode = 'speed'  # 'speed' or 'target'
        self.target_angle = None
        self.target_velocity = 0.0

        # Inner PID: controls acceleration to achieve target velocity
        # Output is acceleration command
        self.velocity_pid = PIDController(
            kp=10.0,
            ki=0.0,
            kd=0.0,
            output_limits=(-max_acceleration, max_acceleration),
            integral_time_constant=0.5
        )

        # Outer PID: controls target velocity to reach target position
        # Output is velocity command
        self.position_pid = PIDController(
            kp=24.0,
            ki=0.0,
            kd=8.0,
            output_limits=(-max_speed, max_speed),
            integral_time_constant=1.0
        )

    def set_angle(self, angle):
        """Set the angle immediately without animation.

        Args:
            angle: Angle in degrees (0 = 12 o'clock, clockwise, unbounded)
        """
        self.angle = angle
        self.velocity = 0.0
        self.target_velocity = 0.0
        self.motion_mode = 'speed'
        self.target_angle = None
        self.velocity_pid.reset()
        self.position_pid.reset()
        self._update_rotation()

    def set_speed(self, speed):
        """Set the target speed.

        Args:
            speed: Target angular velocity in degrees/s
        """
        self.target_velocity = max(-self.max_speed, min(speed, self.max_speed))
        self.motion_mode = 'speed'
        self.target_angle = None
        self.position_pid.reset()

    def set_target(self, angle, speed=None):
        """Move to a target angle.

        Args:
            angle: Target angle in degrees (unbounded - e.g., 3600 = 10 full rotations)
            speed: Maximum cruise speed in degrees/s (optional, uses max_speed if not provided)
        """
        self.motion_mode = 'target'
        self.target_angle = angle

        if speed is not None:
            # Temporarily adjust max speed for this move
            self.position_pid.output_limits = (-abs(speed), abs(speed))
        else:
            self.position_pid.output_limits = (-self.max_speed, self.max_speed)

    def update(self, dt):
        """Update the hand's position based on PID control.

        Args:
            dt: Time step in seconds
        """
        if dt <= 0:
            return

        # Step 1: Update position from previous velocity
        self.angle += self.velocity * dt

        # Step 2: Determine target velocity
        if self.motion_mode == 'target' and self.target_angle is not None:
            # Outer PID: calculate target velocity to reach target angle
            # Direct error calculation (no shortest path - support multi-rotation)
            error = self.target_angle - self.angle

            # Use position PID to determine target velocity
            self.target_velocity = self.position_pid.update(self.target_angle, self.angle, dt)

            # Check if we've reached the target
            # if abs(error) < 0.1 and abs(self.velocity) < 1.0:
            #     self.angle = self.target_angle
            #     self.velocity = 0.0
            #     self.target_velocity = 0.0
            #     self.motion_mode = 'speed'
            #     self.target_angle = None
            #     self.velocity_pid.reset()
            #     self.position_pid.reset()
            #     self._update_rotation()
            #     return

        # Step 3: Inner PID: apply acceleration to achieve target velocity
        self.acceleration = self.velocity_pid.update(self.target_velocity, self.velocity, dt)
        self.velocity += self.acceleration * dt

        # Clamp velocity to max_speed
        self.velocity = max(-self.max_speed, min(self.velocity, self.max_speed))

        self._update_rotation()

    def _update_rotation(self):
        """Update the graphics item rotation."""
        # Hands point right (3 o'clock) in the image, so subtract 90
        self.item.setRotation(self.angle - 90)


class ClockGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Clock")

        # Load clock face and determine window size
        clock_face = QtGui.QPixmap('clock_face.png')
        face_width = clock_face.width()
        face_height = clock_face.height()
        self.face_center = (512, 497)

        # Load and extract hands from clock_hands.png
        hands_image = QtGui.QPixmap('clock_hands.png')
        scale_factor = 0.6

        # Handle Qt5/Qt6 differences for aspect ratio mode
        try:
            aspect_ratio_mode = QtCore.Qt.AspectRatioMode.KeepAspectRatio
            transform_mode = QtCore.Qt.TransformationMode.SmoothTransformation
        except AttributeError:
            aspect_ratio_mode = QtCore.Qt.KeepAspectRatio
            transform_mode = QtCore.Qt.SmoothTransformation

        # Extract hour hand: center at (44, 272), y range 225-320
        hour_hand_orig = hands_image.copy(0, 225, hands_image.width(), 95)
        hour_center_orig = (44, 272 - 225)
        self.hour_hand = hour_hand_orig.scaled(
            int(hour_hand_orig.width() * scale_factor),
            int(hour_hand_orig.height() * scale_factor),
            aspect_ratio_mode,
            transform_mode
        )
        self.hour_center = (int(hour_center_orig[0] * scale_factor), int(hour_center_orig[1] * scale_factor))

        # Extract minute hand: center at (36, 365), y range 330-402
        minute_hand_orig = hands_image.copy(0, 330, hands_image.width(), 72)
        minute_center_orig = (36, 365 - 330)
        self.minute_hand = minute_hand_orig.scaled(
            int(minute_hand_orig.width() * scale_factor),
            int(minute_hand_orig.height() * scale_factor),
            aspect_ratio_mode,
            transform_mode
        )
        self.minute_center = (int(minute_center_orig[0] * scale_factor), int(minute_center_orig[1] * scale_factor))

        # Extract second hand: center at (196, 456), y range 437-472
        second_hand_orig = hands_image.copy(0, 437, hands_image.width(), 35)
        second_center_orig = (196, 456 - 437)
        self.second_hand = second_hand_orig.scaled(
            int(second_hand_orig.width() * scale_factor),
            int(second_hand_orig.height() * scale_factor),
            aspect_ratio_mode,
            transform_mode
        )
        self.second_center = (int(second_center_orig[0] * scale_factor), int(second_center_orig[1] * scale_factor))

        # Create scene and view
        self.scene = QtWidgets.QGraphicsScene()
        self.view = QtWidgets.QGraphicsView(self.scene)
        # Handle Qt5/Qt6 differences for antialiasing
        try:
            self.view.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        except AttributeError:
            self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setFixedSize(face_width, face_height)
        self.view.setSceneRect(0, 0, face_width, face_height)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.view)
        self.setLayout(layout)

        # Add clock face to scene
        self.scene.addPixmap(clock_face)

        # Create pixmap items for hands
        hour_item = self.scene.addPixmap(self.hour_hand)
        minute_item = self.scene.addPixmap(self.minute_hand)
        second_item = self.scene.addPixmap(self.second_hand)

        # Set transform origins (rotation centers) for each hand
        hour_item.setTransformOriginPoint(self.hour_center[0], self.hour_center[1])
        minute_item.setTransformOriginPoint(self.minute_center[0], self.minute_center[1])
        second_item.setTransformOriginPoint(self.second_center[0], self.second_center[1])

        # Position hands so their centers are at the clock face center
        hour_item.setPos(self.face_center[0] - self.hour_center[0],
                         self.face_center[1] - self.hour_center[1])
        minute_item.setPos(self.face_center[0] - self.minute_center[0],
                           self.face_center[1] - self.minute_center[1])
        second_item.setPos(self.face_center[0] - self.second_center[0],
                           self.face_center[1] - self.second_center[1])

        # Create Hand instances
        self.hour = Hand(hour_item)
        self.minute = Hand(minute_item)
        self.second = Hand(second_item)

        # Track time for accurate dt calculation
        self.last_update_time = time.perf_counter()

        # Set up animation timer (60 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_animation)
        self.timer.start(16)  # ~60 FPS

    def _update_animation(self):
        """Called by timer to update hand positions."""
        current_time = time.perf_counter()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        self.hour.update(dt)
        self.minute.update(dt)
        self.second.update(dt)

    def set_hand_angles(self, hour=None, minute=None, second=None):
        """Set the angles of the clock hands immediately.

        Args:
            hour: Angle for hour hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            minute: Angle for minute hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            second: Angle for second hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
        """
        if hour is not None:
            self.hour.set_angle(hour)
        if minute is not None:
            self.minute.set_angle(minute)
        if second is not None:
            self.second.set_angle(second)

    def set_hand_speeds(self, hour=None, minute=None, second=None):
        """Set the angular velocities of the clock hands.

        Args:
            hour: Angular velocity for hour hand in degrees/s
            minute: Angular velocity for minute hand in degrees/s
            second: Angular velocity for second hand in degrees/s
        """
        if hour is not None:
            self.hour.set_speed(hour)
        if minute is not None:
            self.minute.set_speed(minute)
        if second is not None:
            self.second.set_speed(second)

    def set_hand_targets(self, hour=None, minute=None, second=None, speed=None):
        """Move hands to target angles with PID control.

        Args:
            hour: Target angle for hour hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            minute: Target angle for minute hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            second: Target angle for second hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            speed: Maximum cruise speed in degrees/s (optional, uses hand's max_speed if not provided)
        """
        if hour is not None:
            self.hour.set_target(hour, speed)
        if minute is not None:
            self.minute.set_target(minute, speed)
        if second is not None:
            self.second.set_target(second, speed)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    clock = ClockGUI()

    # Example: Set initial angles
    clock.set_hand_angles(hour=0, minute=0, second=0)
    clock.show()

    # Example 1: Continuous rotation at different speeds
    # clock.set_hand_speeds(hour=5, minute=60, second=360)

    # Example 2: Move to target angles with PID control
    # Move all hands to 3 o'clock position (90 degrees)
    # clock.set_hand_targets(hour=90, minute=90, second=90, speed=360)

    # Example 3: Multi-rotation - second hand does 10 full rotations
    # clock.second.set_target(540, speed=720)  # 3600 degrees = 10 rotations

    # clock.second.set_speed(720)
    # QtCore.QTimer.singleShot(3000, lambda: clock.second.set_speed(-720))
    clock.second.set_target(360*3)

    def show_state():
        print(f"second state: angle={clock.second.angle:0.1f}, velocity={clock.second.velocity:0.1f}, target_velocity={clock.second.target_velocity:0.1f}, acceleration={clock.second.acceleration:0.1f}")
        print(f"  speed PID: {clock.second.velocity_pid.state_str()}")
        print(f"  position PID: {clock.second.position_pid.state_str()}")
    timer = QtCore.QTimer()
    timer.timeout.connect(show_state)
    timer.start(500)

    if sys.flags.interactive != 1:
        if hasattr(app, 'exec_'):
            sys.exit(app.exec_())
        else:
            sys.exit(app.exec())
