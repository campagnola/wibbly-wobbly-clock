import sys
import time
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
from pid import PIDController


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
            kp=10.0,
            ki=0.0,
            kd=2.67,
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
        self.target_angle = None
        self.position_pid.reset()

    def set_target(self, angle, speed=None):
        """Move to a target angle.

        Args:
            angle: Target angle in degrees (unbounded - e.g., 3600 = 10 full rotations)
            speed: Maximum cruise speed in degrees/s (optional, uses max_speed if not provided)
        """
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
        if self.target_angle is not None:
            self.target_velocity = self.position_pid.update(self.target_angle, self.angle, dt)

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


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    clock = ClockGUI()
    clock.show()

    clock.second.set_target(360*3)
    clock.minute.set_target(180, speed=300)
    clock.hour.set_target(30, speed=50)

    if sys.flags.interactive != 1:
        if hasattr(app, 'exec_'):
            sys.exit(app.exec_())
        else:
            sys.exit(app.exec())
