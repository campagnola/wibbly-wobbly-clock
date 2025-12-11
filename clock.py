import tkinter as tk
import math


class ClockGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Clock")

        self.canvas = tk.Canvas(root, width=400, height=400, bg='white')
        self.canvas.pack()

        self.center_x = 200
        self.center_y = 200
        self.radius = 150

        # Hand angles (in degrees, 0 is at 12 o'clock, increases clockwise)
        self.hour_angle = 0
        self.minute_angle = 0
        self.second_angle = 0

        self.draw_clock()

    def draw_clock(self):
        self.canvas.delete('all')

        # Draw clock circle
        self.canvas.create_oval(
            self.center_x - self.radius,
            self.center_y - self.radius,
            self.center_x + self.radius,
            self.center_y + self.radius,
            outline='black',
            width=2
        )

        # Draw hands
        self._draw_hand(self.hour_angle, self.radius * 0.5, 'black', 6)
        self._draw_hand(self.minute_angle, self.radius * 0.7, 'blue', 4)
        self._draw_hand(self.second_angle, self.radius * 0.9, 'red', 2)

    def _draw_hand(self, angle, length, color, width):
        # Convert angle to radians (0 degrees is at 12 o'clock)
        angle_rad = math.radians(angle - 90)

        end_x = self.center_x + length * math.cos(angle_rad)
        end_y = self.center_y + length * math.sin(angle_rad)

        self.canvas.create_line(
            self.center_x,
            self.center_y,
            end_x,
            end_y,
            fill=color,
            width=width
        )

    def set_hand_angles(self, hour=None, minute=None, second=None):
        """Set the angles of the clock hands.

        Args:
            hour: Angle for hour hand (0-360 degrees, 0 is at 12 o'clock)
            minute: Angle for minute hand (0-360 degrees, 0 is at 12 o'clock)
            second: Angle for second hand (0-360 degrees, 0 is at 12 o'clock)
        """
        if hour is not None:
            self.hour_angle = hour
        if minute is not None:
            self.minute_angle = minute
        if second is not None:
            self.second_angle = second

        self.draw_clock()


if __name__ == '__main__':
    root = tk.Tk()
    clock = ClockGUI(root)

    # Example: Set clock to 3:15:30
    # 3 hours = 90 degrees, 15 minutes = 90 degrees, 30 seconds = 180 degrees
    clock.set_hand_angles(hour=0, minute=30, second=60)

    root.mainloop()
