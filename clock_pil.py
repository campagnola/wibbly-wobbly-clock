import tkinter as tk
import numpy as np
from PIL import Image, ImageTk


class ClockGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Clock")

        # Load clock face and determine window size
        self.clock_face = Image.open('clock_face.png')
        self.face_width, self.face_height = self.clock_face.size
        self.face_center = (512, 497)

        # Load and extract hands from clock_hands.png
        hands_image = Image.open('clock_hands.png')

        scale_factor = 0.6

        # Extract hour hand: center at (44, 272), y range 225-320
        hour_hand_orig = hands_image.crop((0, 225, hands_image.width, 320))
        hour_center_orig = (44, 272 - 225)  # Adjust y for cropped image
        self.hour_hand = hour_hand_orig.resize(
            (int(hour_hand_orig.width * scale_factor), int(hour_hand_orig.height * scale_factor)),
            Image.BICUBIC
        )
        self.hour_center = (int(hour_center_orig[0] * scale_factor), int(hour_center_orig[1] * scale_factor))

        # Extract minute hand: center at (36, 365), y range 330-402
        minute_hand_orig = hands_image.crop((0, 330, hands_image.width, 402))
        minute_center_orig = (36, 365 - 330)
        self.minute_hand = minute_hand_orig.resize(
            (int(minute_hand_orig.width * scale_factor), int(minute_hand_orig.height * scale_factor)),
            Image.BICUBIC
        )
        self.minute_center = (int(minute_center_orig[0] * scale_factor), int(minute_center_orig[1] * scale_factor))

        # Extract second hand: center at (196, 456), y range 437-472
        second_hand_orig = hands_image.crop((0, 437, hands_image.width, 472))
        second_center_orig = (196, 456 - 437)
        self.second_hand = second_hand_orig.resize(
            (int(second_hand_orig.width * scale_factor), int(second_hand_orig.height * scale_factor)),
            Image.BICUBIC
        )
        self.second_center = (int(second_center_orig[0] * scale_factor), int(second_center_orig[1] * scale_factor))

        # Create canvas with exact image size
        self.canvas = tk.Canvas(root, width=self.face_width, height=self.face_height,
                                highlightthickness=0)
        self.canvas.pack()

        # Hand angles (in degrees, 0 is pointing right, increases counter-clockwise)
        self.hour_angle = 0
        self.minute_angle = 0
        self.second_angle = 0

        self.draw_clock()

    def draw_clock(self):
        # Start with the clock face
        result = self.clock_face.copy()

        # Draw each hand by creating a full-size layer, positioning the hand,
        # rotating around the clock center, and compositing
        result = self._composite_hand(result, self.hour_hand, self.hour_center, self.hour_angle)
        result = self._composite_hand(result, self.minute_hand, self.minute_center, self.minute_angle)
        result = self._composite_hand(result, self.second_hand, self.second_center, self.second_angle)

        # Convert to PhotoImage and display
        self.photo = ImageTk.PhotoImage(result)
        self.canvas.delete('all')
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

    def _composite_hand(self, base_image, hand_image, hand_center, angle):
        """Rotate a hand and composite it onto the base image.

        Args:
            base_image: The image to composite onto
            hand_image: The hand image (pointing right)
            hand_center: The rotation center point in the hand image
            angle: Angle in degrees (0 = 12 o'clock, clockwise)
        """
        # Create a transparent layer the same size as the clock face
        layer = Image.new('RGBA', (self.face_width, self.face_height), (0, 0, 0, 0))

        # Calculate where to paste the hand so its center point is at the clock face center
        paste_x = self.face_center[0] - hand_center[0]
        paste_y = self.face_center[1] - hand_center[1]

        # Paste the hand onto the layer
        layer.paste(hand_image, (paste_x, paste_y), hand_image)

        # Rotate the entire layer around the clock center
        # Hands point right (3 o'clock), so we need to rotate by (90 - angle)
        # PIL rotates counter-clockwise, and we want clockwise, so negate
        rotation_angle = -(angle - 90)

        rotated_layer = layer.rotate(
            rotation_angle,
            center=self.face_center,
            resample=Image.BICUBIC
        )

        # Composite onto the base image
        base_image = Image.alpha_composite(base_image.convert('RGBA'), rotated_layer)

        return base_image

    def set_hand_angles(self, hour=None, minute=None, second=None):
        """Set the angles of the clock hands.

        Args:
            hour: Angle for hour hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            minute: Angle for minute hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
            second: Angle for second hand (0-360 degrees, 0 is at 12 o'clock, clockwise)
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

    # Example: Set clock to different angles
    clock.set_hand_angles(hour=0, minute=30, second=60)

    # root.mainloop()
