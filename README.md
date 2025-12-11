It's a clock!
-------------

Its time-wimey is a bit wibbly-wobbly.

You'll need:

- python (I tested on 3.12)
- PyQt6
- pyqtgraph

Try running realclock.py for a demo.

Basic usage:

```
from clock import Clock
from qt import run_app
clock = Clock()
clock.show()

# move the second hand continuously at 100 deg/s
clock.second.set_velocity(100)

# move the minute hand to 120 deg from noon at a top speed of 500 deg/s
clock.minute.set_target(120, 500)
```

