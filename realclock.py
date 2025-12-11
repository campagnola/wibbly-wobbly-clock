import time
from qt import QtWidgets, QtCore, run_app
from clock import Clock


app = QtWidgets.QApplication([])
clock = Clock()
clock.show()

def angles():
    now = time.time()
    return (now/3600 + 4)*360/12, now*360/3600, now*360/60

hour, minute, second = angles()
clock.hour.set_angle(hour)
clock.minute.set_angle(minute)
clock.second.set_angle(second)

def update():
    hour, minute, second = angles()
    clock.hour.set_target(hour)
    clock.minute.set_target(minute)
    clock.second.set_target(second)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000)

run_app()