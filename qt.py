import importlib
import sys


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


def run_app():
    app = QtWidgets.QApplication.instance()
    if sys.flags.interactive != 1:
        if hasattr(app, 'exec_'):
            sys.exit(app.exec_())
        else:
            sys.exit(app.exec())
