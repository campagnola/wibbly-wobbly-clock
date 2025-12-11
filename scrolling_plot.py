import time
import pyqtgraph as pg
from collections import deque
from pyqtgraph.Qt import QtWidgets, QtCore


class ScrollingPlot:
    """Generic scrolling plot with multiple subplots and curves."""

    def __init__(self, plot_names, curve_names, max_samples=1000, update_interval=0.1, colors=None):
        """Initialize scrolling plot.

        Args:
            plot_names: List of plot names (one subplot per name)
            curve_names: Either a list of curve names (same curves for all plots),
                        or a dict of {plot_name: [curve_names]} for per-plot curves
            max_samples: Maximum number of samples to keep in history
            update_interval: Minimum time between plot updates in seconds (default: 0.2)
            colors: Optional dict of {plot_name: {curve_name: color}} to specify colors.
                   Color can be a string ('r', 'g', 'b') or RGB tuple (R, G, B)
        """
        self.plot_names = plot_names

        # Handle both list and dict for curve_names
        if isinstance(curve_names, dict):
            self.curve_names_dict = curve_names
        else:
            # Convert list to dict with same curves for all plots
            self.curve_names_dict = {plot_name: curve_names for plot_name in plot_names}

        self.max_samples = max_samples
        self.update_interval = update_interval
        self.plotting_enabled = True
        self.colors = colors or {}

        # Create container widget
        self.container = QtWidgets.QWidget()
        self.container.setWindowTitle('Scrolling Plot')

        # Create graphics layout
        self.layout = pg.GraphicsLayoutWidget()

        # Create toggle button
        self.toggle_button = QtWidgets.QPushButton('Plotting: ON')
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(True)
        self.toggle_button.clicked.connect(self._toggle_plotting)
        self.toggle_button.setFixedSize(120, 30)

        # Create main layout
        main_layout = QtWidgets.QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)

        # Create top bar for button
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addStretch()
        top_bar.addWidget(self.toggle_button)
        top_bar.setContentsMargins(5, 5, 5, 5)

        # Add layouts
        main_layout.addLayout(top_bar)
        main_layout.addWidget(self.layout)
        self.container.setLayout(main_layout)

        # Data storage: {plot_name: {curve_name: deque}}
        self.data = {}
        # Time storage: {plot_name: deque}
        self.time_data = {}
        # Plot items: {plot_name: PlotItem}
        self.plot_items = {}
        # Curve items: {plot_name: {curve_name: PlotDataItem}}
        self.curves = {}

        # Time tracking
        self.start_time = time.perf_counter()
        self.last_update_time = 0.0

        # Create plots
        self._create_plots()

    def _toggle_plotting(self):
        """Toggle plotting on/off."""
        self.plotting_enabled = self.toggle_button.isChecked()
        if self.plotting_enabled:
            self.toggle_button.setText('Plotting: ON')
            self.toggle_button.setStyleSheet('')
        else:
            self.toggle_button.setText('Plotting: OFF')
            self.toggle_button.setStyleSheet('background-color: #ffaaaa;')

    def _create_plots(self):
        """Create plot items and curves."""
        # Default colors if not specified
        default_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'w']

        for i, plot_name in enumerate(self.plot_names):
            # Create plot item
            plot_item = self.layout.addPlot(row=i, col=0)
            plot_item.setLabel('left', plot_name)
            plot_item.setLabel('bottom', 'Time', units='s')
            plot_item.addLegend()
            plot_item.showGrid(x=True, y=True)

            self.plot_items[plot_name] = plot_item

            # Initialize data storage
            self.data[plot_name] = {}
            self.time_data[plot_name] = deque(maxlen=self.max_samples)
            self.curves[plot_name] = {}

            # Create curves for this plot
            curve_names = self.curve_names_dict.get(plot_name, [])
            for j, curve_name in enumerate(curve_names):
                # Get color from colors dict, or use default
                if plot_name in self.colors and curve_name in self.colors[plot_name]:
                    color = self.colors[plot_name][curve_name]
                else:
                    color = default_colors[j % len(default_colors)]

                # Use dashed line for target curves
                if 'Target' in curve_name:
                    try:
                        # Try Qt6 style first
                        pen = pg.mkPen(color=color, style=QtCore.Qt.PenStyle.DashLine, width=2)
                    except AttributeError:
                        # Fall back to Qt5 style
                        pen = pg.mkPen(color=color, style=QtCore.Qt.DashLine, width=2)
                else:
                    pen = pg.mkPen(color=color, width=2)
                curve = plot_item.plot(pen=pen, name=curve_name)
                self.curves[plot_name][curve_name] = curve
                self.data[plot_name][curve_name] = deque(maxlen=self.max_samples)

    def add_sample(self, values):
        """Add samples to the plots.

        Args:
            values: Dict of {plot_name: {curve_name: value}}
        """
        # Ignore samples if plotting is disabled
        if not self.plotting_enabled:
            return

        current_time = time.perf_counter() - self.start_time

        # Add samples to data
        for plot_name, curve_values in values.items():
            if plot_name not in self.data:
                continue

            # Add time point
            self.time_data[plot_name].append(current_time)

            # Add curve values
            for curve_name, value in curve_values.items():
                if curve_name in self.data[plot_name]:
                    self.data[plot_name][curve_name].append(value)

        # Update plots if enough time has passed
        if current_time - self.last_update_time >= self.update_interval:
            self._update_plots()
            self.last_update_time = current_time

    def _update_plots(self):
        """Update all plot curves."""
        for plot_name in self.plot_names:
            if plot_name not in self.data:
                continue

            time_array = list(self.time_data[plot_name])
            if not time_array:
                continue

            curve_names = self.curve_names_dict.get(plot_name, [])
            for curve_name in curve_names:
                if curve_name not in self.curves[plot_name]:
                    continue

                data_array = list(self.data[plot_name][curve_name])
                if len(data_array) == len(time_array):
                    self.curves[plot_name][curve_name].setData(time_array, data_array)

    def show(self):
        """Show the plot window."""
        self.container.show()

    def get_widget(self):
        """Get the container widget.

        Returns:
            QtWidgets.QWidget containing the plot and controls
        """
        return self.container
