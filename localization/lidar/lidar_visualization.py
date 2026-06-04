import pathlib
import tkinter.filedialog
from tracker.lidar.lidar_tracker import LiDARTracker


filename: str = tkinter.filedialog.askopenfilename(title="Select LVX point cloud File", filetypes=[("LVX Files", "*.lvx")], initialdir=pathlib.Path.cwd() / '../data/')

tracker = LiDARTracker()
tracker.load(pathlib.Path(filename).with_suffix('.npz').as_posix())

# Plot the states of lidar tracker
pkgs = slice(None)
smoothed = True
confidence_level = 0.05
# tracker.visualize_states(confidence_level=confidence_level, smoothed=smoothed, pkgs=pkgs)

# Visualize point clouds
dt = 100 * 1e6
output_size = (1920, 1080)
plot_range = (0, 25)
speed = 2
tracker.visualize_lidar_results(filename, dt=dt, output_size=output_size, plot_range=plot_range, speed=speed)