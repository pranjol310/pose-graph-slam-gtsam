import matplotlib
matplotlib.use("TkAgg")

import pandas as pd
import matplotlib.pyplot as plt

# Read CSV files
initial= pd.read_csv("results/initialFile.csv")
optimized_batch_slam= pd.read_csv("results/optimizedFile_batch_SLAM.csv")
optimized_isam2= pd.read_csv("results/optimizedFile_isam2.csv")

import numpy as np
d_batch_slam = np.hypot(initial["x"] - optimized_batch_slam["x"], initial["y"] - optimized_batch_slam["y"])
d_isam2 = np.hypot(initial["x"] - optimized_isam2["x"], initial["y"] - optimized_isam2["y"])
print("max position change in batch slam:", d_batch_slam.max(), "m")
print("mean position change in batch slam:", d_batch_slam.mean(), "m")
print("max position change in ISAM2:", d_isam2.max(), "m")
print("mean position change in ISAM2:", d_isam2.mean(), "m")

# Plot initial trajectory
plt.plot(initial["x"],initial["y"],label="Initial Estimate",marker=".",markersize=3)
# Plot optimized trajectory
plt.plot(optimized_batch_slam["x"],optimized_batch_slam["y"],label="Optimized Estimate for batch slam",marker=".",markersize=3)
plt.plot(optimized_isam2["x"],optimized_isam2["y"],label="Optimized Estimate for ISAM2",marker=".",markersize=3)

# Mark starting positions
plt.scatter(initial["x"].iloc[0],initial["y"].iloc[0],label="Start",marker="o")
# Mark ending positions
plt.scatter(initial["x"].iloc[-1],initial["y"].iloc[-1],label="End",marker="x")

plt.xlabel("X(metres)")
plt.ylabel("Y(metres)")
plt.title("Intel Dataset: Initial vs Optimized Trajectory")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

