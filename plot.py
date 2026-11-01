import matplotlib
matplotlib.use("TkAgg")

import pandas as pd
import matplotlib.pyplot as plt

# Read CSV files
initial= pd.read_csv("build/initialFile.csv")
optimized= pd.read_csv("build/optimizedFile.csv")

import numpy as np
d = np.hypot(initial["x"] - optimized["x"], initial["y"] - optimized["y"])
print("max position change:", d.max(), "m")
print("mean position change:", d.mean(), "m")
print(len(initial), len(optimized))
print(initial.head())
print(optimized.head())

# Plot initial trajectory
plt.plot(initial["x"],initial["y"],label="Initial Estimate",marker=".",markersize=3)
# Plot optimized trajectory
plt.plot(optimized["x"],optimized["y"],label="Optimized Estimate",marker=".",markersize=3)

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


