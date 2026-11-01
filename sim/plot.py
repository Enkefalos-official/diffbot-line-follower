"""Plot a simulator run:  python sim/plot.py run.csv [out.png]"""
import csv
import math
import sys

import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "run.csv"
rows = list(csv.DictReader(open(path)))
x = [float(r["x"]) for r in rows]
y = [float(r["y"]) for r in rows]

xs = [i / 100 for i in range(int(max(x) * 100) + 10)]
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(xs, [0.25 * math.sin(2 * v) for v in xs], color="black", lw=6, alpha=0.25, label="track")
ax.plot(x, y, color="#ef4444", lw=1.5, label="robot")
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Differential-drive robot following a line (PID)")
ax.legend(loc="upper right")
fig.tight_layout()
if len(sys.argv) > 2:
    fig.savefig(sys.argv[2], dpi=150)
else:
    plt.show()
