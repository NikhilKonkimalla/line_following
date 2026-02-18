import math
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import construct_map as cm


# ----------------------------
# Build occupancy grid
# ----------------------------
def build_occupancy(img: np.ndarray) -> np.ndarray:
    return img[:, :, 0] > 0  # red channel = obstacle


# ----------------------------
# Inflate obstacles (square)
# ----------------------------
def inflate_occupancy(occ: np.ndarray, r: int) -> np.ndarray:
    if r <= 0:
        return occ.copy()

    H, W = occ.shape
    inflated = occ.copy()

    for y in range(H):
        for x in range(W):
            if occ[y, x]:
                y0 = max(0, y - r)
                y1 = min(H, y + r + 1)
                x0 = max(0, x - r)
                x1 = min(W, x + r + 1)
                inflated[y0:y1, x0:x1] = True

    return inflated


# ----------------------------
# Inflate border
# ----------------------------
def inflate_border(occ: np.ndarray, r: int) -> np.ndarray:
    if r <= 0:
        return occ

    occ2 = occ.copy()
    H, W = occ.shape

    occ2[:r, :] = True
    occ2[-r:, :] = True
    occ2[:, :r] = True
    occ2[:, -r:] = True

    return occ2


# ----------------------------
# Wavefront BFS
# ----------------------------
def wavefront_cost(occ: np.ndarray, goal):
    H, W = occ.shape
    gx, gy = goal

    if occ[gy, gx]:
        raise ValueError("Goal is blocked after inflation.")

    cost = -np.ones((H, W), dtype=int)
    cost[gy, gx] = 0

    q = deque([(gx, gy)])
    nbrs = [(1,0), (-1,0), (0,1), (0,-1)]

    while q:
        x, y = q.popleft()
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                if not occ[ny, nx] and cost[ny, nx] == -1:
                    cost[ny, nx] = cost[y, x] + 1
                    q.append((nx, ny))

    return cost


# ----------------------------
# Backtrack shortest path
# ----------------------------
def extract_path(cost, start, goal):
    sx, sy = start
    gx, gy = goal

    if cost[sy, sx] < 0:
        raise ValueError("Start unreachable or blocked.")

    path = [(sx, sy)]
    x, y = sx, sy
    nbrs = [(1,0), (-1,0), (0,1), (0,-1)]

    while (x, y) != (gx, gy):
        cur = cost[y, x]
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cost.shape[1] and 0 <= ny < cost.shape[0]:
                if cost[ny, nx] == cur - 1:
                    x, y = nx, ny
                    path.append((x, y))
                    break

    return path


# ----------------------------
# Convert path to commands
# ----------------------------
DIR_ORDER = ["N", "E", "S", "W"]

def path_to_commands(path, start_heading, resolution):
    headings = []

    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        dx, dy = x1 - x0, y1 - y0
        if (dx, dy) == (1, 0): headings.append("E")
        elif (dx, dy) == (-1, 0): headings.append("W")
        elif (dx, dy) == (0, 1): headings.append("N")
        elif (dx, dy) == (0, -1): headings.append("S")

    cmds = []
    cur = start_heading
    step_in = 1.0 / resolution

    i = 0
    while i < len(headings):
        h = headings[i]

        cur_i = DIR_ORDER.index(cur)
        tgt_i = DIR_ORDER.index(h)
        delta = (tgt_i - cur_i) % 4

        if delta == 1:
            cmds.append(("TURN", 90.0))
        elif delta == 2:
            cmds.append(("TURN", 180.0))
        elif delta == 3:
            cmds.append(("TURN", -90.0))

        j = i
        while j < len(headings) and headings[j] == h:
            j += 1

        dist = (j - i) * step_in
        cmds.append(("DRIVE", dist))

        cur = h
        i = j

    return cmds


# ----------------------------
# Main planner
# ----------------------------
def plan_path(
    is_easy,
    resolution,
    start_in,
    goal_in,
    start_heading,
    clearance_in=6.0,
    border_clearance_in=3.0,
    show_plot=True
):
    img = cm.construct_map(is_easy, resolution)
    occ = build_occupancy(img)

    clearance_cells = int(math.ceil(clearance_in * resolution))
    border_cells = int(math.ceil(border_clearance_in * resolution))

    occ = inflate_occupancy(occ, clearance_cells)
    occ = inflate_border(occ, border_cells)

    sx = int(start_in[0] * resolution)
    sy = int(start_in[1] * resolution)
    gx = int(goal_in[0] * resolution)
    gy = int(goal_in[1] * resolution)

    cost = wavefront_cost(occ, (gx, gy))
    path = extract_path(cost, (sx, sy), (gx, gy))
    cmds = path_to_commands(path, start_heading, resolution)

    print("\n=== COMMAND LIST ===")
    for cmd in cmds:
        if cmd[0] == "TURN":
            print(f"TURN {cmd[1]:+.0f}°")
        else:
            print(f"DRIVE {cmd[1]:.2f} in")

    if show_plot:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        plt.figure()
        plt.imshow(~occ, origin="lower")
        plt.plot(xs, ys)
        plt.scatter([sx], [sy], marker="o")
        plt.scatter([gx], [gy], marker="x")
        plt.title("Simple Wavefront Path")
        plt.show()
        print(cmds)
    return cmds, path


# ----------------------------
# Example run
# ----------------------------
if __name__ == "__main__":
    plan_path(
        is_easy=True,
        resolution=10,
        start_in=(67, 49),
        goal_in=(5, 5),
        start_heading="N",
        clearance_in=6.0,
        border_clearance_in=3.0,
        show_plot=True
    )