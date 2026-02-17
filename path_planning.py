# path_planning.py
# Wavefront path planning with 5" clearance (C-space inflation + wall band)
# Outputs executable commands for NewODO: [("TURN", deg), ("DRIVE", inches), ...]
# Also draws the course/path when it finds one.

import math
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import construct_map as cm  # your provided map constructor


# ----------------------------
# Constants / direction helpers
# ----------------------------
DIR_ORDER = ["N", "E", "S", "W"]  # clockwise


# ----------------------------
# Map + occupancy
# ----------------------------
def build_occupancy(img: np.ndarray) -> np.ndarray:
    """
    construct_map returns an RGB array where obstacles are encoded in the RED channel.
    Treat any red>0 as obstacle.
    Returns occ[y,x] boolean: True=obstacle, False=free.
    """
    return img[:, :, 0] > 0


# ----------------------------
# C-space inflation utilities
# ----------------------------
def _or_shift_no_wrap(dst: np.ndarray, src: np.ndarray, dx: int, dy: int) -> None:
    """
    dst |= src shifted by (dx,dy) WITHOUT wrap-around.
    dx>0 shifts right. dy>0 shifts down (increasing row index).
    """
    H, W = src.shape

    y0_src = max(0, -dy)
    y1_src = min(H, H - dy)
    x0_src = max(0, -dx)
    x1_src = min(W, W - dx)

    y0_dst = max(0, dy)
    y1_dst = min(H, H + dy)
    x0_dst = max(0, dx)
    x1_dst = min(W, W + dx)

    if y0_src >= y1_src or x0_src >= x1_src:
        return

    dst[y0_dst:y1_dst, x0_dst:x1_dst] |= src[y0_src:y1_src, x0_src:x1_src]


def inflate_occupancy(occ: np.ndarray, inflation_radius_cells: int) -> np.ndarray:
    """
    Grow obstacles by an axis-aligned SQUARE of radius inflation_radius_cells.
    This makes obstacles expand as larger rectangles (no rounded corners).
    """
    if inflation_radius_cells <= 0:
        return occ.copy()

    r = inflation_radius_cells
    src = occ.astype(bool)
    inflated = src.copy()

    # OR shifted copies for every (dx,dy) within the square
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            _or_shift_no_wrap(inflated, src, dx, dy)

    return inflated

def apply_border_clearance_band(occ: np.ndarray, clearance_cells: int) -> np.ndarray:
    """
    Enforce clearance from the map border by marking a band of thickness clearance_cells as obstacles.
    This avoids the "inflate a 1-cell border line" off-by-one behavior.
    """
    occ2 = occ.copy()
    c = clearance_cells
    if c <= 0:
        return occ2

    occ2[:c, :] = True
    occ2[-c:, :] = True
    occ2[:, :c] = True
    occ2[:, -c:] = True
    return occ2


# ----------------------------
# Wavefront cost + path extraction
# ----------------------------
def wavefront_cost(occ: np.ndarray, goal_xy: tuple[int, int]) -> np.ndarray:
    """
    4-neighbor BFS wavefront from goal.
    Returns cost[y,x]:
      -1 for obstacles/unreachable
       0 at goal
       increasing integer cost outward
    """
    H, W = occ.shape
    gx, gy = goal_xy

    if not (0 <= gx < W and 0 <= gy < H):
        raise ValueError("Goal is out of bounds.")
    if occ[gy, gx]:
        raise ValueError("Goal is inside obstacle (after clearance).")

    cost = np.full((H, W), -1, dtype=np.int32)
    cost[gy, gx] = 0

    q = deque([(gx, gy)])
    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while q:
        x, y = q.popleft()
        c = cost[y, x]
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                if occ[ny, nx]:
                    continue
                if cost[ny, nx] == -1:
                    cost[ny, nx] = c + 1
                    q.append((nx, ny))

    return cost


def extract_path(cost: np.ndarray, start_xy: tuple[int, int], goal_xy: tuple[int, int]) -> list[tuple[int, int]]:
    """
    Backtrack start->goal by stepping to a neighbor with cost-1 each move.
    """
    H, W = cost.shape
    sx, sy = start_xy
    gx, gy = goal_xy

    if not (0 <= sx < W and 0 <= sy < H):
        raise ValueError("Start is out of bounds.")
    if cost[sy, sx] < 0:
        raise ValueError("Start is unreachable or blocked (after clearance).")

    path = [(sx, sy)]
    x, y = sx, sy
    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while (x, y) != (gx, gy):
        cur = cost[y, x]
        nxt = None
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and cost[ny, nx] == cur - 1:
                nxt = (nx, ny)
                break
        if nxt is None:
            raise RuntimeError("Backtrack failed (no decreasing neighbor).")
        x, y = nxt
        path.append((x, y))

    return path


# ----------------------------
# Convert path -> executable NewODO commands
# ----------------------------
def path_to_headings(path: list[tuple[int, int]]) -> list[str]:
    """
    Convert each step in the cell path into a heading letter N/E/S/W.
    """
    hs: list[str] = []
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        dx, dy = x1 - x0, y1 - y0
        if (dx, dy) == (1, 0):
            hs.append("E")
        elif (dx, dy) == (-1, 0):
            hs.append("W")
        elif (dx, dy) == (0, 1):
            hs.append("N")
        elif (dx, dy) == (0, -1):
            hs.append("S")
        else:
            raise ValueError(f"Non-cardinal step found: {(dx, dy)}")
    return hs


def headings_to_commands(headings: list[str], start_heading: str, resolution: int) -> list[tuple[str, float]]:
    """
    Output commands that NewODO can execute:
      ("TURN", degrees)   # degrees: +CCW, -CW
      ("DRIVE", inches)
    """
    if start_heading not in DIR_ORDER:
        raise ValueError("start_heading must be one of: N, E, S, W")

    cmds: list[tuple[str, float]] = []
    cur = start_heading
    step_in = 1.0 / resolution

    i = 0
    while i < len(headings):
        h = headings[i]

        cur_i = DIR_ORDER.index(cur)
        tgt_i = DIR_ORDER.index(h)
        delta = (tgt_i - cur_i) % 4  # 0,1,2,3

        if delta == 0:
            turn = 0
        elif delta == 1:
            turn = +90
        elif delta == 2:
            turn = 180
        else:
            turn = -90

        # compress consecutive moves in same heading
        j = i
        while j < len(headings) and headings[j] == h:
            j += 1
        steps = j - i
        dist_in = steps * step_in

        if turn != 0:
            cmds.append(("TURN", float(turn)))
        if dist_in > 0:
            cmds.append(("DRIVE", float(dist_in)))

        cur = h
        i = j

    return cmds


# ----------------------------
# Inches -> cell mapping
# ----------------------------
def inches_to_cell_floor(
    x_in: float,
    y_in: float,
    resolution: int,
    W: int,
    H: int,
    clearance_cells: int,
) -> tuple[int, int]:
    """
    Convert inches to grid cells. Uses floor (not round) to avoid jumping into blocked areas.
    Clamps into the legal interior so start/goal never land inside the clearance band.
    """
    xc = int(x_in * resolution)
    yc = int(y_in * resolution)

    xc = max(clearance_cells, min(W - clearance_cells - 1, xc))
    yc = max(clearance_cells, min(H - clearance_cells - 1, yc))
    return xc, yc


# ----------------------------
# Planner that prints + draws
# ----------------------------
def plan_path(
    is_easy: bool,
    resolution: int,
    start_in: tuple[float, float],
    goal_in: tuple[float, float],
    start_heading: str,
    clearance_in: float = 5.0,
    show_plot: bool = True,
    show_cost: bool = False,
):
    """
    Plans a wavefront path with 5" clearance and returns:
      cmds: list[("TURN", deg), ("DRIVE", inches)]  # directly executable by NewODO.execute_commands
      path: list[(x_cell, y_cell)]
    Also prints the course and draws the path when found.
    """
    # Build map and occupancy
    img = cm.construct_map(is_easy, resolution)
    occ0 = build_occupancy(img)

    # Clearance in cells
    clearance_cells = int(math.ceil(clearance_in * resolution))

    # Inflate obstacles for obstacle clearance
    occ = inflate_occupancy(occ0, clearance_cells)

    # Enforce wall clearance with a band
    occ = apply_border_clearance_band(occ, clearance_cells)

    H, W = occ.shape

    # Convert inches -> cells (floor + clamp)
    sx, sy = inches_to_cell_floor(start_in[0], start_in[1], resolution, W, H, clearance_cells)
    gx, gy = inches_to_cell_floor(goal_in[0], goal_in[1], resolution, W, H, clearance_cells)

    # Sanity prints
    print(f"\nresolution={resolution} px/in, clearance={clearance_in} in => {clearance_cells} cells")
    print("Start inches:", start_in, "-> cell:", (sx, sy), "blocked?", bool(occ[sy, sx]))
    print("Goal  inches:", goal_in,  "-> cell:", (gx, gy), "blocked?", bool(occ[gy, gx]))

    if occ[sy, sx]:
        raise ValueError("Start landed in blocked cell after mapping (check start coordinates).")
    if occ[gy, gx]:
        raise ValueError("Goal landed in blocked cell after mapping (check goal coordinates).")

    # Wavefront + path
    cost = wavefront_cost(occ, (gx, gy))
    path = extract_path(cost, (sx, sy), (gx, gy))

    # Convert to executable commands
    headings = path_to_headings(path)
    cmds = headings_to_commands(headings, start_heading, resolution)

    # Print course
    print("\nCOURSE (commands NewODO can execute):")
    for typ, val in cmds:
        if typ == "TURN":
            print(f"  TURN {val:+.0f}°")
        else:
            print(f"  DRIVE {val:.2f} in")

    # Draw path
    if show_plot:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        plt.figure()
        plt.title("Inflated obstacles (dark) + planned path")
        plt.imshow(~occ, origin="lower")  # free bright, blocked dark
        plt.plot(xs, ys)
        plt.scatter([sx], [sy], marker="o")  # start
        plt.scatter([gx], [gy], marker="x")  # goal
        plt.xlabel("x (cells)")
        plt.ylabel("y (cells)")
        plt.show()

        if show_cost:
            plt.figure()
            plt.title("Wavefront cost-to-go (goal=0, obstacles/unreachable=-1)")
            plt.imshow(cost, origin="lower")
            plt.colorbar()
            plt.plot(xs, ys)
            plt.scatter([sx], [sy], marker="o")
            plt.scatter([gx], [gy], marker="x")
            plt.xlabel("x (cells)")
            plt.ylabel("y (cells)")
            plt.show()

    return cmds, path


# ----------------------------
# Example run
# ----------------------------
if __name__ == "__main__":
    #from NewODO import execute_commands

    cmds, path = plan_path(
        is_easy=True,
        resolution=1,
        start_in=(7.5, 49.0),
        goal_in=(65.0, 40.0),
        start_heading="W",
        clearance_in=5,
        show_plot=True,
        show_cost=False,
    )

    print("\nClose the plot window to execute the path...")
    input("Press Enter to execute commands on robot...")

    print(cmds)

