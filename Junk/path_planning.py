import math
import heapq
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import construct_map as cm  # your provided map constructor
# ----------------------------
# Constants / direction helpers
# ----------------------------
DIR_ORDER = ["N", "E", "S", "W"]  # clockwise
PATH_NBRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]
STEP_TO_HEADING = {
    (0, 1): "N",
    (1, 0): "E",
    (0, -1): "S",
    (-1, 0): "W",
}
HEADING_STEP_SCALE = {
    "N": 1.0,
    "E": 1.0,
    "S": 1.0,
    "W": 1.0,
}


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
def wavefront_cost(
    occ: np.ndarray,
    goal_xy: tuple[int, int],
    clearance_prefer: np.ndarray | None = None,
    clearance_cap_cells: int = 0,
    clearance_gain: float = 0.0,
) -> np.ndarray:
    """
    4-neighbor cost-to-go from goal using Dijkstra.

    Base step cost is 1.0. If clearance_prefer is provided, each step into a
    lower-clearance cell adds a penalty up to clearance_gain:
      step_cost = 1 + clearance_gain * ((cap - clr) / cap),  for clr < cap

    Returns cost[y,x]:
      +inf for obstacles/unreachable
       0.0 at goal
       increasing float cost outward
    """
    H, W = occ.shape
    gx, gy = goal_xy

    if not (0 <= gx < W and 0 <= gy < H):
        raise ValueError("Goal is out of bounds.")
    if occ[gy, gx]:
        raise ValueError("Goal is inside obstacle (after clearance).")

    cost = np.full((H, W), np.inf, dtype=np.float64)
    cost[gy, gx] = 0.0

    pq: list[tuple[float, int, int]] = [(0.0, gx, gy)]
    nbrs = PATH_NBRS

    while pq:
        c, x, y = heapq.heappop(pq)
        if c != cost[y, x]:
            continue

        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if occ[ny, nx]:
                continue

            step_cost = 1.0
            if (
                clearance_prefer is not None
                and clearance_cap_cells > 0
                and clearance_gain > 0.0
            ):
                clr = float(clearance_prefer[ny, nx])
                if clr < clearance_cap_cells:
                    frac = (clearance_cap_cells - clr) / float(clearance_cap_cells)
                    step_cost += clearance_gain * frac

            nc = c + step_cost
            if nc < cost[ny, nx]:
                cost[ny, nx] = nc
                heapq.heappush(pq, (nc, nx, ny))

    return cost


def obstacle_clearance_map(occ: np.ndarray) -> np.ndarray:
    """
    Multi-source BFS distance-to-nearest-obstacle map (4-neighbor / Manhattan).
    Returns clearance[y,x]:
      0 at obstacle cells
      increasing positive integers in free space
    """
    H, W = occ.shape
    clearance = np.full((H, W), -1, dtype=np.int32)
    q = deque()

    ys, xs = np.where(occ)
    for y, x in zip(ys, xs):
        clearance[y, x] = 0
        q.append((x, y))

    # Fallback for maps without any obstacles.
    if not q:
        clearance.fill(H + W)
        return clearance

    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    while q:
        x, y = q.popleft()
        d = clearance[y, x]
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and clearance[ny, nx] == -1:
                clearance[ny, nx] = d + 1
                q.append((nx, ny))

    return clearance


def extract_path(
    cost: np.ndarray,
    start_xy: tuple[int, int],
    goal_xy: tuple[int, int],
    clearance_prefer: np.ndarray | None = None,
    clearance_cap_cells: int = 0,
    turn_smooth_bias: float = 1.0,
    start_heading: str | None = None,
) -> list[tuple[int, int]]:
    """
    Backtrack start->goal by stepping to a strictly lower-cost neighbor.
    Optional tie-break/smoothing among equal-cost moves:
      - prefer continuing in the current heading (or start heading on first step)
      - then prefer larger obstacle clearance
      - then apply a small direction-change penalty to avoid zig-zag chatter
    """
    H, W = cost.shape
    sx, sy = start_xy
    gx, gy = goal_xy

    if not (0 <= sx < W and 0 <= sy < H):
        raise ValueError("Start is out of bounds.")
    if not np.isfinite(cost[sy, sx]):
        raise ValueError("Start is unreachable or blocked (after clearance).")

    if start_heading is not None and start_heading not in DIR_ORDER:
        raise ValueError("start_heading must be one of: N, E, S, W")

    path = [(sx, sy)]
    x, y = sx, sy
    nbrs = PATH_NBRS
    prev_dir: tuple[int, int] | None = None
    if start_heading is not None:
        prev_dir = {
            "N": (0, 1),
            "E": (1, 0),
            "S": (0, -1),
            "W": (-1, 0),
        }[start_heading]

    eps = 1e-9
    while (x, y) != (gx, gy):
        cur = cost[y, x]
        candidates = []
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            nc = cost[ny, nx]
            if not np.isfinite(nc):
                continue
            if not (nc < cur - eps):
                continue

            raw_clr = 0
            capped = 0
            if clearance_prefer is not None and clearance_cap_cells > 0:
                raw_clr = int(clearance_prefer[ny, nx])
                capped = min(raw_clr, clearance_cap_cells)

            candidates.append((nx, ny, dx, dy, nc, capped, raw_clr))

        if not candidates:
            raise RuntimeError("Backtrack failed (no decreasing neighbor).")

        # Hard heading persistence:
        # if continuing straight is still strictly downhill, do that and do not turn yet.
        if prev_dir is not None:
            straight = [c for c in candidates if (c[2], c[3]) == prev_dir]
            if straight:
                nx, ny, _, _, _, _, _ = straight[0]
                nxt = (nx, ny)
                prev_dir = (nxt[0] - x, nxt[1] - y)
                x, y = nxt
                path.append((x, y))
                continue

        # Otherwise choose globally best neighbor cost, then tie-break.
        min_nc = min(c[4] for c in candidates)
        equal_cost = [c for c in candidates if abs(c[4] - min_nc) <= eps]

        def tie_key(cand):
            _, _, dx, dy, _, capped, raw_clr = cand
            continue_pen = 0.0 if (prev_dir is None or prev_dir == (dx, dy)) else 1.0
            return (
                continue_pen,                         # prioritize straight motion
                turn_smooth_bias * continue_pen,      # preserve optional smoothness control
                -capped,                              # then prefer obstacle clearance
                -raw_clr,
            )

        nx, ny, _, _, _, _, _ = min(equal_cost, key=tie_key)
        nxt = (nx, ny)
        prev_dir = (nxt[0] - x, nxt[1] - y)
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
        h = STEP_TO_HEADING.get((dx, dy))
        if h is None:
            raise ValueError(f"Non-cardinal step found: {(dx, dy)}")
        hs.append(h)
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
        dist_in = steps * step_in * HEADING_STEP_SCALE[h]

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
    prefer_clearance_in: float = 7.0,
    clearance_aggression: float = 4.0,
    turn_smooth_bias: float = 1.5,
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

    # Cost-to-go with aggressive clearance weighting.
    clearance_map = obstacle_clearance_map(occ)
    prefer_cells = int(math.ceil(prefer_clearance_in * resolution))
    if prefer_cells > 0:
        print(f"Clearance preference cap: {prefer_clearance_in} in => {prefer_cells} cells")
    print(f"Clearance aggression gain: {clearance_aggression:.2f}")
    print(f"Turn smoothing bias: {turn_smooth_bias:.2f}")
    cost = wavefront_cost(
        occ,
        (gx, gy),
        clearance_prefer=clearance_map,
        clearance_cap_cells=prefer_cells,
        clearance_gain=clearance_aggression,
    )
    path = extract_path(
        cost,
        (sx, sy),
        (gx, gy),
        clearance_prefer=clearance_map,
        clearance_cap_cells=prefer_cells,
        turn_smooth_bias=turn_smooth_bias,
        start_heading=start_heading,
    )

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
            plt.title("Weighted cost-to-go (goal=0, obstacles/unreachable=inf)")
            cost_vis = np.where(np.isfinite(cost), cost, np.nan)
            plt.imshow(cost_vis, origin="lower")
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
        resolution=10,
        start_in= (5, 5),
        goal_in= (65, 40),
        start_heading="S",
        clearance_in=5,
        prefer_clearance_in=7.0,
        clearance_aggression=4.0,
        turn_smooth_bias=1.5,
        show_plot=True,
        show_cost=False,
    )

    print("\nClose the plot window to execute the path...")
    input("Press Enter to execute commands on robot...")

    print(cmds)

