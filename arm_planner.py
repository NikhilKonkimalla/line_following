#!/usr/bin/env python3
"""
arm_planner.py — Path planning server + visualization for the 2-DOF arm.

Runs on your PC. Handles all heavy computation (collision grid, A*, proximity costs)
and displays the planned paths in C-space and workspace.
The robot connects via TCP and sends planning requests.

Usage:
    python arm_planner.py [--port 9090]
"""

import math
import heapq
import json
import socket
import threading
import argparse
import numpy as np
from collections import deque

# ── Arm geometry & constants (must match arm_controller.py) ──────────────────

L1 = 4.0
L2 = 4.5

T1_MIN = math.radians(7)
T1_MAX = math.radians(175)
T2_MIN = -math.pi
T2_MAX = math.pi

OBSTACLES = [
    (-4.1, -1.9, 3.9, 6.1),
    ( 0.9,  3.1, 4.9, 7.1),
]
OBS_MARGIN = 0

PROX_THRESHOLD = 3.25
PROX_WEIGHT = 150.0

N_SAMPLES = 60
_GRID_DEG = 5


# ── Kinematics ───────────────────────────────────────────────────────────────

def fk(t1, t2):
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


def ik(x, y, elbow_up=True):
    r2 = x * x + y * y
    cos_t2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(cos_t2) > 1.0:
        return None
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2 * cos_t2))
    if elbow_up:
        sin_t2 = -sin_t2
    t2 = math.atan2(sin_t2, cos_t2)
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    t1 = t1 % (2 * math.pi)
    if t1 > math.pi:
        t1_alt = t1 - 2 * math.pi
        if T1_MIN <= t1_alt <= T1_MAX:
            t1 = t1_alt
        else:
            return None
    if not (T1_MIN <= t1 <= T1_MAX):
        return None
    if not (T2_MIN <= t2 <= T2_MAX):
        return None
    return t1, t2


# ── Collision / proximity ────────────────────────────────────────────────────

def _arm_sample_points(t1, t2, n_per_link=25):
    pts = []
    ex = L1 * math.cos(t1)
    ey = L1 * math.sin(t1)
    eex, eey = fk(t1, t2)
    for i in range(n_per_link + 1):
        a = i / n_per_link
        pts.append((a * ex, a * ey))
    for i in range(n_per_link + 1):
        a = i / n_per_link
        pts.append((ex + a * (eex - ex), ey + a * (eey - ey)))
    return pts


def _pt_in_obstacle(x, y):
    for (x0, x1, y0, y1) in OBSTACLES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


def _dist_to_obstacles(x, y):
    min_d = float('inf')
    for (x0, x1, y0, y1) in OBSTACLES:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        min_d = min(min_d, math.sqrt(dx * dx + dy * dy))
    return min_d


def _config_proximity_cost(t1, t2):
    min_dist = float('inf')
    for (x, y) in _arm_sample_points(t1, t2):
        min_dist = min(min_dist, _dist_to_obstacles(x, y))
    if min_dist >= PROX_THRESHOLD:
        return 0.0
    if min_dist < OBS_MARGIN:
        return 2500.0
    return PROX_WEIGHT * (1.0 - min_dist / PROX_THRESHOLD)


def config_in_collision(t1, t2):
    for (x, y) in _arm_sample_points(t1, t2):
        if _pt_in_obstacle(x, y):
            return True
    return False


# ── Path planning ────────────────────────────────────────────────────────────

def _segment_clear(t1a, t2a, t1b, t2b):
    for i in range(N_SAMPLES + 1):
        a = i / N_SAMPLES
        if config_in_collision(t1a + a * (t1b - t1a), t2a + a * (t2b - t2a)):
            return False
    return True


def _segment_has_clearance(t1a, t2a, t1b, t2b):
    for i in range(N_SAMPLES + 1):
        a = i / N_SAMPLES
        t1 = t1a + a * (t1b - t1a)
        t2 = t2a + a * (t2b - t2a)
        for (x, y) in _arm_sample_points(t1, t2):
            if _dist_to_obstacles(x, y) < OBS_MARGIN:
                return False
    return True


def _segment_safe(t1a, t2a, t1b, t2b):
    return _segment_clear(t1a, t2a, t1b, t2b) and _segment_has_clearance(t1a, t2a, t1b, t2b)


def _try_sequential_moves(t1a, t2a, t1b, t2b):
    mid = (t1b, t2a)
    if (not config_in_collision(*mid) and
            _segment_safe(t1a, t2a, *mid) and
            _segment_safe(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    mid = (t1a, t2b)
    if (not config_in_collision(*mid) and
            _segment_safe(t1a, t2a, *mid) and
            _segment_safe(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    return None


# ── A* grid ──────────────────────────────────────────────────────────────────

def _angle_to_cell(angle, a_min):
    return int(round((angle - a_min) / math.radians(_GRID_DEG)))

def _cell_to_angle(cell, a_min):
    return a_min + cell * math.radians(_GRID_DEG)

def _build_collision_grid():
    n1 = _angle_to_cell(T1_MAX, T1_MIN) + 1
    n2 = _angle_to_cell(T2_MAX, T2_MIN) + 1
    free = set()
    cost = {}
    for i1 in range(n1):
        for i2 in range(n2):
            t1 = _cell_to_angle(i1, T1_MIN)
            t2 = _cell_to_angle(i2, T2_MIN)
            if not config_in_collision(t1, t2):
                free.add((i1, i2))
                cost[(i1, i2)] = _config_proximity_cost(t1, t2)
    return free, cost, n1, n2

_cspace_cache = None

def _get_cspace():
    global _cspace_cache
    if _cspace_cache is None:
        print("Building collision grid + proximity costs...")
        _cspace_cache = _build_collision_grid()
        free, cost, n1, n2 = _cspace_cache
        print(f"  Grid: {n1}x{n2}, {len(free)} free cells")
    return _cspace_cache


def _snap_to_free(i1, i2, free, n1, n2):
    if (i1, i2) in free:
        return (i1, i2)
    visited = {(i1, i2)}
    queue = deque([(i1, i2)])
    while queue:
        ci1, ci2 = queue.popleft()
        for d1 in (-1, 0, 1):
            for d2 in (-1, 0, 1):
                ni1, ni2 = ci1 + d1, ci2 + d2
                if (ni1, ni2) in visited:
                    continue
                if not (0 <= ni1 < n1 and 0 <= ni2 < n2):
                    continue
                if (ni1, ni2) in free:
                    return (ni1, ni2)
                visited.add((ni1, ni2))
                queue.append((ni1, ni2))
    return None


def _astar_grid(start_t1, start_t2, goal_t1, goal_t2):
    free, cost, n1, n2 = _get_cspace()

    s_i1 = max(0, min(n1 - 1, _angle_to_cell(start_t1, T1_MIN)))
    s_i2 = max(0, min(n2 - 1, _angle_to_cell(start_t2, T2_MIN)))
    g_i1 = max(0, min(n1 - 1, _angle_to_cell(goal_t1, T1_MIN)))
    g_i2 = max(0, min(n2 - 1, _angle_to_cell(goal_t2, T2_MIN)))

    snapped_s = _snap_to_free(s_i1, s_i2, free, n1, n2)
    snapped_g = _snap_to_free(g_i1, g_i2, free, n1, n2)
    if snapped_s is None or snapped_g is None:
        return None
    s_i1, s_i2 = snapped_s
    g_i1, g_i2 = snapped_g

    start = (s_i1, s_i2)
    goal = (g_i1, g_i2)

    if start == goal:
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    def heuristic(node):
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)

    open_set = [(heuristic(start), 0, start)]
    came_from = {}
    g_score = {start: 0}
    neighbors_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                   (0, 1), (1, -1), (1, 0), (1, 1)]

    while open_set:
        _, g_cost, current = heapq.heappop(open_set)

        if current == goal:
            cells = [current]
            while current in came_from:
                current = came_from[current]
                cells.append(current)
            cells.reverse()

            wp_idx = [0]
            i = 0
            while i < len(cells) - 1:
                best = i + 1
                for j in range(i + 2, len(cells)):
                    a1 = _cell_to_angle(cells[i][0], T1_MIN)
                    a2 = _cell_to_angle(cells[i][1], T2_MIN)
                    b1 = _cell_to_angle(cells[j][0], T1_MIN)
                    b2 = _cell_to_angle(cells[j][1], T2_MIN)
                    if _segment_safe(a1, a2, b1, b2):
                        best = j
                    else:
                        break
                wp_idx.append(best)
                i = best

            result = [(start_t1, start_t2)]
            for idx in wp_idx[1:-1]:
                result.append((_cell_to_angle(cells[idx][0], T1_MIN),
                               _cell_to_angle(cells[idx][1], T2_MIN)))
            result.append((goal_t1, goal_t2))
            return result

        if g_cost > g_score.get(current, float('inf')):
            continue

        ct1 = _cell_to_angle(current[0], T1_MIN)
        ct2 = _cell_to_angle(current[1], T2_MIN)
        for d1, d2 in neighbors_8:
            nb = (current[0] + d1, current[1] + d2)
            if nb not in free:
                continue
            nt1 = _cell_to_angle(nb[0], T1_MIN)
            nt2 = _cell_to_angle(nb[1], T2_MIN)
            if not _segment_clear(ct1, ct2, nt1, nt2):
                continue
            step_cost = (1.414 if (d1 != 0 and d2 != 0) else 1.0) + cost[nb]
            new_cost = g_cost + step_cost
            if new_cost < g_score.get(nb, float('inf')):
                g_score[nb] = new_cost
                heapq.heappush(open_set, (new_cost + heuristic(nb), new_cost, nb))
                came_from[nb] = current

    return None


def plan_joint_path(start_t1, start_t2, goal_t1, goal_t2):
    return _astar_grid(start_t1, start_t2, goal_t1, goal_t2)


# ── High-level planning ──────────────────────────────────────────────────────

def plan_move(start_t1, start_t2, x_goal, y_goal):
    sol = ik(x_goal, y_goal, elbow_up=True)
    if sol is None:
        sol = ik(x_goal, y_goal, elbow_up=False)
        if sol is None:
            return {"error": "target is out of reach"}

    goal_t1, goal_t2 = sol

    if config_in_collision(goal_t1, goal_t2):
        alt = ik(x_goal, y_goal, elbow_up=False)
        if alt is not None and not config_in_collision(*alt):
            goal_t1, goal_t2 = alt
        else:
            return {"error": "goal is inside an obstacle"}

    path = plan_joint_path(start_t1, start_t2, goal_t1, goal_t2)
    if path is None:
        return {"error": "no collision-free path found"}

    return {"path": [[t1, t2] for t1, t2 in path]}


def plan_multi(start_t1, start_t2, waypoints):
    """Plan paths for a sequence of (x,y) waypoints. Returns all paths or error."""
    all_paths = []
    cur_t1, cur_t2 = start_t1, start_t2

    for i, (gx, gy) in enumerate(waypoints):
        result = plan_move(cur_t1, cur_t2, gx, gy)
        if "error" in result:
            return {"error": f"waypoint {i+1} ({gx:.2f},{gy:.2f}): {result['error']}"}
        path = result["path"]
        all_paths.append({"path": path, "x": gx, "y": gy})
        cur_t1, cur_t2 = path[-1]

    return {"paths": all_paths}


# ── Visualization ────────────────────────────────────────────────────────────

def show_planned_paths(all_paths):
    """Display C-space + workspace with planned paths. Non-blocking."""
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    free, cost, n1, n2 = _get_cspace()

    # Build C-space image
    grid = np.full((n2, n1), -1.0)
    for (i1, i2), c in cost.items():
        grid[i2, i1] = c

    cmap = plt.cm.RdYlGn_r.copy()
    cmap.set_under("black")

    fig, (ax_cs, ax_ws) = plt.subplots(1, 2, figsize=(16, 7))

    # ── Left: Configuration space ──
    t1_edges = np.linspace(math.degrees(T1_MIN), math.degrees(T1_MAX), n1 + 1)
    t2_edges = np.linspace(math.degrees(T2_MIN), math.degrees(T2_MAX), n2 + 1)

    im = ax_cs.pcolormesh(t1_edges, t2_edges, grid, cmap=cmap,
                          vmin=0, vmax=max(PROX_WEIGHT, 1), shading="flat")
    fig.colorbar(im, ax=ax_cs, label="Proximity cost")

    colors = ["#00BFFF", "#FF6600", "#AA00FF", "#00CC44", "#FF1493", "#FFD700"]
    for idx, entry in enumerate(all_paths):
        path = entry["path"]
        gx, gy = entry["x"], entry["y"]
        t1s = [math.degrees(p[0]) for p in path]
        t2s = [math.degrees(p[1]) for p in path]
        c = colors[idx % len(colors)]
        ax_cs.plot(t1s, t2s, "o-", color=c, markersize=4, linewidth=2,
                   label=f"WP{idx+1} ({gx:.1f},{gy:.1f})")
        ax_cs.plot(t1s[0], t2s[0], "s", color=c, markersize=10)
        ax_cs.plot(t1s[-1], t2s[-1], "*", color=c, markersize=14)

    ax_cs.set_xlabel("theta1 (deg)")
    ax_cs.set_ylabel("theta2 (deg)")
    ax_cs.set_title("Configuration Space")
    ax_cs.legend(fontsize=8)
    ax_cs.grid(True, alpha=0.3)

    # ── Right: Workspace ──
    ax_ws.set_xlim(-10, 10)
    ax_ws.set_ylim(-5, 10)
    ax_ws.set_aspect("equal")
    ax_ws.set_xlabel("x (inches)")
    ax_ws.set_ylabel("y (inches)")
    ax_ws.set_title("Workspace")
    ax_ws.grid(True, alpha=0.3)

    for (x0, x1, y0, y1) in OBSTACLES:
        rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                 linewidth=2, edgecolor="darkred",
                                 facecolor="red", alpha=0.5, zorder=5)
        ax_ws.add_patch(rect)

    theta = np.linspace(0, np.pi, 200)
    ax_ws.plot((L1 + L2) * np.cos(theta), (L1 + L2) * np.sin(theta),
              "--", color="gray", alpha=0.3, linewidth=1)
    ax_ws.plot(0, 0, "ko", markersize=8, zorder=10)

    for idx, entry in enumerate(all_paths):
        path = entry["path"]
        gx, gy = entry["x"], entry["y"]
        xs = [fk(p[0], p[1])[0] for p in path]
        ys = [fk(p[0], p[1])[1] for p in path]
        c = colors[idx % len(colors)]
        ax_ws.plot(xs, ys, "o-", color=c, markersize=3, linewidth=1.5,
                   label=f"WP{idx+1} ({gx:.1f},{gy:.1f})")
        ax_ws.plot(gx, gy, "*", color=c, markersize=14, zorder=11)

    ax_ws.legend(fontsize=8)
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)


# ── TCP server ───────────────────────────────────────────────────────────────

def handle_client(conn, addr):
    print(f"Robot connected from {addr}")
    buf = b""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    req = json.loads(line.decode())
                except json.JSONDecodeError:
                    conn.sendall((json.dumps({"error": "bad JSON"}) + "\n").encode())
                    continue

                cmd = req.get("cmd", "plan")

                if cmd == "plan":
                    result = plan_move(req["start_t1"], req["start_t2"],
                                       req["x_goal"], req["y_goal"])
                    if "path" in result:
                        show_planned_paths([{"path": result["path"],
                                             "x": req["x_goal"], "y": req["y_goal"]}])

                elif cmd == "plan_multi":
                    result = plan_multi(req["start_t1"], req["start_t2"],
                                        req["waypoints"])
                    if "paths" in result:
                        show_planned_paths(result["paths"])

                elif cmd == "ping":
                    result = {"ok": True}
                else:
                    result = {"error": f"unknown cmd: {cmd}"}

                conn.sendall((json.dumps(result) + "\n").encode())

                if "error" in result:
                    print(f"  [{addr}] {cmd} -> error: {result['error']}")
                elif cmd == "plan_multi":
                    print(f"  [{addr}] {cmd} -> {len(result['paths'])} paths")
                else:
                    n = len(result.get("path", []))
                    print(f"  [{addr}] {cmd} -> {n} waypoints")

    except ConnectionResetError:
        pass
    finally:
        conn.close()
        print(f"Robot disconnected: {addr}")


def main():
    parser = argparse.ArgumentParser(description="Arm path planning server")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    _get_cspace()  # Pre-build grid

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(2)
    print(f"\nPlanner server listening on 0.0.0.0:{args.port}")
    print("Waiting for robot to connect...\n")

    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\nServer stopped.")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
