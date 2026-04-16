#!/usr/bin/env python3
"""
arm_sim.py – Visual simulator for the 2-DOF planar arm.

Replicates all math from arm_controller.py (FK, IK, collision, path planning)
without hardware dependencies. Uses matplotlib for interactive visualization.

Controls:
    - Click & drag the green end-effector dot to manually pose the arm
    - Type "x y" in the text box and press Enter to command a move
    - The arm animates through the planned path with obstacle avoidance
    - Type "home" to return to (6.25, 0)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import TextBox
from matplotlib.animation import FuncAnimation

# ── Constants (mirrored from arm_controller.py) ──────────────────────────────

L1 = 4.0
L2 = 4.5

T1_MIN = 0.0
T1_MAX = math.pi
T2_MIN = -math.radians(160)
T2_MAX = math.radians(160)

OBSTACLES = [
    (-4.0, -2.0, 4.0, 6.0),
    ( 1.0,  3.0, 5.0, 7.0),
]
OBS_MARGIN = 0.1

N_SAMPLES = 60

HOME_X = 6.25
HOME_Y = 0.0


# ── Kinematics (identical to arm_controller.py) ─────────────────────────────

def fk(t1: float, t2: float) -> tuple[float, float]:
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


def elbow_pos(t1: float) -> tuple[float, float]:
    return L1 * math.cos(t1), L1 * math.sin(t1)


def ik(x: float, y: float, elbow_up: bool = True):
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


# ── Collision (identical to arm_controller.py) ───────────────────────────────

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
        if (x0 - OBS_MARGIN <= x <= x1 + OBS_MARGIN and
                y0 - OBS_MARGIN <= y <= y1 + OBS_MARGIN):
            return True
    return False


def config_in_collision(t1, t2):
    for (x, y) in _arm_sample_points(t1, t2):
        if _pt_in_obstacle(x, y):
            return True
    return False


# ── Path planning ────────────────────────────────────────────────────────────
#
# Strategy:
#   1. Try direct line in joint space
#   2. Try one-joint-at-a-time sequential moves
#   3. Fall back to A* grid search over configuration space

import heapq

def _segment_clear(t1a, t2a, t1b, t2b):
    for i in range(N_SAMPLES + 1):
        a = i / N_SAMPLES
        if config_in_collision(t1a + a * (t1b - t1a), t2a + a * (t2b - t2a)):
            return False
    return True


def _try_sequential_moves(t1a, t2a, t1b, t2b):
    mid = (t1b, t2a)
    if (not config_in_collision(*mid) and
            _segment_clear(t1a, t2a, *mid) and
            _segment_clear(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    mid = (t1a, t2b)
    if (not config_in_collision(*mid) and
            _segment_clear(t1a, t2a, *mid) and
            _segment_clear(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    return None


# ── A* over a discretised configuration space ────────────────────────────────

# Grid resolution (degrees per cell) — 5° gives ~36x64 grid, fast enough
_GRID_DEG = 5

def _angle_to_cell(angle: float, a_min: float) -> int:
    return int(round((angle - a_min) / math.radians(_GRID_DEG)))

def _cell_to_angle(cell: int, a_min: float) -> float:
    return a_min + cell * math.radians(_GRID_DEG)

def _build_collision_grid():
    """Pre-compute which (t1, t2) grid cells are collision-free."""
    n1 = _angle_to_cell(T1_MAX, T1_MIN) + 1
    n2 = _angle_to_cell(T2_MAX, T2_MIN) + 1
    free = set()
    for i1 in range(n1):
        for i2 in range(n2):
            t1 = _cell_to_angle(i1, T1_MIN)
            t2 = _cell_to_angle(i2, T2_MIN)
            if not config_in_collision(t1, t2):
                free.add((i1, i2))
    return free, n1, n2

# Cache the grid on first use
_cspace_cache = None

def _get_cspace():
    global _cspace_cache
    if _cspace_cache is None:
        _cspace_cache = _build_collision_grid()
    return _cspace_cache


def _astar_grid(start_t1, start_t2, goal_t1, goal_t2):
    """A* search on the configuration-space grid. Returns waypoint list or None."""
    free, n1, n2 = _get_cspace()

    s_i1 = max(0, min(n1 - 1, _angle_to_cell(start_t1, T1_MIN)))
    s_i2 = max(0, min(n2 - 1, _angle_to_cell(start_t2, T2_MIN)))
    g_i1 = max(0, min(n1 - 1, _angle_to_cell(goal_t1, T1_MIN)))
    g_i2 = max(0, min(n2 - 1, _angle_to_cell(goal_t2, T2_MIN)))

    if (g_i1, g_i2) not in free:
        return None

    start = (s_i1, s_i2)
    goal = (g_i1, g_i2)

    if start == goal:
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    open_set = [(heuristic(start), 0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0}
    neighbors_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                   (0, 1), (1, -1), (1, 0), (1, 1)]

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            cells = [current]
            while current in came_from:
                current = came_from[current]
                cells.append(current)
            cells.reverse()

            # Greedy merge: skip cells as long as segment stays clear
            wp_idx = [0]
            i = 0
            while i < len(cells) - 1:
                best = i + 1
                for j in range(i + 2, len(cells)):
                    a1 = _cell_to_angle(cells[i][0], T1_MIN)
                    a2 = _cell_to_angle(cells[i][1], T2_MIN)
                    b1 = _cell_to_angle(cells[j][0], T1_MIN)
                    b2 = _cell_to_angle(cells[j][1], T2_MIN)
                    if _segment_clear(a1, a2, b1, b2):
                        best = j
                    else:
                        break
                wp_idx.append(best)
                i = best

            # Convert cell indices to angles, exact start/goal for endpoints
            result = [(start_t1, start_t2)]
            for idx in wp_idx[1:-1]:
                result.append((_cell_to_angle(cells[idx][0], T1_MIN),
                               _cell_to_angle(cells[idx][1], T2_MIN)))
            result.append((goal_t1, goal_t2))
            return result

        if cost > g_score.get(current, float('inf')):
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
            step_cost = 1.414 if (d1 != 0 and d2 != 0) else 1.0
            new_cost = cost + step_cost
            if new_cost < g_score.get(nb, float('inf')):
                g_score[nb] = new_cost
                heapq.heappush(open_set, (new_cost + heuristic(nb), new_cost, nb))
                came_from[nb] = current

    return None


def plan_joint_path(start_t1, start_t2, goal_t1, goal_t2):
    """
    Plan a collision-free joint-space path.
    1. Direct line
    2. Sequential (one joint at a time)
    3. A* grid search over configuration space
    """
    # 1. Direct
    if _segment_clear(start_t1, start_t2, goal_t1, goal_t2):
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    # 2. Sequential
    seq = _try_sequential_moves(start_t1, start_t2, goal_t1, goal_t2)
    if seq is not None:
        return seq

    # 3. A* grid search
    path = _astar_grid(start_t1, start_t2, goal_t1, goal_t2)
    if path is not None:
        return path

    return None


# ── Interpolate path into dense frames ───────────────────────────────────────

def interpolate_path(waypoints: list[tuple[float, float]],
                     steps_per_seg: int = 80) -> list[tuple[float, float]]:
    """Linearly interpolate joint-space waypoints into dense animation frames."""
    frames = []
    for i in range(len(waypoints) - 1):
        t1a, t2a = waypoints[i]
        t1b, t2b = waypoints[i + 1]
        for s in range(steps_per_seg + 1):
            a = s / steps_per_seg
            frames.append((t1a + a * (t1b - t1a), t2a + a * (t2b - t2a)))
    return frames


# ── Simulator GUI ────────────────────────────────────────────────────────────

class ArmSimulator:
    def __init__(self):
        # Start at home (6.25, 0)
        sol = ik(HOME_X, HOME_Y, elbow_up=True)
        if sol is None:
            sol = ik(HOME_X, HOME_Y, elbow_up=False)
        self.t1, self.t2 = sol

        self.dragging = False
        self.animating = False
        self.anim_frames: list[tuple[float, float]] = []
        self.anim_idx = 0
        self.trail_x: list[float] = []
        self.trail_y: list[float] = []
        self.anim = None

        self._build_gui()

    def _build_gui(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 10))
        self.fig.subplots_adjust(bottom=0.12)

        ax = self.ax
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect("equal")
        ax.set_xlabel("x (inches)")
        ax.set_ylabel("y (inches)")
        ax.set_title("Arm Simulator — drag tip or enter coords")
        ax.grid(True, which="both", linewidth=0.5, alpha=0.5)
        ax.set_xticks(range(-10, 11, 1))
        ax.set_yticks(range(-10, 11, 1))

        # Reachable workspace circle
        theta = np.linspace(0, np.pi, 200)
        ax.plot((L1 + L2) * np.cos(theta), (L1 + L2) * np.sin(theta),
                "--", color="gray", alpha=0.3, linewidth=1, label="max reach")
        ax.plot(abs(L1 - L2) * np.cos(theta), abs(L1 - L2) * np.sin(theta),
                "--", color="gray", alpha=0.3, linewidth=1)

        # Obstacles
        for (x0, x1, y0, y1) in OBSTACLES:
            rect = patches.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                     linewidth=2, edgecolor="darkred",
                                     facecolor="red", alpha=0.5, zorder=5)
            ax.add_patch(rect)
            # margin outline
            rect_m = patches.Rectangle(
                (x0 - OBS_MARGIN, y0 - OBS_MARGIN),
                (x1 - x0) + 2 * OBS_MARGIN, (y1 - y0) + 2 * OBS_MARGIN,
                linewidth=1, edgecolor="red", facecolor="none",
                linestyle=":", alpha=0.4, zorder=5)
            ax.add_patch(rect_m)

        # Shoulder base
        ax.plot(0, 0, "ko", markersize=10, zorder=10)

        # Arm links
        self.link1_line, = ax.plot([], [], "o-", color="#2196F3",
                                   linewidth=4, markersize=8,
                                   solid_capstyle="round", zorder=8)
        self.link2_line, = ax.plot([], [], "o-", color="#4CAF50",
                                   linewidth=4, markersize=8,
                                   solid_capstyle="round", zorder=8)

        # End-effector (draggable)
        self.ee_dot, = ax.plot([], [], "o", color="#4CAF50", markersize=14,
                               zorder=12, picker=True, pickradius=15)

        # EE trail during animation
        self.trail_line, = ax.plot([], [], "-", color="orange", linewidth=1.5,
                                   alpha=0.7, zorder=6)

        # Target marker
        self.target_dot, = ax.plot([], [], "x", color="red", markersize=12,
                                   markeredgewidth=2, zorder=11)

        # Status text
        self.status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                                   fontsize=9, verticalalignment="top",
                                   fontfamily="monospace",
                                   bbox=dict(boxstyle="round,pad=0.3",
                                             facecolor="white", alpha=0.8))

        # Collision indicator
        self.collision_text = ax.text(0.98, 0.98, "", transform=ax.transAxes,
                                      fontsize=11, verticalalignment="top",
                                      horizontalalignment="right",
                                      fontweight="bold",
                                      bbox=dict(boxstyle="round,pad=0.3",
                                                facecolor="white", alpha=0.8))

        # Text input box
        ax_box = self.fig.add_axes([0.15, 0.02, 0.7, 0.04])
        self.textbox = TextBox(ax_box, "Cmd: ",
                               initial="",
                               textalignment="left")
        self.textbox.on_submit(self._on_submit)

        # Mouse events
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)

        self._update_drawing()

    def _update_drawing(self):
        ex, ey = elbow_pos(self.t1)
        eex, eey = fk(self.t1, self.t2)

        self.link1_line.set_data([0, ex], [0, ey])
        self.link2_line.set_data([ex, eex], [ey, eey])
        self.ee_dot.set_data([eex], [eey])

        col = config_in_collision(self.t1, self.t2)
        self.collision_text.set_text("COLLISION" if col else "")
        self.collision_text.get_bbox_patch().set_facecolor(
            "#ff5252" if col else "white")

        self.status_text.set_text(
            f"EE: ({eex:.2f}, {eey:.2f})\n"
            f"t1: {math.degrees(self.t1):.1f}deg  "
            f"t2: {math.degrees(self.t2):.1f}deg")

        self.fig.canvas.draw_idle()

    def _try_ik_to(self, x: float, y: float) -> bool:
        sol = ik(x, y, elbow_up=True)
        if sol is None:
            sol = ik(x, y, elbow_up=False)
        if sol is None:
            return False
        self.t1, self.t2 = sol
        return True

    # ── Mouse drag ───────────────────────────────────────────────────────────

    def _on_press(self, event):
        if self.animating or event.inaxes != self.ax:
            return
        eex, eey = fk(self.t1, self.t2)
        if event.button == 1:
            dist = math.sqrt((event.xdata - eex) ** 2 +
                             (event.ydata - eey) ** 2)
            if dist < 0.8:
                self.dragging = True

    def _on_release(self, event):
        self.dragging = False

    def _on_motion(self, event):
        if not self.dragging or event.inaxes != self.ax:
            return
        self._try_ik_to(event.xdata, event.ydata)
        self._update_drawing()

    # ── Command input ────────────────────────────────────────────────────────

    def _on_submit(self, text: str):
        text = text.strip().lower()
        if not text:
            return

        if text == "home":
            self._command_move(HOME_X, HOME_Y)
            self.textbox.set_val("")
            return

        try:
            parts = text.replace(",", " ").split()
            gx, gy = float(parts[0]), float(parts[1])
        except (ValueError, IndexError):
            self.status_text.set_text("Usage: x y  (e.g. '3.0 4.5')\nor 'home'")
            self.fig.canvas.draw_idle()
            self.textbox.set_val("")
            return

        self._command_move(gx, gy)
        self.textbox.set_val("")

    def _command_move(self, x_goal: float, y_goal: float):
        if self.animating:
            return

        # Solve IK for goal
        sol = ik(x_goal, y_goal, elbow_up=True)
        if sol is None:
            sol = ik(x_goal, y_goal, elbow_up=False)
        if sol is None:
            self.status_text.set_text(f"Target ({x_goal:.2f}, {y_goal:.2f}) "
                                      f"is out of reach!")
            self.fig.canvas.draw_idle()
            return

        goal_t1, goal_t2 = sol

        # If goal config collides, try other elbow solution
        if config_in_collision(goal_t1, goal_t2):
            alt = ik(x_goal, y_goal, elbow_up=False)
            if alt is not None and not config_in_collision(*alt):
                goal_t1, goal_t2 = alt
            else:
                self.status_text.set_text(f"Target ({x_goal:.2f}, {y_goal:.2f}) "
                                          f"is inside an obstacle!")
                self.fig.canvas.draw_idle()
                return

        # Plan path
        path = plan_joint_path(self.t1, self.t2, goal_t1, goal_t2)
        if path is None:
            self.status_text.set_text("No collision-free path found!")
            self.fig.canvas.draw_idle()
            return

        # Show target marker
        self.target_dot.set_data([x_goal], [y_goal])

        # Interpolate and animate
        self.anim_frames = interpolate_path(path, steps_per_seg=80)
        self.anim_idx = 0
        self.trail_x = []
        self.trail_y = []
        self.animating = True

        n_segs = len(path) - 1
        self.status_text.set_text(
            f"Moving to ({x_goal:.2f}, {y_goal:.2f})\n"
            f"Path: {n_segs} segment(s)")
        self.fig.canvas.draw_idle()

        self.anim = FuncAnimation(self.fig, self._anim_step,
                                  frames=len(self.anim_frames),
                                  interval=15, repeat=False,
                                  blit=False)
        self.fig.canvas.draw_idle()

    def _anim_step(self, frame):
        if self.anim_idx >= len(self.anim_frames):
            self.animating = False
            if self.anim is not None:
                self.anim.event_source.stop()
            return

        self.t1, self.t2 = self.anim_frames[self.anim_idx]
        self.anim_idx += 1

        eex, eey = fk(self.t1, self.t2)
        self.trail_x.append(eex)
        self.trail_y.append(eey)
        self.trail_line.set_data(self.trail_x, self.trail_y)

        self._update_drawing()

        if self.anim_idx >= len(self.anim_frames):
            self.animating = False
            eex, eey = fk(self.t1, self.t2)
            self.status_text.set_text(
                f"EE: ({eex:.2f}, {eey:.2f})\n"
                f"t1: {math.degrees(self.t1):.1f}deg  "
                f"t2: {math.degrees(self.t2):.1f}deg\n"
                f"Move complete")
            self.fig.canvas.draw_idle()

    def run(self):
        print("Arm Simulator")
        print("  - Drag the green dot to manually pose the arm")
        print("  - Type 'x y' in the text box to command a move")
        print("  - Type 'home' to return to (6.25, 0)")
        plt.show()


if __name__ == "__main__":
    sim = ArmSimulator()
    sim.run()
