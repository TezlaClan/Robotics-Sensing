"""
frame.py

Shared frame-drawing routine used by BOTH the live Renderer and the
VideoWriter, so on-screen and recorded output stay identical.

Which visual layers are drawn is controlled by a `layers` dict
(see config["render_layers"]).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# Default: draw everything. Used when no layer config is supplied.
DEFAULT_LAYERS = {
    "map": True,
    "fog": True,
    "tint": True,
    "start_goal": True,
    "path": True,
    "particles": True,
    "agents": True,
}


def draw_frame(ax, environment, agents, layers=None):
    """
    Draw a single simulation frame onto `ax`.

    Clears the axes first, so this can be called repeatedly to animate.
    """

    layers = layers if layers is not None else DEFAULT_LAYERS

    ax.clear()

    grid = environment.map.grid
    h = len(grid)
    w = len(grid[0])

    # =========================
    # 1. Base occupancy image
    # =========================
    img = np.zeros((h, w, 3))

    for y in range(h):
        for x in range(w):
            if grid[y][x] == 1:
                img[y][x] = [0.2, 0.2, 0.2]
            else:
                img[y][x] = [1.0, 1.0, 1.0]

    # =========================
    # 2. Fog-of-War + tint overlay (per-agent belief)
    # =========================
    if layers.get("fog", True):
        for agent in agents:
            internal = agent.internal_map

            for y in range(h):
                for x in range(w):
                    prob = internal[y][x]

                    # Confidence: distance from 0.5 (0 = unknown, 1 = certain)
                    confidence = abs(prob - 0.5) * 2
                    fog_strength = 1.0 - confidence

                    # Darken by uncertainty
                    img[y][x] *= (1.0 - 0.6 * fog_strength)

                    # Optional tint on confidently known cells
                    if layers.get("tint", True):
                        if prob > 0.6:
                            img[y][x] = img[y][x] * 0.7 + np.array([0.6, 0.2, 0.2]) * 0.3
                        elif prob < 0.4:
                            img[y][x] = img[y][x] * 0.7 + np.array([0.2, 0.8, 0.2]) * 0.3

    # =========================
    # 3. Draw the composited image (base map + fog) in one pass.
    # extent aligns tile (x, y) to span x..x+1 / y..y+1 (center at x+0.5, y+0.5),
    # matching the +0.5 offsets used for markers, agents, and paths.
    # =========================
    if layers.get("map", True):
        ax.imshow(img, cmap="gray_r", interpolation="nearest", extent=[0, w, h, 0])

    # =========================
    # 4. Start / Goal markers
    # =========================
    if layers.get("start_goal", True):
        sx, sy = environment.map.start
        gx, gy = environment.map.goal

        ax.scatter(sx + 0.5, sy + 0.5, c='green', s=100, zorder=1, label="Start")
        ax.scatter(gx + 0.5, gy + 0.5, c='yellow', s=100, zorder=1, label="Goal")

    # =========================
    # 5. Planned path
    # =========================
    if layers.get("path", True):
        for agent in agents:
            if agent.current_path:
                xs = [c[0] + 0.5 for c in agent.current_path]
                ys = [c[1] + 0.5 for c in agent.current_path]
                ax.plot(xs, ys, 'cyan', linewidth=2.5, alpha=0.6, zorder=2)

    # =========================
    # 5b. Localization particle cloud (if the localizer exposes one)
    # =========================
    if layers.get("particles", True):
        for agent in agents:
            get_particles = getattr(
                getattr(agent, "localization_model", None), "get_particles", None
            )
            if get_particles is None:
                continue
            particles = get_particles()
            if not particles:
                continue
            ax.scatter(
                [p[0] for p in particles],
                [p[1] for p in particles],
                c='orange', s=4, alpha=0.35, zorder=2, linewidths=0,
            )

    # =========================
    # 6. Agents (true = red, believed = blue)
    # =========================
    if layers.get("agents", True):
        for agent in agents:
            tx, ty = agent.true_position
            bx, by = agent.believed_position
            radius = agent.radius

            ax.add_patch(patches.Circle(
                (tx, ty), radius,
                color='red', fill=True, alpha=0.9, zorder=3
            ))
            ax.add_patch(patches.Circle(
                (bx, by), radius * 0.8,
                color='blue', fill=True, alpha=0.7, zorder=3
            ))

    # =========================
    # 7. Framing
    # =========================
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title("Mapping Simulation")
