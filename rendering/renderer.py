import matplotlib.pyplot as plt
import numpy as np


class Renderer:
    def __init__(self, environment, agents):

        self.environment = environment
        self.agents = agents

        h = environment.map.height
        w = environment.map.width

        # ✅ Dynamic figure size based on map
        scale = max(h, w)
        fig_size = 8 if scale < 100 else 10

        self.fig, self.ax = plt.subplots(figsize=(fig_size, fig_size))

        plt.ion()
        plt.show()

    def render(self):
        self.ax.clear()

        grid = self.environment.map.grid
        h = len(grid)
        w = len(grid[0])

        # =========================
        # 1. Draw Map
        # =========================
        img = np.zeros((h, w, 3))

        for y in range(h):
            for x in range(w):
                if grid[y][x] == 1:
                    img[y][x] = [0.2, 0.2, 0.2]
                else:
                    img[y][x] = [1.0, 1.0, 1.0]

        # =========================
        # 2. Fog-of-War Overlay
        # =========================
        for agent in self.agents:
            internal = agent.internal_map

            for y in range(h):
                for x in range(w):
                    prob = internal[y][x]

                    # Confidence: distance from 0.5
                    confidence = abs(prob - 0.5) * 2   # 0 → unknown, 1 → certain

                    # Fog intensity (inverse confidence)
                    fog_strength = 1.0 - confidence

                    # Apply dark fog smoothly
                    img[y][x] *= (1.0 - 0.6 * fog_strength)

                    # Optional tinting (more subtle)
                    if prob > 0.6:
                        img[y][x] = img[y][x] * 0.7 + np.array([0.6, 0.2, 0.2]) * 0.3
                    elif prob < 0.4:
                      img[y][x] = img[y][x] * 0.7 + np.array([0.2, 0.8, 0.2]) * 0.3

        # Draw the fully composited image (base map + fog) in one pass
        self.ax.imshow(img, cmap="gray_r", interpolation="nearest")

        # =========================
        # 2. Draw Start/Goal (under everything)
        # =========================
        sx, sy = self.environment.map.start
        gx, gy = self.environment.map.goal

        self.ax.scatter(
            sx + 0.5, sy + 0.5,
            c='green', s=100, zorder=1, label="Start"
        )

        self.ax.scatter(
            gx + 0.5, gy + 0.5,
            c='yellow', s=100, zorder=1, label="Goal"
        )

        # =========================
        # 3. Draw Agents with Size
        # =========================
        for agent in self.agents:

            tx, ty = agent.true_position
            bx, by = agent.believed_position

            radius = agent.radius

            # ✅ Draw circles instead of points
            true_circle = plt.Circle(
                (tx, ty),
                radius,
                color='red',
                fill=True,
                alpha=0.9,
                zorder=3
            )

            belief_circle = plt.Circle(
                (bx, by),
                radius * 0.8,
                color='blue',
                fill=True,
                alpha=0.7,
                zorder=3
            )

            self.ax.add_patch(true_circle)
            self.ax.add_patch(belief_circle)

            # =========================
            # Draw Path
            # =========================
            if agent.current_path:
                xs = [c[0] + 0.5 for c in agent.current_path]
                ys = [c[1] + 0.5 for c in agent.current_path]

                self.ax.plot(xs, ys, 'cyan', linewidth=1, alpha=0.6, zorder=2)

        # =========================
        # 4. Scaling + Aspect
        # =========================
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(h, 0)

        self.ax.set_aspect('equal')

        # Remove axis for clean view
        self.ax.axis('off')

        self.ax.set_title("Mapping Simulation")

        plt.tight_layout()
        plt.pause(0.001)