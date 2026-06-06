"""
renderer.py

Live, interactive matplotlib renderer. Draws each simulation frame to an
on-screen window using the shared draw_frame() routine.
"""

import matplotlib.pyplot as plt

from rendering.frame import draw_frame, DEFAULT_LAYERS


class Renderer:
    def __init__(self, environment, agents, config=None):

        self.environment = environment
        self.agents = agents

        config = config or {}
        self.layers = config.get("render_layers", DEFAULT_LAYERS)
        self.comm_range = config.get("communication_range", None)

        h = environment.map.height
        w = environment.map.width

        # Dynamic figure size based on map
        scale = max(h, w)
        fig_size = 8 if scale < 100 else 10

        self.fig, self.ax = plt.subplots(figsize=(fig_size, fig_size))

        plt.ion()
        plt.show()

    def render(self):
        draw_frame(self.ax, self.environment, self.agents, self.layers, self.comm_range)
        plt.tight_layout()
        plt.pause(0.001)

    def close(self):
        # Leave the final frame on screen but stop interactive mode.
        plt.ioff()
