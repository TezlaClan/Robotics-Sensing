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

        # Closing the window sets this flag; the simulation polls is_closed() and
        # terminates cleanly, instead of stepping on into a dead canvas (which is
        # what forced the old "close window then spam Ctrl-C" dance).
        self.closed = False
        self.fig.canvas.mpl_connect("close_event", self._on_close)

        plt.ion()
        plt.show()

    def _on_close(self, _event):
        self.closed = True

    def is_closed(self) -> bool:
        return self.closed

    def render(self):
        if self.closed:
            return
        try:
            draw_frame(self.ax, self.environment, self.agents, self.layers, self.comm_range)
            plt.tight_layout()
            plt.pause(0.001)
        except Exception:
            # Window was torn down mid-draw - treat as a close request.
            self.closed = True

    def close(self):
        # Leave the final frame on screen but stop interactive mode.
        if not self.closed:
            plt.ioff()
