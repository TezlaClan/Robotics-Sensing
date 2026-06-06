"""
video_writer.py

Records the simulation to an mp4 file, frame by frame, using the same
layered frame drawing as the live renderer.

Recordings are auto-named and stored in a videos folder:

    videos/<N>_<map>_<localization>_<agents>_<comm>.mp4

where N increments as recordings are added. Each mp4 is paired with a
matching .json setup file describing the run (config, seeds, steps, etc.).

Rendering is done on an offscreen Agg figure (independent of any interactive
backend), and frames are encoded with imageio + the imageio-ffmpeg backend,
so no system ffmpeg install is required.

    pip install imageio imageio-ffmpeg
"""

import os
import re
import json
import datetime

import numpy as np

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from rendering.frame import draw_frame, DEFAULT_LAYERS


class VideoWriter:
    def __init__(self, environment, agents, config=None):

        self.environment = environment
        self.agents = agents
        self.config = config = config or {}

        self.layers = config.get("render_layers", DEFAULT_LAYERS)
        self.comm_range = config.get("communication_range", None)
        self.fps = config.get("video_fps", 30)
        self.dpi = config.get("video_dpi", 100)

        # =========================
        # Output naming
        # =========================
        self.video_dir = config.get("video_dir", "videos")
        os.makedirs(self.video_dir, exist_ok=True)

        index = self._next_index(self.video_dir)
        self.name = f"{index}_{self._build_slug()}"
        self.video_file = self.name + ".mp4"
        self.path = os.path.join(self.video_dir, self.video_file)
        self.meta_path = os.path.join(self.video_dir, self.name + ".json")

        self.frame_count = 0

        # =========================
        # Offscreen figure
        # =========================
        h = environment.map.height
        w = environment.map.width
        scale = max(h, w)
        fig_size = 8 if scale < 100 else 10

        # Agg figure never pops up a window, so it works alongside the live
        # renderer and on headless machines.
        self.fig = Figure(figsize=(fig_size, fig_size), dpi=self.dpi)
        self.canvas = FigureCanvasAgg(self.fig)
        self.ax = self.fig.add_subplot(111)

        self._writer = self._open_writer()

    # =========================
    # Naming helpers
    # =========================

    def _next_index(self, video_dir) -> int:
        """Lowest unused integer prefix among existing recordings (0, 1, ...)."""
        indices = []
        for fname in os.listdir(video_dir):
            match = re.match(r"(\d+)_", fname)
            if match:
                indices.append(int(match.group(1)))
        return max(indices) + 1 if indices else 0

    def _build_slug(self) -> str:
        """
        Descriptive name tokens: map type, localization, agent count, comms.
        e.g. "maze_odometry_1agent_global".
        """
        map_type = self.config.get("map_type", "map")
        n = len(self.agents)

        if self.agents:
            loc = self._short_name(
                type(self.agents[0].localization_model).__name__, "Localization"
            )
            comm = getattr(self.agents[0].communication_model, "mode", "comm")
        else:
            loc, comm = "none", "none"

        return f"{map_type}_{loc}_{n}agent_{comm}"

    @staticmethod
    def _short_name(class_name: str, suffix: str) -> str:
        """'OdometryLocalization' + 'Localization' -> 'odometry'."""
        name = class_name.replace(suffix, "")
        return name.lower() if name else class_name.lower()

    # =========================
    # Setup / teardown
    # =========================

    def _open_writer(self):
        """
        Open the imageio mp4 writer. Returns None (and prints guidance) if the
        backend is unavailable, so the simulation can still run without video.
        """
        try:
            import imageio
        except ImportError:
            print(
                "[VideoWriter] Recording disabled: 'imageio' is not installed.\n"
                "              Install it with:  pip install imageio imageio-ffmpeg"
            )
            return None

        try:
            return imageio.get_writer(
                self.path,
                fps=self.fps,
                codec="libx264",
                quality=8,
                macro_block_size=None,  # don't silently resize odd dimensions
            )
        except Exception as e:
            print(f"[VideoWriter] Could not open '{self.path}' for writing: {e}")
            print("              You may be missing the ffmpeg backend:  pip install imageio-ffmpeg")
            return None

    # =========================
    # Per-frame
    # =========================

    def render(self):
        if self._writer is None:
            return

        draw_frame(self.ax, self.environment, self.agents, self.layers, self.comm_range)
        self.fig.tight_layout()
        self.canvas.draw()

        # Grab the RGBA buffer and drop the alpha channel -> RGB frame.
        # Figure pixel size is fixed (figsize * dpi), so every frame is the
        # same shape, which ffmpeg requires.
        buf = np.asarray(self.canvas.buffer_rgba())
        frame = buf[:, :, :3].copy()

        self._writer.append_data(frame)
        self.frame_count += 1

    def close(self):
        if self._writer is None:
            return

        self._writer.close()
        self._writer = None
        print(f"[VideoWriter] Saved recording to '{self.path}'")

        self._write_metadata()
        print(f"[VideoWriter] Saved setup to '{self.meta_path}'")

    # =========================
    # Metadata sidecar (JSON)
    # =========================

    def _write_metadata(self):
        m = self.environment.map
        agent = self.agents[0] if self.agents else None

        summary = {
            "map_type": self.config.get("map_type"),
            "map_size": {"width": m.width, "height": m.height},
            "start_cell": list(m.start),
            "goal_cell": list(m.goal),
            "agents": len(self.agents),
            "steps_taken": self.frame_count,
        }

        if agent is not None:
            summary.update({
                "localization": type(agent.localization_model).__name__,
                "exploration": type(agent.exploration_strategy).__name__,
                "planner": type(agent.planner).__name__,
                "communication": getattr(agent.communication_model, "mode", None),
                "sensor": {
                    "mode": getattr(agent.sensor_model, "mode", None),
                    "range": getattr(agent.sensor_model, "sensor_range", None),
                },
            })

        metadata = {
            "name": self.name,
            "video_file": self.video_file,
            "recorded": datetime.datetime.now().isoformat(timespec="seconds"),
            "summary": summary,
            "seeds": {
                "map_seed": self.config.get("map_seed"),
                "behaviour_seed": self.config.get("behaviour_seed"),
            },
            "config": self.config,
        }

        with open(self.meta_path, "w") as f:
            json.dump(metadata, f, indent=4, default=str)
