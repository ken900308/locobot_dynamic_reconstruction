#!/usr/bin/env python3
"""Terminal UI launcher for quick_start scripts 1, 3, 4, and 5.

This is intentionally a TUI, similar in spirit to nmtui: it runs inside the
current terminal, needs no X11 DISPLAY, and keeps each script's live logs in a
separate pane.
"""

from __future__ import annotations

import argparse
import curses
import os
import queue
import signal
import subprocess
import textwrap
import threading
import time
from dataclasses import dataclass, field


DEFAULT_CONTAINER = "mast3r_locobot_jazzy"
DEFAULT_DOMAIN_ID = "50"
CONTAINER_THOR_DIR = "/workspace/thor"


@dataclass(frozen=True)
class ScriptSpec:
    key: str
    title: str
    script: str
    stop_pattern: str


SCRIPTS: tuple[ScriptSpec, ...] = (
    ScriptSpec(
        key="1",
        title="1 MASt3R SLAM",
        script="1_ipc_bridge.sh",
        stop_pattern="mast3r_slam_node|mast3r_slam_visual_IGBR.py|quick_start/1_ipc_bridge.sh|quick_start/2_mast3r_slam.sh",
    ),
    ScriptSpec(
        key="3",
        title="3 IPC Receiver (unused)",
        script="3_ipc_receiver.sh",
        stop_pattern="quick_start/3_ipc_receiver.sh",
    ),
    ScriptSpec(
        key="4",
        title="4 Auto Anchor",
        script="4_auto_anchor.sh",
        stop_pattern="auto_anchor_from_pointcloud_stretch3|quick_start/4_auto_anchor.sh",
    ),
    ScriptSpec(
        key="5",
        title="5 PC2 to Map",
        script="5_pc2_to_map.sh",
        stop_pattern="pc2_to_map_stretch3|quick_start/5_pc2_to_map.sh",
    ),
)


@dataclass
class PaneState:
    spec: ScriptSpec
    proc: subprocess.Popen[str] | None = None
    reader_thread: threading.Thread | None = None
    log_queue: queue.Queue[str] = field(default_factory=queue.Queue)
    lines: list[str] = field(default_factory=list)
    scrollback: int = 0

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def status(self) -> str:
        if self.is_running():
            return "RUNNING"
        if self.proc is None:
            return "STOPPED"
        return f"EXIT {self.proc.returncode}"


class QuickStartTui:
    def __init__(self, container: str, domain_id: str, mode: str) -> None:
        self.container = container
        self.domain_id = domain_id
        self.mode = mode
        self.panes = [PaneState(spec) for spec in SCRIPTS]
        self.selected = 0
        self.message = "Ready"
        self.running = True

    def run(self) -> None:
        curses.wrapper(self._main)

    def _main(self, stdscr: curses.window) -> None:
        curses.curs_set(0)
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_RED, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        curses.init_pair(4, curses.COLOR_CYAN, -1)
        stdscr.nodelay(True)
        stdscr.timeout(100)

        while self.running:
            self._drain_logs()
            self._draw(stdscr)
            key = stdscr.getch()
            if key != -1:
                self._handle_key(key)
            self._reap_exited()
            time.sleep(0.03)

    def _handle_key(self, key: int) -> None:
        if key in (ord("q"), ord("Q")):
            if any(pane.is_running() for pane in self.panes):
                self.stop_all()
                self.message = "Stopping all scripts before quit; press q again to close"
                return
            self.running = False
            return
        if key in (ord("a"), ord("A")):
            self.start_all()
        elif key in (ord("x"), ord("X")):
            self.stop_all()
        elif key in (ord("s"), ord("S")):
            self.start_selected()
        elif key in (ord("t"), ord("T")):
            self.stop_selected()
        elif key in (ord("c"), ord("C")):
            self.clear_selected()
        elif key in (ord("C") - 64,):
            self.clear_all()
        elif key in (9, curses.KEY_RIGHT, curses.KEY_DOWN):
            self.selected = (self.selected + 1) % len(self.panes)
        elif key in (curses.KEY_LEFT, curses.KEY_UP):
            self.selected = (self.selected - 1) % len(self.panes)
        elif key in (ord("j"), ord("J")):
            self.panes[self.selected].scrollback = max(0, self.panes[self.selected].scrollback - 1)
        elif key in (ord("k"), ord("K")):
            self.panes[self.selected].scrollback += 1
        elif key in (ord("g"), ord("G")):
            self.panes[self.selected].scrollback = 0
        elif chr(key) in {pane.spec.key for pane in self.panes if 0 <= key < 256}:
            self.toggle_by_key(chr(key))

    def start_all(self) -> None:
        for index in range(len(self.panes)):
            self.start(index)

    def stop_all(self) -> None:
        for index in range(len(self.panes)):
            self.stop(index)

    def clear_all(self) -> None:
        for pane in self.panes:
            pane.lines.clear()
            pane.scrollback = 0
        self.message = "Cleared all panes"

    def start_selected(self) -> None:
        self.start(self.selected)

    def stop_selected(self) -> None:
        self.stop(self.selected)

    def clear_selected(self) -> None:
        pane = self.panes[self.selected]
        pane.lines.clear()
        pane.scrollback = 0
        self.message = f"Cleared {pane.spec.title}"

    def toggle_by_key(self, key: str) -> None:
        for index, pane in enumerate(self.panes):
            if pane.spec.key == key:
                self.selected = index
                if pane.is_running():
                    self.stop(index)
                else:
                    self.start(index)
                return

    def start(self, index: int) -> None:
        pane = self.panes[index]
        if pane.is_running():
            self.message = f"{pane.spec.title} already running"
            return

        command = self.build_launch_command(pane.spec)
        pane.lines.append(f"[launcher] Starting {pane.spec.title}")
        pane.lines.append(f"[launcher] {self.launch_description(pane.spec)}")
        pane.scrollback = 0

        try:
            pane.proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError as exc:
            pane.proc = None
            pane.lines.append(f"[launcher] Command not found: {exc}")
            self.message = f"Failed to start {pane.spec.title}"
            return
        except Exception as exc:  # pragma: no cover - terminal error path
            pane.proc = None
            pane.lines.append(f"[launcher] Launch failed: {exc}")
            self.message = f"Failed to start {pane.spec.title}"
            return

        pane.reader_thread = threading.Thread(target=self._read_output, args=(pane,), daemon=True)
        pane.reader_thread.start()
        self.message = f"Started {pane.spec.title}"

    def stop(self, index: int) -> None:
        pane = self.panes[index]
        if pane.is_running() and pane.proc is not None:
            pane.lines.append("[launcher] Sending SIGINT...")
            try:
                os.killpg(os.getpgid(pane.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            pane.lines.append("[launcher] Not running; checking for matching process...")

        threading.Thread(
            target=subprocess.run,
            args=(self.build_stop_command(pane.spec),),
            kwargs={"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL},
            daemon=True,
        ).start()
        self.message = f"Stop requested for {pane.spec.title}"

    def build_launch_command(self, spec: ScriptSpec) -> list[str]:
        script_path = f"{CONTAINER_THOR_DIR}/quick_start/{spec.script}"
        inner = (
            f"cd {CONTAINER_THOR_DIR} && "
            f"source {CONTAINER_THOR_DIR}/environment.sh {self.domain_id} && "
            "set +u && source /opt/ros/jazzy/setup.bash && set -u && "
            "if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then "
            "source /workspace/thor/ros2_ws/install/setup.bash; "
            "fi && "
            "if command -v stdbuf >/dev/null 2>&1; then "
            f"exec stdbuf -oL -eL {script_path}; "
            f"else exec {script_path}; fi"
        )
        if self.mode == "container":
            return ["bash", "-lc", inner]
        return ["docker", "exec", "-i", self.container, "bash", "-lc", inner]

    def build_stop_command(self, spec: ScriptSpec) -> list[str]:
        inner = f"pkill -INT -f '{spec.stop_pattern}' >/dev/null 2>&1 || true"
        if self.mode == "container":
            return ["bash", "-lc", inner]
        return ["docker", "exec", "-i", self.container, "bash", "-lc", inner]

    def launch_description(self, spec: ScriptSpec) -> str:
        if self.mode == "container":
            return f"container-local bash ... {spec.script}"
        return f"docker exec {self.container} ... {spec.script}"

    def _read_output(self, pane: PaneState) -> None:
        assert pane.proc is not None
        assert pane.proc.stdout is not None
        for line in pane.proc.stdout:
            pane.log_queue.put(line.rstrip("\n"))
        return_code = pane.proc.wait()
        pane.log_queue.put(f"[launcher] Process exited with code {return_code}")

    def _drain_logs(self) -> None:
        for pane in self.panes:
            while True:
                try:
                    line = pane.log_queue.get_nowait()
                except queue.Empty:
                    break
                pane.lines.append(line)
                if len(pane.lines) > 2000:
                    del pane.lines[:500]

    def _reap_exited(self) -> None:
        for pane in self.panes:
            if pane.proc is not None and pane.proc.poll() is not None:
                continue

    def _draw(self, stdscr: curses.window) -> None:
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()
        if max_y < 20 or max_x < 80:
            stdscr.addstr(0, 0, "Terminal too small. Please resize to at least 80x20.")
            stdscr.refresh()
            return

        header = (
            f"LoCoBot quick_start 1/3/4/5 | mode={self.mode} | "
            f"container={self.container} | ROS_DOMAIN_ID={self.domain_id}"
        )
        self._addnstr(stdscr, 0, 0, header, max_x - 1, curses.A_BOLD)
        help_line = "a StartAll  x StopAll  1/3/4/5 Toggle  Tab Select  s Start  t Stop  c Clear  k/j Scroll  g Tail  q Quit"
        self._addnstr(stdscr, 1, 0, help_line, max_x - 1, curses.color_pair(4))
        self._addnstr(stdscr, 2, 0, self.message, max_x - 1, curses.color_pair(3))

        top = 4
        grid_h = max_y - top - 1
        pane_h = grid_h // 2
        pane_w = max_x // 2
        positions = ((top, 0), (top, pane_w), (top + pane_h, 0), (top + pane_h, pane_w))

        for index, pane in enumerate(self.panes):
            y, x = positions[index]
            height = pane_h if index < 2 else max_y - y - 1
            width = pane_w if index % 2 == 0 else max_x - x
            self._draw_pane(stdscr, pane, y, x, height, width, selected=index == self.selected)

        stdscr.refresh()

    def _draw_pane(
        self,
        stdscr: curses.window,
        pane: PaneState,
        y: int,
        x: int,
        height: int,
        width: int,
        selected: bool,
    ) -> None:
        attr = curses.A_BOLD if selected else curses.A_NORMAL
        try:
            stdscr.attron(attr)
            self._box(stdscr, y, x, height, width)
            stdscr.attroff(attr)
        except curses.error:
            return

        status_attr = curses.color_pair(1) if pane.is_running() else curses.color_pair(2)
        title = f" {pane.spec.title} [{pane.status()}] "
        self._addnstr(stdscr, y, x + 2, title, max(0, width - 4), status_attr | curses.A_BOLD)

        inner_w = max(1, width - 4)
        inner_h = max(1, height - 3)
        wrapped = self._wrapped_lines(pane.lines, inner_w)
        end = max(0, len(wrapped) - pane.scrollback)
        start = max(0, end - inner_h)
        visible = wrapped[start:end]

        for row in range(inner_h):
            line = visible[row] if row < len(visible) else ""
            self._addnstr(stdscr, y + 1 + row, x + 2, line, inner_w)

        if pane.scrollback:
            marker = f" scroll +{pane.scrollback} "
            self._addnstr(stdscr, y + height - 1, x + width - len(marker) - 2, marker, len(marker), curses.color_pair(3))

    def _wrapped_lines(self, lines: list[str], width: int) -> list[str]:
        wrapped: list[str] = []
        for line in lines:
            if not line:
                wrapped.append("")
                continue
            chunks = textwrap.wrap(line, width=width, replace_whitespace=False, drop_whitespace=False)
            wrapped.extend(chunks or [""])
        return wrapped

    def _box(self, stdscr: curses.window, y: int, x: int, height: int, width: int) -> None:
        if height < 2 or width < 2:
            return
        stdscr.addch(y, x, curses.ACS_ULCORNER)
        stdscr.addch(y, x + width - 1, curses.ACS_URCORNER)
        stdscr.addch(y + height - 1, x, curses.ACS_LLCORNER)
        stdscr.addch(y + height - 1, x + width - 1, curses.ACS_LRCORNER)
        for col in range(x + 1, x + width - 1):
            stdscr.addch(y, col, curses.ACS_HLINE)
            stdscr.addch(y + height - 1, col, curses.ACS_HLINE)
        for row in range(y + 1, y + height - 1):
            stdscr.addch(row, x, curses.ACS_VLINE)
            stdscr.addch(row, x + width - 1, curses.ACS_VLINE)

    def _addnstr(self, win: curses.window, y: int, x: int, text: str, limit: int, attr: int = 0) -> None:
        if limit <= 0:
            return
        try:
            win.addnstr(y, x, text, limit, attr)
        except curses.error:
            pass


def is_inside_container() -> bool:
    if os.path.exists("/.dockerenv"):
        return True
    try:
        with open("/proc/1/cgroup", "r", encoding="utf-8") as cgroup:
            return any(token in cgroup.read() for token in ("docker", "containerd"))
    except OSError:
        return False


def resolve_mode(requested: str) -> str:
    if requested == "auto":
        return "container" if is_inside_container() else "host"
    return requested


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("auto", "host", "container"),
        default="auto",
        help="Launch mode: host uses docker exec, container runs scripts directly (default: auto)",
    )
    parser.add_argument(
        "--container",
        default=DEFAULT_CONTAINER,
        help=f"Docker container name (default: {DEFAULT_CONTAINER})",
    )
    parser.add_argument(
        "--domain-id",
        default=DEFAULT_DOMAIN_ID,
        help=f"ROS_DOMAIN_ID passed to environment.sh (default: {DEFAULT_DOMAIN_ID})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mode = resolve_mode(args.mode)
    app = QuickStartTui(container=args.container, domain_id=args.domain_id, mode=mode)
    app.run()


if __name__ == "__main__":
    main()
