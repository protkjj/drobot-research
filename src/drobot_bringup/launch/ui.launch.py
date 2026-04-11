#!/usr/bin/env python3
"""
UI launch file.

Usage:
  ros2 launch drobot_bringup ui.launch.py
"""

import sys
import os
import signal
import subprocess
import shutil
import tempfile
import atexit
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

_spawned_procs: list[subprocess.Popen] = []
_shutdown_requested = False


def _kill_gazebo_processes():
    """Kill any remaining Gazebo simulator processes."""
    gazebo_patterns = [
        "gzserver",
        "gzclient",
        "gz sim",
        "ign gazebo",
        "parameter_bridge",
        "ros2 launch drobot_bringup navigation.launch.py",
        "ros2 launch drobot_bringup perception.launch.py",
        "ros2 run drobot_controller teleop_keyboard",
    ]
    for pattern in gazebo_patterns:
        try:
            subprocess.run(
                ["pkill", "-f", pattern],
                capture_output=True,
                timeout=5,
            )
        except Exception:
            pass


def _cleanup():
    """Kill processes spawned by this launch file and Gazebo processes."""
    for proc in _spawned_procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                pass

    _kill_gazebo_processes()

    # Give processes a moment to shut down gracefully, then force-kill survivors
    import time
    time.sleep(2)

    for proc in _spawned_procs:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                pass

    # Force-kill any stubborn Gazebo processes
    for pattern in ["gzserver", "gzclient", "gz sim", "ign gazebo"]:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", pattern],
                capture_output=True,
                timeout=5,
            )
        except Exception:
            pass

    # Final cleanup: kill all related ROS/Gazebo processes
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "gz|rviz|nav2|slam|ekf|goal"],
            capture_output=True,
            timeout=5,
        )
    except Exception:
        pass

    _spawned_procs.clear()


def load_world_files(worlds_root: Path):
    """Return sorted world file paths relative to worlds_root."""
    if not worlds_root.exists():
        return []

    world_files = []
    for pattern in ("**/*.sdf", "**/*.world"):
        world_files.extend(worlds_root.glob(pattern))

    return sorted(
        p.relative_to(worlds_root).as_posix()
        for p in world_files
        if p.is_file()
    )


def resolve_worlds_root() -> Path:
    """Resolve drobot worlds path in install/source environments."""
    try:
        return Path(get_package_share_directory("drobot_description")) / "worlds" / "original"
    except Exception:
        # Fallback for direct source execution
        return Path(__file__).resolve().parents[3] / "src" / "drobot_description" / "worlds" / "original"


def resolve_world_generator_script() -> Path:
    """Resolve world_generator.py in install/source environments."""
    try:
        return Path(get_package_share_directory("drobot_description")) / "scripts" / "world_generator.py"
    except Exception:
        return Path(__file__).resolve().parents[3] / "src" / "drobot_description" / "scripts" / "world_generator.py"


def resolve_workspace_dir() -> Path:
    """Resolve ros2 workspace directory."""
    try:
        # Installed path example:
        # <ws>/install/drobot_bringup/share/drobot_bringup
        share_dir = Path(get_package_share_directory("drobot_bringup")).resolve()
        for parent in share_dir.parents:
            if parent.name == "install":
                return parent.parent
    except Exception:
        pass

    # Source path fallback:
    # <ws>/src/drobot_bringup/launch/ui.launch.py
    return Path(__file__).resolve().parents[3]


def resolve_bringup_launch_dir() -> Path:
    """Resolve drobot_bringup launch directory in install/source environments."""
    try:
        return Path(get_package_share_directory("drobot_bringup")) / "launch"
    except Exception:
        return Path(__file__).resolve().parents[2] / "launch"


def load_option_launch_files(launch_dir: Path) -> list[str]:
    """List launch files to expose as options, excluding ui.launch.py."""
    if not launch_dir.exists():
        return []
    files = sorted(
        p.name for p in launch_dir.glob("*.launch.py")
        if p.is_file() and p.name != "ui.launch.py"
    )
    return files


def launch_in_terminator(commands: list[tuple[str, str]]):
    """Launch all commands in one Terminator window with split panes."""
    n = len(commands)
    if n == 0:
        return

    # Write each command to a temp script
    script_paths = []
    for title, cmd in commands:
        fd, path = tempfile.mkstemp(suffix=".sh", prefix="drobot_")
        with os.fdopen(fd, "w") as f:
            f.write("#!/bin/bash -l\n")
            f.write(f"{cmd}\n")
            f.write("exec bash\n")
        os.chmod(path, 0o755)
        script_paths.append(path)

    # Build Terminator config — custom_command goes in profiles, layout references profiles
    cfg = [
        "[global_config]",
        "  dbus = False",
        "  suppress_multiple_term_dialog = True",
        "[keybindings]",
        "[profiles]",
    ]

    # First command uses the default profile, rest get named profiles
    for i in range(n):
        profile_name = "default" if i == 0 else f"drobot_{i}"
        cfg += [
            f"  [[{profile_name}]]",
            "    use_custom_command = True",
            f"    custom_command = {script_paths[i]}",
        ]

    cfg += [
        "[layouts]",
        "  [[default]]",
        "    [[[window0]]]",
        "      type = Window",
        '      parent = ""',
    ]

    if n == 1:
        cfg += [
            "    [[[term0]]]",
            "      type = Terminal",
            "      parent = window0",
            "      profile = default",
        ]
    elif n == 2:
        cfg += [
            "    [[[child1]]]",
            "      type = HPaned",
            "      parent = window0",
        ]
        for i in range(2):
            profile = "default" if i == 0 else f"drobot_{i}"
            cfg += [
                f"    [[[term{i}]]]",
                "      type = Terminal",
                "      parent = child1",
                f"      profile = {profile}",
            ]
    elif n == 3:
        cfg += [
            "    [[[child1]]]",
            "      type = VPaned",
            "      parent = window0",
            "    [[[child2]]]",
            "      type = HPaned",
            "      parent = child1",
        ]
        for i in range(2):
            profile = "default" if i == 0 else f"drobot_{i}"
            cfg += [
                f"    [[[term{i}]]]",
                "      type = Terminal",
                "      parent = child2",
                f"      profile = {profile}",
            ]
        cfg += [
            "    [[[term2]]]",
            "      type = Terminal",
            "      parent = child1",
            "      profile = drobot_2",
        ]
    else:  # 4+: 2x2 grid
        cfg += [
            "    [[[child1]]]",
            "      type = VPaned",
            "      parent = window0",
            "    [[[child2]]]",
            "      type = HPaned",
            "      parent = child1",
            "    [[[child3]]]",
            "      type = HPaned",
            "      parent = child1",
        ]
        for i in range(min(n, 4)):
            parent = "child2" if i < 2 else "child3"
            profile = "default" if i == 0 else f"drobot_{i}"
            cfg += [
                f"    [[[term{i}]]]",
                "      type = Terminal",
                f"      parent = {parent}",
                f"      profile = {profile}",
            ]

    cfg.append("[plugins]")
    config_text = "\n".join(cfg) + "\n"

    fd, config_path = tempfile.mkstemp(suffix=".cfg", prefix="drobot_terminator_")
    with os.fdopen(fd, "w") as f:
        f.write(config_text)

    proc = subprocess.Popen(
        ["terminator", "-g", config_path, "--maximise"],
        start_new_session=True,
    )
    _spawned_procs.append(proc)


def run_ui():
    """Run Tkinter UI app."""
    atexit.register(_cleanup)

    def _request_shutdown(signum=None, frame=None):
        global _shutdown_requested
        _shutdown_requested = True

    signal.signal(signal.SIGTERM, _request_shutdown)
    signal.signal(signal.SIGINT, _request_shutdown)
    window_width = 1320
    window_height = 560
    header_pady = (0, 10)
    value_pady = (0, 10)
    button_pady = 3

    root = tk.Tk()
    root.title("Start UI")
    root.geometry(f"{window_width}x{window_height}")
    root.resizable(False, False)

    def _poll_shutdown():
        if _shutdown_requested:
            _cleanup()
            root.destroy()
            return
        root.after(200, _poll_shutdown)

    root.after(200, _poll_shutdown)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=0, minsize=300)  # worlds
    root.grid_columnconfigure(1, weight=0, minsize=8)    # sep1
    root.grid_columnconfigure(2, weight=0, minsize=0)    # unused spacer
    root.grid_columnconfigure(3, weight=0, minsize=200)  # goals
    root.grid_columnconfigure(4, weight=0, minsize=8)    # sep2
    root.grid_columnconfigure(5, weight=0, minsize=220)  # options
    root.grid_columnconfigure(6, weight=0, minsize=584)  # summary

    worlds_root = resolve_worlds_root()
    world_files = load_world_files(worlds_root)
    world_generator_script = resolve_world_generator_script()
    workspace_dir = resolve_workspace_dir()
    bringup_launch_dir = resolve_bringup_launch_dir()
    option_files = load_option_launch_files(bringup_launch_dir)

    worlds1 = ttk.Frame(root, padding="10")
    worlds1.grid(row=0, column=0, sticky="nsew")
    worlds1.grid_propagate(False)
    ttk.Label(worlds1, text="Worlds").pack(anchor="w", pady=header_pady)

    summary_col = ttk.Frame(root)
    summary_col.grid(row=0, column=6, sticky="nsew")
    summary_col.grid_propagate(False)
    summary_col.grid_rowconfigure(0, weight=1)
    summary_col.grid_columnconfigure(1, weight=1)

    # Thicker divider between the last column and the rest of the UI.
    summary_sep = tk.Frame(summary_col, width=4, bg="#808080")
    summary_sep.grid(row=0, column=0, sticky="ns")

    summary_content = ttk.Frame(summary_col, padding="10")
    summary_content.grid(row=0, column=1, sticky="nsew")
    ttk.Label(summary_content, text="Selection").pack(anchor="w", pady=header_pady)

    selected_world = tk.StringVar(value="(none)")
    selected_goal = tk.StringVar(value="(none)")
    selected_option = tk.StringVar(value="(none)")
    ttk.Label(summary_content, text="Selected:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_world).pack(anchor="w", pady=value_pady)
    ttk.Label(summary_content, text="Goal:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_goal).pack(anchor="w", pady=value_pady)
    ttk.Label(summary_content, text="Option:").pack(anchor="w")
    ttk.Label(summary_content, textvariable=selected_option).pack(anchor="w", pady=value_pady)

    next_btn = ttk.Button(summary_content, text="Next")

    def run_in_new_terminal(launch_cmd: str):
        """Run a shell command in a new terminal if available."""

        # Prefer opening in a real terminal so launch logs are visible.
        terminal = None
        for candidate in ("gnome-terminal", "konsole", "xfce4-terminal", "mate-terminal", "xterm"):
            if shutil.which(candidate):
                terminal = candidate
                break

        proc = None
        if terminal == "gnome-terminal":
            proc = subprocess.Popen(
                [terminal, "--", "bash", "-lc", f"{launch_cmd}; exec bash"],
                start_new_session=True,
            )
        elif terminal == "konsole":
            proc = subprocess.Popen(
                [terminal, "-e", "bash", "-lc", f"{launch_cmd}; exec bash"],
                start_new_session=True,
            )
        elif terminal == "xfce4-terminal":
            proc = subprocess.Popen(
                [terminal, "--command", f"bash -lc '{launch_cmd}; exec bash'"],
                start_new_session=True,
            )
        elif terminal == "mate-terminal":
            proc = subprocess.Popen(
                [terminal, "--", "bash", "-lc", f"{launch_cmd}; exec bash"],
                start_new_session=True,
            )
        elif terminal == "xterm":
            proc = subprocess.Popen(
                [terminal, "-e", "bash", "-lc", f"{launch_cmd}; exec bash"],
                start_new_session=True,
            )
        else:
            # Fallback: detached background shell.
            proc = subprocess.Popen(
                ["bash", "-lc", launch_cmd],
                start_new_session=True,
            )

        if proc:
            _spawned_procs.append(proc)

    def launch_selected_stack(output_world_name: str, selected_options: list[str]):
        """Launch navigation and optionally controller/perception based on selected options."""
        world_arg = Path(output_world_name).stem

        # Collect all commands to launch
        commands = []

        nav_cmd = (
            f"cd {workspace_dir} && "
            "source install/setup.bash && "
            f"ros2 launch drobot_bringup navigation.launch.py world:={world_arg}"
        )
        commands.append(("Navigation", nav_cmd))
        print(f"Launching generated world: {world_arg}", flush=True)

        teleop_cmd = (
            f"cd {workspace_dir} && "
            "source install/setup.bash && "
            "sleep 5 && ros2 run drobot_controller teleop_keyboard"
        )
        commands.append(("Teleop", teleop_cmd))

        if "perception.launch.py" in selected_options:
            perception_cmd = (
                f"cd {workspace_dir} && "
                "source install/setup.bash && "
                "sleep 3 && ros2 launch drobot_bringup perception.launch.py"
            )
            commands.append(("Perception", perception_cmd))
            print("Launching perception stack", flush=True)

        # Use Terminator split panes if available, else fallback to separate windows
        if shutil.which("terminator"):
            launch_in_terminator(commands)
        else:
            for _, cmd in commands:
                run_in_new_terminal(cmd)

    def parse_output_world_name(stdout_text: str) -> str | None:
        for line in stdout_text.splitlines():
            if line.startswith("OUTPUT_WORLD_NAME="):
                value = line.split("=", 1)[1].strip()
                if value:
                    return value
        return None

    def show_load_world_popup(base_world_name: str, output_world_name: str, selected_options: list[str]):
        """Show completion popup with a real 'Load World' button."""
        popup = tk.Toplevel(root)
        popup.title("World Generated")
        popup.geometry("360x140")
        popup.resizable(False, False)
        popup.transient(root)
        popup.grab_set()

        ttk.Label(
            popup,
            text=f"Launching '{base_world_name}'.",
            padding=(12, 16, 12, 8),
        ).pack(anchor="w")

        ttk.Button(
            popup,
            text="Load World",
            command=lambda: (launch_selected_stack(output_world_name, selected_options), popup.destroy()),
        ).pack(anchor="e", padx=12, pady=(0, 12))

    def create_world():
        base_world_name = selected_world.get()
        base_goal = selected_goal.get()
        selected_options = [
            name for name, var in option_vars.items() if var.get()
        ]
        base_option = ",".join(selected_options) if selected_options else "(none)"
        if base_world_name == "(none)":
            print("Create World skipped: no world selected")
            return
        if base_goal == "(none)":
            print("Create World skipped: no goal selected")
            return
        if base_option == "(none)":
            print("Create World skipped: no option selected")
            return
        requires_navigation = any(
            opt in ("perception.launch.py",)
            for opt in selected_options
        )
        has_navigation = "navigation.launch.py" in selected_options
        if requires_navigation and not has_navigation:
            messagebox.showerror(
                "Invalid Options",
                "controller/perception requires navigation.\n"
                "Please also select navigation.launch.py",
            )
            return
        if not world_generator_script.is_file():
            print(f"Create World skipped: script not found: {world_generator_script}")
            return

        env = os.environ.copy()
        env["DROBOT_WORKSPACE_DIR"] = str(workspace_dir)
        env["DROBOT_BASE_WORLD_NAME"] = base_world_name
        env["DROBOT_BASE_GOALS"] = base_goal
        env["DROBOT_BASE_OPTIONS"] = base_option
        result = subprocess.run(
            [sys.executable, str(world_generator_script)],
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            print("Create World failed")
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(result.stderr)
            return

        output_world_name = parse_output_world_name(result.stdout)
        if not output_world_name:
            print("Create World failed: OUTPUT_WORLD_NAME not found in generator output")
            if result.stdout:
                print(result.stdout)
            return

        show_load_world_popup(base_world_name, output_world_name, selected_options)

    create_world_btn = ttk.Button(summary_content, command=create_world, text="Create World")



    goals_col = ttk.Frame(root, padding="10")
    goals_col.grid_propagate(False)
    ttk.Label(goals_col, text="Goals").pack(anchor="w", pady=header_pady)
    goals_buttons = ttk.Frame(goals_col)
    goals_buttons.pack(fill="both", expand=True)

    for goal_name in ["0", "1", "2", "3", "random"]:
        ttk.Button(
            goals_buttons,
            text=goal_name,
            command=lambda name=goal_name: on_select_goal(name)
        ).pack(fill="x", pady=button_pady)

    options_col = ttk.Frame(root, padding="10")
    options_col.grid_propagate(False)
    ttk.Label(options_col, text="Options").pack(anchor="w", pady=header_pady)
    options_buttons = ttk.Frame(options_col)
    options_buttons.pack(fill="both", expand=True)

    option_vars: dict[str, tk.BooleanVar] = {}

    def refresh_selected_options():
        selected = [name for name, var in option_vars.items() if var.get()]
        if selected:
            selected_option.set(", ".join(selected))
            if not create_world_btn.winfo_ismapped():
                create_world_btn.pack(anchor="w", pady=(button_pady, 0))
        else:
            selected_option.set("(none)")
            create_world_btn.pack_forget()
        next_btn.pack_forget()

    sep2 = ttk.Separator(root, orient="vertical")
    sep2.grid(row=0, column=4, sticky="ns")

    def reset_option_step():
        """Clear selected option and force option re-selection."""
        selected_option.set("(none)")
        for var in option_vars.values():
            var.set(False)
        create_world_btn.pack_forget()
        if options_col.winfo_ismapped():
            options_col.grid_remove()

    if option_files:
        for option_name in option_files:
            var = tk.BooleanVar(value=False)
            option_vars[option_name] = var
            ttk.Checkbutton(
                options_buttons,
                text=option_name,
                variable=var,
                command=refresh_selected_options,
            ).pack(fill="x", pady=button_pady)
    else:
        ttk.Label(
            options_buttons,
            text=f"No launch options found under:\n{bringup_launch_dir}",
            justify="left",
        ).pack(anchor="w")

    def on_next_pressed():
        # Step 1: show goals. Step 2: show options.
        if not goals_col.winfo_ismapped():
            goals_col.grid(row=0, column=3, sticky="nsew")
            next_btn.pack_forget()
            return

        if not options_col.winfo_ismapped():
            options_col.grid(row=0, column=5, sticky="nsew")
            next_btn.pack_forget()

    next_btn.configure(command=on_next_pressed)

    def on_select_world(world_name: str):
        selected_world.set(world_name)
        print(f"Selected world: {world_name}")
        selected_goal.set("(none)")
        reset_option_step()

        if goals_col.winfo_ismapped():
            goals_col.grid_remove()
        
        if not next_btn.winfo_ismapped() and not create_world_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(button_pady, 0))

    def on_select_goal(goal_name: str):
        selected_goal.set(goal_name)
        print(f"Selected goal: {goal_name}")
        reset_option_step()

        if not next_btn.winfo_ismapped():
            next_btn.pack(anchor="w", pady=(4, 0))
    
    worlds_list = ttk.Frame(worlds1)
    worlds_list.pack(fill="both", expand=True)

    if world_files:
        for world_name in world_files:
            ttk.Button(
                worlds_list,
                text=world_name,
                command=lambda name=world_name: on_select_world(name)
            ).pack(fill="x", pady=button_pady)
    else:
        ttk.Label(
            worlds_list,
            text=f"No world files found under:\n{worlds_root}",
            justify="left"
        ).pack(anchor="w")

    sep1 = ttk.Separator(root, orient="vertical")
    sep1.grid(row=0, column=1, sticky="ns")

    root.protocol("WM_DELETE_WINDOW", lambda: (_cleanup(), root.destroy()))
    root.mainloop()


def generate_launch_description():
    """Launch UI in a child Python process."""
    this_file = str(Path(__file__).resolve())
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, this_file, "--run-ui"],
            output="screen",
        ),
    ])


if __name__ == "__main__":
    if "--run-ui" in sys.argv:
        run_ui()
