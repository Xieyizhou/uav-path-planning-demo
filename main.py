import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
ASTAR_FLIGHT_SCRIPT = PROJECT_ROOT / "scripts" / "flight" / "fly_astar_path.py"
ASTAR_ANALYSIS_SCRIPT = PROJECT_ROOT / "scripts" / "analysis" / "analyze_astar_log.py"
from src.logging.output_registry import get_previews_dir

ASTAR_PREVIEW_DIR = get_previews_dir("static_astar") / "as_preview"


HELP_EPILOG = """\
Recommended current workflow:
Terminal A:
  cd ~/projects/drone-ai
  bash scripts/flight/start_px4_substation.sh

Terminal B:
  cd ~/projects/drone-ai
  source .venv/bin/activate
  python main.py astar preview --obstacle-config config/substation_obstacles.json --return-home --altitude 1.5
  python main.py astar fly --obstacle-config config/substation_obstacles.json --return-home --altitude 1.5 --max-speed 0.8 --return-speed-scale 0.7 --waypoint-acceptance 0.3
  python main.py astar analyze --obstacle-config config/substation_obstacles.json

Perception-response comparison:
  python main.py astar fly --obstacle-config config/substation_obstacles.json --return-home --altitude 1.5 --max-speed 1.0 --return-speed-scale 0.8 --waypoint-acceptance 0.4 --enable-perception --risk-action log_only
  python main.py astar fly --obstacle-config config/substation_obstacles.json --return-home --altitude 1.5 --max-speed 1.0 --return-speed-scale 0.8 --waypoint-acceptance 0.4 --enable-perception --risk-action slow_down
  python scripts/analysis/summarize_experiments.py
"""


def display_path(path):
    try:
        return path.resolve().relative_to(PROJECT_ROOT)
    except ValueError:
        return path.resolve()


def say(message=""):
    print(message, flush=True)


def run_subprocess(command, description):
    say(f"\n{description}")
    say("Command:")
    say("  " + " ".join(str(part) for part in command))

    result = subprocess.run(command, cwd=PROJECT_ROOT)
    if result.returncode != 0:
        say(f"\nError: {description.lower()} failed.")
        say(f"Exit code: {result.returncode}")
        sys.exit(result.returncode)


def append_obstacle_config(command, obstacle_config):
    if obstacle_config is not None:
        command.extend(["--obstacle-config", str(obstacle_config)])


def append_common_astar_options(command, args):
    for option_name, flag in [
        ("altitude", "--altitude"),
        ("max_speed", "--max-speed"),
        ("return_speed_scale", "--return-speed-scale"),
        ("waypoint_acceptance", "--waypoint-acceptance"),
        ("waypoint_timeout", "--waypoint-timeout"),
        ("risk_action", "--risk-action"),
        ("min_risk_speed", "--min-risk-speed"),
    ]:
        value = getattr(args, option_name, None)
        if value is not None:
            command.extend([flag, str(value)])
    if getattr(args, "enable_perception", False):
        command.append("--enable-perception")
    if getattr(args, "enable_local_replan", False):
        command.append("--enable-local-replan")
    for option_name, flag in [
        ("replan_mode", "--replan-mode"),
        ("replan_risk_level", "--replan-risk-level"),
        ("replan_cooldown", "--replan-cooldown"),
        ("dynamic_obstacle_inflation", "--dynamic-obstacle-inflation"),
        ("max_replans", "--max-replans"),
    ]:
        value = getattr(args, option_name, None)
        if value is not None:
            command.extend([flag, str(value)])


def add_common_astar_options(parser):
    parser.add_argument(
        "--altitude",
        type=float,
        help="Target altitude above takeoff in meters.",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        help="Maximum horizontal A* speed in m/s.",
    )
    parser.add_argument(
        "--return-speed-scale",
        type=float,
        help="Return route speed multiplier.",
    )
    parser.add_argument(
        "--waypoint-acceptance",
        type=float,
        help="Horizontal waypoint acceptance radius in meters.",
    )
    parser.add_argument(
        "--waypoint-timeout",
        help="Per-waypoint timeout in seconds, or 'auto' to estimate from flight conditions.",
    )
    parser.add_argument(
        "--enable-perception",
        action="store_true",
        help="Enable simulated local obstacle detection in the telemetry log.",
    )
    parser.add_argument(
        "--risk-action",
        choices=["log_only", "slow_down", "stop_and_land"],
        help="Optional behavior for perception risk.",
    )
    parser.add_argument(
        "--min-risk-speed",
        type=float,
        help="Minimum horizontal commanded speed in m/s when slow_down reduces speed.",
    )
    parser.add_argument(
        "--enable-local-replan",
        action="store_true",
        help="Enable local replan attempts when perception risk reaches the threshold.",
    )
    parser.add_argument(
        "--replan-mode",
        choices=["log_only", "active"],
        help="Local replanning mode.",
    )
    parser.add_argument(
        "--replan-risk-level",
        choices=["detected", "warning", "danger"],
        help="Minimum perception risk level that triggers a local replan attempt.",
    )
    parser.add_argument(
        "--replan-cooldown",
        type=float,
        help="Minimum seconds between local replan attempts.",
    )
    parser.add_argument(
        "--dynamic-obstacle-inflation",
        type=int,
        help="Grid-cell inflation applied to perception-derived dynamic obstacles.",
    )
    parser.add_argument(
        "--max-replans",
        type=int,
        help="Maximum local replan attempts per flight.",
    )


def run_astar_preview(args):
    say("Running A* preview.")
    say("This plans the path and saves preview files only.")
    say("Reminder: this does not connect to PX4 and does not fly the drone.")
    say(f"Preview output folder: {display_path(ASTAR_PREVIEW_DIR)}")
    if args.return_home:
        say("Return-home preview enabled: the return route is the reversed A* path.")

    command = [sys.executable, str(ASTAR_FLIGHT_SCRIPT), "--dry-run"]
    append_obstacle_config(command, args.obstacle_config)
    append_common_astar_options(command, args)
    if args.return_home:
        command.append("--return-home")

    run_subprocess(
        command,
        "Starting A* dry-run preview",
    )

    say("\nA* preview complete.")
    say(f"Preview image: {display_path(ASTAR_PREVIEW_DIR / 'grid_path.png')}")
    say(f"Preview JSON: {display_path(ASTAR_PREVIEW_DIR / 'path_preview.json')}")
    say("Reminder: no drone flight was started.")


def run_astar_flight(args):
    say("Starting A* flight.")
    say("Reminder: PX4 SITL must already be running.")
    say("You can use QGroundControl and Gazebo to observe the drone.")
    if args.return_home:
        say("Return-home enabled: the drone will fly the reversed A* path back to start before landing.")

    command = [sys.executable, str(ASTAR_FLIGHT_SCRIPT)]
    append_obstacle_config(command, args.obstacle_config)
    append_common_astar_options(command, args)
    if args.return_home:
        command.append("--return-home")

    run_subprocess(
        command,
        "Starting A* flight script",
    )


def run_astar_analysis(obstacle_config=None):
    say("Running A* analysis.")
    say("The analysis script will use the newest astar_*.csv log by default.")

    command = [sys.executable, str(ASTAR_ANALYSIS_SCRIPT)]
    append_obstacle_config(command, obstacle_config)

    run_subprocess(command, "Starting A* log analysis")


def run_astar_workflow(args):
    run_astar_preview(args)

    answer = input("\nPX4 must already be running. Continue with flight? [y/N] ")
    if answer.strip().lower() not in {"y", "yes"}:
        say("Flight cancelled. Preview files are still available.")
        return

    run_astar_flight(args)
    run_astar_analysis(obstacle_config=args.obstacle_config)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Unified beginner-friendly command line for the drone-ai project.",
        epilog=HELP_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="topic", metavar="command")

    astar_parser = subparsers.add_parser(
        "astar",
        help="Run the current A* planning, flight, and analysis workflow.",
    )
    astar_subparsers = astar_parser.add_subparsers(
        dest="astar_command",
        metavar="action",
    )
    preview_parser = astar_subparsers.add_parser(
        "preview",
        help="Plan the A* path and save preview outputs without flying.",
    )
    preview_parser.add_argument(
        "--return-home",
        action="store_true",
        help="Also preview the reversed A* path back to the start.",
    )
    preview_parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Obstacle config JSON to use for the A* map.",
    )
    add_common_astar_options(preview_parser)

    fly_parser = astar_subparsers.add_parser(
        "fly",
        help="Fly the A* path in PX4 SITL. PX4 must already be running.",
    )
    fly_parser.add_argument(
        "--return-home",
        action="store_true",
        help="Fly the reversed A* path back to the start before landing.",
    )
    fly_parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Obstacle config JSON to use for the A* map.",
    )
    add_common_astar_options(fly_parser)

    analyze_parser = astar_subparsers.add_parser(
        "analyze",
        help="Analyze the newest astar_*.csv log.",
    )
    analyze_parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Obstacle config JSON to overlay and validate against.",
    )

    run_parser = astar_subparsers.add_parser(
        "run",
        help="Run preview, ask for confirmation, fly, then analyze.",
    )
    run_parser.add_argument(
        "--return-home",
        action="store_true",
        help="Preview and fly the reversed A* path back to start before landing.",
    )
    run_parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Obstacle config JSON to use for preview, flight, and analysis.",
    )
    add_common_astar_options(run_parser)

    return parser.parse_args()


def main():
    args = parse_args()

    if args.topic != "astar":
        say("Choose a workflow command. For help, run:")
        say("  python main.py --help")
        sys.exit(2)

    if args.astar_command == "preview":
        run_astar_preview(args)
    elif args.astar_command == "fly":
        run_astar_flight(args)
    elif args.astar_command == "analyze":
        run_astar_analysis(obstacle_config=args.obstacle_config)
    elif args.astar_command == "run":
        run_astar_workflow(args)
    else:
        say("Choose an A* action. For help, run:")
        say("  python main.py astar --help")
        sys.exit(2)


if __name__ == "__main__":
    main()
