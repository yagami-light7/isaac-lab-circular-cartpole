# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to print available environments registered in Gym."""

"""Launch Isaac Sim Simulator first."""

from isaaclab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app


"""Rest everything follows."""

import gymnasium as gym
from prettytable import PrettyTable
import argparse

import circular_cartpole_sim2real.tasks  # noqa: F401


def main():
    """Print registered environments, optionally filtered by keyword."""
    parser = argparse.ArgumentParser(description="List registered Gym environments.")
    parser.add_argument(
        "--keyword",
        type=str,
        default=None,
        help="Optional keyword to filter task names.",
    )
    parser.add_argument(
        "--only-project",
        action="store_true",
        default=False,
        help="Show only environments registered by circular_cartpole_sim2real.",
    )
    args = parser.parse_args()

    # print all the available environments
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in Isaac Lab"
    # set alignment of table columns
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    # count of environments
    index = 0
    keyword = args.keyword.lower() if args.keyword else None

    # acquire all registered environment names
    for task_spec in gym.registry.values():
        task_name = task_spec.id
        if keyword and keyword not in task_name.lower():
            continue

        entry_point = str(task_spec.entry_point)
        env_cfg_entry = str(task_spec.kwargs.get("env_cfg_entry_point", ""))
        if args.only_project:
            joined_text = f"{task_name} {entry_point} {env_cfg_entry}".lower()
            if "circular_cartpole_sim2real" not in joined_text:
                continue

        env_cfg = task_spec.kwargs.get("env_cfg_entry_point", "-")

        # add details to table
        table.add_row(
            [
                index + 1,
                task_name,
                entry_point,
                env_cfg,
            ]
        )
        # increment count
        index += 1

    print(table)


if __name__ == "__main__":
    try:
        # run the main function
        main()
    except Exception as e:
        raise e
    finally:
        # close the app
        simulation_app.close()
