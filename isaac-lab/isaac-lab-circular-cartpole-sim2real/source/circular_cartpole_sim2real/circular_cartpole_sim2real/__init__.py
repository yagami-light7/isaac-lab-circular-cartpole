# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Python module serving as a project/extension template.
"""

def _is_optional_runtime_dependency_error(exc: ModuleNotFoundError) -> bool:
	missing_name = getattr(exc, "name", "") or ""
	return missing_name.split(".", 1)[0] in {"pxr", "omni", "isaacsim"}


try:
	# Register Gym environments.
	from .tasks import *
except ModuleNotFoundError as exc:
	if not _is_optional_runtime_dependency_error(exc):
		raise

try:
	# Register UI extensions.
	from .ui_extension_example import *
except ModuleNotFoundError as exc:
	if not _is_optional_runtime_dependency_error(exc):
		raise
