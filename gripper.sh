#!/usr/bin/env bash
set -euo pipefail
gripper_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${GRIPPER_PYTHON:-}" ]]; then
  gripper_python="$GRIPPER_PYTHON"
elif [[ -x "$gripper_dir/.venv/bin/python" ]]; then
  gripper_python="$gripper_dir/.venv/bin/python"
else
  gripper_python="python3"
fi
exec "$gripper_python" "$gripper_dir/gripper.py" "$@"
