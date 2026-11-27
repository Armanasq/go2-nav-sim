#!/bin/bash
# One-time setup for the first-principles modules: a project-local venv with MuJoCo and Pinocchio, and the Go2 MJCF.
set -eo pipefail
cd "$(dirname "$0")/.."
[ -d .venv-modules ] || python3 -m venv .venv-modules
.venv-modules/bin/pip install -q --upgrade pip
.venv-modules/bin/pip install -q mujoco pin numpy scipy matplotlib pytest
modules/fetch_go2.sh
.venv-modules/bin/python -c "import mujoco, pinocchio; print('mujoco', mujoco.__version__, '| pinocchio', pinocchio.__version__)"
