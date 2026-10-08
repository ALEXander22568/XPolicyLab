#!/bin/bash
# RoboShell policy environment: the agent container image, the policy-server dependency (websockets) in the Isaac
# Sim Python, and the RGB-D observation configs. Settings come from config.env next to this file.
#   bash install.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "${SCRIPT_DIR}/config.env" ] || { echo "[INSTALL] copy config.env.example to config.env and fill it in first" >&2; exit 1; }
exec "${SCRIPT_DIR}/roboshell.sh" setup
