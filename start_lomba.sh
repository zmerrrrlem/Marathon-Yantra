#!/bin/bash
# Wrapper to launch start_lomba.py
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/start_lomba.py"
