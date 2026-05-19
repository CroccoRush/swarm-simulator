#!/usr/bin/env bash
# Unified swarm simulator launcher (see lib.sh for behavior).

SWARM_SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SWARM_SIM_ROOT
export SSIM_ME="${0##*/}"
cd "$SWARM_SIM_ROOT"

# shellcheck source=lib.sh
source "$SWARM_SIM_ROOT/lib.sh"
ssim_main "$@"
