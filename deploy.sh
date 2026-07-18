#!/usr/bin/env bash
# Build the app (assumes the QNX SDP env is already sourced:
#   source ~/qnx800/qnxsdp-env.sh) and deploy + run it on the Pi.
#
# Usage:
#   TARGET_HOST=<pi-ip> ./deploy.sh [app args, e.g. --probe]
set -euo pipefail

: "${TARGET_HOST:?set TARGET_HOST to the Pi's IP or hostname}"

DEST=/data/home/qnxuser/bathat
BIN=nto/aarch64/o.le/bathat

make
[ -f "$BIN" ] || { echo "build produced no binary at $BIN" >&2; exit 1; }

ssh "qnxuser@${TARGET_HOST}" "mkdir -p ${DEST}/bin"
scp "$BIN" "qnxuser@${TARGET_HOST}:${DEST}/bin/bathat"
echo "deployed to ${TARGET_HOST}:${DEST}/bin/bathat"

# shellcheck disable=SC2029  # we intentionally expand args client-side
ssh "qnxuser@${TARGET_HOST}" "cd ${DEST} && ./bin/bathat $*"
