#!/usr/bin/env bash
#
# Installs the /dev/rf udev rule and reloads udev.
#
# Run it from the package directory, or from an installed workspace:
#   sudo $(ros2 pkg prefix rf_joy)/share/rf_joy/udev/install_udev_rules.sh
#
# It is idempotent: running it again just overwrites the rule and reloads.

set -euo pipefail

RULE_NAME="72-ibus-rf.rules"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_RULE="${SOURCE_DIR}/${RULE_NAME}"
TARGET_RULE="/etc/udev/rules.d/${RULE_NAME}"
# 99-ibus-rf.rules로 먼저 깔았던 흔적이 있으면 지운다(같은 SYMLINK 규칙이 두 번 돈다).
STALE_RULE="/etc/udev/rules.d/99-ibus-rf.rules"

if [[ ! -f "${SOURCE_RULE}" ]]; then
  echo "error: ${SOURCE_RULE} not found" >&2
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  echo "error: run this with sudo" >&2
  exit 1
fi

if [[ -f "${STALE_RULE}" ]]; then
  rm -f "${STALE_RULE}"
  echo "removed stale ${STALE_RULE}"
fi

install -m 0644 "${SOURCE_RULE}" "${TARGET_RULE}"
echo "installed ${TARGET_RULE}"

udevadm control --reload-rules
# usb-serial까지 같이 걸어야 FTDI latency_timer 규칙이 다시 적용된다.
udevadm trigger --subsystem-match=tty --subsystem-match=usb-serial
echo "udev rules reloaded"

# The trigger is asynchronous, so give the symlink a moment before reporting on it.
udevadm settle --timeout=5 || true

if [[ -e /dev/rf ]]; then
  echo "ok: /dev/rf -> $(readlink -f /dev/rf)"
  ls -l /dev/rf
else
  echo
  echo "/dev/rf does not exist yet. Check that the adapter is plugged in and that its" >&2
  echo "vendor and product ids match a rule in ${RULE_NAME}:" >&2
  echo >&2
  lsusb >&2 || true
  echo >&2
  ls -l /dev/serial/by-id/ >&2 2>/dev/null || echo "  (no /dev/serial/by-id entries)" >&2
  exit 1
fi
