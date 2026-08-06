#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="/var/lib/containerd"
TARGET_DIR="${1:-/data/containerd}"
CONFIG_FILE="/etc/containerd/config.toml"
BACKUP_DIR="${SOURCE_DIR}.before-data-move"
CONFIG_BACKUP="${CONFIG_FILE}.before-data-move"
MIGRATION_COMPLETE=0
HAD_CONFIG=0

if [ "${EUID}" -ne 0 ]; then
  echo "Run this script with sudo:" >&2
  echo "  sudo $0 ${TARGET_DIR}" >&2
  exit 1
fi

for command_name in containerd docker findmnt rsync systemctl; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Required command not found: ${command_name}" >&2
    exit 1
  fi
done

if [ "${TARGET_DIR}" = "${SOURCE_DIR}" ] || [[ "${TARGET_DIR}" != /data/* ]]; then
  echo "Target must be a directory below /data: ${TARGET_DIR}" >&2
  exit 1
fi

if ! findmnt -rn -T /data -o TARGET | grep -qx /data; then
  echo "/data is not a separate mounted filesystem; migration aborted." >&2
  exit 1
fi

if [ -e "${BACKUP_DIR}" ] || [ -e "${CONFIG_BACKUP}" ]; then
  echo "A previous migration backup already exists. Resolve it first:" >&2
  echo "  ${BACKUP_DIR}" >&2
  echo "  ${CONFIG_BACKUP}" >&2
  exit 1
fi

if [ -d "${TARGET_DIR}" ] && [ -n "$(find "${TARGET_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
  echo "Target directory is not empty: ${TARGET_DIR}" >&2
  exit 1
fi

rollback() {
  if [ "${MIGRATION_COMPLETE}" -eq 1 ]; then
    return
  fi

  echo "Migration failed; restoring the previous containerd configuration." >&2
  systemctl stop docker.service docker.socket containerd.service >/dev/null 2>&1 || true

  if [ "${HAD_CONFIG}" -eq 1 ] && [ -f "${CONFIG_BACKUP}" ]; then
    mv "${CONFIG_BACKUP}" "${CONFIG_FILE}"
  else
    rm -f "${CONFIG_FILE}"
  fi

  if [ -d "${BACKUP_DIR}" ]; then
    rm -rf "${SOURCE_DIR}"
    mv "${BACKUP_DIR}" "${SOURCE_DIR}"
  fi

  systemctl start containerd.service docker.service >/dev/null 2>&1 || true
}
trap rollback EXIT

echo "[1/6] Stopping Docker and containerd"
systemctl stop docker.service docker.socket
systemctl stop containerd.service

echo "[2/6] Copying containerd data to ${TARGET_DIR}"
mkdir -p "${TARGET_DIR}"
rsync -aHAX --numeric-ids "${SOURCE_DIR}/" "${TARGET_DIR}/"

echo "[3/6] Writing containerd configuration"
mkdir -p "$(dirname "${CONFIG_FILE}")"
if [ -f "${CONFIG_FILE}" ]; then
  HAD_CONFIG=1
  cp -a "${CONFIG_FILE}" "${CONFIG_BACKUP}"
fi
containerd config default | sed "s#^root = .*#root = '${TARGET_DIR}'#" >"${CONFIG_FILE}.tmp"
install -m 0644 "${CONFIG_FILE}.tmp" "${CONFIG_FILE}"
rm -f "${CONFIG_FILE}.tmp"

echo "[4/6] Preserving the old data for rollback"
mv "${SOURCE_DIR}" "${BACKUP_DIR}"

echo "[5/6] Starting services and verifying Docker"
systemctl start containerd.service
systemctl start docker.service
docker info >/dev/null

CONFIGURED_ROOT="$(containerd config dump | sed -n "s/^root = '\(.*\)'/\1/p" | head -n 1)"
if [ "${CONFIGURED_ROOT}" != "${TARGET_DIR}" ]; then
  echo "containerd is using '${CONFIGURED_ROOT}', expected '${TARGET_DIR}'." >&2
  exit 1
fi

echo "[6/6] Removing the verified old copy from the root filesystem"
rm -rf "${BACKUP_DIR}"
rm -f "${CONFIG_BACKUP}"
MIGRATION_COMPLETE=1
trap - EXIT

echo "containerd storage migrated successfully: ${TARGET_DIR}"
df -h / /data
docker system df
