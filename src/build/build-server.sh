#!/usr/bin/env bash
#
# Build nqserver for FrikBotNex.
#
# Uses NexQuake's build system from vendor/NexQuake, overlays our
# bot server code (nav_bot, nav_mesh, net_bot) and patches.
#
set -euo pipefail

FRIKBOT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NQ_ROOT="${FRIKBOT_ROOT}/vendor/NexQuake"

if [[ ! -d "${NQ_ROOT}/build" ]]; then
  echo "error: vendor/NexQuake not found. Run: git submodule update --init" >&2
  exit 1
fi

if [[ ! -d "${FRIKBOT_ROOT}/vendor/recastnavigation/Detour" ]]; then
  echo "error: vendor/recastnavigation not found. Run: git submodule update --init" >&2
  exit 1
fi

BUILD_TMP="${FRIKBOT_ROOT}/src/build/tmp"
OUT_DIR="${BUILD_TMP}/server"
OUT="${OUT:-${BUILD_TMP}/bin/nqserver}"

mkdir -p "$(dirname "${OUT}")" "${OUT_DIR}"

# Use NexQuake's prepare-upstream to set up the scratch dir
export UPSTREAM_QUAKE_DIR="${BUILD_TMP}"
export OUT_DIR
pushd "${NQ_ROOT}" >/dev/null
bash build/prepare-upstream.sh server
popd >/dev/null

# Overlay our bot server code
cp "${FRIKBOT_ROOT}/src/server/net_bot.c" "${FRIKBOT_ROOT}/src/server/net_bot.h" "${OUT_DIR}/"
cp "${FRIKBOT_ROOT}/src/server/nav_bot.c" "${FRIKBOT_ROOT}/src/server/nav_bot.h" "${OUT_DIR}/"
cp "${FRIKBOT_ROOT}/src/server/nav_mesh.h" "${FRIKBOT_ROOT}/src/server/nav_mesh.cpp" "${OUT_DIR}/"

# Apply our patches (on top of NexQuake's)
for patch in "${FRIKBOT_ROOT}"/src/server/*.patch; do
  [[ -f "${patch}" ]] || continue
  echo "  patch: $(basename "${patch}")"
  (cd "${OUT_DIR}" && patch -p0 --forward -r /dev/null < "${patch}") || true
done

# Vendor Recast/Detour
mkdir -p "$(dirname "${OUT_DIR}")/vendor/recastnavigation"
cp -r "${FRIKBOT_ROOT}/vendor/recastnavigation/Recast" "$(dirname "${OUT_DIR}")/vendor/recastnavigation/"
cp -r "${FRIKBOT_ROOT}/vendor/recastnavigation/Detour" "$(dirname "${OUT_DIR}")/vendor/recastnavigation/"

# Build
make_jobs="$(nproc 2>/dev/null || echo 1)"
pushd "${OUT_DIR}" >/dev/null
make -j "${make_jobs}" -f Makefile.dedicated
popd >/dev/null

bin_path="$(ls -1 "${OUT_DIR}"/build-netquake-*/nqserver 2>/dev/null | head -n 1 || true)"
if [[ -z "${bin_path}" ]]; then
  echo "error: nqserver binary not found" >&2
  exit 1
fi

cp -f "${bin_path}" "${OUT}"
chmod +x "${OUT}" || true
echo "Built server: ${OUT}"
