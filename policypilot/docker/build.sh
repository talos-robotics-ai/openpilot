#!/usr/bin/env bash
# Build the policypilot Docker image.
#
# Context is the policypilot/ repo root, so the Dockerfile can COPY
# policy_runtime/ and docker/vendor/ directly out of the source tree.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POLICYPILOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_TAG="${POLICYPILOT_IMAGE_TAG:-policypilot:latest}"

echo "[build] context = ${POLICYPILOT_DIR}"
echo "[build] image   = ${IMAGE_TAG}"

docker build -t "${IMAGE_TAG}" -f "${SCRIPT_DIR}/Dockerfile" "${POLICYPILOT_DIR}"
