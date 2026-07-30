#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
grail_root="${1:-/workspace/fangs1@xiaopeng.com/workspace_fs/GRAIL}"
sonic_python="${SONIC_PYTHON:-/opt/conda/envs/sonic/bin/python}"
checkpoint_rel="out/faceE_sonic_v1_1_noheight_v65/dagger_v69iter100_balanced_freezeenc_fixedlr2e6_30iter_v73/model_step_000025.pt"
checkpoint="${grail_root}/${checkpoint_rel}"
expected_checkpoint_sha="610bc1bd21e30ca9dd9690a5096ddf3f54ca7b3fe7fb3ddf3ed73c65ead049c9"

if [[ ! -x "${sonic_python}" ]]; then
  echo "SONIC Python not found: ${sonic_python}" >&2
  exit 1
fi
if [[ ! -f "${checkpoint}" ]]; then
  echo "Checkpoint not found: ${checkpoint}" >&2
  exit 1
fi
actual_checkpoint_sha="$(sha256sum "${checkpoint}" | awk '{print $1}')"
if [[ "${actual_checkpoint_sha}" != "${expected_checkpoint_sha}" ]]; then
  echo "Checkpoint checksum mismatch: ${actual_checkpoint_sha}" >&2
  exit 1
fi

export ISAACLAB_PYTHON="${sonic_python}"
cd "${grail_root}/imports/SONIC"
"${sonic_python}" -u gear_sonic/eval_agent_trl.py \
  "+checkpoint=${checkpoint}" \
  +headless=True \
  ++num_envs=1 \
  ++export_onnx_only=True \
  ++manager_env.config.robot.type=g1_model_12 \
  ++manager_env.config.terrain_type=plane \
  ++manager_env.commands.motion.use_height_map=False

cd "${repo_root}"
"${sonic_python}" tools/package_facee_v73_noheight_policy.py \
  --grail-root "${grail_root}"
