#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
policy_dir="$repo_root/gear_sonic_deploy/policy/facee_stage1_generalist_yaw"
oss_prefix="oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/facee_stage1_generalist_yaw"
ossutil_bin=${OSSUTIL_BIN:-ossutil64}
ossutil_config=${OSSUTIL_CONFIG:-}
with_pt=false
with_full_catalog=false

while (($#)); do
  case "$1" in
    --ossutil) ossutil_bin=$2; shift 2 ;;
    --config) ossutil_config=$2; shift 2 ;;
    --with-pt) with_pt=true; shift ;;
    --with-full-catalog) with_full_catalog=true; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ossutil_args=()
if [[ -n "$ossutil_config" ]]; then
  ossutil_args+=(--config-file "$ossutil_config")
fi
mkdir -p "$policy_dir"

fetch() {
  local source=$1
  local destination=$2
  "$ossutil_bin" cp "$oss_prefix/$source" "$policy_dir/$destination" \
    --force "${ossutil_args[@]}"
}

fetch model_step_000096_encoder.onnx model_encoder.onnx
fetch model_step_000096_decoder.onnx model_decoder.onnx
if [[ "$with_pt" == true ]]; then
  fetch model_step_000096.pt model_step_000096.pt
fi
if [[ "$with_full_catalog" == true ]]; then
  full_catalog_dir="$repo_root/gear_sonic/vigil_bridge/data/facee_stage1_generalist_yaw_full403"
  mkdir -p "$full_catalog_dir"
  "$ossutil_bin" cp "$oss_prefix/motion_catalog_full403/" "$full_catalog_dir/" \
    --recursive --force "${ossutil_args[@]}"
  python3 - "$full_catalog_dir/manifest.json" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if manifest.get("name") != "facee_stage1_generalist_yaw_full403_sit_only":
    raise SystemExit("unexpected full-catalog name")
if manifest.get("mode") != "sit" or manifest.get("count") != 403:
    raise SystemExit("full catalog must contain exactly 403 Sit motions")
if any("stand" in str(row.get("tag", "")) for row in manifest["motions"]):
    raise SystemExit("full catalog unexpectedly contains Stand motion")
print("full Sit catalog: 403 motions verified")
PY
fi

(
  cd "$policy_dir"
  printf '%s  %s\n' \
    e275f478131651dd6c315386bb6096e707cd8da1195cbc018c85f98e34f331d4 \
    model_encoder.onnx \
    ced9b5afbb7f10c816c6a5166625876db3eca63071c346c7463ac6df4d6fd21f \
    model_decoder.onnx | sha256sum --check --strict
  if [[ "$with_pt" == true ]]; then
    printf '%s  %s\n' \
      dc543f99fb51207727799d9c939cbd6b58e83af354947ddc843dd9ffa913607d \
      model_step_000096.pt | sha256sum --check --strict
  fi
)

echo "FaceE Stage-1 generalist assets are ready in $policy_dir"
