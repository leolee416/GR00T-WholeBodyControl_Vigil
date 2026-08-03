#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sonic_python="${FACEE_SONIC_PYTHON:-/opt/conda/envs/sonic/bin/python}"
facee_mujoco_pythonpath="${FACEE_MUJOCO_PYTHONPATH:-/workspace/fangs1@xiaopeng.com/workspace_fs/sit_chair/.runtime/mujoco_sit_python}"
distance_m="${2:-1.50}"
scene_rel="gear_sonic/data/assets/robot_description/mjcf/scene_facee_training_primitives.xml"
catalog_rel="gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json"
policy_prefix="policy/facee_e0019_exact_v3/model"
policy_obs="policy/facee_e0019_exact_v3/observation_config.yaml"
cpp_runtime_setup="${repo_root}/tools/setup_facee_cpp_runtime.sh"

usage() {
  echo "Usage: $0 build|scene|dds-probe|deploy|bridge|request [distance_m]"
  echo ""
  echo "Run scene, deploy, and bridge in three terminals, then request in a fourth."
  echo "This helper always selects MuJoCo/sim; it never starts real-robot mode."
}

if [[ ! -x "${sonic_python}" ]]; then
  echo "FACEE_SONIC_PYTHON is not executable: ${sonic_python}" >&2
  exit 1
fi

case "${1:-}" in
  build)
    source "${cpp_runtime_setup}"
    cd "${repo_root}/gear_sonic_deploy"
    exec just build
    ;;
  scene)
    source "${cpp_runtime_setup}"
    if [[ ! -d "${facee_mujoco_pythonpath}/mujoco" ]]; then
      echo "Pinned MuJoCo Python tree not found: ${facee_mujoco_pythonpath}" >&2
      echo "Set FACEE_MUJOCO_PYTHONPATH to a mujoco==3.3.4 target directory." >&2
      exit 1
    fi
    export PYTHONPATH="${facee_mujoco_pythonpath}:${FACEE_CYCLONEDDS_PYTHONPATH}:${repo_root}/external_dependencies/unitree_sdk2_python:${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"
    mujoco_version="$("${sonic_python}" -c 'import mujoco; print(mujoco.__version__)')"
    if [[ "${mujoco_version}" != "3.3.4" ]]; then
      echo "FaceE E0019 repository validation requires mujoco==3.3.4; got ${mujoco_version}." >&2
      exit 1
    fi
    cd "${repo_root}"
    exec "${sonic_python}" -u gear_sonic/scripts/run_sim_loop.py \
      --interface lo \
      --robot-scene "${scene_rel}" \
      --facee-chair-distance-m "${distance_m}" \
      --facee-chair-catalog "${catalog_rel}" \
      --no-enable-elastic-band \
      --no-auto-reset-on-fall \
      --wait-for-low-cmd-before-step \
      --wait-for-policy-control-before-step \
      --facee-initialize-reference-pose \
      --facee-preserve-authored-world-frame \
      --facee-policy-lockstep-decimation 4 \
      --no-with-hands \
      --no-enable-onscreen \
      --no-enable-offscreen
    ;;
  dds-probe)
    source "${cpp_runtime_setup}"
    probe_dir="${repo_root}/gear_sonic_deploy/target/release"
    probe_binary="${probe_dir}/dds_lowstate_probe"
    mkdir -p "${probe_dir}"
    g++ -std=c++17 -O2 "${repo_root}/tools/dds_lowstate_probe.cpp" \
      -I"${repo_root}/gear_sonic_deploy/thirdparty/unitree_sdk2/include" \
      -I"${repo_root}/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/include/ddscxx" \
      -I"${repo_root}/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/include" \
      -L"${repo_root}/gear_sonic_deploy/thirdparty/unitree_sdk2/lib/x86_64" \
      -L"${repo_root}/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64" \
      -l:libunitree_sdk2.a -lddscxx -lddsc -lpthread \
      -o "${probe_binary}"
    exec "${probe_binary}" lo 5
    ;;
  deploy)
    source "${cpp_runtime_setup}"
    cd "${repo_root}/gear_sonic_deploy"
    deploy_binary="target/release/g1_deploy_onnx_ref"
    if [[ ! -x "${deploy_binary}" ]]; then
      echo "Missing ${deploy_binary}; run '$0 build' first." >&2
      exit 1
    fi
    exec "${deploy_binary}" lo \
      "${policy_prefix}_decoder.onnx" \
      reference/example/ \
      --obs-config "${policy_obs}" \
      --encoder-file "${policy_prefix}_encoder.onnx" \
      --planner-file "${FACEE_PLANNER_ONNX}" \
      --input-type zmq_manager \
      --output-type all \
      --zmq-host localhost \
      --disable-crc-check
    ;;
  bridge)
    source "${cpp_runtime_setup}"
    export PYTHONPATH="${FACEE_CYCLONEDDS_PYTHONPATH}:${repo_root}/external_dependencies/unitree_sdk2_python:${repo_root}${PYTHONPATH:+:${PYTHONPATH}}"
    cd "${repo_root}"
    exec "${sonic_python}" -u gear_sonic_deploy/scripts/run_vigil_bridge.py \
      --backend mujoco \
      --runtime-mode mujoco \
      --host 127.0.0.1 \
      --port 8765 \
      --command-bind-host 127.0.0.1 \
      --command-port 5556 \
      --state-host 127.0.0.1 \
      --state-port 5557 \
      --state-timeout 10 \
      --odom-source dds \
      --dds-interface lo \
      --chair-motion-catalog "${catalog_rel}"
    ;;
  request)
    "${sonic_python}" - "${distance_m}" <<'PY'
import json
import sys
import urllib.request

distance_m = float(sys.argv[1])
payload = {
    "episode_id": "facee_e0019_repo_mujoco",
    "step_id": 1,
    "runtime_mode": "mujoco",
    "skill_name": "sonic.sit_chair",
    "arguments": {"chair_distance_m": distance_m},
    "safety": {},
}
request = urllib.request.Request(
    "http://127.0.0.1:8765/execute_action",
    data=json.dumps(payload).encode("utf-8"),
    headers={"Content-Type": "application/json"},
    method="POST",
)
print(urllib.request.urlopen(request, timeout=20).read().decode("utf-8"))
PY
    ;;
  *)
    usage
    exit 2
    ;;
esac
