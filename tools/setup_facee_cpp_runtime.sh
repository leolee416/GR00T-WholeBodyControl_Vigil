#!/usr/bin/env bash
# Source this file before building or running the FaceE C++ deploy process.

facee_runtime_default="/workspace/fangs1@xiaopeng.com/workspace_fs/sonic_cpp_runtime_x86_64_20260803"
export FACEE_CPP_RUNTIME_ROOT="${FACEE_CPP_RUNTIME_ROOT:-${facee_runtime_default}}"
export TensorRT_ROOT="${FACEE_CPP_RUNTIME_ROOT}/tensorrt-10.13.3/sdk"
export onnxruntime_ROOT="${FACEE_CPP_RUNTIME_ROOT}/onnxruntime-linux-x64-1.16.3"
export FACEE_PLANNER_ONNX="${FACEE_CPP_RUNTIME_ROOT}/planner/target_vel/V2/planner_sonic.onnx"
export FACEE_CYCLONEDDS_PREFIX="${FACEE_CPP_RUNTIME_ROOT}/cyclonedds-0.10.2-prefix"
export FACEE_CYCLONEDDS_PYTHONPATH="${FACEE_CPP_RUNTIME_ROOT}/cyclonedds-python-0.10.2-py311"
export CUDAToolkit_ROOT="${CUDAToolkit_ROOT:-/usr/local/cuda}"
export CUDA_HOME="${CUDA_HOME:-${CUDAToolkit_ROOT}}"
export HAS_ROS2="${HAS_ROS2:-0}"

facee_runtime_libs="${TensorRT_ROOT}/lib:${onnxruntime_ROOT}/lib:${FACEE_CYCLONEDDS_PREFIX}/lib:${CUDAToolkit_ROOT}/lib64"
export LD_LIBRARY_PATH="${facee_runtime_libs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

facee_runtime_missing=0
for facee_runtime_file in \
  "${TensorRT_ROOT}/include/NvInfer.h" \
  "${TensorRT_ROOT}/lib/libnvinfer.so.10" \
  "${TensorRT_ROOT}/lib/libnvonnxparser.so.10" \
  "${onnxruntime_ROOT}/include/onnxruntime_cxx_api.h" \
  "${onnxruntime_ROOT}/lib/libonnxruntime.so" \
  "${FACEE_CYCLONEDDS_PREFIX}/lib/libddsc.so" \
  "${FACEE_CYCLONEDDS_PYTHONPATH}/cyclonedds/_clayer.cpython-311-x86_64-linux-gnu.so" \
  "${FACEE_PLANNER_ONNX}"; do
  if [[ ! -e "${facee_runtime_file}" ]]; then
    echo "Missing FaceE C++ runtime dependency: ${facee_runtime_file}" >&2
    facee_runtime_missing=1
  fi
done

if [[ "${facee_runtime_missing}" -ne 0 ]]; then
  return 1 2>/dev/null || exit 1
fi

unset facee_runtime_default facee_runtime_libs facee_runtime_file facee_runtime_missing
