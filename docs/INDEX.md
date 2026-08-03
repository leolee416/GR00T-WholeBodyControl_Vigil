# Documentation Index

This index lists the Codex-facing documentation for this repository.

## Contracts

- `docs/contracts/repo_role.md`  
  Repository role, Vigil/GR00T split, and ownership boundaries.

- `docs/contracts/protected_areas.md`  
  Protected code areas, allowed bridge locations, and editing rules.

- `docs/contracts/bridge_contract.md`  
  Bridge API semantics: what the bridge may and must not return.

- `docs/contracts/safety_rules.md`  
  Real-robot safety rules and hardware execution constraints.

## Integration

- [FaceE 按椅距选择坐下 reference](integration/facee_chair_distance_motion.md)

- [FaceE 真机 reference joint order 根因、源码证据与复现](integration/facee_reference_joint_order_root_cause.md)

- `RUNTIME_BRIDGE_OVERVIEW.md`
  Short runtime bridge overview for external agents/clients and operator lifecycle.

- `docs/integration/vigil_bridge.md`  
  Vigil bridge architecture, runtime modes, and adapter design.

- `docs/integration/vigil_bridge_quickstart.md`  
  Short operator quickstart for SSH, launcher startup, and curl smoke tests.

- `docs/integration/vigil_bridge_interface.md`  
  Current bridge protocol, capabilities, message shapes, and extension table.

- [FaceE E0019 仓库内 MuJoCo 历史诊断](integration/vigil_mujoco_facee_validation_20260803.md)
- [FaceE E0019 模型的 MuJoCo Sim 完整验证流程](integration/facee_e0019_mujoco_sim_validation_workflow.md)

- [Vigil Bridge 真机 Rollout 录制与导出](integration/vigil_rollout_recording.md)

- `docs/integration/g1_audio_io.md`
  Unitree G1 audio input/output debugging notes and future runtime extension
  guidance.

- `docs/integration/g1_audio_bridge_connectivity.md`
  G1/Host-VLT audio bridge connectivity record, startup commands, smoke tests,
  known failures, and speaker/microphone configuration.

- `docs/integration/g1_tts_joint_debug_quickstart.md`
  Short G1-side startup and Host/VLT-side standalone client commands for TTS
  joint debugging.

## Workflows

- `docs/workflows/codex_workflows.md`  
  Codex workflows, including neat-freak and bridge development routines.

## Original Project Documentation

The Sphinx documentation under `docs/source/` remains the original
GR00T/GEAR-SONIC runtime/control documentation. Treat it as robot runtime
documentation, not Vigil benchmark documentation.
