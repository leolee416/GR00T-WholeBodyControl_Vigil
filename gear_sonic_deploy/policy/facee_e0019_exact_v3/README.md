# FaceE E0019 exact-v3（仿真 opt-in）

该目录是 E0019 exact-v3 最终 reference 集合配套的 E0017 iteration-8 共享 actor。

- 一个共享 actor，18 条 reference 共用同一 encoder/decoder；
- 不输入 height map、视觉、椅子位姿、距离或 reference selector；
- `encoder_mode_4` 是原 SONIC 的 g1/teleop/smpl 输入模态标记，不是距离 selector；
- 启动 history 契约为 `repeat_reset`；
- 不替换 `policy/release` 或 `facee_v73_noheight`，必须显式选择；
- 当前只授权仿真验证，不据此授权真机运行。

配套 reference catalog：
`gear_sonic/vigil_bridge/data/facee_chair_13s_exact_v3/manifest.json`。

配套 MuJoCo 场景：
`gear_sonic/data/assets/robot_description/mjcf/scene_facee_training_primitives.xml`。

两个 ONNX 是可移植且由 Git LFS 管理的策略输入。`*.trt` 是首次运行时根据当前
TensorRT/GPU 生成的本机缓存，不提交；`Init Done` 前不要发送 reference。
