# FaceE v73 no-height candidate policy

This directory packages the FaceE v73 student actor as a deployment-format
encoder/decoder ONNX pair.

- Architecture: one shared actor, one encoder and one decoder. There is no
  distance expert bank, distance-conditioned route, or v119 three-way selector.
- Distance handling: the bridge selects a reference-motion file before policy
  inference; all distances then run through the same actor.
- `encoder_mode_4` is the official universal encoder's input-modality token
  (`g1`/`teleop`/`smpl`), not a chair-distance selector.
- Actor observations: reference motion plus proprioception.
- Excluded observations: height map and chair pose.
- Encoder interface: `obs_dict[1,1751] -> encoded_tokens[1,64]`.
- Decoder interface: `obs_dict[1,994] -> action[1,29]`.
- PyTorch/ONNX parity: pass; see the two `*.parity.json` files.

The package is mirrored at:

```text
oss://xrobot-data/fs/r2s_ego_exp/GR00T-WholeBodyControl_Vigil/policy/facee_v73_noheight/
```

The policy is experimental and is not the launcher default. Independent
MuJoCo screening at d=1.35/1.40 completed the 13-second references and made
continuous seat contact, but ended with approximately 75–77 degrees of torso
tilt and a waist-pitch joint-limit violation. It is therefore not authorized
for real-robot execution.

Use the existing release policy by default. See
`docs/integration/facee_chair_distance_motion.md` for the opt-in command,
reference-motion mapping, checksums, and evidence paths.
