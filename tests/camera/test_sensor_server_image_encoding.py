from __future__ import annotations

import numpy as np

from gear_sonic.camera.sensor_server import ImageUtils
from gear_sonic.utils.mujoco_sim.sensor_server import ImageUtils as MujocoImageUtils


def test_camera_image_utils_preserves_rgb_channel_order() -> None:
    image = np.zeros((24, 24, 3), dtype=np.uint8)
    image[:, :] = [255, 0, 0]

    decoded = ImageUtils.decode_image(ImageUtils.encode_image(image))

    assert decoded[..., 0].mean() > 200
    assert decoded[..., 2].mean() < 50


def test_mujoco_image_utils_preserves_rgb_channel_order() -> None:
    image = np.zeros((24, 24, 3), dtype=np.uint8)
    image[:, :] = [255, 0, 0]

    decoded = MujocoImageUtils.decode_image(MujocoImageUtils.encode_image(image))

    assert decoded[..., 0].mean() > 200
    assert decoded[..., 2].mean() < 50
