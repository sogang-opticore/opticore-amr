from __future__ import annotations

import numpy as np


def scale_action(action: np.ndarray | list[float], v_max: float, w_max: float) -> tuple[float, float]:
    values = np.asarray(action, dtype=np.float32)
    if values.shape != (2,) or not np.isfinite(values).all():
        return 0.0, 0.0
    a_v, a_w = np.clip(values, -1.0, 1.0)
    return float((a_v + 1.0) * 0.5 * v_max), float(a_w * w_max)


def normalize_command(v: float, w: float, v_max: float, w_max: float) -> np.ndarray:
    return np.asarray([2.0 * np.clip(v / v_max, 0.0, 1.0) - 1.0,
                       np.clip(w / w_max, -1.0, 1.0)], dtype=np.float32)

