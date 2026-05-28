from __future__ import annotations

from collections.abc import Sequence

import numpy as np

HsvTriplet = tuple[int, int, int]


def to_hsv_triplet(values: Sequence[int] | np.ndarray) -> HsvTriplet:
    hsv_values = np.asarray(values).reshape(-1)
    if hsv_values.size != 3:
        raise ValueError(f"HSV bounds must contain exactly 3 values, got {values!r}.")
    return (int(hsv_values[0]), int(hsv_values[1]), int(hsv_values[2]))


def to_hsv_array(values: Sequence[int] | np.ndarray) -> np.ndarray:
    return np.asarray(to_hsv_triplet(values), dtype=np.uint8)
