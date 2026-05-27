from __future__ import annotations

from collections.abc import Sequence

import numpy as np

HsvBound = Sequence[int] | np.ndarray
HsvTriplet = tuple[int, int, int]


def hsv_bound_triplet(bound: HsvBound) -> HsvTriplet:
    values = np.asarray(bound).reshape(-1)
    if values.size != 3:
        raise ValueError(f"HSV bounds must contain exactly 3 values, got {bound!r}.")
    return (int(values[0]), int(values[1]), int(values[2]))


def hsv_bound_array(bound: HsvBound) -> np.ndarray:
    return np.asarray(hsv_bound_triplet(bound), dtype=np.uint8)


def hsv_bound_or_default(bound: HsvBound, default: HsvTriplet) -> HsvTriplet:
    hsv = hsv_bound_triplet(bound)
    return default if hsv == (0, 0, 0) else hsv
