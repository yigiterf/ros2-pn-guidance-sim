"""2B vektör / geometri yardımcıları."""
from __future__ import annotations

import numpy as np


def cross2(a: np.ndarray, b: np.ndarray) -> float:
    """2B vektörlerin z-bileşenli çapraz çarpımı: a x b."""
    return float(a[0] * b[1] - a[1] * b[0])


def left_normal(v: np.ndarray) -> np.ndarray:
    """v vektörünün +90 derece (CCW) döndürülmüş birim normali."""
    n = np.array([-v[1], v[0]], dtype=float)
    return n


def unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Güvenli birim vektör."""
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros(2, dtype=float)
    return v / n


def heading_to_vec(heading: float, speed: float) -> np.ndarray:
    """Başlangıç yönü ve hızından hız vektörü üretir."""
    return speed * np.array([np.cos(heading), np.sin(heading)], dtype=float)
