"""Proportional Navigation (PN) güdüm yasaları.

Kullanılan büyüklükler
-----------------------
R        : hedeften rokete? hayır -> rokete değil, rokete: R = r_target - r_rocket
Vc       : kapanma hızı (closing speed),  Vc = -(R . V_rel) / |R|
lambda_dot : görüş hattı (LOS) açısal hızı,  = (R x V_rel) / |R|^2

Saf PN (true PN)
    a_cmd = N * Vc * lambda_dot      (LOS'a dik yönde)

Güçlendirilmiş PN (Augmented PN / APN)
    a_cmd = N * Vc * lambda_dot + N/2 * a_target_perp

Burada dönen büyüklük 2B'de skaler (z-bileşeni) olduğu için, komut
vektörü uygun normal doğrultusunda kurulur.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import cross2, unit


@dataclass
class GuidanceState:
    """Bir kontrol adımındaki güdüm telemetrisi."""

    distance: float          # |R| [m]
    closing_speed: float     # Vc  [m/s]
    los_rate: float          # lambda_dot [rad/s]
    los_angle: float         # lambda [rad]
    accel_cmd: np.ndarray    # komut vektörü [m/s^2]

    @property
    def accel_cmd_mag(self) -> float:
        return float(np.linalg.norm(self.accel_cmd))


def los_geometry(
    r_rocket: np.ndarray,
    r_target: np.ndarray,
    v_rocket: np.ndarray,
    v_target: np.ndarray,
) -> tuple[np.ndarray, float, np.ndarray, float, float, float]:
    """Roket-hedef bağıl geometrisini hesaplar."""
    R = r_target - r_rocket
    d = float(np.linalg.norm(R))
    R_hat = unit(R)
    V_rel = v_target - v_rocket
    vc = -float(np.dot(R, V_rel)) / max(d, 1e-9)
    los_rate = cross2(R, V_rel) / max(d * d, 1e-9)
    los_angle = float(np.arctan2(R[1], R[0]))
    return R, d, R_hat, vc, los_rate, los_angle


def pn_accel(
    r_rocket: np.ndarray,
    r_target: np.ndarray,
    v_rocket: np.ndarray,
    v_target: np.ndarray,
    nav_constant: float,
    law: str = "pure",
    a_target: np.ndarray | None = None,
) -> GuidanceState:
    """PN güdüm komutunu hesaplar.

    Komut vektörü, güdüm gecikmesini modeller nitelikte roketin hız
    vektörüne dik yönde uygulanır (AA güdümlü füzelerde yanal ivme
    kanatçıklarla bu yönde üretilir).
    """
    R, d, R_hat, vc, los_rate, los_angle = los_geometry(
        r_rocket, r_target, v_rocket, v_target
    )

    # --- PN skaler komutu ---------------------------------------------------
    a_mag = nav_constant * vc * los_rate

    # --- Komut doğrultusu: roket hızına dik (+90 derece / CCW) ---------------
    v_hat = unit(v_rocket)
    cmd_dir = np.array([-v_hat[1], v_hat[0]], dtype=float)

    # --- Augmented PN düzeltmesi -------------------------------------------
    # a_APN = (N/2) * a_T_perp : hedefin manevra ivmesini telafi eder.
    # Hedef ivmesinin komut doğrultusundaki bileşenini ekleriz.
    if law == "augmented" and a_target is not None:
        a_t_comp = float(np.dot(a_target, cmd_dir))
        a_mag += 0.5 * nav_constant * a_t_comp

    accel_cmd = a_mag * cmd_dir

    return GuidanceState(
        distance=d,
        closing_speed=vc,
        los_rate=los_rate,
        los_angle=los_angle,
        accel_cmd=accel_cmd,
    )
