"""Güdümlü roket (füze) modeli.

Noktasal kütle modeli:
    * Eksenel (itki - sürükleme) ivme -> hızı değiştirir
    * Yanal (güdüm) ivme -> yönü değiştirir, |a| <= a_max ile sınırlı
"""
from __future__ import annotations

import numpy as np

from .config import RHO, G, RocketConfig
from .geometry import heading_to_vec, unit


class Rocket:
    def __init__(self, cfg: RocketConfig):
        self.cfg = cfg
        self.pos = np.array(cfg.pos0, dtype=float)
        self.vel = heading_to_vec(cfg.heading0, cfg.v_init)

        # Telemetri
        self.accel_cmd = np.zeros(2)        # güdüm komutu (vektör)
        self.accel_lat = np.zeros(2)        # uygulanan yanal ivme
        self.accel_total = np.zeros(2)      # toplam ivme
        self.g_load = 0.0                   # yanal yük [g]

    # -- yardımcılar ---------------------------------------------------------
    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.vel))

    @property
    def heading(self) -> float:
        return float(np.arctan2(self.vel[1], self.vel[0]))

    def _drag(self) -> float:
        v = self.speed
        return 0.5 * RHO * self.cfg.drag_coeff * self.cfg.area * v * v

    # -- entegrasyon ---------------------------------------------------------
    def step(self, dt: float, a_lat_cmd: np.ndarray) -> None:
        """Tek zaman adımı ilerlet. a_lat_cmd: istenen yanal ivme vektörü."""
        cfg = self.cfg

        # İvme sınırı (g-limit)
        mag = float(np.linalg.norm(a_lat_cmd))
        if mag > cfg.a_max:
            a_lat_cmd = a_lat_cmd * (cfg.a_max / mag)

        self.accel_cmd = a_lat_cmd
        self.accel_lat = a_lat_cmd
        self.g_load = mag / G

        # Eksenel ivme: (F_itki - D_sürükleme) / m
        a_axial = (cfg.thrust - self._drag()) / cfg.mass
        v_hat = unit(self.vel)

        self.accel_total = a_axial * v_hat + a_lat_cmd

        # Yarı-implicit Euler
        self.vel = self.vel + self.accel_total * dt
        self.pos = self.pos + self.vel * dt

        # Hız zarfını koru
        sp = self.speed
        if sp > cfg.v_max:
            self.vel *= cfg.v_max / sp
        elif sp < cfg.v_min:
            self.vel *= cfg.v_min / max(sp, 1e-6)
