"""Akıllı kaçış manevrası yapan hareketli hedef.

Kaçış stratejisi (heuristik, güdümlü-füze literatüründeki "barrel /
weave" + "flee" karışımı):

    * LOS'a DİK yönde salınım (jink)  -> füzenin LOS açısal hızını bozar
    * Uzaktayken LOS boyunca KAÇIŞ      -> mesafeyi açar
    * Yaklaşıldığında yanal ağırlık artar -> son anda sıyırma (miss)

Hedef ivmesi |a| <= a_max ile sınırlıdır; hızı [v_min, v_max] aralığında
küçük dalgalanmalarla değişir (throttle).
"""
from __future__ import annotations

import numpy as np

from .config import G, TargetConfig
from .geometry import heading_to_vec, unit


class Target:
    def __init__(self, cfg: TargetConfig, rng: np.random.Generator | None = None):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.pos = np.array(cfg.pos0, dtype=float)
        self.vel = heading_to_vec(cfg.heading0, cfg.speed0)

        self.accel = np.zeros(2)
        self.phase = float(self.rng.uniform(0, 2 * np.pi))

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.vel))

    # -- kaçış yasası --------------------------------------------------------
    def evasive_accel(self, r_rocket: np.ndarray, t: float) -> np.ndarray:
        cfg = self.cfg

        los = r_rocket - self.pos
        d = float(np.linalg.norm(los))
        los_hat = unit(los)

        away = -los_hat                                  # rokete sırtını dön
        perp = np.array([-los_hat[1], los_hat[0]])       # LOS'a dik

        # Yanal jink (sinüzoidal) + hafif ikinci harmonik -> düzensiz görünüm
        weave = (np.sin(2 * np.pi * cfg.weave_freq * t + self.phase)
                 + 0.35 * np.sin(2 * np.pi * cfg.weave_freq * 2.3 * t + 0.7))

        # Mesafeye göre kaçış/yanal karışımı
        blend = float(np.clip(d / cfg.d_ref, 0.0, 1.0))  # 1: uzak, 0: yakın

        dir_vec = blend * away + (1.0 - 0.35 * blend) * cfg.weave_amp * weave * perp
        n = float(np.linalg.norm(dir_vec))
        if n < 1e-9:
            return np.zeros(2)
        return (dir_vec / n) * cfg.a_max

    # -- entegrasyon ---------------------------------------------------------
    def step(self, dt: float, a_cmd: np.ndarray, t: float) -> None:
        cfg = self.cfg

        mag = float(np.linalg.norm(a_cmd))
        if mag > cfg.a_max:
            a_cmd = a_cmd * (cfg.a_max / mag)
        self.accel = a_cmd

        self.vel = self.vel + a_cmd * dt
        self.pos = self.pos + self.vel * dt

        sp = self.speed
        if sp > cfg.v_max:
            self.vel *= cfg.v_max / sp
        elif sp < cfg.v_min:
            self.vel *= cfg.v_min / max(sp, 1e-6)
