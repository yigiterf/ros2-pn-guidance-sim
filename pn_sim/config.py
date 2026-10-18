"""Simülasyon parametreleri.

Tüm birimler SI'dır (metre, saniye, kg, Newton, radyan).
"""
from __future__ import annotations

from dataclasses import dataclass, field

# --- Fiziksel sabitler -------------------------------------------------------
G = 9.81        # yerçekimi ivmesi [m/s^2]
RHO = 1.225     # hava yoğunluğu (deniz seviyesi) [kg/m^3]


@dataclass
class RocketConfig:
    """Güdümlü roket / füze modeli."""

    # Kütle ve itki
    mass: float = 120.0            # [kg]
    thrust: float = 1500.0         # sabit itki [N] (0 => süzülen füze)

    # Aerodinamik sürükleme: D = 0.5 * RHO * Cd * A * V^2
    drag_coeff: float = 0.15       # Cd [-]
    area: float = 0.05             # referans alan [m^2]

    # Yanal (güdüm) ivme limiti -- yapısal/g limiti
    a_max: float = 40.0 * G        # [m/s^2]

    # Hız zarfı
    v_init: float = 650.0          # başlangıç hızı [m/s]
    v_min: float = 300.0           # [m/s]
    v_max: float = 950.0           # [m/s]

    # Başlangıç durumu
    pos0: tuple[float, float] = (0.0, 0.0)
    heading0: float = 0.0          # rad, +x ekseninden CCW


@dataclass
class TargetConfig:
    """Kaçan (evasive) hareketli hedef modeli."""

    speed0: float = 230.0          # başlangıç hızı [m/s]
    heading0: float = 2.62         # başlangıç yönü [rad]
    a_max: float = 8.0 * G         # manevra ivme limiti [m/s^2]

    v_min: float = 140.0           # [m/s]
    v_max: float = 320.0           # [m/s]

    # Kaçış stratejisi
    d_ref: float = 3500.0          # "yakın" kabul edilen mesafe [m]
    weave_freq: float = 0.22       # yanlara salınım (jink) frekansı [Hz]
    weave_amp: float = 0.9         # salınım ağırlığı [-]

    pos0: tuple[float, float] = (5200.0, 1600.0)


@dataclass
class GuidanceConfig:
    """Güdüm yasası (Proportional Navigation) ayarları."""

    law: str = "pure"              # "pure" | "augmented"
    nav_constant: float = 4.0      # N  (klasik 3-5 arası)


@dataclass
class SimConfig:
    """Zaman/entegrasyon ve senaryo ayarları."""

    dt: float = 0.01               # entegrasyon adımı [s]
    t_max: float = 60.0            # azami simülasyon süresi [s]
    hit_radius: float = 8.0        # isabet kabul yarıçapı [m]
    miss_confirm_time: float = 1.5 # artan mesafede "ıska" onayı için süre [s]
    seed: int = 7


@dataclass
class Scenario:
    """Bir senaryoyu oluşturan tüm alt konfigürasyonlar."""

    rocket: RocketConfig = field(default_factory=RocketConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    sim: SimConfig = field(default_factory=SimConfig)
