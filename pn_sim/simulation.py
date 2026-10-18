"""Simülasyon motoru ve senaryo tarihçesi."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config import Scenario
from .guidance import GuidanceState, pn_accel
from .rocket import Rocket
from .target import Target


@dataclass
class History:
    """Kaydedilen zaman serisi (her biri eşit uzunlukta listeler)."""

    t: list[float] = field(default_factory=list)
    rocket_pos: list[np.ndarray] = field(default_factory=list)
    rocket_vel: list[np.ndarray] = field(default_factory=list)
    target_pos: list[np.ndarray] = field(default_factory=list)
    target_vel: list[np.ndarray] = field(default_factory=list)
    target_accel: list[np.ndarray] = field(default_factory=list)
    rocket_accel: list[np.ndarray] = field(default_factory=list)
    distance: list[float] = field(default_factory=list)
    closing_speed: list[float] = field(default_factory=list)
    los_rate: list[float] = field(default_factory=list)
    g_load: list[float] = field(default_factory=list)

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "t": np.array(self.t),
            "rocket_pos": np.array(self.rocket_pos),
            "rocket_vel": np.array(self.rocket_vel),
            "target_pos": np.array(self.target_pos),
            "target_vel": np.array(self.target_vel),
            "target_accel": np.array(self.target_accel),
            "rocket_accel": np.array(self.rocket_accel),
            "distance": np.array(self.distance),
            "closing_speed": np.array(self.closing_speed),
            "los_rate": np.array(self.los_rate),
            "g_load": np.array(self.g_load),
        }


@dataclass
class SimResult:
    history: History
    outcome: str            # "hit" | "miss" | "timeout"
    miss_distance: float    # minimum roket-hedef mesafesi [m]
    t_end: float
    scenario: Scenario


class Simulation:
    def __init__(self, scenario: Scenario):
        self.sc = scenario
        self.rng = np.random.default_rng(scenario.sim.seed)

    def run(self) -> SimResult:
        sc = self.sc
        dt, t_max = sc.sim.dt, sc.sim.t_max
        n_steps = int(round(t_max / dt))

        rocket = Rocket(sc.rocket)
        target = Target(sc.target, self.rng)
        hist = History()

        outcome = "timeout"
        miss_distance = np.inf
        t = 0.0

        # Başlangıç durumunu kaydet
        self._record(hist, t, rocket, target, GuidanceState(
            distance=float(np.linalg.norm(target.pos - rocket.pos)),
            closing_speed=0.0, los_rate=0.0, los_angle=0.0,
            accel_cmd=np.zeros(2),
        ))

        for _ in range(n_steps):
            # 1) Güdüm komutu (mevcut durumla)
            gs = pn_accel(
                rocket.pos, target.pos, rocket.vel, target.vel,
                sc.guidance.nav_constant, sc.guidance.law,
                a_target=target.accel,
            )

            # 2) Roketi ilerlet
            rocket.step(dt, gs.accel_cmd)

            # 3) Hedef kaçış ivmesi ve ilerleme
            t_target = t + dt
            a_t = target.evasive_accel(rocket.pos, t_target)
            target.step(dt, a_t, t_target)

            t += dt

            # 4) Kaydet
            self._record(hist, t, rocket, target, gs, a_t)

            # 5) Sonlanma kontrolleri
            d = float(np.linalg.norm(target.pos - rocket.pos))
            miss_distance = min(miss_distance, d)

            if d <= sc.sim.hit_radius:
                outcome = "hit"
                break

            # Kapanma durdu ve mesafe artıyorsa ıska
            if len(hist.distance) > 2:
                if hist.distance[-1] > hist.distance[-2] and gs.closing_speed < 0:
                    # son miss_confirm_time boyunca sürekli artıyor mu?
                    k = int(sc.sim.miss_confirm_time / dt)
                    if len(hist.distance) > k:
                        recent = hist.distance[-k:]
                        if all(recent[i] < recent[i + 1] for i in range(len(recent) - 1)):
                            outcome = "miss"
                            break

        return SimResult(
            history=hist,
            outcome=outcome,
            miss_distance=float(miss_distance),
            t_end=t,
            scenario=self.sc,
        )

    @staticmethod
    def _record(hist, t, rocket, target, gs, a_t=None):
        hist.t.append(t)
        hist.rocket_pos.append(rocket.pos.copy())
        hist.rocket_vel.append(rocket.vel.copy())
        hist.target_pos.append(target.pos.copy())
        hist.target_vel.append(target.vel.copy())
        hist.target_accel.append((a_t if a_t is not None else np.zeros(2)).copy())
        hist.rocket_accel.append(rocket.accel_lat.copy())
        hist.distance.append(gs.distance)
        hist.closing_speed.append(gs.closing_speed)
        hist.los_rate.append(gs.los_rate)
        hist.g_load.append(rocket.g_load)
