#!/usr/bin/env python3
"""PN (Proportional Navigation) güdüm simülasyonu -- giriş noktası.

Örnekler
--------
    python3 main.py                          # varsayılan senaryo + GIF
    python3 main.py --law augmented -N 3     # güçlendirilmiş PN, N=3
    python3 main.py --compare                # saf vs güçlendirilmiş PN
    python3 main.py --show                   # pencere açarak canlı göster
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from pn_sim.config import GuidanceConfig, RocketConfig, Scenario, SimConfig, TargetConfig
from pn_sim.simulation import Simulation
from pn_sim.visualize import save_animation, save_summary


def _scenario(args) -> Scenario:
    rocket = RocketConfig()
    target = TargetConfig()
    if args.agile_target:
        target = TargetConfig(a_max=12.0 * 9.81, weave_freq=0.35, d_ref=2500.0)
    guidance = GuidanceConfig(law=args.law, nav_constant=args.nav_constant)
    sim = SimConfig(seed=args.seed)
    return Scenario(rocket=rocket, target=target, guidance=guidance, sim=sim)


def _print_summary(res, title: str) -> None:
    h = res.history.arrays()
    print("=" * 62)
    print(f" {title}")
    print("=" * 62)
    print(f"  Sonuç         : {res.outcome.upper()}")
    print(f"  Iska mesafesi : {res.miss_distance:8.2f} m")
    print(f"  Uçuş süresi   : {res.t_end:8.2f} s")
    print(f"  Maks. yanal yük (roket) : {max(h['g_load']):6.1f} g")
    print(f"  Maks. |LOS rate|        : {np.rad2deg(np.max(np.abs(h['los_rate']))):6.2f} deg/s")
    print(f"  Roket son hızı          : {np.linalg.norm(h['rocket_vel'][-1]):6.1f} m/s")
    print(f"  Hedef son hızı          : {np.linalg.norm(h['target_vel'][-1]):6.1f} m/s")
    print("=" * 62)


def cmd_run(args) -> None:
    res = Simulation(_scenario(args)).run()
    _print_summary(res, f"PN güdüm  (law={args.law}, N={args.nav_constant})")

    os.makedirs("output", exist_ok=True)

    if not args.no_gif:
        gif = save_animation(res, "output/pn_simulation.gif", fps=args.fps)
        print(f"  Animasyon GIF  -> {gif}")
        print(f"  Statik kare    -> output/pn_simulation_final.png")
    else:
        png = save_summary(res, "output/pn_simulation_final.png")
        print(f"  Statik kare    -> {png}")

    if args.show:
        import matplotlib.pyplot as plt
        from pn_sim.visualize import SimVisualizer
        plt.rcdefaults()
        viz = SimVisualizer(res)
        plt.show()


def cmd_compare(args) -> None:
    """Saf PN vs Güçlendirilmiş PN karşılaştırması."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    results = {}
    for law in ("pure", "augmented"):
        sc = Scenario(
            rocket=RocketConfig(),
            target=TargetConfig(a_max=12.0 * 9.81, weave_freq=0.35, d_ref=2500.0),
            guidance=GuidanceConfig(law=law, nav_constant=args.nav_constant),
            sim=SimConfig(seed=args.seed),
        )
        res = Simulation(sc).run()
        results[law] = res
        _print_summary(res, f"law={law}")

    # karşılaştırma figürü
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(15, 6.5), facecolor="#0d1117")
    for ax in (ax0, ax1):
        ax.set_facecolor("#111820")
        ax.tick_params(colors="#8aa0b3")
        for sp in ax.spines.values():
            sp.set_color("#1e2a38")
        ax.grid(True, color="#1e2a38", alpha=0.6)

    colors = {"pure": "#ff6b6b", "augmented": "#4dabf7"}
    for law, res in results.items():
        h = res.history.arrays()
        ax0.plot(h["target_pos"][:, 0], h["target_pos"][:, 1], "--",
                 color="#00e5ff", lw=1.2, alpha=0.7)
        ax0.plot(h["rocket_pos"][:, 0], h["rocket_pos"][:, 1],
                 color=colors[law], lw=2.2,
                 label=f"{law}  (iska {res.miss_distance:.1f} m)")
        ax1.plot(h["t"], h["distance"], color=colors[law], lw=2,
                 label=f"{law}  iska {res.miss_distance:.1f} m")

    ax0.set_title("Yörüngeler", color="#c9d6e3", fontsize=12)
    ax0.set_xlabel("x [m]", color="#c9d6e3")
    ax0.set_ylabel("y [m]", color="#c9d6e3")
    ax0.set_aspect("equal")
    ax0.legend(facecolor="#0b1017", edgecolor="#1e2a38", labelcolor="#c9d6e3")

    ax1.set_yscale("log")
    ax1.set_title("Mesafe - zaman", color="#c9d6e3", fontsize=12)
    ax1.set_xlabel("zaman [s]", color="#c9d6e3")
    ax1.set_ylabel("mesafe [m]", color="#c9d6e3")
    ax1.legend(facecolor="#0b1017", edgecolor="#1e2a38", labelcolor="#c9d6e3")

    fig.suptitle("Saf PN  vs  Güçlendirilmiş PN", color="#c9d6e3",
                 fontsize=15, fontweight="bold")
    os.makedirs("output", exist_ok=True)
    out = "output/pn_compare.png"
    fig.savefig(out, dpi=140, facecolor=fig.get_facecolor())
    print(f"\n  Karşılaştırma figürü -> {out}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Proportional Navigation güdüm simülasyonu")
    p.add_argument("--law", choices=["pure", "augmented"], default="pure",
                   help="güdüm yasası (varsayılan: pure)")
    p.add_argument("-N", "--nav-constant", type=float, default=4.0,
                   help="navigasyon sabiti (varsayılan: 4)")
    p.add_argument("--agile-target", action="store_true",
                   help="daha çevik/kaçıcı hedef senaryosu")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--no-gif", action="store_true", help="GIF üretme")
    p.add_argument("--show", action="store_true", help="animasyonu pencerede göster")
    p.add_argument("--compare", action="store_true",
                   help="saf vs güçlendirilmiş PN karşılaştırması")
    return p


def main() -> None:
    args = build_parser().parse_args()
    if args.compare:
        cmd_compare(args)
    else:
        cmd_run(args)


if __name__ == "__main__":
    main()
