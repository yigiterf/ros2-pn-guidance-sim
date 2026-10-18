"""Güçlü görselleştirme: taktik animasyon + telemetri grafikleri.

Üretilen figür:
    [ SOL ] Büyük taktik görünüm  : yörüngeler, hız/ivme vektörleri, LOS
    [ SAĞ ] Telemetri             : mesafe, LOS açısal hızı, yanal yük (g)
"""
from __future__ import annotations

import numpy as np
import matplotlib

matplotlib.use("Agg")  # çizim arka planda; main.py gerekirse show() çağırır
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Polygon

from .simulation import SimResult

# --- Renk paleti (koyu tema) -------------------------------------------------
BG = "#0d1117"
PANEL = "#111820"
GRID = "#1e2a38"
FG = "#c9d6e3"
ROCKET_C = "#ff4d4d"
TARGET_C = "#00e5ff"
LOS_C = "#ffd166"
ACCEL_C = "#7CFC00"

# --- Gövde şekilleri (yerel koordinat, +x ileri) ----------------------------
ROCKET_SHAPE = np.array([[1.6, 0.0], [-1.0, 0.9], [-0.6, 0.0], [-1.0, -0.9]])
TARGET_SHAPE = np.array([[1.3, 0.0], [0.0, 1.0], [-1.3, 0.0], [0.0, -1.0]])


def _place(shape: np.ndarray, pos: np.ndarray, heading: float, scale: float) -> np.ndarray:
    c, s = np.cos(heading), np.sin(heading)
    rot = np.array([[c, -s], [s, c]])
    return (rot @ (shape * scale).T).T + pos


class SimVisualizer:
    def __init__(self, result: SimResult, frame_stride: int | None = None):
        self.res = result
        self.h = result.history.arrays()
        n = len(self.h["t"])
        if frame_stride is None:
            frame_stride = max(1, n // 360)
        self.stride = frame_stride
        self.frames = list(range(0, n, frame_stride))
        if self.frames[-1] != n - 1:
            self.frames.append(n - 1)

        # Görsel ölçekler (dünya birimi ~ metre)
        span = self._span()
        self.unit = span / 220.0
        self.span = span
        self.rocket_scale = span * 0.013
        self.target_scale = span * 0.015
        self.accel_scale = span * 0.0005   # yanal ivme oku ölçeği [m / (m/s^2)]

        self._build_figure()

    # -- yardımcılar ---------------------------------------------------------
    def _span(self) -> float:
        allpos = np.vstack([self.h["rocket_pos"], self.h["target_pos"]])
        return float(np.max(np.ptp(allpos, axis=0)) )

    def _build_figure(self) -> None:
        plt.rcParams["font.family"] = "DejaVu Sans"
        self.fig = plt.figure(figsize=(16, 9), facecolor=BG)
        gs = self.fig.add_gridspec(
            3, 2, width_ratios=[2.15, 1.0], wspace=0.16, hspace=0.5,
            left=0.05, right=0.97, top=0.93, bottom=0.07,
        )
        self.ax = self.fig.add_subplot(gs[:, 0])
        self.ax_d = self.fig.add_subplot(gs[0, 1])
        self.ax_l = self.fig.add_subplot(gs[1, 1])
        self.ax_g = self.fig.add_subplot(gs[2, 1])
        self._style_axes()
        self._setup_main()
        self._setup_telemetry()

        self.fig.suptitle(
            "PROPORTIONAL NAVIGATION (PN) GÜDÜM SİMÜLASYONU",
            color=FG, fontsize=17, fontweight="bold", y=0.975,
        )
        self.sc = self.res.scenario
        self.fig.text(
            0.5, 0.925,
            f"law={self.sc.guidance.law.upper()}   N={self.sc.guidance.nav_constant:.0f}"
            f"   |   roket a_max={self.sc.rocket.a_max/9.81:.0f}g,"
            f"  hedef a_max={self.sc.target.a_max/9.81:.0f}g",
            color="#8aa0b3", ha="center", fontsize=11,
        )

    def _style_axes(self) -> None:
        for a in (self.ax, self.ax_d, self.ax_l, self.ax_g):
            a.set_facecolor(PANEL)
            for sp in a.spines.values():
                sp.set_color(GRID)
            a.tick_params(colors="#7d8fa3", labelsize=9)
            a.grid(True, color=GRID, lw=0.6, alpha=0.6)

    def _setup_main(self) -> None:
        ax = self.ax
        ax.set_aspect("equal")
        ax.set_title("Taktik Görünüm (üstten)", color=FG, fontsize=12, pad=8)

        spos = self.h["rocket_pos"]
        tpos = self.h["target_pos"]
        allpos = np.vstack([spos, tpos])
        pad = 0.10 * max(np.ptp(allpos, axis=0))
        ax.set_xlim(allpos[:, 0].min() - pad, allpos[:, 0].max() + pad)
        ax.set_ylim(allpos[:, 1].min() - pad, allpos[:, 1].max() + pad)

        # hız büyüklüğü ölçeği
        spd = np.linalg.norm(self.h["rocket_vel"], axis=1)
        tspd = np.linalg.norm(self.h["target_vel"], axis=1)
        self.norm = Normalize(vmin=float(spd.min()), vmax=float(spd.max()))
        self.norm_t = Normalize(vmin=float(tspd.min()), vmax=float(tspd.max()))

        # yörünge izleri (hız ile renklenen scatter)
        self.rocket_trail = ax.scatter([], [], c=[], cmap="plasma", norm=self.norm,
                                       s=12, zorder=3,
                                       edgecolors="none", alpha=0.95)
        self.target_trail = ax.scatter([], [], c=[], cmap="winter", norm=self.norm_t,
                                       s=12, zorder=3,
                                       edgecolors="none", alpha=0.95)

        # vektörler
        self.los_line, = ax.plot([], [], "--", color=LOS_C, lw=1.3, alpha=0.85, zorder=2)
        self.v_rocket_arrow, = ax.plot([], [], "-", color=ROCKET_C, lw=2, alpha=0.9, zorder=5)
        self.v_target_arrow, = ax.plot([], [], "-", color=TARGET_C, lw=2, alpha=0.9, zorder=5)
        self.a_rocket_arrow, = ax.plot([], [], "-", color=ACCEL_C, lw=2, alpha=0.9, zorder=5)

        # gövdeler
        self.rocket_poly = Polygon(ROCKET_SHAPE, closed=True, fc=ROCKET_C,
                                   ec="#ffdddd", lw=1.2, zorder=6)
        self.target_poly = Polygon(TARGET_SHAPE, closed=True, fc=TARGET_C,
                                   ec="#d6ffff", lw=1.2, zorder=6)
        ax.add_patch(self.rocket_poly)
        ax.add_patch(self.target_poly)

        # isabet patlaması
        self.blast, = ax.plot([], [], "o", color="#ffb703", ms=12, alpha=0.0, zorder=7)

        # bilgi paneli
        self.info = ax.text(
            0.015, 0.985, "", transform=ax.transAxes, va="top", ha="left",
            color=FG, fontsize=10.5, family="monospace", zorder=8,
            bbox=dict(boxstyle="round,pad=0.5", fc="#0b1017", ec=GRID, alpha=0.9),
        )

        # gösterge
        ax.plot([], [], color=ROCKET_C, lw=3, label="Roket (PN güdümlü)")
        ax.plot([], [], color=TARGET_C, lw=3, label="Hedef (kaçan)")
        ax.plot([], [], "--", color=LOS_C, lw=1.5, label="Görüş Hattı (LOS)")
        leg = ax.legend(loc="lower right", facecolor="#0b1017", edgecolor=GRID,
                        labelcolor=FG, fontsize=9, framealpha=0.9)
        self.cbar = self.fig.colorbar(self.rocket_trail, ax=ax, fraction=0.035, pad=0.02)
        self.cbar.set_label("Roket hızı [m/s]", color=FG, fontsize=9)
        self.cbar.ax.yaxis.set_tick_params(color=FG)
        plt.setp(self.cbar.ax.get_yticklabels(), color=FG)

    def _setup_telemetry(self) -> None:
        specs = [
            (self.ax_d, "Mesafe [m]", "#ff6b6b", self.h["distance"]),
            (self.ax_l, "LOS açısal hızı [deg/s]", "#ffd166",
             np.rad2deg(self.h["los_rate"])),
            (self.ax_g, "Roket yanal yükü [g]", ACCEL_C, self.h["g_load"]),
        ]
        self.markers = []
        for ax, ylab, color, data in specs:
            ax.set_ylabel(ylab, color=FG, fontsize=9)
            ax.plot(self.h["t"], data, color=color, lw=1.2, alpha=0.55)
            m, = ax.plot([], [], "o", color=color, ms=6, zorder=5)
            vl = ax.axvline(0, color="#3a4a5a", lw=0.8, ls=":")
            self.markers.append((m, vl, data))
        self.ax_g.set_xlabel("zaman [s]", color=FG, fontsize=9)

    # -- animasyon -----------------------------------------------------------
    def _update(self, fidx: int):
        i = self.frames[fidx]
        h = self.h

        # izler
        self.rocket_trail.set_offsets(h["rocket_pos"][:i + 1])
        self.rocket_trail.set_array(np.linalg.norm(h["rocket_vel"][:i + 1], axis=1))
        self.target_trail.set_offsets(h["target_pos"][:i + 1])
        self.target_trail.set_array(np.linalg.norm(h["target_vel"][:i + 1], axis=1))

        rp, tp = h["rocket_pos"][i], h["target_pos"][i]
        rv, tv = h["rocket_vel"][i], h["target_vel"][i]
        rh = float(np.arctan2(rv[1], rv[0]))
        th = float(np.arctan2(tv[1], tv[0]))

        # gövdeler
        self.rocket_poly.set_xy(_place(ROCKET_SHAPE, rp, rh, self.rocket_scale))
        self.target_poly.set_xy(_place(TARGET_SHAPE, tp, th, self.target_scale))

        # LOS
        self.los_line.set_data([rp[0], tp[0]], [rp[1], tp[1]])

        # hız vektörleri
        vs = 0.35
        self.v_rocket_arrow.set_data([rp[0], rp[0] + rv[0] * vs],
                                     [rp[1], rp[1] + rv[1] * vs])
        self.v_target_arrow.set_data([tp[0], tp[0] + tv[0] * vs],
                                     [tp[1], tp[1] + tv[1] * vs])

        # roket yanal ivme vektörü (ölçekli)
        ra = h["rocket_accel"][i]
        asf = self.accel_scale
        self.a_rocket_arrow.set_data([rp[0], rp[0] + ra[0] * asf],
                                     [rp[1], rp[1] + ra[1] * asf])

        # bilgi
        vc = h["closing_speed"][i]
        d = h["distance"][i]
        sstr = np.linalg.norm(rv)
        tstr = np.linalg.norm(tv)
        self.info.set_text(
            f"t = {h['t'][i]:6.2f} s\n"
            f"mesafe      = {d:8.1f} m\n"
            f"kapanma hız = {vc:8.1f} m/s\n"
            f"LOS rate    = {np.rad2deg(h['los_rate'][i]):7.2f} deg/s\n"
            f"roket hızı  = {sstr:7.1f} m/s\n"
            f"hedef hızı  = {tstr:7.1f} m/s\n"
            f"yanal yük   = {h['g_load'][i]:7.1f} g"
        )

        # patlama
        if self.res.outcome == "hit" and i == len(h["t"]) - 1:
            self.blast.set_data([tp[0]], [tp[1]])
            self.blast.set_alpha(0.9)
        else:
            self.blast.set_alpha(0.0)

        # telemetri imleçleri
        for m, vl, data in self.markers:
            m.set_data([h["t"][i]], [data[i]])
            vl.set_xdata([h["t"][i], h["t"][i]])

        return (self.rocket_poly, self.target_poly, self.los_line, self.info,
                self.v_rocket_arrow, self.v_target_arrow, self.a_rocket_arrow,
                self.blast, self.rocket_trail, self.target_trail,
                *[m for m, _, _ in self.markers])

    def animate(self) -> FuncAnimation:
        return FuncAnimation(
            self.fig, self._update, frames=len(self.frames),
            interval=self.stride * self.sc.sim.dt * 1000, blit=False,
        )

    def render_final(self) -> None:
        """Son kareyi sabit çiz (PNG çıktısı için)."""
        self._update(len(self.frames) - 1)


def save_animation(result: SimResult, path: str = "output/pn_simulation.gif",
                   fps: int = 25, dpi: int = 110) -> str:
    viz = SimVisualizer(result)
    anim = viz.animate()
    # Kareyi animasyon kaydedilmeden ÖNCE son duruma getir, sonra PNG al.
    viz.render_final()
    viz.fig.savefig("output/pn_simulation_final.png", dpi=140,
                    facecolor=viz.fig.get_facecolor())
    anim.save(path, writer="pillow", fps=fps, dpi=dpi,
              savefig_kwargs={"facecolor": viz.fig.get_facecolor()})
    plt.close(viz.fig)
    return path


def save_summary(result: SimResult, path: str = "output/pn_summary.png",
                 dpi: int = 140) -> str:
    """Çoklu senaryo/özet çizimi için statik yörünge + telemetri figürü."""
    viz = SimVisualizer(result)
    viz.render_final()
    viz.fig.savefig(path, dpi=dpi, facecolor=viz.fig.get_facecolor())
    plt.close(viz.fig)
    return path
