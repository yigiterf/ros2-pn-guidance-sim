# ros2-pn-guidance-sim

**Proportional Navigation (PN) güdüm algoritması — 2B dinamik simülasyon ve görselleştirme.**

Güdümlü bir roket, akıllı kaçış manevraları yapan hareketli bir hedefi
PN (oransal seyrüsefer) yasasıyla kovalar. Her şey saf Python + NumPy ile
modellenir, Matplotlib ile animasyonlu olarak görselleştirilir.

> İsimdeki `ros2` sonraki adımlar içindir; mevcut çekirdek **ROS2'ye bağımlı
> değildir** ve tek komutla çalışır. İleride `pn_sim` paketi bir ROS2
> node'una sarmalanabilir.

---

## İçindekiler
- [Ne yapar?](#ne-yapar)
- [Güdüm yasası (teori)](#güdüm-yasası-teori)
- [Kurulum ve çalıştırma](#kurulum-ve-çalıştırma)
- [Senaryolar ve parametreler](#senaryolar-ve-parametreler)
- [Dosya yapısı](#dosya-yapısı)
- [Örnek sonuçlar](#örnek-sonuçlar)

---

## Ne yapar?

| Bileşen | Model |
|---|---|
| **Roket (avcı)** | Noktasal kütle: itki − sürükleme (hızı değiştirir) + güdüm yanal ivmesi (yönü değiştirir, g-limitli) |
| **Hedef (kaçan)** | LOS'a dik salınım (jink/weave) + mesafeye göre kaçış; hız zarfı ve ivme limiti var |
| **Güdüm** | Saf PN ve Güçlendirilmiş PN (APN) |
| **Görselleştirme** | Taktik üstten görünüm + hız/ivme vektörleri + LOS + canlı telemetri grafikleri |

Hedefin konumu **her an** değişir: sürekli yanlara salınır, roketten
uzaklaşır, mesafe azaldıkça yanal manevrayı sertleştirir. Yani dinamik,
kapalı-çevrim (closed-loop) bir kovalamaca simülasyonudur.

---

## Güdüm yasası (teori)

Görüş hattı (Line-of-Sight, **LOS**) rokete hedefi birleştiren vektördür.
Klasik **Saf PN** yasası, LOS açısal hızını sıfıra sürmek için rokete
görüş hattına dik yanal ivme komutu verir:

```
a_cmd = N · Vc · λ̇
```

- `N`   : navigasyon sabiti (klasik olarak 3–5)
- `Vc`  : kapanma hızı, `Vc = −(R · V_rel) / |R|`
- `λ̇`   : LOS açısal hızı, `λ̇ = (R × V_rel) / |R|²`

**Güçlendirilmiş PN (Augmented PN)** ise hedefin kendi manevra ivmesini
telafi eder:

```
a_cmd = N · Vc · λ̇  +  (N/2) · a_T⊥
```

`a_T⊥`, hedef ivmesinin LOS'a dik bileşenidir. Manevralı hedeflere karşı
daha küçük ıska mesafesi ve daha gürültüsüz LOS hızı verir (bkz. örnekler).

---

## Kurulum ve çalıştırma

```bash
# Gereksinimler (NumPy, Matplotlib, SciPy, Pillow)
pip install -r requirements.txt

# Varsayılan senaryo: saf PN, N=4 → GIF + PNG üretir
python3 main.py

# Güçlendirilmiş PN, N=3
python3 main.py --law augmented -N 3

# Daha çevik/kaçıcı hedef senaryosu
python3 main.py --agile-target

# Saf PN vs Güçlendirilmiş PN karşılaştırma figürü
python3 main.py --compare -N 3

# Animasyonu ekranda canlı göster (GUI gerekir)
python3 main.py --show
```

Çıktılar `output/` klasörüne yazılır:

| Dosya | Açıklama |
|---|---|
| `output/pn_simulation.gif` | Tam animasyon |
| `output/pn_simulation_final.png` | Son karenin statik hâli |
| `output/pn_compare.png` | Saf vs Güçlendirilmiş PN karşılaştırması |

---

## Senaryolar ve parametreler

Tüm ayarlar [`pn_sim/config.py`](pn_sim/config.py) içindeki dataclass'larda:

```python
RocketConfig   # kütle, itki, sürükleme (Cd, alan), a_max, hız zarfı, başlangıç
TargetConfig   # a_max, hız zarfı, weave_freq, d_ref, başlangıç
GuidanceConfig # law ("pure"/"augmented"), nav_constant (N)
SimConfig      # dt, t_max, hit_radius, seed
```

Kod tarafında özel senaryo kurmak için:

```python
from pn_sim.config import Scenario, RocketConfig, TargetConfig, GuidanceConfig, SimConfig
from pn_sim.simulation import Simulation
from pn_sim.visualize import save_animation

sc = Scenario(
    rocket=RocketConfig(v_init=700.0, a_max=30*9.81),
    target=TargetConfig(a_max=12*9.81, weave_freq=0.4),
    guidance=GuidanceConfig(law="augmented", nav_constant=4),
    sim=SimConfig(seed=1),
)
result = Simulation(sc).run()
print(result.outcome, result.miss_distance)
save_animation(result, "output/custom.gif")
```

### Hedef kaçış stratejisi

[`pn_sim/target.py`](pn_sim/target.py) içinde:

- **Jink (salınım):** LOS'a dik yönde `sin` tabanlı düzensiz salınım →
  füzenin LOS açısal hızını bozar.
- **Kaçış:** uzaktayken LOS boyunca roketten uzaklaşır.
- **Yakın manevra:** mesafe azaldıkça yanal ağırlık artar → son anda sıyırma.

---

## Dosya yapısı

```
pn-algorithm/
├── main.py                 # CLI giriş noktası (run / compare / show)
├── requirements.txt
├── pn_sim/
│   ├── __init__.py
│   ├── config.py           # tüm parametreler
│   ├── geometry.py         # 2B vektör yardımcıları
│   ├── guidance.py         # PN / APN güdüm yasaları + LOS geometrisi
│   ├── rocket.py           # roket dinamiği (itki, sürükleme, g-limit)
│   ├── target.py           # kaçan hedef modeli
│   ├── simulation.py       # entegrasyon döngüsü + isabet/ıska tespiti
│   └── visualize.py        # animasyon + telemetri grafikleri
└── output/                 # üretilen GIF / PNG'ler
```

---

## Örnek sonuçlar

Varsayılan senaryo (saf PN, N=4):

```
Sonuç         : HIT
Iska mesafesi :     7.52 m
Uçuş süresi   :    12.79 s
Maks. yanal yük (roket) :   21.0 g
Maks. |LOS rate|        :    7.01 deg/s
```

Çevik hedef senaryosu (`--agile-target`), N=3 — APN'in avantajı:

| Yasa | Iska mesafesi | Maks. \|LOS rate\| |
|---|---|---|
| `pure` | 6.60 m | 18.63 deg/s |
| `augmented` | **5.11 m** | **9.91 deg/s** |

---

## Sonraki adımlar (yol haritası)

- [ ] 3B'ye genişletme (yükseklik + yerçekimi telafisi)
- [ ] Fırlatma gecikmesi / otopilot zaman sabiti (1. mertebeden gecikme modeli)
- [ ] Gürültülü arayıcı (seeker) ölçümleri + Kalman filtresi
- [ ] ROS2 node'ları: `guidance_node`, `target_node`, `viz_node`
- [ ] Monte-Carlo koşuları ile isabet istatistikleri

---

## Lisans

Eğitim/prototip amaçlıdır.
