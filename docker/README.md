# Docker Localization Benchmark

İki bağımsız Docker container'da **AMCL+EKF** ve **FGO** lokalizasyonunu canlı karşılaştırır.

## Dosya Yapısı

```
docker/
├── Dockerfile             — Base image (bağımlılıklar, kaynak kod YOK)
├── docker-compose.yml     — 2 servis: ekf_sim + fgo_sim
├── .env                   — Yol ve display değişkenleri
├── entrypoint_ekf.sh      — ekf_sim başlatma scripti
├── entrypoint_fgo.sh      — fgo_sim başlatma scripti
├── live_compare.py        — Host terminalde canlı karşılaştırma tablosu
└── rviz/
    └── comparison.rviz    — Benchmark path'lerini overlay gösteren RViz config
```

## İlk Kurulum (Bir Kez)

```bash
# 1. X11 erişimi ver
xhost +local:docker

# 2. Benchmark sonuçları klasörünü oluştur
mkdir -p ~/ros2_ws/benchmark_results

# 3. Docker image'ı build et (~10-15 dk, bir kez yeterli)
cd ~/ros2_ws/src/robot_master/docker
docker compose build

# 4. rich kütüphanesini kur (canlı tablo için)
pip install rich
```

## Çalıştırma

```bash
# Terminal 1 — İki container'ı başlat
cd ~/ros2_ws/src/robot_master/docker
docker compose up

# Terminal 2 — Host'ta canlı karşılaştırma tablosu
python3 ~/ros2_ws/src/robot_master/docker/live_compare.py
```

İki RViz penceresi açılır:
- **ekf_sim** → AMCL path (yeşil) + Ground Truth (kırmızı)
- **fgo_sim** → FGO path (yeşil) + Ground Truth (kırmızı)

Terminal 2'de her iki container'ın metriklerini yan yana görebilirsin.

## Geliştirme Workflow'u (Kod Değişikliği)

```bash
# Host'ta kodu değiştir → build et
cd ~/ros2_ws
colcon build --packages-select <değişen_paket>

# Sadece ilgili container'ı yeniden başlat (~3 saniye)
docker compose restart ekf_sim    # EKF tarafı değiştiyse
docker compose restart fgo_sim    # FGO tarafı değiştiyse
```

**Image rebuild sadece şunda gerekli:** `Dockerfile` içine yeni `apt` paketi eklediğinde.
```bash
docker compose build --no-cache
```

## Kontroller

```bash
# Sadece bir container'ı çalıştır
docker compose up ekf_sim
docker compose up fgo_sim

# Container içine gir (debug)
docker exec -it ekf_sim bash
docker exec -it fgo_sim bash

# Container içinde kaynak kodunu source'la
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash

# Benchmark sonuçlarını gör
ls -lth ~/ros2_ws/benchmark_results/
```

## Mimari

```
HOST (ROS_DOMAIN_ID=host)
  live_compare.py → CSV okur, terminal tablosu

ekf_sim container (ROS_DOMAIN_ID=1)
  Gazebo + robot_bringup
  AMCL + EKF
  localization_benchmark_amcl → /benchmark/est_path, /benchmark/gt_path
  RViz (comparison.rviz)
  → ~/ros2_ws/benchmark_results/localization_AMCL_*.csv

fgo_sim container (ROS_DOMAIN_ID=2)
  Gazebo + robot_bringup
  FGO (fgo_node + scan_matcher + trust_weight_bridge)
  localization_benchmark → /benchmark/est_path, /benchmark/gt_path
  → ~/ros2_ws/benchmark_results/localization_FGO_*.csv
```

## Benchmark Metrikleri

| Metrik | İyi | Uyarı | Kötü |
|--------|-----|-------|------|
| Position Error | < 5 cm | < 15 cm | > 15 cm |
| Yaw Error | < 2° | < 5° | > 5° |
| ATE RMSE | < 5 cm | < 20 cm | > 20 cm |

## Sorun Giderme

**Gazebo açılmıyor (display hatası):**
```bash
xhost +local:docker
```

**Container build hataları:**
```bash
docker compose build --no-cache --progress=plain 2>&1 | tee build.log
```

**`/clock` bekleniyor ama gelmiyor:**
Container içine girip Gazebo'nun düzgün başlayıp başlamadığını kontrol et:
```bash
docker exec -it ekf_sim bash
ros2 topic list | grep clock
```

**NVIDIA GPU hatası:**
```bash
nvidia-smi                          # driver çalışıyor mu?
docker run --gpus all nvidia/cuda:12.0-base nvidia-smi  # docker GPU erişimi var mı?
```
