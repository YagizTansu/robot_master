#!/usr/bin/env bash
# =============================================================================
#  robot_master — Sıfırdan Kurulum Scripti
#  Hedef OS  : Ubuntu 24.04 LTS (Noble Numbat)
#  ROS 2     : Jazzy Jalisco
#
#  Kullanım:
#    chmod +x install_jazzy.sh
#    ./install_jazzy.sh
#
#  Seçenekler:
#    --skip-ros       ROS 2 kurulumunu atla (zaten kuruluysa)
#    --skip-build     colcon build adımını atla
#    --ws-dir <yol>   Workspace dizini (varsayılan: ~/ros2_ws)
# =============================================================================

set -euo pipefail

# ── Renkli çıktı ──────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Argüman ayrıştırma ────────────────────────────────────────────────────────
SKIP_ROS=false
SKIP_BUILD=false
WS_DIR="$HOME/ros2_ws"
REPO_URL=""   # git klonu için: --repo-url https://github.com/...
SRC_PATH=""   # yerel kopya için: --src-path /media/usb/robot_master

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-ros)   SKIP_ROS=true ;;
        --skip-build) SKIP_BUILD=true ;;
        --ws-dir)     WS_DIR="$2"; shift ;;
        --repo-url)   REPO_URL="$2"; shift ;;
        --src-path)   SRC_PATH="$2"; shift ;;
        *) warn "Bilinmeyen argüman: $1" ;;
    esac
    shift
done

SRC_DIR="$WS_DIR/src"

# ── Ubuntu 24.04 kontrolü ─────────────────────────────────────────────────────
info "İşletim sistemi kontrol ediliyor..."
if [[ ! -f /etc/os-release ]]; then
    error "Bu script yalnızca Ubuntu üzerinde çalışır."
fi
source /etc/os-release
if [[ "$ID" != "ubuntu" || "$VERSION_CODENAME" != "noble" ]]; then
    error "Bu script Ubuntu 24.04 Noble gerektirir. Mevcut: $PRETTY_NAME"
fi
success "Ubuntu 24.04 Noble tespit edildi."

# ── sudo teyidi ───────────────────────────────────────────────────────────────
if ! sudo -v 2>/dev/null; then
    error "Bu script sudo yetkisi gerektirir."
fi

# =============================================================================
# 1. TEMEL SİSTEM GÜNCELLEMESİ
# =============================================================================
info "Sistem güncelleniyor..."
sudo apt-get update -qq
sudo apt-get upgrade -y -qq
sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    ca-certificates \
    locales \
    python3-pip \
    python3-dev \
    net-tools \
    x11-apps \
    libgl1 \
    libglib2.0-0
success "Sistem güncellendi."

# =============================================================================
# 2. LOCALE AYARI
# =============================================================================
info "Locale ayarlanıyor (UTF-8)..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
success "Locale hazır."

# =============================================================================
# 3. ROS 2 JAZZY KURULUMU
# =============================================================================
if [[ "$SKIP_ROS" == "false" ]]; then
    info "ROS 2 Jazzy Jalisco kuruluyor..."

    # Universe repo ekle
    sudo add-apt-repository universe -y

    # ROS 2 GPG anahtarı
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # ROS 2 apt kaynağı
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y ros-jazzy-desktop-full
    success "ROS 2 Jazzy Desktop Full kuruldu."
else
    warn "ROS 2 kurulumu atlandı (--skip-ros)."
fi

# =============================================================================
# 4. ROS 2 ARAÇLARI
# =============================================================================
info "ROS 2 geliştirme araçları kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-setuptools \
    ros-jazzy-ament-cmake \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime
success "ROS 2 araçları hazır."

# =============================================================================
# 5. NAVİGASYON / NAV2 PAKETLERİ
# =============================================================================
info "Nav2 ve navigasyon paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-behavior-tree \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-util \
    ros-jazzy-nav2-msgs
success "Nav2 kuruldu."

# =============================================================================
# 6. LOKALIZASYON / SLAM
# =============================================================================
info "Lokalizasyon ve SLAM paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox
success "Lokalizasyon paketleri kuruldu."

# =============================================================================
# 7. ROBOT DESCRIPTION / STATE PUBLISHER
# =============================================================================
info "Robot description paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro
success "Robot description paketleri kuruldu."

# =============================================================================
# 8. TF2 PAKETLERİ
# =============================================================================
info "TF2 paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations
success "TF2 paketleri kuruldu."

# =============================================================================
# 9. SENSOR / PCL / LİDAR
# =============================================================================
info "Sensör ve PCL paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-laser-geometry \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-msgs
success "Sensör paketleri kuruldu."

# =============================================================================
# 10. BEHAVIOUR TREES
# =============================================================================
info "BehaviourTree.CPP paketleri kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-behaviortree-cpp
success "BehaviourTree.CPP kuruldu."

# =============================================================================
# 11. GAZEBO HARMONIC + ROS KÖPRÜSÜ
# =============================================================================
info "Gazebo Harmonic ve ROS köprüsü kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces
success "Gazebo Harmonic kuruldu."

# =============================================================================
# 12. VDA5050 / MQTT BAĞLANTI
# =============================================================================
info "VDA5050 / MQTT bağımlılıkları kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    ros-jazzy-lifecycle-msgs \
    ros-jazzy-action-msgs \
    ros-jazzy-rclcpp-action \
    ros-jazzy-pluginlib \
    python3-paho-mqtt
success "VDA5050 bağımlılıkları kuruldu."

# =============================================================================
# 13. C++ ÜÇÜNCÜ TARAF KÜTÜPHANELERİ
# =============================================================================
info "C++ sistem kütüphaneleri kuruluyor..."

# GTSAM (Factor Graph Optimization için)
info "  GTSAM kuruluyor..."
if ! dpkg -l | grep -q libgtsam-dev 2>/dev/null; then
    # Ubuntu 24.04 için GTSAM PPA
    sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2 2>/dev/null || \
    warn "GTSAM PPA eklenemedi, standart repolardan deneniyor..."
    sudo apt-get update -qq
fi
sudo apt-get install -y --no-install-recommends \
    libgtsam-dev \
    libgtsam-unstable-dev 2>/dev/null || \
    warn "GTSAM apt'tan kurulamadı, kaynak koddan derleme gerekebilir."

# PCL (Point Cloud Library)
sudo apt-get install -y --no-install-recommends \
    libpcl-dev \
    libpcl-all-dev

# JSON (nlohmann)
sudo apt-get install -y --no-install-recommends \
    nlohmann-json3-dev

# GeographicLib
sudo apt-get install -y --no-install-recommends \
    libgeographiclib-dev \
    libgeographic-dev \
    geographiclib-tools

# Diğer C++ bağımlılıkları
sudo apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libeigen3-dev \
    libyaml-cpp-dev

success "C++ sistem kütüphaneleri kuruldu."

# =============================================================================
# 14. PYTHON GUI (BT VİZÜALİZÖR)
# =============================================================================
info "Python GUI araçları kuruluyor..."
sudo apt-get install -y --no-install-recommends \
    python3-tk \
    python3-numpy \
    python3-scipy \
    python3-matplotlib
success "Python GUI araçları kuruldu."

# =============================================================================
# 15. PYTHON PIP PAKETLERİ
# =============================================================================
info "Python pip paketleri kuruluyor..."
pip3 install --break-system-packages \
    xacro \
    graphviz \
    requests \
    transforms3d
success "Python pip paketleri kuruldu."

# =============================================================================
# 16. ROSDEP BAŞLATMA
# =============================================================================
info "rosdep başlatılıyor..."
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update
success "rosdep hazır."

# =============================================================================
# 17. WORKSPACE KURULUMU
# =============================================================================
info "Workspace dizini oluşturuluyor: $WS_DIR/src"
mkdir -p "$SRC_DIR"
success "$WS_DIR/src dizini hazır."

if [[ -n "$REPO_URL" ]]; then
    # ── Git'ten klonla ────────────────────────────────────────────────────────
    info "Repo klonlanıyor: $REPO_URL"
    if [[ -d "$SRC_DIR/robot_master" ]]; then
        warn "robot_master zaten mevcut, git pull yapılıyor..."
        git -C "$SRC_DIR/robot_master" pull
    else
        git clone "$REPO_URL" "$SRC_DIR/robot_master"
    fi
    success "Repo klonlandı: $SRC_DIR/robot_master"

elif [[ -n "$SRC_PATH" ]]; then
    # ── Yerel dizinden kopyala (USB, ağ paylaşımı, vb.) ──────────────────────
    if [[ ! -d "$SRC_PATH" ]]; then
        error "Kaynak yol bulunamadı: $SRC_PATH"
    fi
    info "Kaynak kod kopyalanıyor: $SRC_PATH → $SRC_DIR/robot_master"
    rsync -av --progress "$SRC_PATH/" "$SRC_DIR/robot_master/"
    success "Kaynak kod kopyalandı."

elif [[ -d "$SRC_DIR/robot_master" ]]; then
    # ── Zaten mevcut ──────────────────────────────────────────────────────────
    success "Mevcut workspace kullanılıyor: $SRC_DIR/robot_master"

else
    warn "---------------------------------------------------------------"
    warn "Kaynak kod yok! Seçenekler:"
    warn "  1) Git ile: --repo-url https://github.com/kullanici/robot_master"
    warn "  2) USB/disk: --src-path /media/usb/robot_master"
    warn "  3) Manuel:  kodu $SRC_DIR/robot_master dizinine kopyalayın"
    warn "---------------------------------------------------------------"
    warn "Bağımlılıklar kurulmaya devam edilecek, derleme atlanacak."
    SKIP_BUILD=true
fi

# =============================================================================
# 18. ROSDEP BAĞIMLILIKLARINI ÇÖZME
# =============================================================================
if [[ -d "$SRC_DIR/robot_master" ]]; then
    info "rosdep bağımlılıkları çözülüyor..."
    # navigation2 içindeki nav2_route gibi subpackage'ler için --ignore-src
    rosdep install \
        --from-paths "$SRC_DIR" \
        --ignore-src \
        --rosdistro jazzy \
        -y \
        --skip-keys="libgtsam-dev libgtsam-unstable-dev" \
        2>/dev/null || warn "Bazı rosdep paketleri çözülemedi (normalde sorun değil)."
    success "rosdep tamamlandı."
fi

# =============================================================================
# 19. COLCON BUILD
# =============================================================================
if [[ "$SKIP_BUILD" == "false" && -d "$SRC_DIR/robot_master" ]]; then
    info "Workspace derleniyor (colcon build)..."
    info "Bu işlem birkaç dakika sürebilir..."

    # ROS environment'ı kaynak al
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash

    cd "$WS_DIR"
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-ignore navigation2 \
        2>&1 | tee "$WS_DIR/build.log"

    BUILD_EXIT=${PIPESTATUS[0]}
    if [[ $BUILD_EXIT -eq 0 ]]; then
        success "Derleme başarılı!"
    else
        warn "Derleme tamamlandı ancak bazı paketlerde hata oluştu."
        warn "Detaylar için: $WS_DIR/build.log"
    fi
elif [[ "$SKIP_BUILD" == "true" ]]; then
    warn "Derleme atlandı (--skip-build)."
fi

# =============================================================================
# 20. .BASHRC AYARLARI
# =============================================================================
info ".bashrc güncelleniyor..."

BASHRC="$HOME/.bashrc"
SETUP_MARKER="# >>> robot_master ROS2 setup >>>"
SETUP_MARKER_END="# <<< robot_master ROS2 setup <<<"

if ! grep -q "$SETUP_MARKER" "$BASHRC" 2>/dev/null; then
    cat >> "$BASHRC" << EOF

$SETUP_MARKER
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# robot_master workspace (derlendiyse)
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source "$WS_DIR/install/setup.bash"
fi

# Otomatik tamamlama
eval "\$(register-python-argcomplete3 ros2)"
eval "\$(register-python-argcomplete3 colcon)"

# ROS domain (aynı ağdaki robotlarla çakışmaması için değiştirin)
export ROS_DOMAIN_ID=42

# Log renkleri
export RCUTILS_COLORIZED_OUTPUT=1

$SETUP_MARKER_END
EOF
    success ".bashrc güncellendi."
else
    warn ".bashrc zaten yapılandırılmış, atlandı."
fi

# =============================================================================
# ÖZET
# =============================================================================
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║           KURULUM TAMAMLANDI                             ║${NC}"
echo -e "${GREEN}╠══════════════════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}║${NC}  ROS 2 Jazzy Jalisco   : /opt/ros/jazzy               ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  Workspace             : $WS_DIR              ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  Build log             : $WS_DIR/build.log    ${GREEN}║${NC}"
echo -e "${GREEN}╠══════════════════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}║${NC}  Terminali yeniden başlatın veya:                     ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}    source ~/.bashrc                                   ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}                                                       ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  Workspace'i manuel derlemek için:                    ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}    cd $WS_DIR && colcon build --symlink-install${GREEN}║${NC}"
echo -e "${GREEN}╠══════════════════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}║${NC}  Kullanım örnekleri:                                   ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  Git ile: ./install_jazzy.sh --repo-url <url>          ${GREEN}║${NC}"
echo -e "${GREEN}║${NC}  USB ile: ./install_jazzy.sh --src-path /media/usb/... ${GREEN}║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
