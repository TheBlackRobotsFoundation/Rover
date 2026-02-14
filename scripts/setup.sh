#!/bin/bash
# Setup script para Rover
# Instala todas las dependencias necesarias

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_ROOT="$(dirname "$SCRIPT_DIR")"

echo "==============================================="
echo "Rover Setup - Instalación de Dependencias"
echo "==============================================="
echo ""

# Verificar que estamos en ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 no está sourced"
    echo "Por favor ejecuta: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Detectado ROS2 $ROS_DISTRO"
echo ""

# Actualizar repositorios
echo "Actualizando repositorios..."
sudo apt-get update

# Instalar dependencias de ROS2
echo ""
echo "Instalando dependencias de ROS2..."
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Instalar dependencias específicas del rover
echo ""
echo "Instalando dependencias del rover..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-rplidar-ros \
    python3-pip \
    python3-serial \
    python3-yaml

# Gazebo (opcional, puede no estar disponible en ARM64)
echo ""
echo "Instalando Gazebo (opcional)..."
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-ros-pkgs || echo "AVISO: Gazebo no disponible, saltando (no es crítico para hardware real)"

# Instalar dependencias Python
echo ""
echo "Instalando dependencias Python..."
pip3 install --user pyserial pyyaml

# Inicializar rosdep si no está inicializado
if [ ! -d /etc/ros/rosdep ]; then
    echo ""
    echo "Inicializando rosdep..."
    sudo rosdep init
fi

echo ""
echo "Actualizando rosdep..."
rosdep update

# Instalar dependencias con rosdep
echo ""
echo "Instalando dependencias con rosdep..."
cd "$ROVER_ROOT"
rosdep install --from-paths . --ignore-src -r -y || true

# Configurar permisos para puertos seriales
echo ""
echo "Configurando permisos de puertos seriales..."
sudo usermod -a -G dialout $USER

# Crear udev rules para dispositivos
echo ""
echo "Configurando udev rules..."
sudo tee /etc/udev/rules.d/99-rover.rules > /dev/null << 'EOF'
# RoboClaw controllers
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", SYMLINK+="roboclaw_left", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", SYMLINK+="roboclaw_right", MODE="0666"

# PX4
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0032", SYMLINK+="px4", MODE="0666"

# RPLiDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "==============================================="
echo "Setup completado!"
echo "==============================================="
echo ""
echo "NOTA IMPORTANTE:"
echo "  Se han configurado los permisos de puertos seriales."
echo "  Necesitas cerrar sesión y volver a iniciar sesión"
echo "  para que los cambios surtan efecto."
echo ""
echo "Siguiente paso:"
echo "  ./scripts/build_all.sh"
echo ""
