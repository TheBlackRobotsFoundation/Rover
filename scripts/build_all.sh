#!/bin/bash
# Script para compilar el Rover completo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_ROOT="$(dirname "$SCRIPT_DIR")"

echo "==============================================="
echo "Rover Build - Compilando proyecto completo"
echo "==============================================="
echo ""

# Verificar que estamos en ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 no está sourced"
    echo "Por favor ejecuta: source /opt/ros/humble/setup.bash"
    exit 1
fi

cd "$ROVER_ROOT"

echo "Compilando todos los paquetes..."
echo ""

# Compilar todo de una vez
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "==============================================="
echo "Compilación completada!"
echo "==============================================="
echo ""
echo "Para usar el rover:"
echo "  source $ROVER_ROOT/install/setup.bash"
echo ""
