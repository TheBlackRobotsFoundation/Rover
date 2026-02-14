#!/bin/bash
# Script para ejecutar tests en todos los workspaces

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_ROOT="$(dirname "$SCRIPT_DIR")"

echo "==============================================="
echo "Rover Test - Ejecutando tests"
echo "==============================================="
echo ""

if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 no está sourced"
    exit 1
fi

cd "$ROVER_ROOT"

WORKSPACES=(
    "rover_core"
    "rover_drivers"
    "rover_control"
    "rover_navigation"
    "rover_perception"
    "rover_simulation"
    "rover_tools"
    "rover_bringup"
)

FAILED_TESTS=()

for ws in "${WORKSPACES[@]}"; do
    if [ -d "$ws/src" ]; then
        echo ""
        echo ">>> Testeando $ws..."
        cd "$ROVER_ROOT/$ws"
        if colcon test; then
            echo ">>> $ws: tests OK"
        else
            echo ">>> $ws: tests FAILED"
            FAILED_TESTS+=("$ws")
        fi
    fi
done

echo ""
echo "==============================================="
echo "Tests completados"
echo "==============================================="
echo ""

if [ ${#FAILED_TESTS[@]} -eq 0 ]; then
    echo "Todos los tests pasaron correctamente"
    exit 0
else
    echo "Tests fallidos en:"
    for ws in "${FAILED_TESTS[@]}"; do
        echo "  - $ws"
    done
    exit 1
fi
