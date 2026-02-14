#!/bin/bash
# Script para sincronizar documentación con la página web

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_ROOT="$(dirname "$SCRIPT_DIR")"
WEB_ROOT="/home/rover/Desktop/theblackrobotsfoundation.github.io"

echo "==============================================="
echo "Rover Docs Sync - Sincronizando documentación"
echo "==============================================="
echo ""

if [ ! -d "$WEB_ROOT" ]; then
    echo "ERROR: No se encuentra el directorio de la web en $WEB_ROOT"
    exit 1
fi

# Directorio destino en la web
DOCS_DEST="$WEB_ROOT/docs/rover"

# Crear directorio si no existe
mkdir -p "$DOCS_DEST"

# Copiar documentación
echo "Copiando documentación desde $ROVER_ROOT/docs a $DOCS_DEST..."
rsync -av --delete \
    --exclude='.git' \
    --exclude='node_modules' \
    "$ROVER_ROOT/docs/" "$DOCS_DEST/"

echo ""
echo "Documentación sincronizada!"
echo ""
echo "Para publicar:"
echo "  cd $WEB_ROOT"
echo "  npm run build"
echo "  git add docs/rover"
echo "  git commit -m 'Update rover documentation'"
echo "  git push"
echo ""
