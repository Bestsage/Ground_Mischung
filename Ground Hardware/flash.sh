#!/bin/bash
set -e

# Charger la toolchain Xtensa ESP (gcc linker + clang)
source "$HOME/export-esp.sh"

echo "============================================"
echo "  ESP32-S3 N16R8 - Build & Flash Tool"
echo "============================================"

PROJECT_DIR="/home/bestsage/Ground_Mischung/Ground Hardware"
cd "$PROJECT_DIR"

# Fonction: trouver le port série
find_port() {
    for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
        if [ -e "$p" ]; then
            echo "$p"
            return 0
        fi
    done
    return 1
}

PORT=$(find_port) || true
if [ -z "$PORT" ]; then
    echo "❌ Aucun port série trouvé!"
    echo "   Vérifiez que l'ESP32-S3 est branché."
    echo "   Pour le mode download: maintenir BOOT + appuyer RESET, puis relâcher BOOT"
    exit 1
fi

echo "📡 Port détecté: $PORT"

echo ""
echo "🔨 Compilation..."

# Choix du binaire (par défaut: ground-station)
BIN="${1:-ground-station}"
cargo +esp build --release --bin "$BIN"

BINARY="target/xtensa-esp32s3-none-elf/release/$BIN"
if [ ! -f "$BINARY" ]; then
    echo "❌ Binaire non trouvé après compilation!"
    exit 1
fi

echo ""
echo "⚡ Flash sur ESP32-S3 N16R8..."
echo "   (Si ça échoue: maintenir BOOT + appuyer RESET, puis relâcher BOOT)"
echo ""

# Flash SANS reset automatique après flash.
# Sur ESP32-S3 USB natif (ttyACM0), le reset via DTR/RTS remet le chip
# en mode download (boot:0x0) au lieu de démarrer l'application.
espflash flash \
    --chip esp32s3 \
    --port "$PORT" \
    --baud 921600 \
    --min-chip-rev 0.0 \
    --no-skip \
    "$BINARY"

echo ""
echo "✅ Flash terminé!"
echo ""
echo "============================================"
echo "  IMPORTANT - ESP32-S3 USB natif (ttyACM0)"
echo "============================================"
echo ""
echo "  1. DÉBRANCHEZ le câble USB"
echo "  2. REBRANCHEZ le câble USB"
echo "  3. Appuyez sur Entrée ici"
echo ""
echo "  (Un simple RESET ne suffit pas avec l'USB natif,"
echo "   il faut un power cycle complet)"
echo ""
read -r

# Attendre que le port réapparaisse après power cycle
echo "🔍 Recherche du port série..."
RETRY=0
PORT=""
while [ $RETRY -lt 15 ]; do
    PORT=$(find_port) || true
    if [ -n "$PORT" ]; then
        break
    fi
    sleep 1
    RETRY=$((RETRY + 1))
    echo "   Attente... ($RETRY/15)"
done

if [ -z "$PORT" ]; then
    echo "❌ Port série non retrouvé après power cycle!"
    echo "   Essayez: espflash monitor --port /dev/ttyACM0"
    exit 1
fi

echo "📡 Port retrouvé: $PORT"
echo "🖥️  Lancement du monitor (serial direct, pas de reset)..."
echo "   Ctrl+C pour quitter"
echo ""

# On utilise cat au lieu de espflash monitor car ce dernier
# reset le chip via DTR/RTS (visible dans le log: "Using flash stub")
# ce qui remet l'ESP32-S3 en mode download.
stty -F "$PORT" 115200 raw -echo
cat "$PORT"
