#!/bin/bash
set -e

echo "============================================"
echo "  ESP32-C3 Super Mini - Build & Flash Tool"
echo "============================================"

PROJECT_DIR="/home/bestsage/Documents/GroundMischung/Ground Hardware"
cd "$PROJECT_DIR"

# Détection automatique du port série
PORT=""
for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$p" ]; then
        PORT="$p"
        break
    fi
done

if [ -z "$PORT" ]; then
    echo "❌ Aucun port série trouvé!"
    echo "   Vérifiez que l'ESP32-C3 est branché."
    echo "   Si le port n'est pas reconnu, maintenir BOOT + appuyer RESET"
    exit 1
fi

echo "📡 Port détecté: $PORT"

echo ""
echo "🔨 Compilation..."

# Choix du binaire (par défaut: ground-station)
BIN="${1:-ground-station}"
cargo build --release --bin "$BIN"

BINARY="target/riscv32imc-unknown-none-elf/release/$BIN"
if [ ! -f "$BINARY" ]; then
    echo "❌ Binaire non trouvé après compilation!"
    exit 1
fi

echo ""
echo "⚡ Flash sur ESP32-C3 Super Mini..."
echo "   (Si ça échoue: maintenir BOOT + appuyer RESET, puis relâcher BOOT)"
echo ""

# Flash avec options optimisées pour ESP32-C3 Super Mini
# --baud 921600 : vitesse optimale
# --port : port détecté
# --chip esp32c3 : cible
espflash flash \
    --chip esp32c3 \
    --port "$PORT" \
    --baud 921600 \
    --monitor \
    "$BINARY"

echo ""
echo "✅ Flash terminé!"
