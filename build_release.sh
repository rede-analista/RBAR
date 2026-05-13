#!/bin/bash
set -e

VERSION=$1
KEEP_VERSIONS=5

if [ -z "$VERSION" ]; then
    echo "Uso: bash build_release.sh <versão>"
    echo "Exemplo: bash build_release.sh 0.3.1"
    exit 1
fi

FIRMWARE_DIR="firmware/v${VERSION}"
HEX_NAME="RBAR_v${VERSION}_firmware.hex"
ELF_NAME="RBAR_v${VERSION}_firmware.elf"

# ── 1. Atualizar versão em globals.h ──────────────────────────────────────────
sed -i "s/#define FIRMWARE_VERSION \".*\"/#define FIRMWARE_VERSION \"${VERSION}\"/" include/globals.h
echo "[1/4] Versão atualizada para ${VERSION}"

# ── 2. Compilar ───────────────────────────────────────────────────────────────
echo "[2/4] Compilando..."
platformio run --environment atmega324p

# ── 3. Copiar binários para firmware/v<versão>/ ───────────────────────────────
mkdir -p "${FIRMWARE_DIR}"
cp .pio/build/atmega324p/firmware.hex "${FIRMWARE_DIR}/${HEX_NAME}"
cp .pio/build/atmega324p/firmware.elf "${FIRMWARE_DIR}/${ELF_NAME}"

SIZE=$(du -sh "${FIRMWARE_DIR}/${HEX_NAME}" | cut -f1)
echo "[3/4] Firmware salvo em ${FIRMWARE_DIR}/ (${SIZE})"

# ── 4. Manter apenas as últimas N versões ─────────────────────────────────────
mapfile -t VERSIONS < <(ls -d firmware/v* 2>/dev/null | sort -V)
COUNT=${#VERSIONS[@]}
if [ "$COUNT" -gt "$KEEP_VERSIONS" ]; then
    DELETE=$(( COUNT - KEEP_VERSIONS ))
    for i in $(seq 0 $(( DELETE - 1 ))); do
        echo "Removendo versão antiga: ${VERSIONS[$i]}"
        rm -rf "${VERSIONS[$i]}"
    done
fi

echo "[4/4] Versões mantidas: $(ls -d firmware/v* | sort -V | tr '\n' ' ')"

# ── Resumo ────────────────────────────────────────────────────────────────────
echo ""
echo "Build concluída → ${FIRMWARE_DIR}/${HEX_NAME}"
echo ""
echo "Para gravar agora:"
echo "  platformio run --target upload --environment atmega324p"
echo ""
echo "Para commitar e publicar:"
echo "  git add include/globals.h firmware/v${VERSION}/"
echo "  git commit -m \"Release v${VERSION}\""
echo "  git push origin main"
echo "  git tag v${VERSION} && git push origin v${VERSION}"
