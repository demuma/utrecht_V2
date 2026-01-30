#!/bin/bash
set -e

ARCH=$(uname -m)

case "$ARCH" in
  x86_64) BIN="docker-compose-linux-x86_64" ;;
  aarch64|arm64) BIN="docker-compose-linux-aarch64" ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac

PLUGIN_DIR="$HOME/.docker/cli-plugins"

echo "→ Creating plugin directory"
mkdir -p "$PLUGIN_DIR"

echo "→ Downloading docker-compose for $ARCH"
curl -L \
  "https://github.com/docker/compose/releases/latest/download/$BIN" \
  -o "$PLUGIN_DIR/docker-compose"

echo "→ Making it executable"
chmod +x "$PLUGIN_DIR/docker-compose"

echo "→ Done. Testing:"
docker compose version
