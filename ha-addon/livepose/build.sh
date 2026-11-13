#!/usr/bin/env bash
# Assemble the local add-on folder (config.yaml, Dockerfile, overlay/ with the changed server files, web/ with the
# rebuilt app, README.md, icons):
#
#   nix-shell -p nodejs pnpm --run ./build.sh [OUT_DIR]       # default OUT_DIR: ./out/vacuula_livepose
#
# WEB_DIST=/path/to/dist ./build.sh   uses an already built app instead of running pnpm.
#
# Then copy it to the HA host (nothing is installed or started by this script):
#   scp -r out/vacuula_livepose root@<ha-host>:/addons/vacuula_livepose
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../.." && pwd)
out=${1:-$here/out/vacuula_livepose}

# Server files this change adds or modifies, relative to server/src/vacuula_server. NOT _build_version.py: the stock
# image already carries its own. Everything else in the image stays as upstream built it.
server_files=(
    livepose.py
    livepose_core.py
    __init__.py
    conn.py
    env.py
    state.py
    handlers/app.py
    handlers/route_matcher.py
    gen3/event_handler.py
    gen3/handlers/nucleo.py
)

rm -rf "$out"
mkdir -p "$out/overlay/vacuula_server" "$out/web"

cp "$here/config.yaml" "$here/Dockerfile" "$here/README.md" "$here/base-sha256.txt" "$out/"
cp "$repo/ha-addon/icon.png" "$repo/ha-addon/logo.png" "$out/"

for f in "${server_files[@]}"; do
    src="$repo/server/src/vacuula_server/$f"
    [ -f "$src" ] || { echo "missing $src" >&2; exit 1; }
    mkdir -p "$out/overlay/vacuula_server/$(dirname "$f")"
    cp "$src" "$out/overlay/vacuula_server/$f"
done

if [ -n "${WEB_DIST:-}" ]; then
    cp -r "$WEB_DIST/." "$out/web/"
else
    (cd "$repo/app" && pnpm install --frozen-lockfile && pnpm vite build --outDir "$out/web" --emptyOutDir)
fi
[ -f "$out/web/index.html" ] || { echo "web app build missing" >&2; exit 1; }

echo "built $out"
sha1sum "$out/overlay/vacuula_server/livepose_core.py"
echo "next: scp -r '$out' root@<ha-host>:/addons/vacuula_livepose   (then follow README.md; nothing was deployed)"
