#!/usr/bin/env bash
# Assembles the GitHub Pages site (the pages workflow runs this):
#   <out>/index.html      the front page, docs/site/index.html
#   <out>/img/            its pictures, from docs/img and viewer/docs
#   <out>/viewer/         the viewer with its two sample splats, unchanged
# To look at it locally: docs/site/build.sh _site && python3 -m http.server -d _site 8000
set -euo pipefail
out=${1:?usage: docs/site/build.sh OUT_DIR}
root=$(cd "$(dirname "$0")/../.." && pwd)
rm -rf "$out"
mkdir -p "$out/img" "$out/viewer"
cp "$root/docs/site/index.html" "$out/index.html"
cp "$root/docs/img/hero.svg" "$root/viewer/docs/viewer.jpg" "$out/img/"
cp -r "$root/viewer/index.html" "$root/viewer/js" "$root/viewer/data" "$out/viewer/"
touch "$out/.nojekyll"   # serve the files as they are, without Jekyll
echo "Site in $out: $(find "$out" -type f | wc -l) files, $(du -sh "$out" | cut -f1)"
