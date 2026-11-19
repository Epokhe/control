#!/bin/sh
# Fetch the game at the commit these solutions were written against.
set -e
dir="$(dirname "$0")/.game"
[ -d "$dir/.git" ] || git clone -q https://github.com/janismac/ControlChallenges "$dir"
git -C "$dir" checkout -q 027b7d091509f2fdbb9a73ad5b34b111128de7ab
echo "game source ready in $dir"
