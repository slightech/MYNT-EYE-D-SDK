#!/usr/bin/env bash

srcdir="$1"
dstdir="$2"

[ -e "$srcdir" ] || exit 0
[ -e "$dstdir" ] || exit 0

BASE_DIR=$(cd "$(dirname "$0")" && pwd)

ECHO="echo -e"
FIND="$($BASE_DIR/getfind.sh)"

SRCDIR_PATT="$(echo $srcdir | sed -e 's/\./\\\./g' | sed -e 's/\//\\\//g')"
# echo "SRCDIR_PATT=$SRCDIR_PATT"

$FIND "$srcdir" -type f | while read -r src; do
    dst="$dstdir/$(echo $src | sed -e "s/.*${SRCDIR_PATT}\///g")"
    if [ -e "$dst" ]; then
        rm -f "$dst" && $ECHO "RM: $dst"
    fi
done

[ -e "$dstdir" ] && $FIND "$dstdir" -type d -empty -delete

exit 0