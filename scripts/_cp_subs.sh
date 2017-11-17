#!/usr/bin/env bash

srcdir="$1"
dstdir="$2"

[ -e "$srcdir" ] || exit 0

if [ -n "$3" ]; then
    override="False"
else
    override="True"
fi

BASE_DIR=$(cd "$(dirname "$0")" && pwd)

ECHO="echo -e"
FIND="$($BASE_DIR/getfind.sh)"

SRCDIR_PATT="$(echo $srcdir | sed -e 's/\./\\\./g' | sed -e 's/\//\\\//g')"
# echo "SRCDIR_PATT=$SRCDIR_PATT"

$FIND "$srcdir" -type f | while read -r src; do
    dst="$dstdir/$(echo $src | sed -e "s/.*${SRCDIR_PATT}\///g")"
    if [ -e "$dst" ] && [ "$override" = "False" ]; then
        $ECHO "CP: $src > $dst already done"
    else
        $ECHO "CP: $src > $dst"
        _dir="$(dirname "$dst")" && [ -d "$_dir" ] || mkdir -p "$_dir"
        cp "$src" "$dst"
    fi
done

exit 0