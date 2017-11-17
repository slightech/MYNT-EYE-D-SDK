#!/usr/bin/env bash

if which where >/dev/null; then
    for i in `where find`; do
        if [ `echo $i | grep -c "msys64"` -gt 0 ]; then
            echo "${i%.exe}" | tr -d "[:space:]" | sed -e 's/\\/\//g'
            exit
        fi
    done
fi

echo "find"