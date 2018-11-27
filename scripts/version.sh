#!/usr/bin/env bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(dirname "$BASE_DIR")
CONFIG_FILE="$ROOT_DIR/CMakeLists.txt"

version=$(cat "$CONFIG_FILE" | grep -m1 "mynteyed VERSION")
version=$(echo "${version%LANGUAGES*}")
version=$(echo "${version#*VERSION}" | tr -d '[:space:]')

echo "$version"
