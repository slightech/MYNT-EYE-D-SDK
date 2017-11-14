#!/usr/bin/env bash

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(dirname "$BASE_DIR")
CONFIG_FILE="$ROOT_DIR/modules/mynteye_config.h"

ver_major=$(cat "$CONFIG_FILE" | grep -m1 MYNTEYE_VERSION_MAJOR)
ver_major=$(echo "${ver_major#*MYNTEYE_VERSION_MAJOR}" | tr -d '[:space:]')

ver_minor=$(cat "$CONFIG_FILE" | grep -m1 MYNTEYE_VERSION_MINOR)
ver_minor=$(echo "${ver_minor#*MYNTEYE_VERSION_MINOR}" | tr -d '[:space:]')

version="$ver_major.$ver_minor"
echo "$version"
