#!/bin/bash
# 保存 SLAM 构建的地图
# 用法:
#   rosrun rm_ep_navigation save_map.sh [地图名称]
# 地图保存到 maps/<地图名称>/ 下，每次建图独立一个文件夹

MAP_NAME=${1:-map_test}
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
MAP_DIR="$(cd "$SCRIPT_DIR/../maps" 2>/dev/null && pwd || echo "$SCRIPT_DIR/../maps")"

SAVE_DIR="${MAP_DIR}/${MAP_NAME}"

if [ ! -d "$SAVE_DIR" ]; then
    mkdir -p "$SAVE_DIR"
fi

echo "保存地图到: ${SAVE_DIR}/${MAP_NAME}"

rosrun map_server map_saver -f "${SAVE_DIR}/${MAP_NAME}"

if [ -f "${SAVE_DIR}/${MAP_NAME}.pgm" ] && [ -f "${SAVE_DIR}/${MAP_NAME}.yaml" ]; then
    echo "地图保存成功:"
    echo "  ${SAVE_DIR}/${MAP_NAME}.pgm"
    echo "  ${SAVE_DIR}/${MAP_NAME}.yaml"
else
    echo "地图保存失败！请确认 gmapping 正在运行。"
    exit 1
fi
