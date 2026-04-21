#!/bin/bash

# 获取当前脚本所在目录
APPDIR="$(cd "$(dirname "$0")" && pwd)"
SDK_LIB_DIR="$APPDIR/../HumanoidArms_SDK/usrlib"

export QML2_IMPORT_PATH="$APPDIR/qml"

# 优先使用发布包自带 Qt 库，并补上 SDK 中的 CAN 依赖库。
if [ -d "$SDK_LIB_DIR" ]; then
    export LD_LIBRARY_PATH="$APPDIR/lib:$SDK_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
else
    export LD_LIBRARY_PATH="$APPDIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# 启动你的程序
"$APPDIR/Ti5Control" "$@"
