#!/bin/bash
# The SDK build of ci/build_with_vex_sdk.sh on a Windows PC, using the VEX SDK and tools that the
# VEX VS Code extension has already installed: nothing is downloaded or installed. Its clang is the
# same one VEXcode compiles with. Run it from Git Bash: ci/build_local_windows.sh
set -euo pipefail

VEX="${APPDATA:?run this from Git Bash on Windows}/Code/User/globalStorage/vexrobotics.vexcode"
TOOLS="$VEX/tools/cpp/toolchain_win32"
if [ ! -x "$TOOLS/clang/bin/clang.exe" ]; then
    echo "VEX VS Code extension tools not found in $TOOLS" >&2
    echo "Install the extension and open (or create) a V5 C++ project once, so it downloads them." >&2
    exit 1
fi

export VEX_SDK_HOME="$VEX/sdk/cpp/V5"
export CXX="$TOOLS/clang/bin/clang.exe"
export LD="$TOOLS/gcc/bin/arm-none-eabi-ld.exe"
export OBJCOPY="$TOOLS/gcc/bin/arm-none-eabi-objcopy.exe"
exec "$(dirname "$0")/build_with_vex_sdk.sh" "$@"
