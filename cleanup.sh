#!/bin/bash

# Remove non-public sources
rm -rf src/swe/engine/java/src/com/
rm -rf src/swe/engine/java/src/org/
rm -rf src/content/public/android/java/src/com/

# Remove qualcomm proprietary libraries
rm -f src/third_party/libsweadrenoext/lib/libsweadrenoext_plugin.so
rm -f src/third_party/libnetxt/lib/target/libgetzip_plugin.so
rm -f src/third_party/libnetxt/lib/target/libpp_proc_plugin.so
rm -f src/third_party/libnetxt/lib/target/libqmodem_plugin.so
rm -f src/third_party/libnetxt/lib/target/libredirect_plugin.so
rm -f src/third_party/libnetxt/lib/target/libspl_proc_plugin.so
rm -f src/third_party/libnetxt/lib/target/libtcp_connections_plugin.so
rm -f src/third_party/libnetxt/lib/target/libtcp_fin_aggregation_plugin.so

# Remove non-buildable files
rm -rf open/
rm -rf prebuilt/
rm -rf src/third_party/android_tools/ndk/toolchains/arm-linux-androideabi-4.4.3/
rm -rf src/third_party/android_tools/ndk/toolchains/arm-linux-androideabi-4.7/
rm -rf src/third_party/android_tools/ndk/toolchains/arm-linux-androideabi-clang3.1/
rm -rf src/third_party/android_tools/ndk/toolchains/arm-linux-androideabi-clang3.2/
rm -rf src/third_party/android_tools/ndk/toolchains/llvm-3.1/
rm -rf src/third_party/android_tools/ndk/toolchains/llvm-3.2/
rm -rf src/third_party/android_tools/ndk/toolchains/mipsel-linux-android-4.4.3/
rm -rf src/third_party/android_tools/ndk/toolchains/mipsel-linux-android-4.6/
rm -rf src/third_party/android_tools/ndk/toolchains/mipsel-linux-android-4.7/
rm -rf src/third_party/android_tools/ndk/toolchains/mipsel-linux-android-clang3.1/
rm -rf src/third_party/android_tools/ndk/toolchains/mipsel-linux-android-clang3.2/
rm -rf src/third_party/android_tools/ndk/toolchains/x86-4.4.3/
rm -rf src/third_party/android_tools/ndk/toolchains/x86-4.6/
rm -rf src/third_party/android_tools/ndk/toolchains/x86-4.7/
rm -rf src/third_party/android_tools/ndk/toolchains/x86-clang3.1/
rm -rf src/third_party/android_tools/ndk/toolchains/x86-clang3.2/
rm -rf src/third_party/hunspell_dictionaries/
rm -rf src/third_party/llvm-snapdragon/
rm -rf src/third_party/WebKit/LayoutTests/
rm -rf src/third_party/WebKit/ManualTests/
rm -rf src/third_party/WebKit/PerformanceTests/

# Remove non-buildable files except gyp or gypi
find src/chrome/test/ -type f ! \( -name "*.gyp" -o -name "*.gypi" \) -exec rm {} \;
find src/third_party/trace-viewer/ -type f ! \( -name "*.gyp" -o -name "*.gypi" \) -exec rm {} \;


# Remove git dirs
find . -name ".git" -exec rm -rf {} \;
find . -name ".gitignore" -exec rm -f {} \;
