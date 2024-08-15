#!/bin/bash

CURDIR=$(pwd)
WORKDIR=$(cd $(dirname $0); pwd)

DO_DISPATCH=0
ALL_ARCHS=1
if [[ "$1" == "-d" || "$1" == "--dispatch" ]]; then
    DO_DISPATCH=1
elif [[ "$1" == "-s" || "$1" == "--size" ]]; then
    ALL_ARCHS=0
elif [[ "$1" == "-h" || "$1" == "--help" || "$1" = "?" ]]; then
    echo "Usage: $(basename $0) [-d|-a|-h]"
    echo "    -d | --dispatch     Copy the built libraries to target dispath directory."
    echo "    -s | --size         Build the minimum size to support all iPhone platforms,"
    echo "                        only architectures of arm7 and arm64 will be generated for"
    echo "                        iPhone devices, and i386 and x86_64 for Simulator devices."
    echo "                        By default, all armv7, armv7s, arm64 and arm64e architectures"
    echo "                        will be generated for all iPhone devices."
    echo "                        with this option, minimum size but not best performance."
    echo "                        By default without this option, maximum size but best performance."
    echo "    -h | --help         Show this help information."
    echo;
    cd $CURDIR
    exit 0
fi

rm -rf ${WORKDIR}/output 2>/dev/null
mkdir -p ${WORKDIR}/output/release
mkdir -p ${WORKDIR}/output/debug
MY_OUTPUT_DIR=${WORKDIR}/output
MY_BUILD_DIR=${WORKDIR}/build
MY_PRODUCT_NAME_SIMULATOR_DEBUG=$(xcodebuild -showBuildSettings -sdk 'iphonesimulator' -configuration 'Debug' | grep 'FULL_PRODUCT_NAME' | head -n 1 | awk '{print $NF}')
MY_PRODUCT_NAME_DEBUG=$(xcodebuild -showBuildSettings -sdk 'iphoneos' -configuration 'Debug' | grep 'FULL_PRODUCT_NAME' | head -n 1 | awk '{print $NF}')
MY_PRODUCT_NAME=$(xcodebuild -showBuildSettings -sdk 'iphoneos' -configuration 'Release' | grep 'FULL_PRODUCT_NAME' | head -n 1 | awk '{print $NF}')
# When MY_IPHONE_ARCHS and MY_SIMULATOR_ARCHS are not set, generate the minimux size but not best performance libraries.
# By default, all architectures will be built out for best performance but with maximum size.
MY_IPHONE_ARCHS=
MY_SIMULATOR_ARCHS=
if [ $ALL_ARCHS -eq 1 ]; then
    MY_IPHONE_ARCHS="-arch armv7 -arch armv7s -arch arm64 -arch arm64e"
    MY_SIMULATOR_ARCHS="-arch i386 -arch x86_64"
fi

MY_PRODUCT_NAME_IPHONESIMULATOR_DEBUG=$(basename $MY_PRODUCT_NAME_SIMULATOR_DEBUG .a)_iphonesimulator_debug.a
xcodebuild $MY_SIMULATOR_ARCHS -target wicedmesh -destination 'generic/platform=iOS Simulator' $IPHONE_ARCHS -sdk iphonesimulator -configuration Debug clean build
if [ $? -ne 0 ]; then
    echo "error: run xcodebuild for iphonesimulator Debug build failed, error code: $?"
    xcodebuild clean
    cd $CURDIR
    exit 1
else
    cp "$MY_BUILD_DIR/Debug-iphonesimulator/$MY_PRODUCT_NAME" "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONESIMULATOR_DEBUG"
fi

MY_PRODUCT_NAME_IPHONEOS_DEBUG=$(basename $MY_PRODUCT_NAME_DEBUG .a)_iphoneos_debug.a
xcodebuild $MY_IPHONE_ARCHS -target wicedmesh -destination 'generic/platform=iOS' -sdk iphoneos -configuration Debug clean build
if [ $? -ne 0 ]; then
    echo "error: run xcodebuild for iphoneos Debug build failed, error code: $?"
    xcodebuild clean
    cd $CURDIR
    exit 1
else
    cp "$MY_BUILD_DIR/Debug-iphoneos/$MY_PRODUCT_NAME" "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONEOS_DEBUG"
fi

MY_PRODUCT_NAME_IPHONEOS_RELEASE=$(basename $MY_PRODUCT_NAME .a)_iphoneos.a
xcodebuild $MY_IPHONE_ARCHS -target wicedmesh -destination generic/platform=iOS -sdk iphoneos -configuration Release clean build
if [ $? -ne 0 ]; then
    echo "error: run xcodebuild for iphoneos Release build failed, error code: $?"
    xcodebuild clean
    cd $CURDIR
    exit 1
else
    cp "$MY_BUILD_DIR/Release-iphoneos/$MY_PRODUCT_NAME" "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONEOS_RELEASE"
fi

xcodebuild clean 2>/dev/null
ls -l "$MY_OUTPUT_DIR"

MY_PRODUCT_NAME_DEBUG=debug/$(basename $MY_PRODUCT_NAME .a).a
MY_PRODUCT_NAME_RELEASE=release/$(basename $MY_PRODUCT_NAME .a).a
lipo -create "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONEOS_DEBUG" "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONESIMULATOR_DEBUG" -output "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_DEBUG"
cp "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_IPHONEOS_RELEASE" "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_RELEASE"

rm $MY_OUTPUT_DIR/*.a 2>/dev/null

echo;
ls -l "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_DEBUG"
lipo -info "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_DEBUG"

echo;
ls -l "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_RELEASE"
lipo -info "$MY_OUTPUT_DIR/$MY_PRODUCT_NAME_RELEASE"

echo;
echo "xcodebuild done success"
echo;
cd $CURDIR

# Additional function to copy built result to target directory.
if [ $DO_DISPATCH -eq 0 ]; then
    exit 0
fi

function dispatch_copy()
{
    my_target_dir=$(cd "$1"; pwd)
    if [ -d $my_target_dir ]; then
        rm -f "$my_target_dir/$MY_PRODUCT_NAME_DEBUG" 1>/dev/null 2>&1
        rm -f "$my_target_dir/$MY_PRODUCT_NAME_RELEASE" 1>/dev/null 2>&1
        cp -fR "$MY_OUTPUT_DIR"/* "$my_target_dir"/
        rm -f "$my_target_dir/$MY_PRODUCT_NAME" 1>/dev/null 2>&1
        cp -f "$my_target_dir/$MY_PRODUCT_NAME_DEBUG" "$my_target_dir"/
        echo "done, dispath copied to \"$my_target_dir\""
    fi
    cd $CURDIR
}
# Update the libwicedmesh.a libraries in the iOS MeshApp project.
dispatch_copy "$WORKDIR/../MeshApp/MeshFramework/meshcore/libwicedmesh/libs"
