#!/bin/bash

export CROSS_COMPILE="../../arm-eabi-4.8/bin/arm-eabi-"
export TARGET_PRODUCT=lenovo89_cu_jb
export MTK_ROOT_CUSTOM=../mediatek

make distclean

rm -rf ../mediatek/config/out/
rm -rf ../mediatek/custom/out/
rm kernel_.bin
find . -name "modules.builtin" -type f -print -delete
