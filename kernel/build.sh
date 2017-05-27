#!/bin/bash
#Stop script if something is broken
set -e

#Export CROSS_COMPILE to point toolchain
export CROSS_COMPILE="ccache ../../arm-cortex_a7-linux-gnueabihf-linaro_4.9.4-2015.06/bin/arm-eabi-"
export TARGET_BUILD_VARIANT=user
export TARGET_PRODUCT=lenovo89_cu_jb
export MTK_ROOT_CUSTOM=../mediatek/custom/
export ARCH=arm


#Echo actual vars
echo "We are actually building for $TARGET_PRODUCT with $CROSS_COMPILE"

#Workaround for + appended on kernelrelease
export LOCALVERSION=Proton-Kernel-1.0

#Create vars for OUT, SCRIPTS and RAMDISK directories
OUT_DIRECTORY=../out/$TARGET_PRODUCT
RAMDISK_DIRECTORY=../ramdisk/$TARGET_PRODUCT
SCRIPTS_DIRECTORY=../scripts/$TARGET_PRODUCT
CERTIFICATES_DIRECTORY=../.certificates

#Create and clean out directory for your device
mkdir -p $OUT_DIRECTORY
if [ "$(ls -A $OUT_DIRECTORY)" ]; then
rm $OUT_DIRECTORY/* -R
fi

#Kernel part
make -j8

#Add MTK header to zImage
cp arch/arm/boot/zImage ../mtk-tools/zImageOld

echo "Adding MTK header to zImage"
cd ../mtk-tools
./mkimage zImageOld KERNEL > zImage
echo "zImage with MTK header ready in mtk-tools folder"

#Repack part
if [ -d "$RAMDISK_DIRECTORY" ]; then

echo "GZIP ramdisk and adding mtk header"
./mkbootfs $RAMDISK_DIRECTORY | gzip >ramdisk.gz
./mkimage ramdisk.gz ROOTFS > ramdisk.img

echo "Repacking boot.img"
./mkbootimg --kernel ./zImage --ramdisk ./ramdisk.img -o boot.img
mv boot.img $OUT_DIRECTORY/
echo "Repacked boot.img is ready in $OUT_DIRECTORY"

cd ../kernel

#Modules part
while test -n "$1"; do
    case "$1" in
    -m | -modules)
        make INSTALL_MOD_STRIP=--strip-unneeded INSTALL_MOD_PATH=$OUT_DIRECTORY/system INSTALL_MOD_DIR=$OUT_DIRECTORY/system android_modules_install
    ;;
    *)
        shift
    ;;
    esac
    shift
done

#../mtk-tools/repack-MT65xx.pl -boot $OUT_DIRECTORY/zImage $RAMDISK_DIRECTORY $OUT_DIRECTORY/boot.img
#rm $OUT_DIRECTORY/zImage

#Cleaning
echo "Cleaning..."
#rm ../mtk-tools/zImage*
rm ../mtk-tools/ramdisk*

#Flashable zip build
if [ -d "$SCRIPTS_DIRECTORY" ]; then
cp $SCRIPTS_DIRECTORY/* $OUT_DIRECTORY -R
FLASHABLE_ZIP="$OUT_DIRECTORY/`cat DEVICE_NAME`-$LOCALVERSION-"$(date +'%y%m%d%H%M')
FLASHABLE_ZIP_2="`cat DEVICE_NAME`-$LOCALVERSION-"$(date +'%y%m%d%H%M')
echo "Creating flashable at '$FLASHABLE_ZIP'.zip"
pushd $OUT_DIRECTORY
zip -r -0 "$FLASHABLE_ZIP_2".zip .
popd
if [ ! -d "$CERTIFICATES_DIRECTORY" ]; then
echo "Warning ! We can't sign flashable.zip, you need to run ./certificates.sh"
else
java -jar $SCRIPTS_DIRECTORY/../signapk.jar $CERTIFICATES_DIRECTORY/certificate.pem $CERTIFICATES_DIRECTORY/key.pk8 "$FLASHABLE_ZIP".zip "$FLASHABLE_ZIP"-signed.zip
fi
fi
fi
