#!/bin/sh

echo "SC-02C KERNEL IMAGE BUILD START!!!"

read -p "build? [(a)ll/(u)pdate/(z)Image default:update] " ANS

echo "copy initramfs..."
if [ -e /tmp/sc02c_initramfs ]; then
  rm -rf /tmp/sc02c_initramfs
fi
if [ ! -e /tmp/sc02c_initramfs ]; then
  mkdir /tmp/sc02c_initramfs
fi
cp -a ../sc02c_initramfs /tmp/sc02c_initramfs/
rm -rf /tmp/sc02c_initramfs/.git
find /tmp/sc02c_initramfs -name .gitignore | xargs rm

# make start
if [ "$ANS" = 'all' -o "$ANS" = 'a' ]; then
  echo "cleaning..."
  make clean
  make c1_rev02_jpn_ntt_defconfig
fi

if [ "$ANS" != 'zImage' -a "$ANS" != 'z' ]; then
  echo "build start..."
  if [ -e make.log ]; then
    mv make.log make_old.log
  fi
  make 2>&1 | tee make.log
  if [ $? != 0 ]; then
    echo "NG make!!!"
    exit
  fi
  # *.ko replace
  find -name '*.ko' -exec cp -av {} /tmp/sc02c_initramfs/lib/modules/ \;
fi

# build zImage
echo "make zImage..."
make zImage

# release zImage
if [ ! -e out ]; then
  mkdir out
fi
cp arch/arm/boot/zImage ./out/

echo "copy zImage to ./out/zImage"
echo 'download example "sudo heimdall flash --kernel ./out/zImage --verbose"'

echo "SC-02C KERNEL IMAGE BUILD COMPLETE!!!"
