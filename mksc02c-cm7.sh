#!/bin/sh

echo "SC-02C KERNEL IMAGE BUILD START!!!"

read -p "build? [(a)ll/(u)pdate/(z)Image default:update] " ANS

echo "copy initramfs..."
if [ -d /tmp/sc02c_initramfs ]; then
  rm -rf /tmp/sc02c_initramfs  
fi
cp -a ../sc02c_initramfs /tmp/
rm -rf /tmp/sc02c_initramfs/.git
find /tmp/sc02c_initramfs -name .gitignore | xargs rm

# make start
if [ "$ANS" = 'all' -o "$ANS" = 'a' ]; then
  echo "cleaning..."
  make clean
  make c1_cm7_sakuramilk_defconfig
fi

if [ "$ANS" != 'zImage' -a "$ANS" != 'z' ]; then
  echo "build start..."
  if [ -e make.log ]; then
    mv make.log make_old.log
  fi
  make -j2 2>&1 | tee make.log
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
echo 'Please download and run command "sudo heimdall flash --kernel ./out/zImage --verbose"'

echo "SC-02C KERNEL IMAGE BUILD COMPLETE!!!"