echo Compile ARM KENEL ...
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- gem5_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j4

cp vmlinux ../../COSSIM/kernels/binaries/vmlinux.aarch64.Accel

echo Mount the .img ...
sudo mount -o loop,offset=32256 $HOME/ACSIM/kernels/disks/aarch64-ubuntu-trusty-headless.img /mnt/

echo Compile UserSpace Driver ...
aarch64-linux-gnu-gcc -c kernel/AccelDriver.c

echo Add this file to a library...
ar rvs AccelDriver.a AccelDriver.o

echo Copy the AccelDriver.a to Application directory...
cp AccelDriver.a ../Application

echo Copy the Header file to Application directory...
cp kernel/AccelDriver.h ../Application


echo Compile APP.C using library...
cd ../Application
make all

echo Copy the executable in /mnt...
sudo sudo cp AccelApp /mnt/
echo Umount the .img ...
sudo umount /mnt