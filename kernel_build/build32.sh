cd linux-arm32-gem5

echo Compile ARM KENEL ...
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- vexpress_gem5_server_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -j4

cp vmlinux ../../kernels/binaries/vmlinux.aarch32.Accel

echo Mount the .img ...
sudo mount -o loop,offset=32256 ../../kernels/disks/linux-aarch32-ael.img /mnt/

echo Compile UserSpace Driver ...
arm-linux-gnueabi-gcc -c kernel/AccelDriver.c

echo Add this file to a library...
ar rvs AccelDriver.a AccelDriver.o

echo Copy the AccelDriver.a to Application directory...
cp AccelDriver.a ../Application

echo Copy the Header file to Application directory...
cp kernel/AccelDriver.h ../Application


echo Compile APP.C using library...
cd ../Application
make all32

echo Copy the executable in /mnt...
sudo sudo cp AccelApp /mnt/
echo Umount the .img ...
sudo umount /mnt
