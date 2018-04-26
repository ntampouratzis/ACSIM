# ACSIM
This work presents a novel simulation environment comprised of a generic SystemC accelerator and probably the most widely known fullsystem
simulator (i.e. GEM5). 

Specifically, we introduce a novel flow that enables us to rapidly prototype synthesisable SystemC hardware accelerators in conjunction with a full-system simulator without worrying about communication and synchronisation issues. SystemC is selected because of its cycle-accurate simulation features, while it is one of the most widely used input languages for the HLS tools. In addition, the official effort for SystemC definition and promotion known as Open SystemC Initiative (OSCI), now known as Accellera, provides an open-source proof-of-concept simulator while it has been approved by the IEEE Standards Association. The base full-system simulator utilized is GEM5, since it is a fully featured system which is, also, the most widely-used open-source one. We use the Full-System (FS) mode of the simulator so as to be able to simulate a complete system comprised of a number of devices and a full Operating System (OS). As a result, the user can verify his/her application in a cycle-accurate manner via whole system simulation, including memory hierarchy, caches, peripherals, etc, with full operating system interaction (e.g. scheduler, drivers etc.), thus making the simulation more realistic/accurate. In the current state, the ACSIM can support both ARM-32 and ARM-64 Linux kernels.

ACSIM can be an extension of the COSSIM simulation framework and it can integrate, in a novel and efficient way,
a combined system and network simulator with a SystemC simulator, in a transparent to the end-used way.

## cgem5 Repository
The cgem5 repository includes the modified gem5 simulator that ipmlements the ACSIM simulator. The ACSIM can be executed either standalone or within COSSIM project. It should be noted that cgem5 can be used independently of ACSIM as a standalone package incorporating all the changes that have been integrated to the official GEM5 October 2017 release.

## kernel_build Repository
The kernel_build repository includes both ARM-32 and ARM-64 source kernel files with a refernce application. 

## ACSIM Installation

### Download and install SystemC 2.3.0

```
sudo wget -O systemc-2.3.0a.tar.gz http://www.accellera.org/images/downloads/standards/systemc/systemc-2.3.0a.tar.gz
tar -xzvf systemc-2.3.0a.tar.gz
cd systemc-2.3.0a
sudo mkdir -p /usr/local/systemc-2.3.0/
mkdir objdir
cd objdir
../configure --prefix=/usr/local/systemc-2.3.0
make
sudo make install

echo "#SystemC 2.3.0 exports" >> ~/.bashrc
echo "export SYSTEMC_HOME=/usr/local/systemc-2.3.0" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/systemc-2.3.0/lib-linux64" >> ~/.bashrc

source ~/.bashrc
```


### Installing cgem5

```
cd $HOME
git clone https://github.com/ntampouratzis/ACSIM
echo "#GEM5 exports" >> ~/.bashrc
echo "export GEM5=$HOME/ACSIM/cgem5" >> ~/.bashrc
echo "export M5_PATH=$HOME/ACSIM/kernels" >> ~/.bashrc

cd $HOME/ACSIM/cgem5
source ~/.bashrc
scons build/ARM/gem5.opt -j4
```


### Download and copy the kernels
Download the kernels.tar.gz and put it in your $HOME directory from [here](http://kition.mhl.tuc.gr:8000/f/a16324207b/) (it includes gem5 images and kernels).

```
mv $HOME/kernels.tar.gz $HOME/ACSIM
cd $HOME/ACSIM
tar -zxvf kernels.tar.gz
```

## How to execute ACSIM?
We provide you a reference [SystemC Accelerator](cgem5/src/dev/arm/SystemC_Accelerator/dev0/SystemCDevice0.cc) in order to execute the ACSIM environment as well as a User Space [Application](kernel_build/Application/TestApp.c) which calls the Accelerator from gem5 OS.

The reference SystemC Accelerator is compiled with the cgem5, while we have implemented two scripts in order to build the User Space Application for [ARM-32](kernel_build/build32.sh) and [ARM-64](kernel_build/build64.sh).

So, first of all, you need to compile one of your prefered ARM kernel using, and finally to execute them.


### ARM-32
The following script compliles the kernel with [User Application](Application/TestApp.c) and mount it inside the linux-aarch32-ael.img. In addition, it creates vmlinux.aarch32.Accel and put it in kernels/binaries directory.
```
cd $HOME/ACSIM/kernel_build
./build32.sh
cd $HOME/ACSIM/cgem5
./run32.sh
```

### ARM-64
The following script compliles the kernel with [User Application](Application/TestApp.c) and mount it inside the aarch64-ubuntu-trusty-headless.img. In addition, it creates vmlinux.aarch64.Accel and put it in kernels/binaries directory.
```
cd $HOME/ACSIM/kernel_build
./build64.sh
cd $HOME/ACSIM/cgem5
./run64.sh
```

Finally, in order to execute the ACSIM, we have implemented the following script

## Acknowledgement
Our work is published in this paper: T. Nikolaos, K. Georgopoulos and Y. Papaefstathiou, "A novel way to efficiently simulate complex full systems incorporating hardware accelerators," Design, Automation & Test in Europe Conference & Exhibition (DATE), 2017, Lausanne, 2017, pp. 658-661.
