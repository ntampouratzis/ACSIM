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


Our work is published in this paper: T. Nikolaos, K. Georgopoulos and Y. Papaefstathiou, "A novel way to efficiently simulate complex full systems incorporating hardware accelerators," Design, Automation & Test in Europe Conference & Exhibition (DATE), 2017, Lausanne, 2017, pp. 658-661.


