# ACSIM
This work presents a novel simulation environment comprised of a generic SystemC accelerator and probably the most widely known fullsystem
simulator (i.e. GEM5). 

Specifically, we introduce a novel flow that enables us to rapidly prototype synthesisable SystemC hardware accelerators in conjunction with a full-system simulator without worrying about communication and synchronisation issues. SystemC is selected because of its cycleaccurate simulation features, while it is one of the most widely used input languages for the HLS tools. In addition, the
official effort for SystemC definition and promotion known as Open SystemC Initiative (OSCI), now known as Accellera, provides an open-source proof-of-concept simulator while it has been approved by the IEEE Standards Association. The base full-system simulator utilized is GEM5, since it is a fully featured system which is, also, the most widelyused open-source one. We use the full-system mode of the
simulator so as to be able to simulate a complete system comprised of a number of devices and a full Operating System
(OS). As a result, the user can verify his/her application in a cycle-accurate manner via whole system simulation, including
memory hierarchy, caches, peripherals, etc, with full operating system interaction (e.g. scheduler, drivers etc.), thus making
the simulation more realistic/accurate.

ACSIM is an extension of the COSSIM simulation framework and it integrates, in a novel and efficient way,
a combined system and network simulator with a SystemC simulator, in a transparent to the end-used way.
