#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include "hardware.h"
#define IO_SPACE_LIMIT  0xffffffff
#define __io(a) __typesafe_io(pci_convert_base_to_vbase(a))

#endif
