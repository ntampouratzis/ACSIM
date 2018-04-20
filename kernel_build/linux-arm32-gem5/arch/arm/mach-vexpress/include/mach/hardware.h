#ifndef __ASM_ARM_ARCH_HARDWARE_H
#define __ASM_ARM_ARCH_HARDWARE_H

#define IRQ_V2M_PCIE		(32 + 36)

/* The __io macro calls this function. */
u32 pci_convert_base_to_vbase(u32 a);

/* #defines replaced with u32 */
extern u32 dt_vexpress_pci_cfg_base;
extern u32 dt_vexpress_pci_cfg_size;
extern u32 dt_vexpress_pci_cfg_limit;
extern u32 dt_vexpress_pci_cfg_vbase;
extern u32 dt_vexpress_pci_mem_vbase;
extern u32 dt_vexpress_pci_memp_vbase;
extern u32 dt_vexpress_pci_io_vbase;
extern u32 dt_vexpress_pci_mem_base;
extern u32 dt_vexpress_pci_memp_base;
extern u32 dt_vexpress_pci_io_base;
extern u32 dt_vexpress_pci_mem_size;
extern u32 dt_vexpress_pci_memp_size;
extern u32 dt_vexpress_pci_io_size;
extern u32 dt_vexpress_pci_mem_limit;
extern u32 dt_vexpress_pci_memp_limit;
extern u32 dt_vexpress_pci_io_limit;

#endif
