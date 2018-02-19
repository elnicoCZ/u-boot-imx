/*
 * Copyright (C) 2017 Elnico s.r.o.
 *
 * Author: Petr Kubiznak <kubiznak.petr@elnico.cz>
 *
 * Configuration settings for the Elnico SQM4-SX6-EasyBoard.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __CONFIG_H
#define __CONFIG_H

/******************************************************************************/
/* Commands */
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_I2C
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_USB
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_TIME

/******************************************************************************/
#include "sqm4sx6.h"

#define CONFIG_DBG_MONITOR

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

/******************************************************************************/
/* UART */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/******************************************************************************/
/* MMC */
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_MMC_ENV_DEV		0	/*USDHC4*/
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT			"/dev/mmcblk3p2"  /* USDHC4; THIS WILL BE OVERWRITTEN BY board_late_mmc_env_init(), calling mmc_map_to_kernel_blk() ! */

#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC4_BASE_ADDR

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define CONFIG_EXTRA_ENV_SETTINGS \
	"mtdparts=" MTDPARTS_DEFAULT "\0"\
	"mtdids=" MTDIDS_DEFAULT "\0"\
	\
	"console=ttymxc0" "\0"\
	\
	"runlevel=5" "\0"\
	\
	"loadaddr=0x80800000" "\0"\
	"fdt_addr=0x83000000" "\0"\
	"aux_addr="__stringify(CONFIG_SYS_AUXCORE_BOOTDATA) "\0"\
	\
	"init=run init_dev init_bootargs" "\0"\
	"init_dev=run init_nand" "\0"\
	"init_mmc=mmc rescan;setenv load_cmd ${load_cmd_mmc};setenv kernel ${mmc_name_kernel};setenv fdt_file ${mmc_name_fdt};setenv aux_file ${mmc_name_aux};setenv bootargs_dev ${bootargs_mmc}" "\0"\
	"init_nand=setenv load_cmd ${load_cmd_nand};setenv kernel ${nand_name_kernel};setenv fdt_file ${nand_name_fdt};setenv aux_file ${nand_name_aux};setenv bootargs_dev ${bootargs_nand}" "\0"\
	"init_bootargs=run bootargs_base bootargs_dev; setenv bootargs ${bootargs} ${bootargs_custom} ${runlevel}" "\0"\
	\
	"bootargs_base=setenv bootargs uart_from_osc console=${console},${baudrate} ${mtdparts}" "\0"\
	"bootargs_mmc=setenv bootargs ${bootargs} root=" CONFIG_MMCROOT " rw rootwait" "\0"\
	"bootargs_nand=setenv bootargs ${bootargs} ubi.mtd=rootfs root=ubi0:rootfs rootfstype=ubifs rw rootwait" "\0"\
	\
	"mmc_name_kernel=zImage" "\0"\
	"mmc_name_fdt=zImage-sqm4sx6-uni.dtb" "\0"\
	"mmc_name_aux=m4.bin" "\0"\
	\
	"nand_name_kernel=kernel" "\0"\
	"nand_name_fdt=dtb" "\0"\
	"nand_name_aux=aux" "\0"\
	\
	"load_cmd_mmc=fatload mmc " __stringify(CONFIG_SYS_MMC_ENV_DEV) ":1" "\0"\
	"load_cmd_nand=nand read" "\0"\
	\
	"load_kernel=${load_cmd} ${loadaddr} ${kernel}" "\0"\
	"load_fdt=${load_cmd} ${fdt_addr} ${fdt_file}" "\0"\
	"load_aux=${load_cmd} ${aux_addr} ${aux_file}" "\0"\
	\
	"run_linux=bootz ${loadaddr} - ${fdt_addr}" "\0"\
	"run_aux=bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA) "\0"\
	\
	"boot_linux=run load_kernel load_fdt run_linux" "\0"\
	"boot_aux=run load_aux run_aux" "\0"\
	"boot_dummyaux=mw.l 0x7f8000 0x20008000; mw.l 0x7f8004 0x1fff9001 144; mw.w 0x7f9000 0xe7fe; dcache flush; bootaux 0x7F8000" "\0"\
	\
	"bootcmd_mfg=source" "\0"\
	\
	""

#define CONFIG_BOOTCOMMAND \
	"run init boot_linux"

/******************************************************************************/
#endif				/* __CONFIG_H */
