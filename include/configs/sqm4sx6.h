/*
 * Copyright (C) 2018 Elnico s.r.o.
 *
 * Author: Petr Kubiznak <kubiznak.petr@elnico.cz>
 *
 * Common configuration settings for the Elnico SQM4-SX6 SOM.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#ifndef __CONFIG_COMMON_H
#define __CONFIG_COMMON_H

#include "mx6_common.h"

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_SYS_AUXCORE_BOOTDATA 0x78000000 /* Set to QSPI2 B flash at default */
#define CONFIG_IMX_BOOTAUX

/* Miscellaneous configurable options */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_1G

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_FSL_QSPI		/* Enable the QSPI flash at default */
#define CONFIG_SYS_USE_NAND	/* Enable the NAND flash at default */

#ifdef CONFIG_SYS_AUXCORE_FASTUP	/* M4 fast-up */
# undef CONFIG_FSL_QSPI		/* M4 runs from QSPI -> disable the driver */
# define CONFIG_ENV_IS_IN_MMC
#elif defined CONFIG_SYS_BOOT_QSPI	/* boot from QSPI */
# define CONFIG_ENV_IS_IN_SPI_FLASH
#elif defined CONFIG_SYS_BOOT_NAND	/* boot from NAND */
# define CONFIG_ENV_IS_IN_NAND
#else					/* boot from SD */
# define CONFIG_ENV_IS_IN_MMC
#endif

/******************************************************************************/
#ifdef CONFIG_FSL_QSPI

#define CONFIG_QSPI_BASE		QSPI0_BASE_ADDR
#define CONFIG_QSPI_MEMMAP_BASE		QSPI0_AMBA_BASE

#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_BAR
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_SPI_FLASH_STMICRO
#define	CONFIG_SF_DEFAULT_BUS		0
#define	CONFIG_SF_DEFAULT_CS		0
#define	CONFIG_SF_DEFAULT_SPEED		40000000
#define	CONFIG_SF_DEFAULT_MODE		SPI_MODE_0

#endif /* CONFIG_FSL_QSPI */

/******************************************************************************/
#ifdef CONFIG_SYS_USE_NAND

/* NAND flash commands */
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE

#define MTDIDS_DEFAULT			"nand0=gpmi-nand"
#define MTDPARTS_DEFAULT		"mtdparts=gpmi-nand:8m(boot),256k(env),768k(aux),16m(kernel),4m(dtb),-(rootfs)"

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8

#endif /* CONFIG_SYS_USE_NAND */

/******************************************************************************/
/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

/******************************************************************************/
/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR	0x08

/******************************************************************************/
/* Network */
#define CONFIG_FEC_MXC
#define CONFIG_MII

#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x0
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x1
#endif

#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/******************************************************************************/
/* USB */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#endif

/******************************************************************************/
/* Misc */
#define CONFIG_IMX_THERMAL

/******************************************************************************/
/* ENV */
#define CONFIG_ENV_SIZE			SZ_8K
#if defined(CONFIG_ENV_IS_IN_MMC)
# define CONFIG_ENV_OFFSET		(12 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
# define CONFIG_ENV_OFFSET		(768 * 1024)
# define CONFIG_ENV_SECT_SIZE		(64 * 1024)
# define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
# define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
# define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
# define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
/* This configuration must correspond to the mtdparts settings. */
# define CONFIG_ENV_SECT_SIZE		(128 << 10)	/* 128 kB */
# undef CONFIG_ENV_SIZE
# define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE
# define CONFIG_ENV_OFFSET		(8 << 20)	/* 8 MB offset */
# define CONFIG_ENV_SIZE_REDUND		(CONFIG_ENV_SIZE)
# define CONFIG_ENV_OFFSET_REDUND	((CONFIG_ENV_OFFSET) + (CONFIG_ENV_SIZE))
#endif

/******************************************************************************/
#if defined(CONFIG_ANDROID_SUPPORT)
#include "sqm4sx6_android.h"
#endif

#endif				/* __CONFIG_COMMON_H */
