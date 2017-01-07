/*
 * include/asm/warp.h
 */

#ifndef _ASM_WARP_H
#define _ASM_WARP_H

#ifdef CONFIG_ARCH_ROCKCHIP

#ifdef WARP_NONE_FLOATING

#define WARP_HIBDRV_PHYS        0x24100000
#define WARP_HIBDRV_VIRT        0xc3f00000
#define WARP_HIBDRV_SIZE        0x00100000

#define WARP_DRV_INFO                                                   \
{                                                                       \
    /* Hibernation driver */                                            \
    .mode                       = WARP_DRV_FIXED,                       \
    .drv.fixed.phys             = WARP_HIBDRV_PHYS,                     \
    .drv.fixed.virt             = WARP_HIBDRV_VIRT,                     \
    .drv.fixed.size             = WARP_HIBDRV_SIZE,                     \
},

#else   /* !WARP_NONE_FLOATING */

#define WARP_DRV_INFO                                                   \
{                                                                       \
    /* Hibernation driver */                                            \
    .mode                       = WARP_DRV_FLOATING,                    \
    .drv.floating.load          = WARP_LOAD_DEV,                        \
    .drv.floating.dev.name      = "mmcblk0p5",                          \
    .drv.floating.offs          = 0x00000000,                           \
},

#endif  /* WARP_NONE_FLOATING */

#define WARP_SAVEAREA                                                   \
{                                                                       \
    .sw_bootflag.load           = WARP_LOAD_DEV,                        \
    .sw_bootflag.dev.name       = "mmcblk0p5",                          \
    .sw_bootflag.offs           = 0x00000100,                           \
    .bootflag.dev               = WARP_DEV(SD, 0, 0),                   \
    .bootflag.size              = 0x00000002,                           \
    .bootflag.offs              = 0x0001f100,                           \
    .snapshot[0].dev            = WARP_DEV(SD, 0, 0),                   \
    .snapshot[0].size           = 0x0007fefe,                           \
    .snapshot[0].offs           = 0x0001f102,                           \
},

#define WARP_CONSOLE    1
#define WARP_BPS        115200

//#define WARP_MMC_SNAPSHOT_COPY

#endif  /* CONFIG_ARCH_ROCKCHIP */

#define WARP_PFN_IS_NOSAVE

#endif  /* _ASM_WARP_H */

