/*
 * Warp!! parameter define  Rev. 5.0.1
 *
 */

#ifndef _LINUX_WARP_PARAM_H
#define _LINUX_WARP_PARAM_H

#include <linux/warp.h>
#include <asm/warp.h>

#define WARP_PARAM_VER_MAJOR    0x0005
#define WARP_PARAM_VER_MINOR    0x0000

#define WARP_HEADER_ID          0x00
#define WARP_HEADER_VERSION     0x04
#define WARP_HEADER_CAPS        0x08
#define WARP_HEADER_LOWMEM_END  0x0c
#define WARP_HEADER_TEXT_END    0x10
#define WARP_HEADER_DATA_END    0x14
#define WARP_HEADER_BSS_END     0x18
#define WARP_HEADER_SNAPSHOT    0x20
#define WARP_HEADER_HIBERNATE   0x28
#define WARP_HEADER_SWITCH      0x30

#define WARP_ID_DRIVER          0x44483557      /* W5HD */
#define WARP_ID_BOOTFLAG        0x46423557      /* W5BF */
#define WARP_ID_USER_API        0x41553257      /* W2UA */

#define WARP_PART_SHIFT         0
#define WARP_LUN_SHIFT          8
#define WARP_DEV_SHIFT          16

#define WARP_PART_MASK          (0xff << WARP_PART_SHIFT)
#define WARP_LUN_MASK           (0xff << WARP_LUN_SHIFT)
#define WARP_DEV_MASK           (0xff << WARP_DEV_SHIFT)

#define WARP_DEV_AUTO           (0x00 << WARP_DEV_SHIFT)
#define WARP_DEV_NOR            (0x01 << WARP_DEV_SHIFT)
#define WARP_DEV_NAND           (0x02 << WARP_DEV_SHIFT)
#define WARP_DEV_ATA            (0x03 << WARP_DEV_SHIFT)
#define WARP_DEV_SD             (0x04 << WARP_DEV_SHIFT)
#define WARP_DEV_MEM            (0x05 << WARP_DEV_SHIFT)
#define WARP_DEV_SPI            (0x06 << WARP_DEV_SHIFT)
#define WARP_DEV_USER           (0x7e << WARP_DEV_SHIFT)
#define WARP_DEV_EXT            (0x7f << WARP_DEV_SHIFT)

#define WARP_DEV(dev, lun, part)        (WARP_DEV_##dev | \
                                         ((lun) << WARP_LUN_SHIFT) | \
                                         ((part) << WARP_PART_SHIFT))

#ifndef WARP_LUN_CONV
#define WARP_LUN_CONV(dev)      (dev)
#endif

#define WARP_DEV_TO_LUN(dev)    WARP_LUN_CONV(((dev) & WARP_LUN_MASK) >> \
                                              WARP_LUN_SHIFT)
#define WARP_DEV_TO_PART(dev)   (((dev) & WARP_PART_MASK) >> WARP_PART_SHIFT)

#define WARP_DRV_INVALID        0
#define WARP_DRV_FIXED          1
#define WARP_DRV_FLOATING       2

#define WARP_LOAD_NONE          0
#define WARP_LOAD_MEM           1
#define WARP_LOAD_DEV           2
#define WARP_LOAD_MTD_NO        3
#define WARP_LOAD_MTD_NAME      4

#define WARP_BF_LEN             0x400

#define WARP_CPU_MAX            8

#define WARP_USERAPI_COMP_MODE  10
#define WARP_USERAPI_MAX        4

#define WARP_DEV_SECTOR_SHIFT   9


#ifndef __ASSEMBLY__

#if BITS_PER_LONG == 64
#define ptr_to_u64(x)           ((u64)(x))
#define u64_to_ptr(x)           ((void *)(x))
#else
#define ptr_to_u64(x)           ((u64)(u32)(x))
#define u64_to_ptr(x)           ((void *)(u32)(x))
#endif

enum warp_progress {
    WARP_PROGRESS_INIT,
    WARP_PROGRESS_SYNC,
    WARP_PROGRESS_FREEZE,
    WARP_PROGRESS_SHRINK,
    WARP_PROGRESS_SUSPEND,
    WARP_PROGRESS_SAVE,
    WARP_PROGRESS_SAVEEND,
    WARP_PROGRESS_RESUME,
    WARP_PROGRESS_THAW,
    WARP_PROGRESS_EXIT,
    WARP_PROGRESS_CANCEL,
};

struct warp_savetbl {
    u32         start;
    u32         end;
};

struct warp_savetbl64 {
    u64         start;
    u64         end;
};

typedef struct warp_area {
    u32         dev;
    u32         size;
    u64         offs;
} warp_area;

struct warp {
    u16         ver_major;
    u16         ver_minor;
    s16         page_shift;
    s16         switch_mode;
    s16         compress;
    s16         oneshot;
    s16         halt;
    s16         silent;
    s32         console;
    s32         bps;
    warp_area   bootflag;
    u64         v2p_offset;
    u64         text_v2p_offset;
    u64         hd_savearea;
    u64         hd_savearea_end;
    u64         subcpu_info;
    u64         cpu_info;
    u32         cpu_num;
    u32         cpu_no;
    u64         zonetbl;
    u64         dramtbl;
    u64         exttbl;
    s32         zonetbl_num;
    s32         dramtbl_num;
    s32         exttbl_num;
    s16         preload_exttbl;
    s16         use_free_mem;
    u32         maxarea;
    u32         maxsize;
    u32         lowmem_maxarea;
    u32         lowmem_maxsize;
    u32         snapshot_id;
    u32         drv_num;
    u64         drv_phys[WARP_USERAPI_MAX + 1];
    u32         drv_floating[WARP_USERAPI_MAX + 1];
    u32         text_size;
    u32         drv_total_size;
    u8          userapi_cpu[WARP_USERAPI_MAX];
    u32         private[4];
    u32         reserve[8];
    u32         stat;
    u32         retry;
};

struct warp_boot {
    u16         cpu_no;
    s16         switch_mode;
    s16         silent;
    s16         console;
    s32         bps;
    u32         drv_total_size;
    u64         v2p_offset;
    u64         text_v2p_offset;
    u32         lowmem_maxarea;
    u32         lowmem_maxsize;
    u64         bg;
    u32         reserve[12];
};

struct warp_cpu_info {
    u32         cpu;
    u32         save_ratio;
    warp_area   snapshot;
};

struct warp_drv_info {
    u16 mode;
    u16 cpu;
    union {
        struct {
            int load;
            union {
                int part;
                char *name;
                void *addr;
            } dev;
            u32 offs;
            u32 size;
        } floating;
        struct {
            u32 size;
            unsigned long phys;
            unsigned long virt;
        } fixed;
    } drv;
};

struct warp_savearea {
    struct {
        u32 load;
        union {
            u32 part;
            char *name;
        } dev;
        u64 offs;
    } sw_bootflag;
    warp_area bootflag;
    warp_area snapshot[WARP_CPU_MAX];
};

struct warp_ops {
    int (*drv_load)(struct warp_drv_info *drv, void *buf, size_t size);
    int (*bf_load)(void *buf, int loadno);
    int (*drv_init)(void);
    int (*device_suspend_early)(void);
    int (*device_suspend)(void);
    int (*pre_snapshot)(void);
    int (*snapshot)(void);
    void (*post_snapshot)(void);
    void (*device_resume)(void);
    void (*device_resume_late)(void);
    void (*drv_uninit)(void);
    void (*putc)(char c);
    void (*progress)(int val);
};

int swsusp_page_is_saveable(struct zone *zone, unsigned long pfn);

int hibdrv_snapshot(void);
int warp_register_machine(struct warp_ops *ops);
int warp_unregister_machine(struct warp_ops *ops);

#ifdef CONFIG_MTD
int warp_mtd_load(int mtdno, void *buf, size_t size);
int warp_mtd_load_nm(const char *mtdname, void *buf, size_t size);
int warp_mtd_bf_load(int mtdno, void *buf, loff_t offs, size_t size);
int warp_mtd_bf_load_nm(const char *mtdname, void *buf,
                        loff_t offs, size_t size);
#endif

#ifdef WARP_HIBDRV_DEV_LOAD
int warp_dev_load(const char *dev, void *buf, size_t size);
#endif

int warp_dev_bf_load(const char *dev, void *buf, loff_t offs, size_t size);

extern struct warp warp_param;

extern struct warp_savearea warp_savearea[];

extern void *warp_hibdrv_addr;
extern void *warp_drv_buf;
extern void *warp_bootflag_buf;
extern void *hd_savearea;

#ifdef WARP_DEBUG_NO_HIBDRV

#define WARP_DRV_ID(drv)                WARP_ID_DRIVER
#define WARP_DRV_LOWMEM_END(drv)        0xffffffff
#define WARP_DRV_TEXT_END(drv)          0x00010000
#define WARP_DRV_DATA_END(drv)          0x00010000
#define WARP_DRV_BSS_END(drv)           0x00020000
#define _WARP_DRV_SNAPSHOT(drv, x)      0
#define _WARP_DRV_SWITCH(drv)           0

#else   /* WARP_DEBUG_NO_HIBDRV */

#define WARP_DRV_ID(drv)        (*(u32 *)((void *)(drv) + WARP_HEADER_ID))
#define WARP_DRV_VERSION(drv)   (*(u32 *)((void *)(drv) + WARP_HEADER_VERSION))
#define WARP_DRV_CAPS(drv)      (*(u32 *)((void *)(drv) + WARP_HEADER_CAPS))
#define WARP_DRV_LOWMEM_END(drv) \
    (*(u32 *)((void *)(drv) + WARP_HEADER_LOWMEM_END))
#define WARP_DRV_TEXT_END(drv)  (*(u32 *)((void *)(drv) + WARP_HEADER_TEXT_END))
#define WARP_DRV_DATA_END(drv)  (*(u32 *)((void *)(drv) + WARP_HEADER_DATA_END))
#define WARP_DRV_BSS_END(drv)   (*(u32 *)((void *)(drv) + WARP_HEADER_BSS_END))

#define _WARP_DRV_SNAPSHOT(drv, x) \
    ((int (*)(void *))((void *)(drv) + WARP_HEADER_SNAPSHOT))(x)
#define _WARP_DRV_SWITCH(drv) \
    ((int (*)(void))((void *)(drv) + WARP_HEADER_SWITCH))()

#endif  /* WARP_DEBUG_NO_HIBDRV */

#ifndef WARP_DRV_SNAPSHOT
#define WARP_DRV_SNAPSHOT(drv, x)       _WARP_DRV_SNAPSHOT(drv, x)
#endif

#ifndef WARP_DRV_SWITCH
#define WARP_DRV_SWITCH(drv)            _WARP_DRV_SWITCH(drv)
#endif

#endif  /* __ASSEMBLY__ */

#endif  /* _LINUX_WARP_PARAM_H */
