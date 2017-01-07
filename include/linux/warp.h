/*
 * Warp!! common define  Rev. 5.0.0
 *
 */

#ifndef _LINUX_WARP_H
#define _LINUX_WARP_H

#define WARP_STATE_NORMAL       0
#define WARP_STATE_SUSPEND      1
#define WARP_STATE_RESUME       2

#define WARP_SHRINK_NONE        0
#define WARP_SHRINK_ALL         1
#define WARP_SHRINK_LIMIT1      2
#define WARP_SHRINK_LIMIT2      3

#ifndef __ASSEMBLY__

extern int pm_device_down;

extern int warp_stat;
extern int warp_error;
extern int warp_retry;
extern int warp_canceled;
extern int warp_saveno;
extern int warp_loadno;
extern int warp_shrink;
extern int warp_separate;
extern int warp_swapout_disable;
extern int warp_separate_pass;

int warp_set_savearea(u64 start, u64 end);
void warp_save_cancel(void);

#ifdef CONFIG_PM_WARP_DEBUG

int warp_printf(const char *fmt, ...);
void warp_putc(char c);

#else

#define warp_printf(fmt...)
#define warp_putc(c)

#endif

#endif  /* __ASSEMBLY__ */

#endif  /* _LINUX_WARP_H */
