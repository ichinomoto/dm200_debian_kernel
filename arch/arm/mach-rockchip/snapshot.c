/*
 * arch/arm/mach-rockchip/snapshot.c
 *
 *  Copyright (C) 2016  Lineo Solutions, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/warp_param.h>
#include <linux/rockchip/iomap.h>

#if 0
static u32 cru_apll_con[3], cru_dpll_con[3], cru_apll_con0_tmp;
static u32 cru_apll_con1_tmp;
static unsigned long rate_old, rate_new;
#endif
static u32 cru_cpll_con[3], cru_gpll_con[3];
static u32 cru_mode_con, cru_clksel_con[35], cru_clkgate_con[11];
static u32 cru_glb_srst_fst_value, cru_glb_srst_snd_value, cru_softrst_con[9];
static u32 cru_misc_con, cru_glb_cnt_th, cru_glb_rst_st, cru_sdmmc_con[2];
static u32 cru_sdio_con[2], cru_emmc_con[2], cru_pll_prg_en;

static u32 grf_gpio_iomux[4][4], grf_gpio2c_iomux2, grf_cif_iomux[2];
static u32 grf_gpio_ds, grf_gpio_pull[8], grf_acodec_con, grf_soc_con[3];
static u32 grf_lvds_con0, grf_dmac_con[3], grf_mac_con[2], grf_tve_con;
static u32 grf_uoc0_con0, grf_uoc1_con[5], grf_ddrc_stat, grf_cpu_con[3];
static u32 grf_os_reg[8], grf_pvtm_con[4], grf_pvtm_status[3];
static u32 grf_nif_fifo1, grf_usbphy_con[2][8], grf_uoc_status0;

static u32 uart_lcr[3], uart_ier[3], uart_fcr[3], uart_mcr[3], uart_dl[3];
static u32 timer[6][5], pmu_regs[18], pwm_regs[4][3];

static unsigned long uart_addrs[] = {
	RK312X_UART0_PHYS,
	RK312X_UART1_PHYS,
	RK312X_UART2_PHYS,
};

extern void rockchip_smp_prepare_cpus (unsigned int max_cpus);

extern void warp_rk818_suspend (void);
extern void warp_rk818_resume (void);

inline int arch_prepare_suspend (void) { return 0; }
inline void save_processor_state (void) {}
inline void restore_processor_state (void) {}


#ifdef CONFIG_PM_WARP_DEBUG

#define UART_ADDR	uart_addrs[WARP_CONSOLE]

static void __iomem *uart_base;

static int warp_drv_init (void)
{
    uart_base = ioremap(UART_ADDR, SZ_16K);
    if (!uart_base)
        return -ENOMEM;
    return 0;
}

static void warp_drv_uninit (void)
{
    if (uart_base)
        iounmap(uart_base);
}

static void warp_putchar (char c)
{
#if 1
    while (!(readl_relaxed(uart_base + 0x7c) & 0x2))
#else
#if 0
    while ((readl_relaxed(uart_base + 0x18) & 0x20))
#else
    while (!(readl_relaxed(uart_base + 0x14) & 0x20))
#endif
#endif
        ;
    writel_relaxed(c, uart_base + 0x00);
}

#endif /* CONFIG_PM_WARP_DEBUG */

static int warp_snapshot (void)
{
    int i, j, ret = 0;
    void __iomem *uart_addr;
    u32 cru_clkgate7_con;
    u64 cntvoff;

    /* UART */
    for (i = 0; i < 3; i++) {
        uart_addr = ioremap(uart_addrs[i], SZ_8K);
        if (!uart_addr)
            continue;
        uart_lcr[i] = readl_relaxed(uart_addr + 0x0c);
        if (uart_lcr[i]) {
            uart_ier[i] = readl_relaxed(uart_addr + 0x04);
            uart_fcr[i] = readl_relaxed(uart_addr + 0x08);
            uart_mcr[i] = readl_relaxed(uart_addr + 0x10);
            while (readl_relaxed(uart_addr + 0x7c) & 0x1);
            writel_relaxed(0x83, uart_addr + 0x0c);
            uart_dl[i] = readl_relaxed(uart_addr + 0x00)
                         | (readl_relaxed(uart_addr + 0x04) << 8);
            while (readl_relaxed(uart_addr + 0x7c) & 0x1);
            writel_relaxed(0x03, uart_addr + 0x0c);
        }
        if (uart_addr)
            iounmap(uart_addr);
    }

    /* PMIC */
    warp_rk818_suspend();

    /* PWM */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++)
            pwm_regs[i][j] = readl_relaxed(RK_PWM_VIRT
                                           + 0x0004 + (i * 16) + (j * 4));
    }
    pwm_regs[4][0] = readl_relaxed(RK_PWM_VIRT + 0x0040);
    pwm_regs[4][1] = readl_relaxed(RK_PWM_VIRT + 0x0044);

    /* PMU */
    for (i = 0; i < 18; i++)
        pmu_regs[i] = readl_relaxed(RK_PMU_VIRT + (i * 4));

    /* Timer */
    cru_clkgate7_con = readl_relaxed(RK_CRU_VIRT + 0x00d0 + (7 * 4));
    if (cru_clkgate7_con & (1 << 7))
        writel_relaxed(0xffff0000 | (cru_clkgate7_con & ~(1 << 7)),
                       RK_CRU_VIRT + 0x00d0 + (7 * 4));
    for (i = 0; i < 6; i++) {
        timer[i][4] = readl_relaxed(RK_TIMER_VIRT + (i * 0x20) + (4 * 4));
        if (timer[i][4] & (1 << 0)) {
            for (j = 0; j < 4; j++)
                timer[i][j] = readl_relaxed(RK_TIMER_VIRT
                                            + (i * 0x20) + (j * 4));
        }
    }
    if (cru_clkgate7_con & (1 << 7))
        writel_relaxed(0xffff0000 | cru_clkgate7_con,
                       RK_CRU_VIRT + 0x00d0 + (7 * 4));

    /* GRF */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            grf_gpio_iomux[i][j] = 0xffff0000
                                   | readl_relaxed(RK_GRF_VIRT + 0x0a8
                                                   + (i * 16) + (j * 4));
        }
    }
    grf_gpio2c_iomux2 = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x0e8);
    grf_cif_iomux[0] = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x0ec);
    grf_cif_iomux[1] = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x0f0);
    grf_gpio_ds = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x100);
    for (i = 0; i < 8; i++) {
        grf_gpio_pull[i] = 0xffff0000
                           | readl_relaxed(RK_GRF_VIRT + 0x118 + (i * 4));
    }
    grf_acodec_con = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x13c);
    for (i = 0; i < 3; i++) {
        grf_soc_con[i] = 0xffff0000
                         | readl_relaxed(RK_GRF_VIRT + 0x140 + (i * 4));
    }
    grf_lvds_con0 = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x150);
    for (i = 0; i < 3; i++) {
        grf_dmac_con[i] = 0xffff0000
                          | readl_relaxed(RK_GRF_VIRT + 0x15c + (i * 4));
    }
    for (i = 0; i < 2; i++) {
        grf_mac_con[i] = 0xffff0000
                         | readl_relaxed(RK_GRF_VIRT + 0x168 + (i * 4));
    }
    grf_tve_con = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x170);
    grf_uoc0_con0 = 0xffff0000 | readl_relaxed(RK_GRF_VIRT + 0x17c);
    for (i = 0; i < 5; i++) {
        grf_uoc1_con[i] = 0xffff0000
                          | readl_relaxed(RK_GRF_VIRT + 0x184 + (i * 4));
    }
    grf_ddrc_stat = readl_relaxed(RK_GRF_VIRT + 0x19c);
    for (i = 0; i < 3; i++) {
        grf_cpu_con[i] = 0xffff0000
                         | readl_relaxed(RK_GRF_VIRT + 0x1a8 + (i * 4));
    }
    for (i = 0; i < 8; i++)
        grf_os_reg[i] = readl_relaxed(RK_GRF_VIRT + 0x1c8 + (i * 4));
    for (i = 0; i < 4; i++)
        grf_pvtm_con[i] = readl_relaxed(RK_GRF_VIRT + 0x200 + (i * 4));
    grf_pvtm_con[0] |= 0xffff0000;
    for (i = 0; i < 3; i++)
        grf_pvtm_status[i] = readl_relaxed(RK_GRF_VIRT + 0x214 + (i * 4));
    grf_nif_fifo1 = readl_relaxed(RK_GRF_VIRT + 0x234);
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 8; j++) {
            if (j == 5)
                continue;
            grf_usbphy_con[i][j] = 0xffff0000
                                   | readl_relaxed(RK_GRF_VIRT + 0x280
                                                   + (i * 32) + (j * 4));
        }
    }
    grf_uoc_status0 = readl_relaxed(RK_GRF_VIRT + 0x2c0);

    /* CRU */
#if 0
    for (i = 0; i < 3; i++) {
        cru_apll_con[i] = readl_relaxed(RK_CRU_VIRT + 0x0000 + (i * 4));
        if (i != 2)
            cru_apll_con[i] |= 0xffff0000;
    }
    for (i = 0; i < 3; i++) {
        cru_dpll_con[i] = readl_relaxed(RK_CRU_VIRT + 0x0010 + (i * 4));
        if (i != 2)
            cru_dpll_con[i] |= 0xffff0000;
    }
#endif
    for (i = 0; i < 3; i++) {
        cru_cpll_con[i] = readl_relaxed(RK_CRU_VIRT + 0x0020 + (i * 4));
        if (i != 2)
            cru_cpll_con[i] |= 0xffff0000;
    }
    for (i = 0; i < 3; i++) {
        cru_gpll_con[i] = readl_relaxed(RK_CRU_VIRT + 0x0030 + (i * 4));
        if (i != 2)
            cru_gpll_con[i] |= 0xffff0000;
    }
    cru_mode_con = 0x11110000 | readl_relaxed(RK_CRU_VIRT + 0x0040);
    for (i = 0; i < 35; i++) {
        if ((i == 16) || (i == 21) || (i == 22) || (i == 33))
            continue;
        cru_clksel_con[i] = readl_relaxed(RK_CRU_VIRT + 0x0044 + (i * 4));
        if ((i >= 0 && i <= 6) || (i >= 9 && i <= 15) || (i >= 23 && i <= 34))
            cru_clksel_con[i] |= 0xffff0000;
    }
    for (i = 0; i < 11; i++)
        cru_clkgate_con[i] = 0xffff0000
                             | readl_relaxed(RK_CRU_VIRT + 0x00d0 + (i * 4));
    cru_glb_srst_fst_value = readl_relaxed(RK_CRU_VIRT + 0x0100);
    cru_glb_srst_snd_value = readl_relaxed(RK_CRU_VIRT + 0x0104);
#if 0
    for (i = 0; i < 9; i++)
        cru_softrst_con[i] = 0xffff0000
                             | readl_relaxed(RK_CRU_VIRT + 0x0110 + (i * 4));
#endif
    cru_misc_con = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x0134);
    cru_glb_cnt_th = readl_relaxed(RK_CRU_VIRT + 0x0140);
    cru_glb_rst_st = readl_relaxed(RK_CRU_VIRT + 0x0150);
#if 0
    cru_sdmmc_con[0] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01c0);
    cru_sdmmc_con[1] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01c4);
    cru_sdio_con[0] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01c8);
    cru_sdio_con[1] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01cc);
    cru_emmc_con[0] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01d8);
    cru_emmc_con[1] = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01dc);
    cru_pll_prg_en = 0xffff0000 | readl_relaxed(RK_CRU_VIRT + 0x01f0);
#endif

    /* CNTVOFF save */
    asm volatile(
        "	cps	0x16\n"
        "	mrc	p15, 0, r1, c1, c1, 0\n"
        "	orr	r0, r1, #1\n"
        "	mcr	p15, 0, r0, c1, c1, 0\n"
        "	isb\n"
        "	mrrc	p15, 4, %Q0, %R0, c14\n"
        "	isb\n"
        "	mcr	p15, 0, r1, c1, c1, 0\n"
        "	isb\n"
        "	cps	0x13\n"
        : "=r" (cntvoff) : : "r0", "r1");

    /* call hibernation driver */
    ret = hibdrv_snapshot();

    /* VOFF restore */
    asm volatile(
        "	cps	0x16\n"
        "	mrc	p15, 0, r1, c1, c1, 0\n"
        "	orr	r0, r1, #1\n"
        "	mcr	p15, 0, r0, c1, c1, 0\n"
        "	isb\n"
        "	mcrr	p15, 4, %Q0, %R0, c14\n"
        "	isb\n"
        "	mcr	p15, 0, r1, c1, c1, 0\n"
        "	isb\n"
        "	cps	0x13\n"
        : : "r" (cntvoff) : "r0", "r1");

    rockchip_smp_prepare_cpus(NR_CPUS);

    /* CRU */
#if 0
    /* ARM PLL */
    cru_apll_con0_tmp = readl_relaxed(RK_CRU_VIRT + 0x0000);
    cru_apll_con1_tmp = readl_relaxed(RK_CRU_VIRT + 0x0004);
    rate_old = 24 / (cru_apll_con1_tmp & 0x3f)
               * (cru_apll_con0_tmp & 0xfff)
               / ((cru_apll_con0_tmp >> 12) & 0x7)
               / ((cru_apll_con1_tmp >> 6) & 0x7);
    rate_new = 24 / (cru_apll_con[1] & 0x3f)
               * (cru_apll_con[0] & 0xfff)
               / ((cru_apll_con[0] >> 12) & 0x7)
               / ((cru_apll_con[1] >> 6) & 0x7);
    if (rate_old <= rate_new) {
        writel_relaxed(cru_clksel_con[0], RK_CRU_VIRT + 0x0044);
        writel_relaxed(cru_clksel_con[1], RK_CRU_VIRT + 0x0048);
    }
    /* select GPLL div2 */
    writel_relaxed(0x00800080, RK_CRU_VIRT + 0x0044);
    for (i = 0; i < 3; i++)
        writel_relaxed(cru_apll_con[i], RK_CRU_VIRT + 0x0000 + (i * 4));
        /* CRU_APLL_CON1 wait pll lock */
        i = 24000000;
        while (i-- > 0) {
            if (readl_relaxed(RK_CRU_VIRT + 0x0004) & (1 << 10))
                break;
        }
        /* select APLL */
        writel_relaxed(0x00800000, RK_CRU_VIRT + 0x0044);
        if (rate_old > rate_new) {
            writel_relaxed(cru_clksel_con[0], RK_CRU_VIRT + 0x0044);
            writel_relaxed(cru_clksel_con[1], RK_CRU_VIRT + 0x0048);
        }
    }
    /* DDR PLL */
    writel_relaxed(0x00100000, RK_CRU_VIRT + 0x0040);
    for (i = 0; i < 3; i++)
        writel_relaxed(cru_dpll_con[i], RK_CRU_VIRT + 0x0010 + (i * 4));
    i = 24000000;
    while (i-- > 0) {
        if (readl_relaxed(RK_CRU_VIRT + 0x0014) & (1 << 10))
            break;
    }
    writel_relaxed(0x00100010, RK_CRU_VIRT + 0x0040);
#endif
    writel_relaxed(0x01000000, RK_CRU_VIRT + 0x0040);
    for (i = 0; i < 3; i++)
        writel_relaxed(cru_cpll_con[i], RK_CRU_VIRT + 0x0020 + (i * 4));
    i = 24000000;
    while (i-- > 0) {
        if (readl_relaxed(RK_CRU_VIRT + 0x0024) & (1 << 10))
            break;
    }
    writel_relaxed(0x01000100, RK_CRU_VIRT + 0x0040);
    writel_relaxed(0x10000000, RK_CRU_VIRT + 0x0040);
    for (i = 0; i < 3; i++)
        writel_relaxed(cru_gpll_con[i], RK_CRU_VIRT + 0x0030 + (i * 4));
    i = 24000000;
    while (i-- > 0) {
        if (readl_relaxed(RK_CRU_VIRT + 0x0034) & (1 << 10))
            break;
    }
    writel_relaxed(0x10001000, RK_CRU_VIRT + 0x0040);
    for (i = 2; i < 35; i++) {
        if ((i == 16) || (i == 21) || (i == 22) || (i == 33))
            continue;
        writel_relaxed(cru_clksel_con[i], RK_CRU_VIRT + 0x0044 + (i * 4));
    }
    for (i = 0; i < 11; i++)
        writel_relaxed(cru_clkgate_con[i], RK_CRU_VIRT + 0x00d0 + (i * 4));
    writel_relaxed(cru_glb_srst_fst_value, RK_CRU_VIRT + 0x0100);
    writel_relaxed(cru_glb_srst_snd_value, RK_CRU_VIRT + 0x0104);
#if 0
    for (i = 0; i < 9; i++)
        writel_relaxed(cru_softrst_con[i], RK_CRU_VIRT + 0x0110 + (i * 4));
#endif
    writel_relaxed(cru_misc_con, RK_CRU_VIRT + 0x0134);
    writel_relaxed(cru_glb_cnt_th, RK_CRU_VIRT + 0x0140);
#if 0
    writel_relaxed(cru_glb_rst_st, RK_CRU_VIRT + 0x0150);
    writel_relaxed(cru_sdmmc_con[0], RK_CRU_VIRT + 0x01c0);
    writel_relaxed(cru_sdmmc_con[1], RK_CRU_VIRT + 0x01c4);
    writel_relaxed(cru_sdio_con[0], RK_CRU_VIRT + 0x01c8);
    writel_relaxed(cru_sdio_con[1], RK_CRU_VIRT + 0x01cc);
    writel_relaxed(cru_emmc_con[0], RK_CRU_VIRT + 0x01d8);
    writel_relaxed(cru_emmc_con[1], RK_CRU_VIRT + 0x01dc);
    writel_relaxed(0xffff5a5a, RK_CRU_VIRT + 0x01f0);
#endif

    /* GRF */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            writel_relaxed(grf_gpio_iomux[i][j],
                           RK_GRF_VIRT + 0x0a8 + (i * 16) + (j * 4));
        }
    }
    writel_relaxed(grf_gpio2c_iomux2, RK_GRF_VIRT + 0x0e8);
    writel_relaxed(grf_cif_iomux[0], RK_GRF_VIRT + 0x0ec);
    writel_relaxed(grf_cif_iomux[1], RK_GRF_VIRT + 0x0f0);
    writel_relaxed(grf_gpio_ds, RK_GRF_VIRT + 0x100);
    for (i = 0; i < 8; i++)
        writel_relaxed(grf_gpio_pull[i], RK_GRF_VIRT + 0x118 + (i * 4));
    writel_relaxed(grf_acodec_con, RK_GRF_VIRT + 0x13c);
    for (i = 0; i < 3; i++)
        writel_relaxed(grf_soc_con[i], RK_GRF_VIRT + 0x140 + (i * 4));
    writel_relaxed(grf_lvds_con0, RK_GRF_VIRT + 0x150);
    for (i = 0; i < 3; i++)
        writel_relaxed(grf_dmac_con[i], RK_GRF_VIRT + 0x15c + (i * 4));
    for (i = 0; i < 2; i++)
        writel_relaxed(grf_mac_con[i], RK_GRF_VIRT + 0x168 + (i * 4));
    writel_relaxed(grf_tve_con, RK_GRF_VIRT + 0x170);
    writel_relaxed(grf_uoc0_con0, RK_GRF_VIRT + 0x17c);
    for (i = 0; i < 5; i++){
		/*	To avoid usb-phy freeze 	*/
		if ( ( i == 1) || ( i == 2)) continue;
        writel_relaxed(grf_uoc1_con[i], RK_GRF_VIRT + 0x184 + (i * 4));
	}
    writel_relaxed(grf_ddrc_stat, RK_GRF_VIRT + 0x19c);
    for (i = 0; i < 3; i++)
        writel_relaxed(grf_cpu_con[i], RK_GRF_VIRT + 0x1a8 + (i * 4));
    for (i = 0; i < 8; i++)
        writel_relaxed(grf_os_reg[i], RK_GRF_VIRT + 0x1c8 + (i * 4));
    for (i = 0; i < 4; i++)
        writel_relaxed(grf_pvtm_con[i], RK_GRF_VIRT + 0x200 + (i * 4));
    for (i = 0; i < 3; i++)
        writel_relaxed(grf_pvtm_status[i], RK_GRF_VIRT + 0x214 + (i * 4));
    writel_relaxed(grf_nif_fifo1, RK_GRF_VIRT + 0x234);
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 8; j++) {
            if (j == 5)
                continue;
            writel_relaxed(grf_usbphy_con[i][j],
                           RK_GRF_VIRT + 0x280 + (i * 32) + (j * 4));
        }
    }
#if 0
    writel_relaxed(grf_uoc_status0, RK_GRF_VIRT + 0x2c0);
#endif

    /* Timer */
    if (cru_clkgate7_con & (1 << 7))
        writel_relaxed(0xffff0000 | (cru_clkgate7_con & ~(1 << 7)),
                       RK_CRU_VIRT + 0x00d0 + (7 * 4));
    for (i = 0; i < 6; i++) {
        if (timer[i][4] & (1 << 0)) {
            writel_relaxed(0, RK_TIMER_VIRT + (i * 0x20) + 0x10);
            dsb();
            for (j = 0; j < 5; j++)
                writel(timer[i][j], RK_TIMER_VIRT + (i * 0x20) + (j * 4));
            dsb();
        }
    }
    if (cru_clkgate7_con & (1 << 7))
        writel_relaxed(0xffff0000 | cru_clkgate7_con,
                       RK_CRU_VIRT + 0x00d0 + (7 * 4));

#if 0
    /* PMU */
    for (i = 0; i < 18; i++)
        writel_relaxed(pmu_regs[i], RK_PMU_VIRT + (i * 4));

    /* PWM */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++)
            writel_relaxed(pwm_regs[i][j],
                           RK_PWM_VIRT + 0x0004 + (i * 16) + (j * 4));
    }
//    writel_relaxed(pwm_regs[4][0], RK_PWM_VIRT + 0x0040);
    writel_relaxed(pwm_regs[4][1], RK_PWM_VIRT + 0x0044);
#endif

    /* PMIC */
    warp_rk818_resume();

    /* UART */
    for (i = 0; i < 3; i++) {
        uart_addr = ioremap(uart_addrs[i], SZ_8K);
        if (!uart_addr)
            continue;
        if (uart_lcr[i]) {
            writel_relaxed(0x1, uart_addr + 0x88);	/* reset all */
            j = 0;
            while (readl_relaxed(uart_addr + 0x7c) & 0x1) {
                if (j++ > 15000)
                    break;
            }
            writel_relaxed(0x83, uart_addr + 0x0c);
            j = 0;
            while (!(readl_relaxed(uart_addr + 0x0c) & 0x80)) {
                if (j++ > 100)
                    break;
            }
            j = 0;
            while (readl_relaxed(uart_addr + 0x7c) & 0x1) {
                if (j++ > 15000)
                    break;
            }
            writel_relaxed(uart_dl[i] & 0xff, uart_addr + 0x00);
            writel_relaxed((uart_dl[i] >> 8) & 0xff, uart_addr + 0x04);
            j = 0;
            while (readl_relaxed(uart_addr + 0x7c) & 0x1) {
                if (j++ > 15000)
                    break;
            }
            writel_relaxed(uart_lcr[i], uart_addr + 0x0c);
            writel_relaxed(uart_ier[i], uart_addr + 0x04);
            writel_relaxed(0x7, uart_addr + 0x08);
            writel_relaxed(uart_mcr[i], uart_addr + 0x10);
        }
        if (uart_addr)
            iounmap(uart_addr);
    }

    return ret;
}

static struct warp_ops warp_machine_ops = {
    .snapshot = warp_snapshot,
#ifdef CONFIG_PM_WARP_DEBUG
    .putc = warp_putchar,
    .drv_init = warp_drv_init,
    .drv_uninit = warp_drv_uninit,
#endif
};

static int __init warp_machine_init (void)
{
    return warp_register_machine(&warp_machine_ops);
}

static void __exit warp_machine_exit (void)
{
    warp_unregister_machine(&warp_machine_ops);
}

module_init(warp_machine_init);
module_exit(warp_machine_exit);

