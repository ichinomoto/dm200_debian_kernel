/*
 * Copyright (C) 2014 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/rockchip/common.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/cru.h>
#include <linux/rockchip/dvfs.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/pmu.h>
/*#include <asm/cpuidle.h>*/
#include <asm/cputype.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "cpu_axi.h"
#include "loader.h"
#include "rk3126b.h"
#define CPU 312x
#include "sram.h"
#include "pm.h"
#include "pm-rk312x.c"
#include <linux/rockchip/cpu.h>
#define RK312X_DEVICE(name) \
	{ \
		.virtual	= (unsigned long) RK_##name##_VIRT, \
		.pfn		= __phys_to_pfn(RK312X_##name##_PHYS), \
		.length		= RK312X_##name##_SIZE, \
		.type		= MT_DEVICE, \
	}

static const char * const rk3126_dt_compat[] __initconst = {
	"rockchip,rk3126",
	NULL,
};

static const char * const rk3128_dt_compat[] __initconst = {
	"rockchip,rk3128",
	NULL,
};

#define RK312X_IMEM_VIRT (RK_BOOTRAM_VIRT + SZ_32K)
#define RK312X_TIMER5_VIRT (RK_TIMER_VIRT + 0xa0)

static struct map_desc rk312x_io_desc[] __initdata = {
	RK312X_DEVICE(CRU),
	RK312X_DEVICE(GRF),
	RK312X_DEVICE(ROM),
	RK312X_DEVICE(PMU),
	RK312X_DEVICE(EFUSE),
	RK312X_DEVICE(TIMER),
	RK312X_DEVICE(CPU_AXI_BUS),
	RK_DEVICE(RK_DEBUG_UART_VIRT, RK312X_UART2_PHYS, RK312X_UART_SIZE),
	RK_DEVICE(RK_DDR_VIRT, RK312X_DDR_PCTL_PHYS, RK312X_DDR_PCTL_SIZE),
	RK_DEVICE(RK_DDR_VIRT + RK312X_DDR_PCTL_SIZE, RK312X_DDR_PHY_PHYS, RK312X_DDR_PHY_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(0), RK312X_GPIO0_PHYS, RK312X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(1), RK312X_GPIO1_PHYS, RK312X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(2), RK312X_GPIO2_PHYS, RK312X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(3), RK312X_GPIO3_PHYS, RK312X_GPIO_SIZE),
	RK_DEVICE(RK_GIC_VIRT, RK312X_GIC_DIST_PHYS, RK312X_GIC_DIST_SIZE),
	RK_DEVICE(RK_GIC_VIRT + RK312X_GIC_DIST_SIZE, RK312X_GIC_CPU_PHYS, RK312X_GIC_CPU_SIZE),
	RK_DEVICE(RK312X_IMEM_VIRT, RK312X_IMEM_PHYS, RK312X_IMEM_SIZE),
	RK_DEVICE(RK_PWM_VIRT, RK312X_PWM_PHYS, RK312X_PWM_SIZE),
};

static void __init rk312x_boot_mode_init(void)
{
	u32 flag = readl_relaxed(RK_PMU_VIRT + RK312X_PMU_SYS_REG0);
	u32 mode = readl_relaxed(RK_PMU_VIRT + RK312X_PMU_SYS_REG1);
	u32 rst_st = readl_relaxed(RK_CRU_VIRT + RK312X_CRU_GLB_RST_ST);

	if (flag == (SYS_KERNRL_REBOOT_FLAG | BOOT_RECOVER))
		mode = BOOT_MODE_RECOVERY;
	if (rst_st & ((1 << 2) | (1 << 3)))
		mode = BOOT_MODE_WATCHDOG;

	rockchip_boot_mode_init(flag, mode);
}

static void usb_uart_init(void)
{
#ifdef CONFIG_RK_USB_UART
	u32 soc_status0 = readl_relaxed(RK_GRF_VIRT + RK312X_GRF_SOC_STATUS0);
#endif
	writel_relaxed(0x34000000, RK_GRF_VIRT + RK312X_GRF_UOC1_CON4);
#ifdef CONFIG_RK_USB_UART
	if (!(soc_status0 & (1 << 5)) && (soc_status0 & (1 << 8))) {
		/* software control usb phy enable */
		writel_relaxed(0x007f0055, RK_GRF_VIRT + RK312X_GRF_UOC0_CON0);
		writel_relaxed(0x34003000, RK_GRF_VIRT + RK312X_GRF_UOC1_CON4);
	}
#endif

	writel_relaxed(0x07, RK_DEBUG_UART_VIRT + 0x88);
	writel_relaxed(0x00, RK_DEBUG_UART_VIRT + 0x04);
	writel_relaxed(0x83, RK_DEBUG_UART_VIRT + 0x0c);
	writel_relaxed(0x0d, RK_DEBUG_UART_VIRT + 0x00);
	writel_relaxed(0x00, RK_DEBUG_UART_VIRT + 0x04);
	writel_relaxed(0x03, RK_DEBUG_UART_VIRT + 0x0c);
}

static void __init rk312x_dt_map_io(void)
{
	u32 v;

	iotable_init(rk312x_io_desc, ARRAY_SIZE(rk312x_io_desc));
	debug_ll_io_init();
	usb_uart_init();

	/* pmu reset by second global soft reset */
	v = readl_relaxed(RK_CRU_VIRT + RK312X_CRU_GLB_CNT_TH);
	v &= ~(3 << 12);
	v |= 1 << 12;
	writel_relaxed(v, RK_CRU_VIRT + RK312X_CRU_GLB_CNT_TH);

	/* enable timer5 for core */
	writel_relaxed(0, RK312X_TIMER5_VIRT + 0x10);
	dsb();
	writel_relaxed(0xFFFFFFFF, RK312X_TIMER5_VIRT + 0x00);
	writel_relaxed(0xFFFFFFFF, RK312X_TIMER5_VIRT + 0x04);
	dsb();
	writel_relaxed(1, RK312X_TIMER5_VIRT + 0x10);
	dsb();
	writel_relaxed(0x80000000, RK_CRU_VIRT + RK312X_CRU_MISC_CON);
	dsb();

	rk312x_boot_mode_init();
	rockchip_efuse_init();
}

static void __init rk3126_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK3126;

	rk312x_dt_map_io();

	if (readl_relaxed(RK_GRF_VIRT + RK312X_GRF_CHIP_TAG) == 0x3136)
		rockchip_soc_id = ROCKCHIP_SOC_RK3126B;
}

static void __init rk3128_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK3128;

	rk312x_dt_map_io();
}
static DEFINE_SPINLOCK(pmu_idle_lock);
static const u8 pmu_idle_map[] = {
	[IDLE_REQ_PERI] = 0,
	[IDLE_REQ_VIDEO] = 1,
	[IDLE_REQ_VIO] = 2,
	[IDLE_REQ_GPU] = 3,
	[IDLE_REQ_CORE] = 4,
	[IDLE_REQ_SYS] = 5,
	[IDLE_REQ_MSCH] = 6,
	[IDLE_REQ_CRYPTO] = 7,

};
static int rk312x_pmu_set_idle_request(enum pmu_idle_req req, bool idle)
{
	u32 val;
	unsigned long flags;
	u32 bit = pmu_idle_map[req];
	u32 idle_mask = BIT(bit) | BIT(bit + 16);
	u32 idle_target = (idle << bit) | (idle << (bit + 16));
	u32 mask = BIT(bit);

	spin_lock_irqsave(&pmu_idle_lock, flags);
	val = pmu_readl(RK312X_PMU_IDLE_REQ);
	if (idle)
		val |= mask;
	else
		val &= ~mask;
	pmu_writel(val, RK312X_PMU_IDLE_REQ);
	dsb();

	while (((pmu_readl(RK312X_PMU_IDLE_ST) & idle_mask) != idle_target))
		;
	spin_unlock_irqrestore(&pmu_idle_lock, flags);
	return 0;
}
static const u8 pmu_pd_map[] = {
	[PD_GPU] = 1,
	[PD_VIDEO] = 2,
	[PD_VIO] = 3,
};

static const u8 pmu_st_map[] = {
	[PD_GPU] = 1,
	[PD_VIDEO] = 2,
	[PD_VIO] = 3,
};

static noinline void rk312x_do_pmu_set_power_domain(enum pmu_power_domain domain
	, bool on)
{
	u8 pd = pmu_pd_map[domain];
	u32 val = pmu_readl(RK312X_PMU_PWRDN_CON);

	if (on)
		val &= ~BIT(pd);
	else
		val |=  BIT(pd);
	pmu_writel(val, RK312X_PMU_PWRDN_CON);
	dsb();

	while ((pmu_readl(RK312X_PMU_PWRDN_ST) & BIT(pmu_st_map[domain])) == on)
		;
}

static bool rk312x_pmu_power_domain_is_on(enum pmu_power_domain pd)
{
	/*1"b0: power on, 1'b1: power off*/
	return !(pmu_readl(RK312X_PMU_PWRDN_ST) & BIT(pmu_st_map[pd]));
}
static DEFINE_SPINLOCK(pmu_pd_lock);
static u32 rga_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 ebc_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 iep_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 lcdc0_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 vip0_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 gpu_qos[RK312X_CPU_AXI_QOS_NUM_REGS];
static u32 video_qos[RK312X_CPU_AXI_QOS_NUM_REGS];

#define SAVE_QOS(array, NAME) RK312X_CPU_AXI_SAVE_QOS(array, RK312X_CPU_AXI_##NAME##_QOS_VIRT)
#define RESTORE_QOS(array, NAME) RK312X_CPU_AXI_RESTORE_QOS(array, RK312X_CPU_AXI_##NAME##_QOS_VIRT)

static int rk312x_pmu_set_power_domain(enum pmu_power_domain pd, bool on)
{
	unsigned long flags;

	spin_lock_irqsave(&pmu_pd_lock, flags);
	if (rk312x_pmu_power_domain_is_on(pd) == on)
		goto out;
	if (!on) {
		if (pd == PD_GPU) {
			SAVE_QOS(gpu_qos, GPU);
			rk312x_pmu_set_idle_request(IDLE_REQ_GPU, true);
		} else if (pd == PD_VIO) {
			SAVE_QOS(rga_qos, VIO_RGA);
			if (!soc_is_rk3126b())
				SAVE_QOS(ebc_qos, VIO_EBC);
			SAVE_QOS(iep_qos, VIO_IEP);
			SAVE_QOS(lcdc0_qos, VIO_LCDC0);
			SAVE_QOS(vip0_qos, VIO_VIP0);
			rk312x_pmu_set_idle_request(IDLE_REQ_VIO, true);
		} else if (pd == PD_VIDEO) {
			SAVE_QOS(video_qos, VIDEO);
			rk312x_pmu_set_idle_request(IDLE_REQ_VIDEO, true);
		}
	}

	rk312x_do_pmu_set_power_domain(pd, on);

	if (on) {
		if (pd == PD_GPU) {
			rk312x_pmu_set_idle_request(IDLE_REQ_GPU, false);
			RESTORE_QOS(gpu_qos, GPU);
		} else if (pd == PD_VIO) {
			rk312x_pmu_set_idle_request(IDLE_REQ_VIO, false);
			RESTORE_QOS(rga_qos, VIO_RGA);
			if (!soc_is_rk3126b())
				RESTORE_QOS(ebc_qos, VIO_EBC);
			RESTORE_QOS(iep_qos, VIO_IEP);
			RESTORE_QOS(lcdc0_qos, VIO_LCDC0);
			RESTORE_QOS(vip0_qos, VIO_VIP0);
		} else if (pd == PD_VIDEO) {
			rk312x_pmu_set_idle_request(IDLE_REQ_VIDEO, false);
			RESTORE_QOS(video_qos, VIDEO);
		}
	}
out:
	spin_unlock_irqrestore(&pmu_pd_lock, flags);

	return 0;
}
extern void secondary_startup(void);
static int rk312x_sys_set_power_domain(enum pmu_power_domain pd, bool on)
{
	u32 clks_save[RK312X_CRU_CLKGATES_CON_CNT];
	u32 clks_ungating[RK312X_CRU_CLKGATES_CON_CNT];
	u32 i, ret = 0;

	for (i = 0; i < RK312X_CRU_CLKGATES_CON_CNT; i++) {
		clks_save[i] = cru_readl(RK312X_CRU_CLKGATES_CON(i));
		clks_ungating[i] = 0;
	}
	for (i = 0; i < RK312X_CRU_CLKGATES_CON_CNT; i++)
		cru_writel(0xffff0000, RK312X_CRU_CLKGATES_CON(i));

	if (on) {
#ifdef CONFIG_SMP
		if (pd >= PD_CPU_1 && pd <= PD_CPU_3) {
			writel_relaxed(0x20000 << (pd - PD_CPU_1),
				       RK_CRU_VIRT + RK312X_CRU_SOFTRSTS_CON(0));
			dsb();
			udelay(10);
			writel_relaxed(virt_to_phys(secondary_startup),
				       RK312X_IMEM_VIRT + 8);
			writel_relaxed(0xDEADBEAF, RK312X_IMEM_VIRT + 4);
			dsb_sev();
		}
#endif
	} else {
#ifdef CONFIG_SMP
		if (pd >= PD_CPU_1 && pd <= PD_CPU_3) {
			writel_relaxed(0x20002 << (pd - PD_CPU_1),
				       RK_CRU_VIRT + RK312X_CRU_SOFTRSTS_CON(0));
			dsb();
		}
#endif
	}

	if (((pd == PD_GPU) || (pd == PD_VIO) || (pd == PD_VIDEO)))
		ret = rk312x_pmu_set_power_domain(pd, on);

	for (i = 0; i < RK312X_CRU_CLKGATES_CON_CNT; i++) {
		cru_writel(clks_save[i] | 0xffff0000
			, RK312X_CRU_CLKGATES_CON(i));
	}

	return ret;
}

static void __init rk312x_dt_init_timer(void)
{
	rockchip_pmu_ops.set_power_domain = rk312x_sys_set_power_domain;
	rockchip_pmu_ops.power_domain_is_on = rk312x_pmu_power_domain_is_on;
	rockchip_pmu_ops.set_idle_request = rk312x_pmu_set_idle_request;
	of_clk_init(NULL);
	clocksource_of_init();
	of_dvfs_init();
}

static void __init rk312x_reserve(void)
{
	/* reserve memory for uboot */
	rockchip_uboot_mem_reserve();

	/* reserve memory for ION */
	rockchip_ion_reserve();
}

#ifdef CONFIG_PM
static u32 rk_pmu_pwrdn_st;

static void rk_pm_soc_pd_suspend(void)
{
	rk_pmu_pwrdn_st = pmu_readl(RK312X_PMU_PWRDN_ST);
	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_GPU])))
		rk312x_sys_set_power_domain(PD_GPU, false);

	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_VIO])))
		rk312x_sys_set_power_domain(PD_VIO, false);

	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_VIDEO])))
		rk312x_sys_set_power_domain(PD_VIDEO, false);
}

static void rk_pm_soc_pd_resume(void)
{
	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_VIDEO])))
		rk312x_sys_set_power_domain(PD_VIDEO, true);

	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_VIO])))
		rk312x_sys_set_power_domain(PD_VIO, true);

	if (!(rk_pmu_pwrdn_st & BIT(pmu_st_map[PD_GPU])))
		rk312x_sys_set_power_domain(PD_GPU, true);
}

static void __init rk312x_init_suspend(void)
{
	pr_info("%s\n", __func__);
	rkpm_pie_init();
	rk312x_suspend_init();
}
#endif

static void __init rk312x_init_late(void)
{
#ifdef CONFIG_PM
	rockchip_suspend_init();
	if (soc_is_rk3126b())
		rk3126b_init_suspend();
	else
		rk312x_init_suspend();
	rkpm_set_ops_pwr_dmns(rk_pm_soc_pd_suspend, rk_pm_soc_pd_resume);
#endif
	if (rockchip_jtag_enabled)
		clk_prepare_enable(clk_get_sys(NULL, "clk_jtag"));
}

static void rk312x_restart(char mode, const char *cmd)
{
	u32 boot_flag, boot_mode;

	rockchip_restart_get_boot_mode(cmd, &boot_flag, &boot_mode);

	/* for loader */
	writel_relaxed(boot_flag, RK_PMU_VIRT + RK312X_PMU_SYS_REG0);
	/* for linux */
	writel_relaxed(boot_mode, RK_PMU_VIRT + RK312X_PMU_SYS_REG1);

	dsb();

	/* pll enter slow mode */
	writel_relaxed(0x11010000, RK_CRU_VIRT + RK312X_CRU_MODE_CON);
	dsb();
	writel_relaxed(0xeca8, RK_CRU_VIRT + RK312X_CRU_GLB_SRST_SND_VALUE);
	dsb();
}

DT_MACHINE_START(RK3126_DT, "Rockchip RK3126")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk3126_dt_map_io,
	.init_time	= rk312x_dt_init_timer,
	.dt_compat	= rk3126_dt_compat,
	.init_late	= rk312x_init_late,
	.reserve	= rk312x_reserve,
	.restart	= rk312x_restart,
MACHINE_END

DT_MACHINE_START(RK3128_DT, "Rockchip RK3128")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk3128_dt_map_io,
	.init_time	= rk312x_dt_init_timer,
	.dt_compat	= rk3128_dt_compat,
	.init_late	= rk312x_init_late,
	.reserve	= rk312x_reserve,
	.restart	= rk312x_restart,
MACHINE_END

#if 1
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/tc3589x.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>

#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		0x04
#define GPIO_INTEN			0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INT_STATUS		0x40
#define GPIO_INT_RAWSTATUS	0x44
#define GPIO_DEBOUNCE		0x48
#define GPIO_PORTS_EOI		0x4c
#define GPIO_EXT_PORT		0x50
#define GPIO_LS_SYNC		0x60

static int get_gpio_b4(void)
{
	uint32_t val;

	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_EXT_PORT);
	return (val & 0x00001000) ? 1 : 0;
}

static int get_gpio_b5(void)
{
	uint32_t val;

	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_EXT_PORT);
	return (val & 0x00002000) ? 1 : 0;
}
static int get_gpio_b6(void)
{
	uint32_t val;

	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_EXT_PORT);
	return (val & 0x00004000) ? 1 : 0;
}
static int get_gpio_b7(void)
{
	uint32_t val;

	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_EXT_PORT);
	return (val & 0x00008000) ? 1 : 0;
}

static void key_gpio_init(void)
{
	uint32_t val;
	grf_readl(RK312X_GRF_GPIO0L_PULL);
	val = grf_readl(RK312X_GRF_GPIO0L_PULL);
	val |= 0xf000f000;
	grf_writel(val, RK312X_GRF_GPIO0L_PULL);
	return;
}

static void onxy2_reset_ctrl(int value)
{
	uint32_t val;

	/* tc35894 reset */
	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_SWPORT_DDR);
    reg_writel(val|0x02000000,RK_GPIO_VIRT(0)+GPIO_SWPORT_DDR);

	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_SWPORT_DR);
    reg_writel(val|0x02000000,RK_GPIO_VIRT(0)+GPIO_SWPORT_DR);
	msleep(1);
	val = reg_readl(RK_GPIO_VIRT(0)+GPIO_SWPORT_DR);
    reg_writel(val&(~0x02000000),RK_GPIO_VIRT(0)+GPIO_SWPORT_DR);
	msleep(1);
	return;
}

#if 0
static const unsigned int onxy2_keymap[] = {
	KEY(7, 0, KEY_ESC),
	KEY(6, 0, KEY_GRAVE),
	KEY(5, 0, KEY_F1),
	KEY(4, 0, KEY_F2),
	KEY(7, 2, KEY_F3),
	KEY(7, 6, KEY_F4),
	KEY(7, 9, KEY_F5),
	KEY(6, 9, KEY_F6),
	KEY(2, 8, KEY_F7),
	KEY(1, 8, KEY_F8),
	KEY(0, 8, KEY_F9),
	KEY(3, 7, KEY_F10),
	KEY(2, 7, KEY_INSERT),
	KEY(1, 7, KEY_DELETE),

	KEY(3, 0, KEY_1),
	KEY(2, 0, KEY_2),
	KEY(7, 1, KEY_3),
	KEY(6, 2, KEY_4),
	KEY(6, 6, KEY_5),
	KEY(5, 6, KEY_6),
	KEY(5, 9, KEY_7),
	KEY(3, 8, KEY_8),
	KEY(4, 7, KEY_9),
	KEY(4, 5, KEY_0),
	KEY(3, 5, KEY_MINUS),
	KEY(2, 5, KEY_EQUAL),
	KEY(1, 5, KEY_YEN),
	KEY(2, 4, KEY_BACKSPACE),

	KEY(1, 0, KEY_TAB),
	KEY(6, 1, KEY_Q),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_E),
	KEY(4, 2, KEY_R),
	KEY(4, 6, KEY_T),
	KEY(3, 6, KEY_Y),
	KEY(4, 9, KEY_U),
	KEY(4, 8, KEY_I),
	KEY(5, 7, KEY_O),
	KEY(6, 5, KEY_P),
	KEY(5, 5, KEY_LEFTBRACE),
	KEY(3, 4, KEY_RIGHTBRACE),
	KEY(5, 3, KEY_ENTER),

	KEY(0, 0, KEY_CAPSLOCK),
	KEY(4, 1, KEY_A),
	KEY(3, 1, KEY_S),
	KEY(3, 2, KEY_D),
	KEY(2, 2, KEY_F),
	KEY(2, 6, KEY_G),
	KEY(3, 9, KEY_H),
	KEY(5, 8, KEY_J),
	KEY(6, 7, KEY_K),
	KEY(7, 5, KEY_L),
	KEY(4, 4, KEY_SEMICOLON),
	KEY(7, 3, KEY_APOSTROPHE),
	KEY(6, 3, KEY_BACKSLASH),

	KEY(0, 11, KEY_LEFTSHIFT), /* GPIO0_B7 */
	KEY(2, 1, KEY_Z),
	KEY(1, 1, KEY_X),
	KEY(1, 2, KEY_C),
	KEY(0, 2, KEY_V),
	KEY(1, 6, KEY_B),
	KEY(2, 9, KEY_N),
	KEY(6, 8, KEY_M),
	KEY(7, 7, KEY_COMMA),
	KEY(6, 4, KEY_DOT),
	KEY(5, 4, KEY_SLASH),
	KEY(4, 3, KEY_RO),
	KEY(3, 3, KEY_UP),
	KEY(1, 11, KEY_RIGHTSHIFT),/* GPIO0_B4 */
	
	KEY(2, 11, KEY_LEFTCTRL), /* GPIO0_B6 */
	KEY(0, 4, KEY_LEFTMETA),
	KEY(3, 11, KEY_LEFTALT), /* GPIO0_B5 */
	KEY(0, 1, KEY_MUHENKAN),
	KEY(0, 6, KEY_SPACE),
	KEY(7, 8, KEY_HENKAN),
	KEY(7, 4, KEY_KATAKANAHIRAGANA),
	KEY(4, 11, KEY_RIGHTCTRL), /* GPIO0_B6 */
	KEY(2, 3, KEY_LEFT),
	KEY(1, 3, KEY_DOWN),
	KEY(0, 3, KEY_RIGHT),
};

#else
static const unsigned int onxy2_keymap[] = {
	KEY(6, 10,KEY_ESC),
	KEY(7, 10,KEY_GRAVE),
	KEY(7, 11,KEY_F1),
	KEY(7, 9, KEY_F2),
	KEY(5, 9, KEY_F3),
	KEY(6, 9, KEY_F4),
	KEY(4, 8, KEY_F5),
	KEY(6, 6, KEY_F6),
	KEY(5, 5, KEY_F7),
	KEY(7, 5, KEY_F8),
	KEY(7, 3, KEY_F9),
	KEY(3, 3, KEY_F10),
	KEY(7, 0, KEY_INSERT),
	KEY(7, 2, KEY_DELETE),

	KEY(3, 10,KEY_1),
	KEY(3, 11,KEY_2),
	KEY(3, 9, KEY_3),
	KEY(3, 8, KEY_4),
	KEY(7, 8, KEY_5),
	KEY(7, 7, KEY_6),
	KEY(3, 7, KEY_7),
	KEY(3, 6, KEY_8),
	KEY(3, 5, KEY_9),
	KEY(3, 4, KEY_0),
	KEY(7, 4, KEY_MINUS),
	KEY(7, 6, KEY_EQUAL),
	KEY(2, 3, KEY_YEN),
	KEY(5, 3, KEY_BACKSPACE),

	KEY(5, 10,KEY_TAB),
	KEY(2, 10,KEY_Q),
	KEY(2, 11,KEY_W),
	KEY(2, 9, KEY_E),
	KEY(2, 8, KEY_R),
	KEY(5, 8, KEY_T),
	KEY(5, 7, KEY_Y),
	KEY(2, 7, KEY_U),
	KEY(2, 6, KEY_I),
	KEY(2, 5, KEY_O),
	KEY(2, 4, KEY_P),
	KEY(5, 4, KEY_LEFTBRACE),
	KEY(5, 6, KEY_RIGHTBRACE),
	KEY(0, 3, KEY_ENTER),

	KEY(5, 11,KEY_CAPSLOCK),
	KEY(1, 10,KEY_A),
	KEY(1, 11,KEY_S),
	KEY(1, 9, KEY_D),
	KEY(1, 8, KEY_F),
	KEY(6, 8, KEY_G),
	KEY(6, 7, KEY_H),
	KEY(1, 7, KEY_J),
	KEY(1, 6, KEY_K),
	KEY(1, 5, KEY_L),
	KEY(1, 4, KEY_SEMICOLON),
	KEY(6, 4, KEY_APOSTROPHE),
	KEY(0, 4, KEY_BACKSLASH),

	KEY(4, 10,KEY_LEFTSHIFT), /* GPIO0_B7 */
	KEY(0, 10,KEY_Z),
	KEY(0, 11,KEY_X),
	KEY(0, 9, KEY_C),
	KEY(0, 8, KEY_V),
	KEY(4, 9, KEY_B),
	KEY(4, 7, KEY_N),
	KEY(0, 7, KEY_M),
	KEY(0, 6, KEY_COMMA),
	KEY(0, 5, KEY_DOT),
	KEY(4, 4, KEY_SLASH),
	KEY(4, 6, KEY_RO),
	KEY(6, 1, KEY_UP),
	KEY(2, 0, KEY_RIGHTSHIFT),/* GPIO0_B4 */

	KEY(4, 11,KEY_LEFTCTRL), /* GPIO0_B6 */
	KEY(6, 11,KEY_LEFTMETA),
	KEY(1, 0, KEY_LEFTALT), /* GPIO0_B5 */
	KEY(4, 5, KEY_MUHENKAN),
	KEY(6, 2, KEY_SPACE),
	KEY(6, 0, KEY_HENKAN),
	KEY(0, 0, KEY_KATAKANAHIRAGANA),
	KEY(3, 0, KEY_RIGHTCTRL), /* GPIO0_B6 */
	KEY(4, 1, KEY_LEFT),
	KEY(4, 2, KEY_DOWN),
	KEY(4, 0, KEY_RIGHT),
};
#endif

static struct matrix_keymap_data onxy2_keymap_data = {
	.keymap		= onxy2_keymap,
	.keymap_size    = ARRAY_SIZE(onxy2_keymap),
};

static struct tc3589x_keypad_platform_data tc35893_data = {
	.krow = 8,
	.kcol = 12,
	.debounce_period = TC_KPD_DEBOUNCE_PERIOD,
	.settle_time = TC_KPD_SETTLE_TIME,
	.irqtype = IRQF_TRIGGER_FALLING  | IRQF_ONESHOT,
	.enable_wakeup = true,
	.keymap_data    = &onxy2_keymap_data,
	.no_autorepeat  = false,
};

static struct tc3589x_platform_data tc3589x_keypad_data = {
	.block = TC3589x_BLOCK_KEYPAD,
	.keypad = &tc35893_data,
	.irq_base = 0,
	.irq_gpio = 7,
	.irq_gpio_lshift = 15,
	.get_gpio_lshift = get_gpio_b7,
	.irq_gpio_rshift = 12,
	.get_gpio_rshift = get_gpio_b4,
	.irq_gpio_ctrl   = 14,
	.get_gpio_ctrl   = get_gpio_b6,
	.irq_gpio_alt    = 13,
	.get_gpio_alt    = get_gpio_b5,
	.gpio_init = key_gpio_init,
	.reset = onxy2_reset_ctrl,
};

static struct i2c_board_info __initdata i2c0_devices_onxy2[] = {
	{
		I2C_BOARD_INFO("tc3589x", 0x45),
		.platform_data = &tc3589x_keypad_data,
		.flags = I2C_CLIENT_WAKE,
	},
};



static int __init rk312x_i2c_init(void)
{
	i2c_register_board_info(0, i2c0_devices_onxy2, ARRAY_SIZE(i2c0_devices_onxy2));
	return 0;
}
arch_initcall(rk312x_i2c_init);

#endif

char PIE_DATA(sram_stack)[1024];
EXPORT_PIE_SYMBOL(DATA(sram_stack));

static int __init rk312x_pie_init(void)
{
	int err;

	if (!cpu_is_rk312x())
		return 0;
	if (soc_is_rk3126b())
		return 0;

	err = rockchip_pie_init();
	if (err)
		return err;

	rockchip_pie_chunk = pie_load_sections(rockchip_sram_pool, rk312x);
	if (IS_ERR(rockchip_pie_chunk)) {
		err = PTR_ERR(rockchip_pie_chunk);
		pr_err("%s: failed to load section %d\n", __func__, err);
		rockchip_pie_chunk = NULL;
		return err;
	}

	rockchip_sram_virt = kern_to_pie(rockchip_pie_chunk, &__pie_common_start[0]);
	rockchip_sram_stack = kern_to_pie(rockchip_pie_chunk, (char *)DATA(sram_stack) + sizeof(DATA(sram_stack)));

	return 0;
}
arch_initcall(rk312x_pie_init);

#include "ddr_rk3126.c"
static int __init rk312x_ddr_init(void)
{
	if (soc_is_rk3128() || soc_is_rk3126()) {
		ddr_change_freq = _ddr_change_freq;
		ddr_round_rate = _ddr_round_rate;
		ddr_set_auto_self_refresh = _ddr_set_auto_self_refresh;
		ddr_bandwidth_get = _ddr_bandwidth_get;
		ddr_init(DDR3_DEFAULT, 0);
	}

	return 0;
}
arch_initcall_sync(rk312x_ddr_init);
