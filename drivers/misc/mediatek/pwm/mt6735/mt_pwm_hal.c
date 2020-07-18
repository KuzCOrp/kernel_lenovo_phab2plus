/*******************************************************************************
* mt_pwm.c PWM Drvier
*
* Copyright (c) 2012, Media Teck.inc
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public Licence,
* version 2, as publish by the Free Software Foundation.
*
* This program is distributed and in hope it will be useful, but WITHOUT
* ANY WARRNTY; without even the implied warranty of MERCHANTABITLITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*/

#include <linux/types.h>

#include <mt-plat/mt_pwm.h>
#include <mach/mt_pwm_hal_pub.h>
#include <mach/mt_pwm_hal.h>
#include <mach/mt_pwm_prv.h>
#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#else
#include <mach/mt_clkmgr.h>
#endif
#include <linux/dma-mapping.h>

#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>

/**********************************
* Global  data
***********************************/
enum {
	PWM_CON,
	PWM_HDURATION,
	PWM_LDURATION,
	PWM_GDURATION,
	PWM_BUF0_BASE_ADDR,
	PWM_BUF0_SIZE,
	PWM_BUF1_BASE_ADDR,
	PWM_BUF1_SIZE,
	PWM_SEND_DATA0,
	PWM_SEND_DATA1,
	PWM_WAVE_NUM,
	PWM_DATA_WIDTH,
	PWM_THRESH,
	PWM_SEND_WAVENUM,
	PWM_VALID
} PWM_REG_OFF;


#if defined(CONFIG_MTK_CLKMGR)
int pwm_power_id[] = {
		MT_CG_PERI_PWM1,
		MT_CG_PERI_PWM2,
		MT_CG_PERI_PWM3,
		MT_CG_PERI_PWM4,
		MT_CG_PERI_PWM5,
		MT_CG_PERI_PWM
	};

#define PWM_CG		5

#endif
#ifdef CONFIG_OF
unsigned long PWM_register[PWM_NUM] = {};
#else
unsigned long PWM_register[PWM_NUM] = {
	(PWM_BASE+0x0010),	   /* PWM1 register base,   15 registers */
	(PWM_BASE+0x0050),	   /* PWM2 register base    15 registers */
	(PWM_BASE+0x0090),	   /* PWM3 register base    15 registers */
	(PWM_BASE+0x00d0),	   /* PWM4 register base    13 registers */
	(PWM_BASE+0x0110),	   /* PWM5 register base    13 registers */
};
#endif
/**************************************************************/

#if !defined(CONFIG_MTK_CLKMGR)
enum {
	PWM1_CLK,
	PWM2_CLK,
	PWM3_CLK,
	PWM4_CLK,
	PWM5_CLK,
	PWM_CLK,
	PWM_CLK_NUM,
};

const char *pwm_clk_name[] = {
	"PWM1-main", "PWM2-main", "PWM3-main", "PWM4-main", "PWM5-main", "PWM-main"
};

struct clk *pwm_clk[PWM_CLK_NUM];

void mt_pwm_power_on_hal(u32 pwm_no, bool pmic_pad, unsigned long *power_flag)
{
	int clk_en_ret;
	/*Set pwm_main , pwm_hclk_main(for memory and random mode) */
	if (0 == (*power_flag)) {
		pr_warn("[PWM][CCF]enable clk PWM_CLK:%p\n", pwm_clk[PWM_CLK]);
		clk_en_ret = clk_prepare_enable(pwm_clk[PWM_CLK]);
		if (clk_en_ret) {
			pr_err("[PWM][CCF]enable clk PWM_CLK failed. ret:%d, clk_pwm_main:%p\n",
			clk_en_ret, pwm_clk[PWM_CLK]);
		} else
			set_bit(PWM_CLK, power_flag);

	}
	/* Set pwm_no clk */
	if (!test_bit(pwm_no, power_flag)) {
		pr_warn("[PWM][CCF]enable clk_pwm%d :%p\n", pwm_no, pwm_clk[pwm_no]);
		clk_en_ret = clk_prepare_enable(pwm_clk[pwm_no]);
		if (clk_en_ret) {
			pr_err("[PWM][CCF]enable clk_pwm_main failed. ret:%d, clk_pwm%d :%p\n",
			clk_en_ret, pwm_no, pwm_clk[pwm_no]);
		} else
			set_bit(pwm_no, power_flag);
	}

}

void mt_pwm_power_off_hal(u32 pwm_no, bool pmic_pad, unsigned long *power_flag)
{
	if (test_bit(pwm_no, power_flag)) {
		pr_debug("[PWM][CCF]disable clk_pwm%d :%p\n", pwm_no, pwm_clk[pwm_no]);
		clk_disable_unprepare(pwm_clk[pwm_no]);
		clear_bit(pwm_no, power_flag);
	}

	/*Disable PWM-main, PWM-HCLK-main */
	pr_debug("[PWM][CCF]disable clk_pwm :%p\n", pwm_clk[PWM_CLK]);

	if (test_bit(PWM_CLK, power_flag)) {
		clk_disable_unprepare(pwm_clk[PWM_CLK]);
		clear_bit(PWM_CLK, power_flag);
	}
}

#else
void mt_pwm_power_on_hal(u32 pwm_no, bool pmic_pad, unsigned long *power_flag)
{
	if (0 == (*power_flag)) {
		PWMDBG("enable_clock: main\n");
		enable_clock(pwm_power_id[PWM_CG], "PWM");
		set_bit(PWM_CG, power_flag);
	}
	if (!test_bit(pwm_no, power_flag)) {
		PWMDBG("enable_clock: %d\n", pwm_no);

			enable_clock(pwm_power_id[pwm_no], "PWM");

		set_bit(pwm_no, power_flag);
		PWMDBG("enable_clock PWM%d\n", pwm_no+1);
	}
}

void mt_pwm_power_off_hal(u32 pwm_no, bool pmic_pad, unsigned long *power_flag)
{
	if (test_bit(pwm_no, power_flag)) {
		PWMDBG("disable_clock: %d\n", pwm_no);
		disable_clock(pwm_power_id[pwm_no], "PWM");
		clear_bit(pwm_no, power_flag);
		PWMDBG("disable_clock PWM%d\n", pwm_no+1);
	}
	if (BIT(PWM_CG) == (*power_flag)) {
		PWMDBG("disable_clock: main\n");
		disable_clock(pwm_power_id[PWM_CG], "PWM");
		clear_bit(PWM_CG, power_flag);
	}
}
#endif

void mt_pwm_init_power_flag(unsigned long *power_flag)
{
#ifdef CONFIG_OF
	PWM_register[PWM1] = (unsigned long)pwm_base + 0x0010;
	PWM_register[PWM2] = (unsigned long)pwm_base + 0x0050;
	PWM_register[PWM3] = (unsigned long)pwm_base + 0x0090;
	PWM_register[PWM4] = (unsigned long)pwm_base + 0x00d0;
	PWM_register[PWM5] = (unsigned long)pwm_base + 0x0110;
#endif
}

s32 mt_pwm_sel_pmic_hal(u32 pwm_no)
{
	PWMDBG("mt_pwm_sel_pmic\n");
	return -EINVALID;
}

s32 mt_pwm_sel_ap_hal(u32 pwm_no)
{
	PWMDBG("mt_pwm_sel_ap\n");
	return -EINVALID;
}

void mt_set_pwm_enable_hal(u32 pwm_no)
{
	SETREG32(PWM_ENABLE, 1 << pwm_no);
}

void mt_set_pwm_disable_hal(u32 pwm_no)
{
	CLRREG32(PWM_ENABLE, 1 << pwm_no);
}

void mt_set_pwm_enable_seqmode_hal(void)
{
}

void mt_set_pwm_disable_seqmode_hal(void)
{
}

s32 mt_set_pwm_test_sel_hal(u32 val)
{
	return 0;
}

void mt_set_pwm_clk_hal(u32 pwm_no, u32 clksrc, u32 div)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	MASKREG32(reg_con, PWM_CON_CLKDIV_MASK, div);
	if ((clksrc & 0x80000000) != 0) {
		clksrc &= ~(0x80000000);
		if (clksrc == CLK_BLOCK_BY_1625_OR_32K) {	/* old mode: 26M/1625 = 16KHz */
			CLRREG32(reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET);	/* bit 4: 0 */
			SETREG32(reg_con, 1 << PWM_CON_CLKSEL_OFFSET);	/* bit 3: 1 */
		} else {	/* old mode 32k clk */
			SETREG32(reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET);
			SETREG32(reg_con, 1 << PWM_CON_CLKSEL_OFFSET);
		}
	} else {
		CLRREG32(reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET);
		if (clksrc == CLK_BLOCK)
			CLRREG32(reg_con, 1 << PWM_CON_CLKSEL_OFFSET);
		else if (clksrc == CLK_BLOCK_BY_1625_OR_32K)
			SETREG32(reg_con, 1 << PWM_CON_CLKSEL_OFFSET);
	}
}

s32 mt_get_pwm_clk_hal(u32 pwm_no)
{
	s32 clk, clksrc, clkdiv;
	unsigned long reg_con, reg_val, reg_en;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;

	reg_val = INREG32(reg_con);
	reg_en = INREG32(PWM_ENABLE);

	if (((reg_val & PWM_CON_CLKSEL_MASK) >> PWM_CON_CLKSEL_OFFSET) == 1)
		if (((reg_en & PWM_CON_OLD_MODE_MASK) >> PWM_CON_OLD_MODE_OFFSET) == 1)
			clksrc = 32 * 1024;
		else
			clksrc = BLOCK_CLK;
	else
		clksrc = BLOCK_CLK / 1625;

	clkdiv = 2 << (reg_val & PWM_CON_CLKDIV_MASK);
	if (clkdiv <= 0) {
		PWMDBG("clkdiv less zero, not valid\n");
		return -ERROR;
	}

	clk = clksrc / clkdiv;
	PWMDBG("CLK is :%d\n", clk);
	return clk;
}

s32 mt_set_pwm_con_datasrc_hal(u32 pwm_no, u32 val)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (val == PWM_FIFO)
		CLRREG32(reg_con, 1 << PWM_CON_SRCSEL_OFFSET);
	else if (val == MEMORY)
		SETREG32(reg_con, 1 << PWM_CON_SRCSEL_OFFSET);
	else
		return 1;
	return 0;
}

s32 mt_set_pwm_con_mode_hal(u32 pwm_no, u32 val)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (val == PERIOD)
		CLRREG32(reg_con, 1 << PWM_CON_MODE_OFFSET);
	else if (val == RAND)
		SETREG32(reg_con, 1 << PWM_CON_MODE_OFFSET);
	else
		return 1;
	return 0;
}

s32 mt_set_pwm_con_idleval_hal(u32 pwm_no, uint16_t val)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (val == IDLE_TRUE)
		SETREG32(reg_con, 1 << PWM_CON_IDLE_VALUE_OFFSET);
	else if (val == IDLE_FALSE)
		CLRREG32(reg_con, 1 << PWM_CON_IDLE_VALUE_OFFSET);
	else
		return 1;
	return 0;
}

s32 mt_set_pwm_con_guardval_hal(u32 pwm_no, uint16_t val)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (val == GUARD_TRUE)
		SETREG32(reg_con, 1 << PWM_CON_GUARD_VALUE_OFFSET);
	else if (val == GUARD_FALSE)
		CLRREG32(reg_con, 1 << PWM_CON_GUARD_VALUE_OFFSET);
	else
		return 1;
	return 0;
}

void mt_set_pwm_con_stpbit_hal(u32 pwm_no, u32 stpbit, u32 srcsel)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (srcsel == PWM_FIFO)
		MASKREG32(reg_con, PWM_CON_STOP_BITS_MASK, stpbit << PWM_CON_STOP_BITS_OFFSET);
	if (srcsel == MEMORY)
		MASKREG32(reg_con, PWM_CON_STOP_BITS_MASK & (0x1f << PWM_CON_STOP_BITS_OFFSET),
			  stpbit << PWM_CON_STOP_BITS_OFFSET);
}

s32 mt_set_pwm_con_oldmode_hal(u32 pwm_no, u32 val)
{
	unsigned long reg_con;

	reg_con = PWM_register[pwm_no] + 4 * PWM_CON;
	if (val == OLDMODE_DISABLE)
		CLRREG32(reg_con, 1 << PWM_CON_OLD_MODE_OFFSET);
	else if (val == OLDMODE_ENABLE)
		SETREG32(reg_con, 1 << PWM_CON_OLD_MODE_OFFSET);
	else
		return 1;
	return 0;
}

void mt_set_pwm_HiDur_hal(u32 pwm_no, uint16_t DurVal)
{				/* only low 16 bits are valid */
	unsigned long reg_HiDur;

	reg_HiDur = PWM_register[pwm_no] + 4 * PWM_HDURATION;
	OUTREG32(reg_HiDur, DurVal);
}

void mt_set_pwm_LowDur_hal(u32 pwm_no, uint16_t DurVal)
{
	unsigned long reg_LowDur;

	reg_LowDur = PWM_register[pwm_no] + 4 * PWM_LDURATION;
	OUTREG32(reg_LowDur, DurVal);
}

void mt_set_pwm_GuardDur_hal(u32 pwm_no, uint16_t DurVal)
{
	unsigned long reg_GuardDur;

	reg_GuardDur = PWM_register[pwm_no] + 4 * PWM_GDURATION;
	OUTREG32(reg_GuardDur, DurVal);
}

void mt_set_pwm_send_data0_hal(u32 pwm_no, u32 data)
{
	unsigned long reg_data0;

	reg_data0 = PWM_register[pwm_no] + 4 * PWM_SEND_DATA0;
	OUTREG32(reg_data0, data);
}

void mt_set_pwm_send_data1_hal(u32 pwm_no, u32 data)
{
	unsigned long reg_data1;

	reg_data1 = PWM_register[pwm_no] + 4 * PWM_SEND_DATA1;
	OUTREG32(reg_data1, data);
}

void mt_set_pwm_wave_num_hal(u32 pwm_no, uint16_t num)
{
	unsigned long reg_wave_num;

	reg_wave_num = PWM_register[pwm_no] + 4 * PWM_WAVE_NUM;
	OUTREG32(reg_wave_num, num);
}

void mt_set_pwm_data_width_hal(u32 pwm_no, uint16_t width)
{
	unsigned long reg_data_width;

	reg_data_width = PWM_register[pwm_no] + 4 * PWM_DATA_WIDTH;
	OUTREG32(reg_data_width, width);
}

void mt_set_pwm_thresh_hal(u32 pwm_no, uint16_t thresh)
{
	unsigned long reg_thresh;

	reg_thresh = PWM_register[pwm_no] + 4 * PWM_THRESH;
	OUTREG32(reg_thresh, thresh);
}

s32 mt_get_pwm_send_wavenum_hal(u32 pwm_no)
{
	unsigned long reg_send_wavenum;

	reg_send_wavenum = PWM_register[pwm_no] + 4 * PWM_SEND_WAVENUM;
	return INREG32(reg_send_wavenum);
}

void mt_set_intr_enable_hal(u32 pwm_intr_enable_bit)
{
	SETREG32(PWM_INT_ENABLE, 1 << pwm_intr_enable_bit);
}

s32 mt_get_intr_status_hal(u32 pwm_intr_status_bit)
{
	int ret;

	ret = INREG32(PWM_INT_STATUS);
	ret = (ret >> pwm_intr_status_bit) & 0x01;
	return ret;
}

void mt_set_intr_ack_hal(u32 pwm_intr_ack_bit)
{
	SETREG32(PWM_INT_ACK, 1 << pwm_intr_ack_bit);
}

void mt_set_pwm_buf0_addr_hal(u32 pwm_no, u32 addr)
{
	unsigned long reg_buff0_addr;

	reg_buff0_addr = PWM_register[pwm_no] + 4 * PWM_BUF0_BASE_ADDR;
	/*OUTREG32(reg_buff0_addr, addr);*/
	OUTREG32(reg_buff0_addr, addr);
}

void mt_set_pwm_buf0_size_hal(u32 pwm_no, uint16_t size)
{
	unsigned long reg_buff0_size;

	reg_buff0_size = PWM_register[pwm_no] + 4 * PWM_BUF0_SIZE;
	OUTREG32(reg_buff0_size, size);
}

void mt_pwm_dump_regs_hal(void)
{
	int i;
	unsigned long reg_val;

	reg_val = INREG32(PWM_ENABLE);
	PWMMSG("\r\n[PWM_ENABLE is:%lx]\n\r ", reg_val);
	reg_val = INREG32(PWM_CK_26M_SEL);
	PWMMSG("\r\n[PWM_26M_SEL is:%lx]\n\r ", reg_val);
	/*PWMDBG("peri pdn0 clock: 0x%x\n", INREG32(INFRA_PDN_STA0));*/

	for (i = PWM1; i < PWM_NUM; i++) {
		reg_val = INREG32(PWM_register[i] + 4 * PWM_CON);
		PWMMSG("\r\n[PWM%d_CON is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_HDURATION);
		PWMMSG("[PWM%d_HDURATION is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_LDURATION);
		PWMMSG("[PWM%d_LDURATION is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_GDURATION);
		PWMMSG("[PWM%d_GDURATION is:%lx]\r\n", i + 1, reg_val);

		reg_val = INREG32(PWM_register[i] + 4 * PWM_BUF0_BASE_ADDR);
		PWMMSG("\r\n[PWM%d_BUF0_BASE_ADDR is:%lx]\r\n", i, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_BUF0_SIZE);
		PWMMSG("\r\n[PWM%d_BUF0_SIZE is:%lx]\r\n", i, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_BUF1_BASE_ADDR);
		PWMMSG("\r\n[PWM%d_BUF1_BASE_ADDR is:%lx]\r\n", i, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_BUF1_SIZE);
		PWMMSG("\r\n[PWM%d_BUF1_SIZE is:%lx]\r\n", i + 1, reg_val);

		reg_val = INREG32(PWM_register[i] + 4 * PWM_SEND_DATA0);
		PWMMSG("[PWM%d_SEND_DATA0 is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_SEND_DATA1);
		PWMMSG("[PWM%d_PWM_SEND_DATA1 is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_WAVE_NUM);
		PWMMSG("[PWM%d_WAVE_NUM is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_DATA_WIDTH);
		PWMMSG("[PWM%d_WIDTH is:%lx]\r\n", i + 1, reg_val);

		reg_val = INREG32(PWM_register[i] + 4 * PWM_THRESH);
		PWMMSG("[PWM%d_THRESH is:%lx]\r\n", i + 1, reg_val);
		reg_val = INREG32(PWM_register[i] + 4 * PWM_SEND_WAVENUM);
		PWMMSG("[PWM%d_SEND_WAVENUM is:%lx]\r\n", i + 1, reg_val);

	}
}

void pwm_debug_store_hal(void)
{
	/* dump clock status */
	/*PWMDBG("peri pdn0 clock: 0x%x\n", INREG32(INFRA_PDN_STA0));*/
}

void pwm_debug_show_hal(void)
{
	mt_pwm_dump_regs_hal();
}

/*----------3dLCM support-----------*/
/*
 base pwm2, select pwm3&4&5 same as pwm2 or inversion of pwm2
 */
void mt_set_pwm_3dlcm_enable_hal(u8 enable)
{
	SETREG32(PWM_3DLCM, 1 << PWM_3DLCM_ENABLE_OFFSET);
}

/*
 set "pwm_no" inversion of pwm base or not
 */
void mt_set_pwm_3dlcm_inv_hal(u32 pwm_no, u8 inv)
{
	/*set "pwm_no" as auxiliary first */
	SETREG32(PWM_3DLCM, 1 << (pwm_no + 16));

	if (inv)
		SETREG32(PWM_3DLCM, 1 << (pwm_no + 1));
	else
		CLRREG32(PWM_3DLCM, 1 << (pwm_no + 1));
}

void mt_set_pwm_3dlcm_base_hal(u32 pwm_no)
{
	CLRREG32(PWM_3DLCM, 0x7F << 8);
	SETREG32(PWM_3DLCM, 1 << (pwm_no + 8));
}

void mt_pwm_26M_clk_enable_hal(u32 enable)
{
	unsigned long reg_con;

	/* select 66M or 26M */
	reg_con = (unsigned long)PWM_CK_26M_SEL;
	if (enable)
		SETREG32(reg_con, 1 << PWM_CK_26M_SEL_OFFSET);
	else
		CLRREG32(reg_con, 1 << PWM_CK_26M_SEL_OFFSET);

}

#if !defined(CONFIG_MTK_CLKMGR)
int mt_get_pwm_clk_src(struct platform_device *pdev)
{
	int i;

	for (i = PWM1_CLK; i < PWM_CLK_NUM; i++) {
		pwm_clk[i] = devm_clk_get(&pdev->dev, pwm_clk_name[i]);
		pr_err("[PWM] get %s clock, %p\n", pwm_clk_name[i], pwm_clk[i]);
		if (IS_ERR(pwm_clk[i])) {
			PWMDBG("cannot get %s clock\n", pwm_clk_name[i]);
			return PTR_ERR(pwm_clk[i]);
		}
	}
	return 0;
}
#endif

#if 1  //use for spi
extern s32 pwm_set_spec_config_for_spi(struct pwm_spec_config *conf_sclk, struct pwm_spec_config *conf_mosi);

void ultra_16_hal(u32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 16);
}

void ultra_24_hal(u32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 24);
}

void ultra_4_hal(u32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 4);
}

void mt_set_pwm_buf1_addr_hal (u32 pwm_no, u32 addr)
{
#if 1
	unsigned long reg_buff1_addr;
        reg_buff1_addr = PWM_register[pwm_no] + 4 * PWM_BUF1_BASE_ADDR;
	OUTREG32( reg_buff1_addr, addr );	//liulei modify
#endif
}

void mt_set_pwm_buf1_size_hal( u32 pwm_no, u16 size)
{
#if 1
	unsigned long reg_buff1_size;
        reg_buff1_size = PWM_register[pwm_no] + 4* PWM_BUF1_SIZE;
        OUTREG32 ( reg_buff1_size, size );
#endif
}

void mt_set_pwm_valid_hal ( u32 pwm_no, u32 buf_valid_bit )
{
	unsigned long reg_valid = 0;
	
	reg_valid = PWM_register[pwm_no] + 4* (PWM_VALID);

	SETREG32 ( reg_valid, 1 << buf_valid_bit );
}

void mt_set_pwm_delay_duration_hal(u32 pwm_no, u16 val)
{
	unsigned long reg_valid = 0;
	if ( pwm_no == 3 ) {
		reg_valid = (unsigned long)PWM3_DELAY;
	} else if ( pwm_no == 4 ) {
		reg_valid = (unsigned long)PWM4_DELAY;
	} else if ( pwm_no == 5 ) {
		reg_valid = (unsigned long)PWM4_DELAY;
	}

	MASKREG32 ( reg_valid, PWM_DELAY_DURATION_MASK, val );
}

void mt_set_pwm_delay_clock_hal (u32 pwm_no, u32 clksrc)
{
	unsigned long reg_valid = 0;
	if ( pwm_no == 3 ) {
		reg_valid = (unsigned long)PWM3_DELAY;
	} else if ( pwm_no == 4 ) {
		reg_valid = (unsigned long)PWM4_DELAY;
	} else if ( pwm_no == 5 ) {
		reg_valid = (unsigned long)PWM4_DELAY;
	}

	MASKREG32 (reg_valid, PWM_DELAY_CLK_MASK, clksrc );
}

void mt_set_pwm_enable_ahb_hal(u32 bit)
{
	SETREG32(CK_SEL, 1 << bit);
}


void mt_set_pwm_seqmode_multiport_enable_hal(u32 pwm_no_sclk, u32 pwm_no_mosi)
{
	u32 res = 1 << PWM_ENABLE_SEQ_OFFSET;
	res |= 1 << pwm_no_sclk;
	res |= 1 << pwm_no_mosi;
	printk("\r\n[PWM_ENABLE is:%x]\n\r ", res);
	SETREG32(PWM_ENABLE, res);
	
}

#define SPI_SCK_GPIO		21
#define SPI_SCK_GPIO_PWM_FUNCTION   2

#define SPI_MOSI_GPIO   	59
#define SPI_MOSI_GPIO_PWM_FUNCTION   5

void spi_gpio_pin_init(void)
{
	//set gpio60,201 to gpio mode,output
	mt_set_gpio_mode(SPI_MOSI_GPIO|0x80000000,0);
	mt_set_gpio_mode(SPI_SCK_GPIO|0x80000000,0);
	mt_set_gpio_dir_base(SPI_MOSI_GPIO, 1); 
	mt_set_gpio_dir_base(SPI_SCK_GPIO, 1); 

}
static void pwm_pin_init(void)
{
	//set gpio60,201 to pwm  mode
	mt_set_gpio_mode(SPI_MOSI_GPIO|0x80000000,SPI_MOSI_GPIO_PWM_FUNCTION);
	mt_set_gpio_mode(SPI_SCK_GPIO|0x80000000,SPI_SCK_GPIO_PWM_FUNCTION);

}
static unsigned char byte_reverse(unsigned char data)
{
	unsigned char data_rev = 0 ,i ,temp;
	for(i = 0; i <= 7; i++)
	{
		temp = (data>>i)&0x01 ;
		data_rev = (data_rev<<1)|temp ;
		
	}
	return data_rev ;
}

static void byte_two(void *val_two,void *val,size_t val_len)
{
	unsigned char *data_char= val ;
	unsigned char *data_two =(unsigned char *) val_two ;
	unsigned short data_short = 0;
	int i,j;
	unsigned char data;
	unsigned char temp ;
	for(j=0; j < val_len; j++)
	{
		data = *(unsigned char *)(data_char+j);
		data_short = 0 ;
		for(i = 8; i > 0; i--)
		{
			temp = (data>>(i-1))&0x01 ;
			if(temp == 1)
				data_short = data_short|(0x03<<(2*(i-1)));	
			
		}
		data_two[2*j] = (unsigned char)((data_short>>8)&0xFF) ;
		data_two[2*j+1]=  (unsigned char)(data_short&0xFF) ;
	}

}
static unsigned char pwm_data_jy[1000*1024];

#if 0
static void debug_packet(struct device *dev, char *name, u8 *ptr,
			 int len)
{
	int i;

	dev_err(dev, "%s: ", name);
	for (i = 0; i < len; i++)
		printk(" %02x", ptr[i]);
	printk("\n");
}
#endif

int pwm_write( struct device *dev ,const void *val, size_t val_len)
{
//	printk(" pwm_write length val_len = %lx \n" ,val_len);
 //   unsigned long flags;
void *virt_2;
dma_addr_t phys;
void *virt;
int clk_len;
dma_addr_t phys_2;
int data_len;
unsigned char *membuff_clk;
unsigned char *membuff_data;
unsigned char *raw_data;
int i;
int ret = 0;
struct pwm_spec_config conf_1 ,conf_2 ;
struct platform_device *pdev = to_platform_device(dev);

	memset(&conf_1, 0, sizeof(struct pwm_spec_config));
	memset(&conf_2, 0, sizeof(struct pwm_spec_config));

//debug_packet(&pdev->dev, "pwm_write(tx data)", (char *)val, val_len);

	printk("enter pwm_write \n" );
//printk("mt_get_gpio_mode(60|0x80000000)_1: %d\n" ,mt_get_gpio_mode(60|0x80000000));
//printk("mt_get_gpio_mode(201|0x80000000)_1: %d\n" ,mt_get_gpio_mode(201|0x80000000));
	pwm_pin_init();
//printk("mt_get_gpio_mode(60|0x80000000)_2: %d\n" ,mt_get_gpio_mode(60|0x80000000));
//printk("mt_get_gpio_mode(201|0x80000000)_2: %d\n" ,mt_get_gpio_mode(201|0x80000000));
	// Memory Setting //
	 data_len = val_len*2 ;
	//struct platform_device *pdev_2 = to_platform_device(dev);
	virt_2 = dma_alloc_coherent(&pdev->dev, data_len+4, &phys_2, GFP_KERNEL);
	if (virt_2 == NULL) {
		printk("dma memory allocte error!!!\n");
		return -ENOMEM;
	}
	
	 clk_len = val_len*2 ;
	//struct platform_device *pdev = to_platform_device(dev);
	virt = dma_alloc_coherent(&pdev->dev, clk_len+4, &phys, GFP_KERNEL);  // ??500KB+4Byes????
	if (virt == NULL) {
		printk("dma memory allocte error!!!\n");
		return -ENOMEM;
	}
	//printk("clk,phys(0x%llx),data, phys_2(0x%llx)\n", phys, phys_2);
	membuff_clk = virt;
	membuff_data = virt_2;
	raw_data = (unsigned char *)val;

    memset(pwm_data_jy,data_len,0x01);
	byte_two(pwm_data_jy,(unsigned char *)raw_data,val_len);
	
	//unsigned char *membuff = val;
	// PWM Clock Channel buffer1
	
	for (i = 0; i < data_len; i++){

		membuff_data[i] = byte_reverse(pwm_data_jy[i]); 
	}
    membuff_data[data_len] = 0x00;
    membuff_data[data_len+1] = 0x00; 
    membuff_data[data_len+2] = 0x00; 
    membuff_data[data_len+3] = 0x00; 
    data_len = data_len +4 ; 
	
	//debug_packet(&pdev->dev, "pwm_write(membuff_data)", membuff_data, data_len);

    membuff_clk[0] = 0x00; 
    membuff_clk[1] = 0x00; 
    clk_len = clk_len +4 ;
	for (i = 2; i < clk_len-2 ; i++){
		membuff_clk[i] = 0xaa; // ??????4Bytes????

	}	
    membuff_clk[clk_len-2] = 0x00;
    membuff_clk[clk_len-1] = 0x00;

	//printk("clk_len(%d)\n", clk_len);
	//debug_packet(&pdev->dev, "pwm_write(membuff_clk)", membuff_clk, clk_len);
/*
	for (i = 0; i < clk_len ; i++){
		membuff_clk[i] = 0xaa; // ??????4Bytes????

	}	*/
  //pwm log .verify the data and clk 
  /*
   	//cccc
   	for (i=0; i < (val_len/16);i++)
    {
      printk("raw_data[%d] 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x \n", 
	  	                            i,  raw_data[i*16],raw_data[i*16+1],raw_data[i*16+2],raw_data[i*16+3],raw_data[i*16+4],raw_data[i*16+5],raw_data[i*16+6],raw_data[i*16+7], 
		                              raw_data[i*16+8],raw_data[i*16+9],raw_data[i*16+10],raw_data[i*16+11],raw_data[i*16+12],raw_data[i*16+13],raw_data[i*16+14],raw_data[i*16+15]);
   	}
	for (i=0; i < (val_len%16);i++)
	{
	 printk("raw_data[i] 0x%x \n ",raw_data[((val_len/16)* 16)+i]);
	}
	
   	printk("dclk_len=%d,data_len=%d   !!!\n", clk_len,data_len);
   	for (i=0; i < (clk_len/16);i++)
    {
      printk("membuff_clk[%d] 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", 
	  	                             i, membuff_clk[i*16],membuff_clk[i*16+1],membuff_clk[i*16+2],membuff_clk[i*16+3],membuff_clk[i*16+4],membuff_clk[i*16+5],membuff_clk[i*16+6],membuff_clk[i*16+7], 
		                              membuff_clk[i*16+8],membuff_clk[i*16+9],membuff_clk[i*16+10],membuff_clk[i*16+11],membuff_clk[i*16+12],membuff_clk[i*16+13],membuff_clk[i*16+14],membuff_clk[i*16+15]);
   	}

	for (i=0; i < (clk_len%16);i++)
	{
	 printk("membuff_clk[i] 0x%x \n ",membuff_clk[((clk_len/16)* 16)+i]);
	}
   	for (i=0; i < (data_len/16);i++)
    {
      printk("membuff_data[%d] 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x \n", 
	  	                            i,  membuff_data[i*16],membuff_data[i*16+1],membuff_data[i*16+2],membuff_data[i*16+3],membuff_data[i*16+4],membuff_data[i*16+5],membuff_data[i*16+6],membuff_data[i*16+7], 
		                              membuff_data[i*16+8],membuff_data[i*16+9],membuff_data[i*16+10],membuff_data[i*16+11],membuff_data[i*16+12],membuff_data[i*16+13],membuff_data[i*16+14],membuff_data[i*16+15]);
   	}

	for (i=0; i < (data_len%16);i++)
	{
	 printk("membuff_data[i] 0x%x \n ",membuff_data[((data_len/16)* 16)+i]);
	}  
	*/

	if(clk_len <= 64000*4) //CH2 and CH3  memory mode
	{
		// PWM Con-Register Setting //
		// clock port setting
		conf_1.mode = PWM_MODE_MEMORY;
		conf_1.pwm_no = 2;
		conf_1.clk_div = CLK_DIV1;
		conf_1.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_1.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_1.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_1.PWM_MODE_MEMORY_REGS.HDURATION = 3;
		conf_1.PWM_MODE_MEMORY_REGS.LDURATION = 3;
		conf_1.PWM_MODE_MEMORY_REGS.GDURATION = 0;
		conf_1.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1;
		conf_1.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31;
		conf_1.PWM_MODE_MEMORY_REGS.BUF0_SIZE = clk_len/4-1  ; // 0~63999  1??32bit????
		conf_1.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = (u32)phys;

#if 1
		// data port setting
		conf_2.mode = PWM_MODE_MEMORY;
		conf_2.pwm_no = 3;
		conf_2.clk_div = CLK_DIV1;
		conf_2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_2.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_2.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_2.PWM_MODE_MEMORY_REGS.HDURATION = 3;
		conf_2.PWM_MODE_MEMORY_REGS.LDURATION = 3;
		conf_2.PWM_MODE_MEMORY_REGS.GDURATION = 0;
		conf_2.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1;
		conf_2.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31;
		conf_2.PWM_MODE_MEMORY_REGS.BUF0_SIZE = data_len/4-1 ;
		conf_2.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = (u32)phys_2;
		
		conf_2.PWM_MODE_MEMORY_REGS.PWM_MEMORY_DUR = 2 + 64 ;//80 ;//64 ;//48;  // delay
		conf_2.PWM_MODE_MEMORY_REGS.PWM_MEMORY_CLK = 0;   // delay clock
#else
// data port setting fifo mode
conf_2.mode = PWM_MODE_FIFO;
conf_2.pwm_no = 4;
conf_2.clk_div = CLK_DIV1;
conf_2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
conf_2.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE;
conf_2.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE;
conf_2.PWM_MODE_FIFO_REGS.HDURATION = 1;
conf_2.PWM_MODE_FIFO_REGS.LDURATION = 1;
conf_2.PWM_MODE_FIFO_REGS.GDURATION = 0;
conf_2.PWM_MODE_FIFO_REGS.WAVE_NUM = 1;
conf_2.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31;
conf_2.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xaaaaaaaa;
conf_2.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xaaaaaaaa;
#endif
	}
	else if(clk_len > 64000*4)//CH2 and CH3  memory mode
	{		
		// PWM Con-Register Setting //
		// clock port setting
		conf_1.mode = PWM_MODE_RANDOM;
		conf_1.pwm_no = 2;
		conf_1.clk_div = CLK_DIV1;
		conf_1.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_1.PWM_MODE_RANDOM_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_1.PWM_MODE_RANDOM_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_1.PWM_MODE_RANDOM_REGS.HDURATION = 2;
		conf_1.PWM_MODE_RANDOM_REGS.LDURATION = 2;
		conf_1.PWM_MODE_RANDOM_REGS.GDURATION = 0;
		conf_1.PWM_MODE_RANDOM_REGS.WAVE_NUM = 1;
		conf_1.PWM_MODE_RANDOM_REGS.STOP_BITPOS_VALUE = 31;
		conf_1.PWM_MODE_RANDOM_REGS.BUF0_SIZE = 63999;	 // 0~63999  1??32bit????
		conf_1.PWM_MODE_RANDOM_REGS.BUF0_BASE_ADDR =(u32) phys;
		conf_1.PWM_MODE_RANDOM_REGS.BUF1_SIZE = (clk_len/4)-64000;
		conf_1.PWM_MODE_RANDOM_REGS.BUF1_BASE_ADDR = (u32)(phys + 64000);

		// data port setting
		conf_2.mode = PWM_MODE_RANDOM;
		conf_2.pwm_no = 3;
		conf_2.clk_div = CLK_DIV1;
		conf_2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_2.PWM_MODE_RANDOM_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_2.PWM_MODE_RANDOM_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_2.PWM_MODE_RANDOM_REGS.HDURATION = 2;
		conf_2.PWM_MODE_RANDOM_REGS.LDURATION = 2;
		conf_2.PWM_MODE_RANDOM_REGS.GDURATION = 0;
		conf_2.PWM_MODE_RANDOM_REGS.WAVE_NUM = 1;
		conf_2.PWM_MODE_RANDOM_REGS.STOP_BITPOS_VALUE = 31;
		conf_2.PWM_MODE_RANDOM_REGS.BUF0_SIZE = 63999;
		conf_2.PWM_MODE_RANDOM_REGS.BUF0_BASE_ADDR = (u32)phys_2;
		conf_2.PWM_MODE_RANDOM_REGS.BUF1_SIZE = (data_len/4)-64000;
		conf_2.PWM_MODE_RANDOM_REGS.BUF1_BASE_ADDR = (u32)(phys + 64000);
		
		conf_2.PWM_MODE_RANDOM_REGS.PWM_RANDOM_DUR = 2 ;//48;  // delay
		conf_2.PWM_MODE_RANDOM_REGS.PWM_RANDOM_CLK = 0;   // delay clock
		
	}else{
		printk("DATA length is bigger than 256KB,and CLK length is bigger than 512KB!\n");
	
	}	
	ret = pwm_set_spec_config_for_spi(&conf_1, &conf_2);//liulei
	spi_gpio_pin_init();
	/*
   	for (i=0; i < (data_len/16);i++)
    {
      printk("membuff_data[%d] 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x \n", 
	  	                            i,  membuff_data[i*16],membuff_data[i*16+1],membuff_data[i*16+2],membuff_data[i*16+3],membuff_data[i*16+4],membuff_data[i*16+5],membuff_data[i*16+6],membuff_data[i*16+7], 
		                              membuff_data[i*16+8],membuff_data[i*16+9],membuff_data[i*16+10],membuff_data[i*16+11],membuff_data[i*16+12],membuff_data[i*16+13],membuff_data[i*16+14],membuff_data[i*16+15]);
   	}

	for (i=0; i < (data_len%16);i++)
	{
	 printk("membuff_data[i] 0x%x \n ",membuff_data[((data_len/16)* 16)+i]);
	}  
	*/
	dma_free_coherent(&pdev->dev, data_len, virt_2,phys_2);
    dma_free_coherent(&pdev->dev, data_len, virt,phys);
	return ret;
}
EXPORT_SYMBOL(pwm_write);

#endif

