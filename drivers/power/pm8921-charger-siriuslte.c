/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#endif


#if defined(CONFIG_PANTECH_SMB_CHARGER)
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <linux/switch.h>
//#define CONFIG_PANTECH_SMB_DEBUG 
#endif

#include <mach/msm_xo.h>

#if defined(CONFIG_PANTECH_PMIC)
#include <linux/usb/msm_hsusb.h>
#else
#include <mach/msm_hsusb.h>
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#if defined(CONFIG_PANTECH_CHARGER)
#include <mach/system.h>
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
#include <linux/input.h>
#endif

#if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST) || defined(PANTECH_CHARGER_MONITOR_TEST)
#include <linux/types.h>
#include <linux/ioctl.h>
#endif

#if defined(CONFIG_PANTECH_PMIC)
#include <mach/msm_smsm.h>
#include <mach/restart.h>

typedef enum {
  CHG_PM_OFFLINE_NORMAL_BOOT_MODE, //OEM_PM_OFFLINE_NORMAL_BOOT_MODE,
  CHG_PM_ONLINE_NORMAL_BOOT_MODE, //OEM_PM_ONLINE_NORMAL_BOOT_MODE
  CHG_PM_ONLINE_FACTORY_CABLE_BOOT_MODE, //OEM_PM_ONLINE_FACTORY_CABLE_BOOT_MODE
  CHG_PM_ONLINE_CABLE_IN_BOOT_MODE, //OEM_PM_ONLINE_CABLE_IN_BOOT_MODE
  CHG_PM_ONLINE_SILENT_BOOT_MODE, //OEM_PM_ONLINE_SILENT_BOOT_MODE
} chg_pm_power_on_mode_type;

static oem_pm_smem_vendor1_data_type *smem_id_vendor1_ptr;

static int chg_pm_read_proc_reset_info(void)
{
	int len = 0;

	smem_id_vendor1_ptr = (oem_pm_smem_vendor1_data_type*)smem_alloc(SMEM_ID_VENDOR1,
		sizeof(oem_pm_smem_vendor1_data_type));

	return len;
}
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/irqs.h>
#include <mach/board.h>

#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_IRQ_BASE (NR_MSM_IRQS + NR_GPIO_IRQS)
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)

#define SMB347_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG_0x00			0x00
#define INPUT_CURRENT_LIMIT_REG_0x01		0x01
#define VAR_FUNC_REG_0x02			0x02
#define FLOAT_VOLTAGE_REG_0x03			0x03
#define CHG_CTRL_REG_0x04			0x04
#define STAT_TIMER_REG_0x05			0x05
#define PIN_ENABLE_CTRL_REG_0x06		0x06
#define THERM_CTRL_A_REG_0x07			0x07
#define SYSOK_USB3_SELECT_REG_0x08		0x08
#define OTHER_CTRL_REG_0x09			0x09
#define OTG_TLIM_THERM_CNTRL_REG_0x0A		0x0A
#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG_0x0B 0x0B
#define FAULT_IRQ_REG_0x0C			0x0C
#define STATUS_IRQ_REG_0x0D			0x0D
#define I2C_BUS_SLAVE_ADDR_REG_0x0E		0x0E
#define CMD_A_REG_0x30		0x30
#define CMD_B_REG_0x31		0x31
#define CMD_C_REG_0x33		0x33
#define IRQ_A_REG_0x35		0x35
#define IRQ_B_REG_0x36		0x36
#define IRQ_C_REG_0x37		0x37
#define IRQ_D_REG_0x38		0x38
#define IRQ_E_REG_0x39		0x39
#define IRQ_F_REG_0x3A		0x3A
#define STATUS_A_REG_0x3B	0x3B
#define STATUS_B_REG_0x3C	0x3C
#define STATUS_C_REG_0x3D	0x3D
#define STATUS_D_REG_0x3E	0x3E
#define STATUS_E_REG_0x3F	0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK		SMB347_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT		BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK			SMB347_MASK(3, 5)
#define PRE_CHG_CURRENT_MASK			SMB347_MASK(2, 3)
#define TERMINATION_CURRENT_MASK		SMB347_MASK(3, 0)
#define USBIN_INPUT_CURRENT_LIMIT_MASK		SMB347_MASK(4, 0)
#define DC_INPUT_CURRENT_LIMIT_MASK		SMB347_MASK(4, 4)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK		SMB347_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK			SMB347_MASK(6, 0)
#define ENABLE_PIN_CONTROL_MASK			SMB347_MASK(2, 5)
#define CHG_ENABLE_BIT			BIT(1)
#define VOLATILE_W_PERM_BIT		BIT(7)
#define USB_SELECTION_BIT		BIT(1)
#define SYSTEM_FET_ENABLE_BIT	BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT			BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT	BIT(2)
#define BATT_OV_END_CHG_BIT		BIT(1)
#define VCHG_FUNCTION			BIT(0)
#define CURR_TERM_END_CHG_BIT	BIT(6)
#endif

#if defined(CONFIG_PANTECH_PMIC_MAX17058) || defined(CONFIG_PANTECH_SMB_CHARGER)
#include <linux/i2c.h>
#endif
#if defined(CONFIG_PANTECH_PMIC_MAX17058)

/* -------------------------------------------------------------------- */
/* max17058 i2c slave address */
/* -------------------------------------------------------------------- */
#define MAX17058_I2C_ADDR			0x36
/* -------------------------------------------------------------------- */
/* max17058 buf max size */
/* -------------------------------------------------------------------- */
#define MAX17058_BUF_MAX			32
/* -------------------------------------------------------------------- */
/* max17058 Rcomp */
/* -------------------------------------------------------------------- */
#if defined(T_STARQ)
#define OEM_BATTERY_STD_RCOMP	0xB0
#define OEM_BATTERY_EXT_RCOMP	0xE0
#elif defined(T_OSCAR)
#define OEM_BATTERY_STD_RCOMP	0x4D
#define OEM_BATTERY_EXT_RCOMP	0x60
#elif defined(T_SIRIUSLTE)
#define OEM_BATTERY_STD_RCOMP	0x47
#define OEM_BATTERY_EXT_RCOMP	0x47
#elif defined(T_EF44S)
#define OEM_BATTERY_STD_RCOMP	0x70
#define OEM_BATTERY_EXT_RCOMP	0x70
#elif defined(T_VEGAPVW)
#define OEM_BATTERY_STD_RCOMP	0x5C
#define OEM_BATTERY_EXT_RCOMP	0x90
#elif defined(T_MAGNUS)
#define OEM_BATTERY_STD_RCOMP	0x60
#define OEM_BATTERY_EXT_RCOMP	0x57
#else
#define OEM_BATTERY_STD_RCOMP	0x70
#define OEM_BATTERY_EXT_RCOMP	0x70
#endif

/* -------------------------------------------------------------------- */
/* max17058 SOC calculation */
/* -------------------------------------------------------------------- */
#if defined(T_OSCAR)
#define OEM_MAX17058_STD_FULL		(970)
#define OEM_MAX17058_STD_EMPTY	(3)
#define OEM_MAX17058_EXT_FULL		(980)
#define OEM_MAX17058_EXT_EMPTY	(8)
#elif defined(T_STARQ)
#define OEM_MAX17058_STD_FULL		(960)
#define OEM_MAX17058_STD_EMPTY	(14)
#define OEM_MAX17058_EXT_FULL		(960)
#define OEM_MAX17058_EXT_EMPTY	(14)
#elif defined(T_EF44S)
#define OEM_MAX17058_STD_FULL		(978)
#define OEM_MAX17058_STD_EMPTY	(8)
#define OEM_MAX17058_EXT_FULL		(978)
#define OEM_MAX17058_EXT_EMPTY	(8)
#elif defined(T_VEGAPVW)
#define OEM_MAX17058_STD_FULL		(977)
#define OEM_MAX17058_STD_EMPTY	(28)
#define OEM_MAX17058_EXT_FULL		(999)
#define OEM_MAX17058_EXT_EMPTY	(7)
#elif defined(T_MAGNUS)
#define OEM_MAX17058_STD_FULL		(980)
#define OEM_MAX17058_STD_EMPTY	(6)
#define OEM_MAX17058_EXT_FULL		(982)
#define OEM_MAX17058_EXT_EMPTY	(28)
#elif defined(T_SIRIUSLTE)
#define OEM_MAX17058_STD_FULL		(977)
#define OEM_MAX17058_STD_EMPTY	(27)
#define OEM_MAX17058_EXT_FULL		(977)
#define OEM_MAX17058_EXT_EMPTY	(27)
#else
#define OEM_MAX17058_STD_FULL		(1020)
#define OEM_MAX17058_STD_EMPTY	(5)
#define OEM_MAX17058_EXT_FULL		(1020)
#define OEM_MAX17058_EXT_EMPTY	(14)
#endif

#if defined(T_STARQ)
#define OEM_SOC(msb, lsb) ((msb * 1000) + (lsb * 1000 / 256))
#else
#define OEM_SOC(msb, lsb) ((msb * 500) + (lsb * 500 / 256))
#endif

#if defined(T_STARQ)
#define OEM_STD_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 100) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)))
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#elif defined(T_OSCAR)
#define OEM_STD_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 100) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)))
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#elif defined(T_SIRIUSLTE)
#define OEM_STD_ADJUSED_SOC(oem_soc) ( ((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 10000) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)) )
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#elif defined(T_EF44S)
#define OEM_STD_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 100) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)))
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#elif defined(T_MAGNUS)
#define OEM_STD_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 100) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)))
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#else
#define OEM_STD_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_STD_EMPTY * 100)) * 100) / ((OEM_MAX17058_STD_FULL * 100) - (OEM_MAX17058_STD_EMPTY * 100)))
#define OEM_EXT_ADJUSED_SOC(oem_soc) (((oem_soc - (OEM_MAX17058_EXT_EMPTY * 100)) * 100) / ((OEM_MAX17058_EXT_FULL * 100) - (OEM_MAX17058_EXT_EMPTY * 100)))
#endif

/* -------------------------------------------------------------------- */
/* max17058 Type Definition */
/* -------------------------------------------------------------------- */

struct max17058_data {
	struct i2c_client   *client;
	struct delayed_work	rcomp_work;
};

typedef enum {
	MAX17058_REG_VCELL_0x2      = 0x02,
	MAX17058_REG_SOC_0x4        = 0x04,
	MAX17058_REG_HIBERNATE_0x0A = 0x0A,
	MAX17058_REG_CONFIG_0x0C    = 0x0C,
	MAX17058_REG_OCV_0x0E       = 0x0E,
	MAX17058_VRESET_0x18        = 0x18,
	MAX17058_REG_UNLOCK_0x3E    = 0x3E,
	MAX17058_REG_TABLE_0x40     = 0x40,
	MAX17058_REG_TABLE_0x50     = 0x50,
	MAX17058_REG_TABLE_0x60     = 0x60,
	MAX17058_REG_TABLE_0x70     = 0x70,
	MAX17058_REG_POR_0xFE       = 0xFE,
} max17058_reg_type;


static struct i2c_client *max17058_client;
static struct max17058_data *the_max17058;
static struct i2c_driver max17058_i2c_driver;

static bool max17058_uses = 0;

/* -------------------------------------------------------------------- */
/* max17058 I2C Read Write block */
/* -------------------------------------------------------------------- */
static s32 max17058_i2c_write(u8 reg, u8 *val, u16 len)
{
#if defined(CONFIG_PANTECH_I2C_MAXIM)
	s32 ret = 0;
	if ((smem_id_vendor1_ptr->hw_rev == 5) || (smem_id_vendor1_ptr->hw_rev == 6))
	{


		ret = i2c_smbus_write_i2c_block_data(the_max17058->client, reg,
					len, val);
		if (ret < 0)
			pr_debug("max17058 failed to write register \n");


	}
	else if (smem_id_vendor1_ptr->hw_rev == 7)
	{
		u8 buf[MAX17058_BUF_MAX];

			struct i2c_msg msg = {
			.addr = MAX17058_I2C_ADDR, .flags = 0, .buf = buf, .len = len + 1
		};

		memset(buf, 0x0, MAX17058_BUF_MAX);

		buf[0] = reg;
		memcpy((void*)&buf[1], (void*)val, len);
	
		if (!max17058_client) {
			return -1;
		}

		if (i2c_transfer(max17058_client->adapter, &msg, 1) < 0) {
			return -EIO;
		}


	}
	return ret;
#else
	s32 ret = 0;
	u8 buf[MAX17058_BUF_MAX];

		struct i2c_msg msg = {
		.addr = MAX17058_I2C_ADDR, .flags = 0, .buf = buf, .len = len + 1
	};

	memset(buf, 0x0, MAX17058_BUF_MAX);

	buf[0] = reg;
	memcpy((void*)&buf[1], (void*)val, len);
	
	if (!max17058_client) {
		return -1;
	}

	if (i2c_transfer(max17058_client->adapter, &msg, 1) < 0) {
		return -EIO;
	}

	return ret;
#endif
}

static int max17058_i2c_read(u8 reg, u8 *val, u16 size)
{
#if defined(CONFIG_PANTECH_I2C_MAXIM)
	if ((smem_id_vendor1_ptr->hw_rev == 5) || (smem_id_vendor1_ptr->hw_rev == 6))
	{
		s32 ret =0;
	
		ret = i2c_smbus_read_i2c_block_data(the_max17058->client, reg, size, val);
		if (ret < 0) {
			pr_debug("max17058 failed to read register \n");
	}

	}
	else if (smem_id_vendor1_ptr->hw_rev == 7)
	{
		struct i2c_msg msg[2] = {
			{ .addr = MAX17058_I2C_ADDR, .flags = 0,        .len = 1,    .buf = &reg },
			{ .addr = MAX17058_I2C_ADDR, .flags = I2C_M_RD, .len = size, .buf = val  },
		};
	
		if (!max17058_client)
			return -1;

		if (i2c_transfer(max17058_client->adapter, msg, 2) < 0)
			return -EIO;
	}
	return 0;	
#else
	struct i2c_msg msg[2] = {
		{ .addr = MAX17058_I2C_ADDR, .flags = 0,        .len = 1,    .buf = &reg },
		{ .addr = MAX17058_I2C_ADDR, .flags = I2C_M_RD, .len = size, .buf = val  },
	};
	
	if (!max17058_client)
		return -1;

	if (i2c_transfer(max17058_client->adapter, msg, 2) < 0)
		return -EIO;
	
	return 0;
#endif
}

/* -------------------------------------------------------------------- */
/* Max17058 interface block */
/* -------------------------------------------------------------------- */
static int get_max17058_voltage(void)
{
	s32 ret;
	u8 val[2] = {0};
	int uvolt = 0;

#if defined(CONFIG_MACH_MSM8960_VEGAPVW) /* JB BUILD FIXED */
	uvolt = 3800000;
	return uvolt;
#endif
	
#if defined (CONFIG_MACH_MSM8960_EF44S) || defined (CONFIG_MACH_MSM8960_MAGNUS) || defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
	uvolt = 3800000;  // sayuss for Factory Cable

	if (smem_id_vendor1_ptr->battery_id !=0)
#endif
	{
		ret = max17058_i2c_read(MAX17058_REG_VCELL_0x2, val, 2);

		if (ret < 0)
			return uvolt;

		uvolt = (val[0] << 4) + ((val[1] & 0xF0) >> 4);
		uvolt = (uvolt * 1250);
	}

	return uvolt;
}

int get_max17058_soc(void)
{
	static unsigned last_soc = 0;
	s32 ret;
	u8 val[2] = {0};
	int soc, oem_soc, adjsoc;
#if defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
	int display_soc;
#endif

#if defined(CONFIG_MACH_MSM8960_VEGAPVW) /* JB BUILD FIXED */
	adjsoc = 50;
	return adjsoc;
#endif

#if defined (CONFIG_MACH_MSM8960_EF44S) || defined (CONFIG_MACH_MSM8960_MAGNUS) || defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
	adjsoc = 50; // sayuss : soc is 50 for Factory Cable
	display_soc = 50;
	if (smem_id_vendor1_ptr->battery_id !=0)
#endif
	{	
		ret = max17058_i2c_read(MAX17058_REG_SOC_0x4, val, 2);

		if (ret < 0)
			return last_soc;

#if defined(T_STARQ)
		soc = val[0] + (val[1] / 256);
		oem_soc = OEM_SOC(val[0], val[1]);
#else
		soc = ((val[0] * 256) + val[1]) / 512;
		oem_soc = OEM_SOC(val[0], val[1]);
#endif

	#if defined(T_OSCAR)
		if (smem_id_vendor1_ptr->battery_id == 1)
			adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
		else if (smem_id_vendor1_ptr->battery_id == 2)
			adjsoc = OEM_EXT_ADJUSED_SOC(oem_soc);
		else
			adjsoc = soc;
	#elif defined(T_STARQ)
		adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
  #elif defined(T_SIRIUSLTE)
		adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
	#elif defined(T_EF44S)
		adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
	#elif defined(T_MAGNUS)
		if (smem_id_vendor1_ptr->battery_id == 1)
			adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
		else if (smem_id_vendor1_ptr->battery_id == 2)
			adjsoc = OEM_EXT_ADJUSED_SOC(oem_soc);
		else
			adjsoc = soc;
	#elif defined(T_VEGAPVW)
		if (smem_id_vendor1_ptr->battery_id == 1)
			adjsoc = OEM_STD_ADJUSED_SOC(oem_soc);
		else if (smem_id_vendor1_ptr->battery_id == 2)
			adjsoc = OEM_EXT_ADJUSED_SOC(oem_soc);
		else
			adjsoc = soc;
	#else
		adjsoc = soc;
	#endif

#if defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
		if( adjsoc >= 9500 )
			display_soc = adjsoc / 100;
		else if( adjsoc >= 6300 )
			display_soc = ((adjsoc  * 12525) - 24078000) / 1000000;

		else if(adjsoc >= 2800)
			display_soc = ((adjsoc  * 9388) - 4313300) / 1000000;
		else
			display_soc = (adjsoc  * 7848) / 1000000;

		if (display_soc >= 100)
			display_soc = 100;

		if (display_soc < 0)
			display_soc = 0;

		last_soc = display_soc;
		pr_debug("oem_soc = %d, adjsoc = %d, display_soc = %d \n", oem_soc, adjsoc, display_soc);
	}

	return display_soc;

#else
		if (adjsoc >= 100)
			adjsoc = 100;
	
		if (adjsoc < 0)
			adjsoc = 0;

		pr_debug("oem_soc = %d, adjsoc = %d\n", oem_soc, adjsoc);

		last_soc = adjsoc;
	}

	return adjsoc;
#endif 
}
EXPORT_SYMBOL_GPL(get_max17058_soc);

#if  defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
static int get_max17058_debug_info(void)
{
	s32 ret;

	u8 val02[2] = {0};
	u8 val04[2] = {0};
	u8 val06[2] = {0};
	u8 val08[2] = {0};
	u8 val0C[2] = {0};
	u8 val18[2] = {0};
	u8 val1A[2] = {0};
	u8 val40[2] = {0};
	u8 valFE[2] = {0};	
	
	int pval02 = 0;
	int pval04 = 0;
	int pval06 = 0;
	int pval08 = 0;
	int pval0C = 0;
	int pval18 = 0;
	int pval1A = 0;
	int pval40 = 0;
	int pvalFE = 0;

	ret = max17058_i2c_read(0x02, val02, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 02h read error\n");
		return -1;
	}	

	ret = max17058_i2c_read(0x04, val04, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 04h read error\n");
		return -1;
	}

	ret = max17058_i2c_read(0x06, val06, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 06h read error\n");
		return -1;
	}

	ret = max17058_i2c_read(0x08, val08, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 08h read error\n");
		return -1;
	}
	
	ret = max17058_i2c_read(0x0C, val0C, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 0Ch read error\n");
		return -1;
	}

	ret = max17058_i2c_read(0x18, val18, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 18h read error\n");
		return -1;
	}	
	
	ret = max17058_i2c_read(0x1A, val1A, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 1Ah read error\n");
		return -1;
	}	


	ret = max17058_i2c_read(0x40, val40, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: 40h read error\n");
		return -1;
	}	

	ret = max17058_i2c_read(0xFE, valFE, 2);
	if (ret < 0){
		printk("[MAX17058] fuelgauge: FEh read error\n");
		return -1;
	}		

	pval02 = (val02[0]) + ((val02[1] & 0xFF) << 8);
	pval04 = (val04[0]) + ((val04[1] & 0xFF) << 8);
	pval06 = (val06[0]) + ((val06[1] & 0xFF) << 8);
	pval08 = (val08[0]) + ((val08[1] & 0xFF) << 8);
	pval0C = (val0C[0]) + ((val0C[1] & 0xFF) << 8);
	pval18 = (val18[0]) + ((val18[1] & 0xFF) << 8);
	pval1A = (val1A[0]) + ((val1A[1] & 0xFF) << 8);
	pval40 = (val40[0]) + ((val40[1] & 0xFF) << 8);
	pvalFE = (valFE[0]) + ((valFE[1] & 0xFF) << 8);	

	printk("[MAX17058] fuelgauge: %04xh, %04xh, %04xh, %04xh, %04xh, %04xh, %04xh, %04xh, %04xh\n", pval02, pval04, pval06, pval08, pval0C, pval18, pval1A, pval40, pvalFE);

	return 0;
}
#endif

static int update_rcomp(u8 *val)
{
	s32 ret = 0 ;

#if defined (CONFIG_MACH_MSM8960_EF44S) || defined (CONFIG_MACH_MSM8960_MAGNUS) || defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
	if (smem_id_vendor1_ptr->battery_id !=0)
#endif
	{
		ret = max17058_i2c_write(MAX17058_REG_CONFIG_0x0C, val, 2);
	}

	if(ret < 0) 
		pr_debug("update_rcomp fail %d\n", ret);	

	return ret;
}

static void rcomp_handler(struct work_struct *work)
{
	int temp;
	int start_rcomp;
	int new_rcomp;
	u8 val[2] = {0};

	if (smem_id_vendor1_ptr->battery_id == 1)
		start_rcomp = OEM_BATTERY_STD_RCOMP;
	else if (smem_id_vendor1_ptr->battery_id == 2)
		start_rcomp = OEM_BATTERY_EXT_RCOMP;
	else
		start_rcomp = OEM_BATTERY_STD_RCOMP;

	if (!the_max17058) {
		schedule_delayed_work(&the_max17058->rcomp_work,
        round_jiffies_relative(msecs_to_jiffies
        (30000)));
		return;
	}
	
	temp = pm8921_batt_temperature();
	temp = temp / 10;

	pr_debug("start_rcomp = %d, temp = %d, battery_id = %d\n", start_rcomp, temp, smem_id_vendor1_ptr->battery_id);

#if defined(T_STARQ)
	if (temp > 20)
		new_rcomp = start_rcomp - (9 / 5 * (temp - 20));
	else if (temp < 20)
		new_rcomp = start_rcomp + (5 * (20 - temp));
	else
		new_rcomp = start_rcomp;
#elif defined(T_OSCAR)
	if (smem_id_vendor1_ptr->battery_id == 1) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 11 / 20);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 7 / 20);
		else
			new_rcomp = start_rcomp;
	} else if (smem_id_vendor1_ptr->battery_id == 2) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 1 / 8);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 17 / 40);
		else
			new_rcomp = start_rcomp;
	} else {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 11 / 20);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 7 / 20);
		else
			new_rcomp = start_rcomp;
	}
#elif defined(T_EF44S)
	if (temp > 20)
		new_rcomp = start_rcomp - ((temp - 20) * 3 / 10);
	else if (temp < 20)
		new_rcomp = start_rcomp - ((temp - 20) * 243 / 40);
	else
		new_rcomp = start_rcomp;
#elif defined(T_SIRIUSLTE)
	if (temp > 20)
		new_rcomp = start_rcomp - ((temp - 20) * 1 / 8);
	else if (temp < 20)
		new_rcomp = start_rcomp - ((temp - 20) * 137 / 40);
	else
		new_rcomp = start_rcomp;
#elif defined(T_MAGNUS)
	if (smem_id_vendor1_ptr->battery_id == 1) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 1 / 8);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 137 / 40);
		else
			new_rcomp = start_rcomp;
	} else if (smem_id_vendor1_ptr->battery_id == 2) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 3 / 10);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 243 / 40);
		else
			new_rcomp = start_rcomp;
	} else {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 1 / 8);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 137 / 40);
		else
			new_rcomp = start_rcomp;
	}
#elif defined(T_VEGAPVW)
	if (smem_id_vendor1_ptr->battery_id == 1) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 0);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 221 / 20);
		else
			new_rcomp = start_rcomp;
	} else if (smem_id_vendor1_ptr->battery_id == 2) {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 1 / 8);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 137 / 40);
		else
			new_rcomp = start_rcomp;
	} else {
		if (temp > 20)
			new_rcomp = start_rcomp - ((temp - 20) * 0);
		else if (temp < 20)
			new_rcomp = start_rcomp - ((temp - 20) * 221 / 20);
		else
			new_rcomp = start_rcomp;
	}
#else
	if (temp > 20)
		new_rcomp = start_rcomp - ((temp - 20) * 3 / 10);
	else if (temp < 20)
		new_rcomp = start_rcomp + ((temp - 20) * 27 / 4);
	else
		new_rcomp = start_rcomp;
#endif

	if (new_rcomp > 255)
		new_rcomp = 255;
	else if (new_rcomp < 0)
		new_rcomp = 0;

	pr_debug("new_rcomp = %d\n", new_rcomp);

	val[0] = (u8)new_rcomp;
	val[1] = 0x1E;

	if (update_rcomp(val) < 0)
		pr_debug("update_rcomp failed\n");

	schedule_delayed_work(&the_max17058->rcomp_work,
        round_jiffies_relative(msecs_to_jiffies
        (30000)));
}

/* -------------------------------------------------------------------- */
/* max17058 driver probe function */
/* -------------------------------------------------------------------- */
static int max17058_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct max17058_data *max17058;
       struct i2c_adapter *adapter;
	   
#if defined(CONFIG_PANTECH_I2C_MAXIM)

	if ((smem_id_vendor1_ptr->hw_rev == 5) || (smem_id_vendor1_ptr->hw_rev == 6))
	{
		max17058 = kzalloc(sizeof(struct max17058_data), GFP_KERNEL);

		if (!max17058) {
			pr_debug("max17058 failed to alloc memory\n");
			return -ENOMEM;
		}

		INIT_DELAYED_WORK(&max17058->rcomp_work, rcomp_handler);

		max17058->client = client;
		i2c_set_clientdata(client, max17058);
		max17058_client = client;

		the_max17058 = max17058;

		i2c_set_clientdata(client, max17058);
	
		schedule_delayed_work(&the_max17058->rcomp_work,
        	round_jiffies_relative(msecs_to_jiffies
        	(30000)));

		// Alert Intterupt Add.

	}
	else if (smem_id_vendor1_ptr->hw_rev == 7)
	{
		adapter = to_i2c_adapter(client->dev.parent);	

		max17058 = kzalloc(sizeof(struct max17058_data), GFP_KERNEL);

		if (!max17058) {
			pr_debug("max17058 failed to alloc memory\n");
			return -ENOMEM;
		}

		if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
			return -EIO;

		INIT_DELAYED_WORK(&max17058->rcomp_work, rcomp_handler);

		max17058->client = client;
		max17058_client = client;

		the_max17058 = max17058;

		i2c_set_clientdata(client, max17058);
	
		schedule_delayed_work(&the_max17058->rcomp_work,
        	round_jiffies_relative(msecs_to_jiffies
        	(30000)));

		// Alert Intterupt Add.


	}

	return 0;	
#else
       adapter = to_i2c_adapter(client->dev.parent);	

	max17058 = kzalloc(sizeof(struct max17058_data), GFP_KERNEL);

	if (!max17058) {
		pr_debug("max17058 failed to alloc memory\n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	INIT_DELAYED_WORK(&max17058->rcomp_work, rcomp_handler);

	max17058->client = client;
	max17058_client = client;

	the_max17058 = max17058;

	i2c_set_clientdata(client, max17058);
	
	schedule_delayed_work(&the_max17058->rcomp_work,
        round_jiffies_relative(msecs_to_jiffies
        (30000)));

	// Alert Intterupt Add.
	pr_err("max17058 probe OK\n");

	return 0;
#endif
}

/* -------------------------------------------------------------------- */
/* max17058 driver remove function */
/* -------------------------------------------------------------------- */
static int max17058_remove(struct i2c_client *client)
{
	struct max17058_data *max17058 = i2c_get_clientdata(client);
	kfree(max17058);
	
	return 0;
}

static const struct i2c_device_id max17058_device_id[] = {
	{"max17058-i2c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max17058_device_id);

static struct i2c_driver max17058_i2c_driver = {
	.driver = {
		.name  = "max17058-i2c",
		.owner = THIS_MODULE,
	},
	.probe     = max17058_probe,
	.remove    = max17058_remove,
	.id_table  = max17058_device_id,
};
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
#define SMB_CHG_SUSP            14
enum
{
	GPIO_LOW_VALUE = 0,
	GPIO_HIGH_VALUE,
};

struct smb347_data {
	struct i2c_client   *client;
	int                        cable_type;
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	int is_dth_detect;
	int is_otg_detect;
	int irq;
	struct delayed_work	dth_detect_work;
#endif
	struct delayed_work	aicl_off_on_work;

	unsigned int 		is_bat_warm;
	unsigned int 		is_bat_cool;
	unsigned int		bat_temp_ok;
	unsigned int		enable_aicl_off_on;
};

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
static struct switch_dev dth_switch_dev = {
	.name		= "dock",
};
#endif

static struct i2c_client *smb347_client;
static struct smb347_data *the_smb347;
static struct i2c_driver smb347_i2c_driver;

static int smb347_read_reg(int reg,
	u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(the_smb347->client, reg);
	if (ret < 0) {
		pr_debug("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else
		*val = ret;

	return 0;
}

static int smb347_write_reg( int reg,
	u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(the_smb347->client, reg, val);
	if (ret < 0) {
		pr_debug("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb347_masked_write(int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb347_read_reg(reg, &temp);
	if (rc) {
		pr_err("smb347_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb347_write_reg(reg, temp);
	if (rc) {
		pr_err("smb347_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

#if defined(CONFIG_PANTECH_SMB_DEBUG)
static void smb347_print_all_reg(void)
{
	u8 is_invalid_temp;
	
	pr_err("=========smb347_reg start========\n");
	smb347_read_reg(0x30, &is_invalid_temp);
	pr_err("0x30 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x31, &is_invalid_temp);
	pr_err("0x31 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x00, &is_invalid_temp);
	pr_err("0x00 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x01, &is_invalid_temp);
	pr_err("0x01 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x02, &is_invalid_temp);
	pr_err("0x02 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x03, &is_invalid_temp);
	pr_err("0x03 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x04, &is_invalid_temp);
	pr_err("0x04 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x05, &is_invalid_temp);
	pr_err("0x05 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x06, &is_invalid_temp);
	pr_err("0x06 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x07, &is_invalid_temp);
	pr_err("0x07 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x08, &is_invalid_temp);
	pr_err("0x08 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x09, &is_invalid_temp);
	pr_err("0x09 value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x0A, &is_invalid_temp);
	pr_err("0x0A value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x0B, &is_invalid_temp);
	pr_err("0x0B value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x0C, &is_invalid_temp);
	pr_err("0x0C value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x0D, &is_invalid_temp);
	pr_err("0x0D value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x0E, &is_invalid_temp);
	pr_err("0x0E value = %.2X\n", is_invalid_temp);
	smb347_read_reg(0x3F, &is_invalid_temp);
	pr_err("0x3F value = %.2X\n", is_invalid_temp);
	pr_err("=========smb347_reg end========\n");
	
	return;
}
#endif

void smb347_set_cable_imax_usb(void);
int smb347_otg_power(bool on)
{
	printk(KERN_ERR "[%s]on = %d\n",__func__, on);
	if (on)
	{
		set_stop_otg_chg(true);
		
		gpio_direction_output(SMB_CHG_SUSP, GPIO_HIGH_VALUE);
		mdelay(100);
		
		smb347_write_reg(0x30,0x91);
		smb347_write_reg(0x31,0x00);
		
			smb347_write_reg(0x0A,0xBB);
		
		smb347_write_reg(0x30,0xD0);   	  
	}
	else
	{
		set_stop_otg_chg(false);
			smb347_set_cable_imax_usb();
		gpio_direction_output(SMB_CHG_SUSP, GPIO_LOW_VALUE);
	}

	the_smb347->is_otg_detect = on;
	smb347_otg_power_supply_changed();

	return 0;
}
EXPORT_SYMBOL_GPL(smb347_otg_power);

/*
enable : 0 - charging off
	 1 = charging on
   */
int smb347_charging_enable(int enable)
{
	int rc;
	u8 temp = 0;

	smb347_write_reg(CMD_A_REG_0x30,0xC3);

	rc = smb347_read_reg(PIN_ENABLE_CTRL_REG_0x06, &temp);
	if (rc) {
		pr_debug("smb347_read_reg to %x value =%d errored = %d\n",
			PIN_ENABLE_CTRL_REG_0x06, temp, rc);
		return -EAGAIN;
	}

	temp &= ~(ENABLE_PIN_CONTROL_MASK);
	pr_debug("enable:%d,temp:%d,bit:0x%x\n",enable,temp,~(ENABLE_PIN_CONTROL_MASK));
	
	rc = smb347_masked_write(PIN_ENABLE_CTRL_REG_0x06, ENABLE_PIN_CONTROL_MASK, temp);
	if (rc) {
		pr_err("smb347_masked_write failed: reg=%03X, rc=%d\n", PIN_ENABLE_CTRL_REG_0x06, rc);
		return rc;
	}

	if(enable)
		temp = CHG_ENABLE_BIT;
	else
		temp = 0;	

	pr_debug("enable:%d,temp:%d,bit:%lu\n",enable,temp,CHG_ENABLE_BIT);

	return smb347_masked_write(CMD_A_REG_0x30, CHG_ENABLE_BIT, temp);
}

int smb347_is_trickle(void)
{
	u8 chg_stat;
	int ret;
	ret = smb347_read_reg(STATUS_C_REG_0x3D, &chg_stat);
	if (ret) {
		pr_debug("smb347_read_reg to %x value =%d errored = %d\n",
			STATUS_C_REG_0x3D, chg_stat, ret);
		return -EAGAIN;
	}
	if(test_bit(1,(void *)&chg_stat) && !(test_bit(2,(void *)&chg_stat))  )
	{
		pr_debug("Trickle...\n");
		return true;
	}
	else
	{
		pr_debug("NotTrickle...\n");
		return false;
	}

}

int smb347_is_charging(void)
{
	u8 chg_stat;
	int ret;
	ret = smb347_read_reg(STATUS_C_REG_0x3D, &chg_stat);
	if (ret) {
		pr_err("smb347_read_reg to %x value =%d errored = %d\n",
			STATUS_C_REG_0x3D, chg_stat, ret);
		return -EAGAIN;
	}

	if(test_bit(0,(void *)&chg_stat))
	{
		pr_debug("charging...\n");
		return true;
	}
	else
	{
		pr_debug("discharging...\n");
		return false;
	}
}

#define SMB347_FAST_CHG_SHIFT	5 
int smb347_get_fast_chg_current(int* val)
{
    	u8 reg;
	int ret;

	ret = smb347_read_reg(CHG_CURRENT_REG_0x00, &reg);
	if (ret) {
		pr_err("smb347_read_reg to %x value =%d errored = %d\n",
			CHG_CURRENT_REG_0x00, reg, ret);
		return -EAGAIN;
	}

	*val = reg >> SMB347_FAST_CHG_SHIFT;

	return 0;
}

#define SMB347_USBIN_INPUT_CURR_SHIFT	0
int smb347_set_USBIN_input_curr_limit(u8 usbin_input_curr_ma)
{
	u8 temp;

	if(usbin_input_curr_ma > 0x0F)
	{
		pr_err("invalid argument\n");
		return -EINVAL;
	}
	temp = usbin_input_curr_ma;
	temp = temp << SMB347_USBIN_INPUT_CURR_SHIFT;
	pr_debug("usbin limit=%d setting %02x\n",
				usbin_input_curr_ma, temp);
	return smb347_masked_write(INPUT_CURRENT_LIMIT_REG_0x01,
			USBIN_INPUT_CURRENT_LIMIT_MASK, temp);
}

#define SMB347_DCIN_INPUT_CURR_SHIFT	4 
int smb347_set_DCIN_input_curr_limit(u8 dcin_input_curr_ma)
{
	u8 temp;

	if(dcin_input_curr_ma > 0x0F)
	{
		pr_err("invalid argument\n");
		return -EINVAL;
	}

	temp = dcin_input_curr_ma;
	temp = temp << SMB347_DCIN_INPUT_CURR_SHIFT;
	pr_debug("dcin limit=%d setting %02x\n",
				dcin_input_curr_ma, temp);
	return smb347_masked_write(INPUT_CURRENT_LIMIT_REG_0x01,
			DC_INPUT_CURRENT_LIMIT_MASK, temp);
}

#define SMB347_FLOAT_VOLTAGE_SHIFT	0
#define SMB347_FLOAT_VOLTAGE_MIN_MV	3500
#define SMB347_FLOAT_VOLTAGE_STEP_MV	20
#define SMB347_FLOAT_VOLTAGE_MAX_MV	4500
int smb347_get_float_voltage(int* float_mv)
{
    	u8 reg;
	s32 rc;
	int temp;

	rc= smb347_read_reg(FLOAT_VOLTAGE_REG_0x03, &reg);
	if (rc) {
		pr_err("smb347_read_reg failed: reg=%03X, rc=%d\n",FLOAT_VOLTAGE_REG_0x03 , rc);
		return rc;
	}

	reg &= FLOAT_VOLTAGE_MASK;
	temp  =  (SMB347_FLOAT_VOLTAGE_MIN_MV + 
			(reg * SMB347_FLOAT_VOLTAGE_STEP_MV));

	if(temp > SMB347_FLOAT_VOLTAGE_MAX_MV)
		temp = SMB347_FLOAT_VOLTAGE_MAX_MV;

	*float_mv = temp;

	pr_debug("reg:%d,float_mv:%d\n",reg,*float_mv);
	return 0;
}

void smb347_set_cable_imax_usb(void)
{

	pr_err("the_smb347->bat_temp_ok: %d\n",the_smb347->bat_temp_ok);
	pr_err("is_bat_warm: %d is_bat_cool: %d\n",the_smb347->is_bat_warm,the_smb347->is_bat_cool);

#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

	smb347_write_reg(CMD_A_REG_0x30,0xC0);
	smb347_write_reg(CMD_B_REG_0x31,0x02); 

	if(the_smb347->bat_temp_ok)
	{
		if(the_smb347->is_bat_warm)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
		}
		else if(the_smb347->is_bat_cool)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
		}
		else
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
		}

		smb347_set_USBIN_input_curr_limit(1);   // USBIN current : 500mA
		//smb347_set_DCIN_input_curr_limit(1);    // DCIN current  : 500mA

		smb347_write_reg(VAR_FUNC_REG_0x02,0x83);
		smb347_write_reg(FLOAT_VOLTAGE_REG_0x03,0xEA);//Float Voltage : 4.34V
		smb347_write_reg(CHG_CTRL_REG_0x04,0x20);
		smb347_write_reg(STAT_TIMER_REG_0x05,0x1E);
		smb347_write_reg(PIN_ENABLE_CTRL_REG_0x06,0x28);
		smb347_write_reg(THERM_CTRL_A_REG_0x07,0xFF);
		smb347_write_reg(SYSOK_USB3_SELECT_REG_0x08,0x18);
		smb347_write_reg(OTHER_CTRL_REG_0x09,0x1D);
		smb347_write_reg(OTG_TLIM_THERM_CNTRL_REG_0x0A,0xB7);
		smb347_write_reg(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG_0x0B,0x01);
		smb347_write_reg(FAULT_IRQ_REG_0x0C,0x00);
		smb347_write_reg(STATUS_IRQ_REG_0x0D,0x00);
		smb347_write_reg(CMD_B_REG_0x31,0x02); 
		smb347_write_reg(CMD_A_REG_0x30,0xC0);
	}
	else
	{
		pr_err("hot or cold batt temp so charging disabled.\n");
		smb347_charging_enable(0);
	}


#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

}

void smb347_set_cable_imax_ta(void)
{

	pr_err("the_smb347->bat_temp_ok: %d\n",the_smb347->bat_temp_ok);
	pr_err("is_bat_warm: %d is_bat_cool: %d\n",the_smb347->is_bat_warm,the_smb347->is_bat_cool);

#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

	smb347_write_reg(CMD_A_REG_0x30,0xC3);
	smb347_write_reg(CMD_B_REG_0x31,0x03); 

	if(the_smb347->bat_temp_ok)
	{

		if(the_smb347->is_bat_warm)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
			smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
			//smb347_set_DCIN_input_curr_limit(1);    // DCIN current  : 500mA
		}
		else if(the_smb347->is_bat_cool)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 
			smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
			//smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA
		}
		else
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 
			smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
			//smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA
		}

		if(the_smb347->enable_aicl_off_on) 
		{
			smb347_write_reg(VAR_FUNC_REG_0x02,0x83);
			schedule_delayed_work(&the_smb347->aicl_off_on_work, round_jiffies_relative(msecs_to_jiffies(500)));
			the_smb347->enable_aicl_off_on = false; 
		}
		else
			smb347_write_reg(VAR_FUNC_REG_0x02,0x93);


		smb347_write_reg(FLOAT_VOLTAGE_REG_0x03,0xEA);//Float Voltage : 4.34V
		smb347_write_reg(CHG_CTRL_REG_0x04,0x20);
		smb347_write_reg(STAT_TIMER_REG_0x05,0x1E);
		smb347_write_reg(PIN_ENABLE_CTRL_REG_0x06,0x08);
		smb347_write_reg(THERM_CTRL_A_REG_0x07,0xFF);
		smb347_write_reg(SYSOK_USB3_SELECT_REG_0x08,0x18);
		smb347_write_reg(OTHER_CTRL_REG_0x09,0x1D);
		smb347_write_reg(OTG_TLIM_THERM_CNTRL_REG_0x0A,0xB7);
		smb347_write_reg(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG_0x0B,0x01);
		smb347_write_reg(FAULT_IRQ_REG_0x0C,0x00);
		smb347_write_reg(STATUS_IRQ_REG_0x0D,0x00);
		smb347_write_reg(CMD_B_REG_0x31,0x03); 
		smb347_write_reg(CMD_A_REG_0x30,0xC3);
	}
	else
	{
		pr_err("hot or cold batt temp so charging disabled.\n");
		smb347_charging_enable(0);
	}


#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

}

void smb347_set_cable_imax_unkown(void)
{

	pr_err("the_smb347->bat_temp_ok: %d\n",the_smb347->bat_temp_ok);
	pr_err("is_bat_warm: %d is_bat_cool: %d\n",the_smb347->is_bat_warm,the_smb347->is_bat_cool);

#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

	smb347_write_reg(CMD_A_REG_0x30,0xC3);
	smb347_write_reg(CMD_B_REG_0x31,0x03); 

	if(the_smb347->bat_temp_ok)
	{

		if(the_smb347->is_bat_warm)
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
		else if(the_smb347->is_bat_cool)
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 
		else
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 

		smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
		smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA

		smb347_write_reg(VAR_FUNC_REG_0x02,0x93);
		smb347_write_reg(FLOAT_VOLTAGE_REG_0x03,0xEA);//Float Voltage : 4.34V
		smb347_write_reg(CHG_CTRL_REG_0x04,0x20);
		smb347_write_reg(STAT_TIMER_REG_0x05,0x1E);
		smb347_write_reg(PIN_ENABLE_CTRL_REG_0x06,0x68);
		smb347_write_reg(THERM_CTRL_A_REG_0x07,0xFF);
		smb347_write_reg(SYSOK_USB3_SELECT_REG_0x08,0x28);
		smb347_write_reg(OTHER_CTRL_REG_0x09,0x1D);
		smb347_write_reg(OTG_TLIM_THERM_CNTRL_REG_0x0A,0xB7);
		smb347_write_reg(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG_0x0B,0x01);
		smb347_write_reg(FAULT_IRQ_REG_0x0C,0x00);
		smb347_write_reg(STATUS_IRQ_REG_0x0D,0x00);
		//smb347_write_reg(I2C_BUS_SLAVE_ADDR_REG_0x0E,0x0D);
		smb347_write_reg(CMD_B_REG_0x31,0x03); 
		smb347_write_reg(CMD_A_REG_0x30,0xC3);
	}
	else
	{
		pr_err("hot or cold batt temp so charging disabled.\n");
		smb347_charging_enable(0);
	}

#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

}

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
void smb347_set_cable_imax_dth(void)
{

	pr_err("the_smb347->bat_temp_ok: %d\n",the_smb347->bat_temp_ok);
	pr_err("is_bat_warm: %d is_bat_cool: %d\n",the_smb347->is_bat_warm,the_smb347->is_bat_cool);

#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

	smb347_write_reg(CMD_A_REG_0x30,0xC3);
	smb347_write_reg(CMD_B_REG_0x31,0x01); 

	if(the_smb347->bat_temp_ok)
	{

		if(the_smb347->is_bat_warm)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x11);// batt_fast_chg_curr : 700ma 
		//	smb347_set_USBIN_input_curr_limit(1);   // USBIN current : 500mA
			smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA
		}
		else if(the_smb347->is_bat_cool)
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 
		//	smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
			smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA
		}
		else
		{
			smb347_write_reg(CHG_CURRENT_REG_0x00,0x71);// batt_fast_chg_curr : 1500ma 
		//	smb347_set_USBIN_input_curr_limit(5);   // USBIN current : 1500mA
			smb347_set_DCIN_input_curr_limit(5);    // DCIN current  : 1500mA
		}

		smb347_write_reg(VAR_FUNC_REG_0x02,0x83);
		smb347_write_reg(FLOAT_VOLTAGE_REG_0x03,0xEA);//Float Voltage : 4.34V
		smb347_write_reg(CHG_CTRL_REG_0x04,0x20);
		smb347_write_reg(STAT_TIMER_REG_0x05,0x1E);
		smb347_write_reg(PIN_ENABLE_CTRL_REG_0x06,0x08);
		smb347_write_reg(THERM_CTRL_A_REG_0x07,0xFF);
		smb347_write_reg(SYSOK_USB3_SELECT_REG_0x08,0x18);
		smb347_write_reg(OTHER_CTRL_REG_0x09,0x1D);
		smb347_write_reg(OTG_TLIM_THERM_CNTRL_REG_0x0A,0xB7);
		smb347_write_reg(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG_0x0B,0x01);
		smb347_write_reg(FAULT_IRQ_REG_0x0C,0x00);
		smb347_write_reg(STATUS_IRQ_REG_0x0D,0x00);
		smb347_write_reg(CMD_B_REG_0x31,0x01); 
		smb347_write_reg(CMD_A_REG_0x30,0xC3);
	}
	else
	{
		pr_err("hot or cold batt temp so charging disabled.\n");
		smb347_charging_enable(0);
	}


#if defined(CONFIG_PANTECH_SMB_DEBUG)
	smb347_print_all_reg();
#endif	

}
#endif

static void aicl_off_on_worker(struct work_struct *work)
{
	pr_err("called......\n");
	smb347_write_reg(VAR_FUNC_REG_0x02,0x93);

	return;
}

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
static void pm_dth_irq_detect(struct work_struct *work);
static irqreturn_t pm_dth_detect_irq_handler(int irq, void *data);
#endif

static int smb347_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct smb347_data *smb347;
	int rc;   
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	int ret; 
#endif

	smb347 = kzalloc(sizeof(struct smb347_data), GFP_KERNEL);

	if (!smb347) {
		pr_debug("smb347 failed to alloc memory\n");
		return -ENOMEM;
	}

	pr_info("smb347_probe called\n");
	smb347->client = client;
	i2c_set_clientdata(client, smb347);
	smb347_client = client;

	pr_info("smb347_client set\n");

	the_smb347 = smb347;
   	the_smb347->cable_type = 0;
	the_smb347->is_bat_warm = 0;
	the_smb347->is_bat_cool = 0;
	the_smb347->bat_temp_ok = 1;
	the_smb347->enable_aicl_off_on = false;
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	the_smb347->is_dth_detect = 0;
	the_smb347->is_otg_detect = 0;
#endif

	INIT_DELAYED_WORK(&the_smb347->aicl_off_on_work, aicl_off_on_worker);
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	INIT_DELAYED_WORK(&the_smb347->dth_detect_work, pm_dth_irq_detect);
#endif

	i2c_set_clientdata(client, smb347);
#if 1
	rc = gpio_request(SMB_CHG_SUSP, "smb_chg_susp");
	if (rc) {
		printk(KERN_ERR "[%s] SMB_CHG_SUSP gpio_request failed: %d \n",__func__, rc);
// test		return rc;
	}	
#endif
	gpio_tlmm_config(GPIO_CFG(SMB_CHG_SUSP, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(SMB_CHG_SUSP, GPIO_LOW_VALUE);

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
       the_smb347->irq = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 35);
	ret = request_irq( the_smb347->irq,
			pm_dth_detect_irq_handler,
			IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING,
			"smb347-i2c", the_smb347);

	pr_err("ret = %x\n", ret);
	if (ret) {
		pr_err("DTH IRQ set failed\n");
	}	

	//enable_irq(the_smb347->irq);
	enable_irq_wake(the_smb347->irq);
	switch_dev_register(&dth_switch_dev);
#endif

	return 0;
}
static int smb347_remove(struct i2c_client *client)
{
	struct max17058_data *smb347 = i2c_get_clientdata(client);
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	free_irq(the_smb347->irq, the_smb347);
	switch_dev_unregister(&dth_switch_dev);
#endif
	kfree(smb347);
	
	return 0;
}

static const struct i2c_device_id smb347_device_id[] = {
	{"smb347-i2c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, smb347_device_id);

static struct i2c_driver smb347_i2c_driver = {
	.driver = {
		.name  = "smb347-i2c",
		.owner = THIS_MODULE,
	},
	.probe     = smb347_probe,
	.remove    = __devexit_p(smb347_remove),
	.id_table  = smb347_device_id,
};
#endif

#ifdef CONFIG_PANTECH_BMS_UPDATE
//#define PANTECH_CHARGER_BMS_WAKELOCK_FIX    //wakelock issues (wakelock.c)
#endif

//#define PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST //20120315_khlee : to write the sysfs power_supply property files
#define PANTECH_BATT_STATUS_CHANGE //20120404_khlee : battery status is a charging while either usb or ac is detected. except over, under temp...etc
//#define PANTECH_USED_ALL_OTHER_WRITABLE_FILES 

#define CHG_BUCK_CLOCK_CTRL	0x14

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6
#define OVP_USB_UVD		0x2B7

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200
#define USB_TRIM_ENTRIES 16

#if defined(CONFIG_PANTECH_CHARGER)
#define PANTECH_CHARGER_FACTORY_BOOT
#define PANTECH_CHARGER_TIME_LIMITATION
#define FEATURE_PANTECH_BATTERY_STATUS
#define PANTECH_CHARGER_THERMAL_MITIGATION_DISABLE
#define PANTECH_CHARGER_BTM_ISR_FIX // for JEITA
#if defined(CONFIG_PANTECH_SMB_CHARGER)
#define PANTECH_CHARGER_SMB_EOC
#endif

#if defined(PANTECH_CHARGER_SMB_EOC)
#define SMB_EOC_CHECK_PERIOD_MS 30000
#endif

#if !defined(FEATURE_AARM_RELEASE_MODE)
#define PANTECH_USED_ALL_OTHER_WRITABLE_FILES
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_ABNORMAL_CHARGER_INFO)
#define PANTECH_CHARGER_INFO_ABNORMAL
#endif
#endif // #if defined(CONFIG_PANTECH_CHARGER)

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
#if defined(PANTECH_BMS_UPDATE_UI_FULL)
#define WIRELESS_RECHARGE_PERCENTS 99
#else
#define WIRELESS_RECHARGE_PERCENTS 96
#endif
#if defined(CONFIG_MACH_MSM8960_VEGAPVW)
#define W_CHG_FULL 0
#define USB_CHG_DET 1
#else
#define W_CHG_FULL 0
#define USB_CHG_DET 1
#endif
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
#define NOTIFY_THR_IN_SLEEP 15 // 15 percent
#define SLEEP_HALF_MINUTE 30 // 30sec
#define SLEEP_ONE_MINUTE 60 // 1 minute
#define SLEEP_THREE_MINUTE 180 // 3 minute
#define SLEEP_FIVE_MINUTE 300 // 5 minute
#define SLEEP_HALF_HOUR 1800 // 30 minute
#define SLEEP_ONE_HOUR 3600 // 60 minute
#endif

#if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST) || defined(PANTECH_CHARGER_MONITOR_TEST)
#define PM8921_CHARER_IOCTL_MAGIC 'p'

#if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST)
#define PM8921_CHARER_TEST_SET_CHARING_CTL        _IOW(PM8921_CHARER_IOCTL_MAGIC, 1, unsigned)
#define PM8921_CHARER_TEST_GET_CHARING_CTL        _IOW(PM8921_CHARER_IOCTL_MAGIC, 2, unsigned)
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
#define PM8921_CHARGER_TEST_SET_PM_CHG_TEST	_IOW(PM8921_CHARER_IOCTL_MAGIC, 3, unsigned)
#endif
#endif // #if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST) || defined(PANTECH_CHARGER_MONITOR_TEST)

enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
enum {
	ERR_NONE = 0,
	ERR_TIMER_EXPIRED,
	ERR_UNKNOWN = 0,	
};
#endif

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
typedef enum {
	OEM_PM_BATTERY_TYPE_UNKNOWN,
	OEM_PM_BATTERY_TYPE_STD,
	OEM_PM_BATTERY_TYPE_EXT, 
	OEM_PM_BATTERY_TYPE_DUMMY, 
} oem_pm_battery_id_type;
#endif

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
	struct	work_struct	work;
#if defined(CONFIG_PANTECH_BMS_UPDATE)
	bool batt_notifier_init;
	bool batt_notifier_state_change;
	bool batt_notifier_periodic_update;
#endif               
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @safety_time:		max time for which charging will happen
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @uvd_voltage_mv:		(PM8917 only) the falling UVD threshold voltage
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 *
 */
struct pm8921_chg_chip {
	struct device			*dev;
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			safety_time;
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	unsigned int			uvd_voltage_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	unsigned int			temp_check_period;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	int				resume_charge_percent;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	int				*usb_trim_table;
	bool				keep_btm_on_suspend;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	bool				dc_unplug_check;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
	struct delayed_work		unplug_check_work;
	struct delayed_work		vin_collapse_check_work;
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	bool				host_mode;
	bool				has_dc_supply;
	u8				active_path;
	int				recent_reported_soc;
		
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
	bool is_early_suspend;
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	struct work_struct update_cable_work;
	struct workqueue_struct	*cable_update_wq;

	unsigned int cable_type;
	unsigned int cable_iusb_max;
	unsigned int cable_ibat_max;
#endif

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
	struct wake_lock		btm_isr_lock;
	struct delayed_work     btm_isr_lock_work;
        bool                               btm_isr_lock_work_is; 
#endif
//youhw
#if defined(PANTECH_CHARGER_SMB_EOC)
	struct delayed_work	smb_eoc_work;
	struct delayed_work	smb_wake_unlock_work;
       int					smb_charging_cable_type;
#endif
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	bool	smb_wake_is;
#endif
#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
	struct delayed_work	update_cable_work_delay;
	unsigned int update_cable_time_delay;
	bool cable_usb_update_delay; 
	bool cable_usb_update_delay_decision;         
#endif

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        bool wireless_recharge_loop;
#endif

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
	unsigned int err_type;
	int err_fsm_type;
	bool err_charge_done;
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
	struct wake_lock work_wake_lock;
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
	int current_uvolt;
	int current_temp;
	int soc;
#endif 

#if defined(PANTECH_CHARGER_MONITOR_TEST)
	bool pm_chg_test;
	u64 cable_adc;
#endif

#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
	int	batt_charge_done_warm_cool;
#endif
};

/* user space parameter to limit usb current */
static unsigned int usb_max_current;
/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
#if defined(CONFIG_PANTECH_SMB_CHARGER)
static int charging_disabled = 1;
#else
static int charging_disabled;
#endif
static int thermal_mitigation;

static struct pm8921_chg_chip *the_chip;

static struct pm8xxx_adc_arb_btm_param btm_config;

#if defined(CONFIG_PANTECH_CHARGER)
static unsigned int chg_usb_type = USB_INVALID_CHARGER;
static unsigned int safety_time_remain = 0;
static unsigned int iusbmax_set_cur;
static unsigned int ibatmax_set_cur;
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
static struct input_dev *bms_input_dev;
static struct platform_device *bms_input_attr_dev;
static atomic_t bms_input_flag;
static atomic_t bms_cutoff_flag;
#endif

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
static bool is_dummy_battery = false;
#endif 

#if defined(PANTECH_CHARGER_MONITOR_TEST)
struct proc_dir_entry *pm8921_charger_dir;
#endif

#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};
static int is_charging_finished(struct pm8921_chg_chip *chip);
#endif

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
struct power_supply_write_item_type
{
    bool is_forced_item;
    int forced_value;
};
static struct power_supply_write_item_type power_supply_write_item[POWER_SUPPLY_TYPE_USB+1][POWER_SUPPLY_PROP_SERIAL_NUMBER+1];
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
static atomic_t g_otg_mode_enabled = ATOMIC_INIT(0);
#endif

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
static void btm_init(struct pm8921_chg_chip *chip);
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
extern void msm_pm_set_max_sleep_time(int64_t sleep_time_ns);
static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip);
static int get_prop_batt_temp(struct pm8921_chg_chip *chip);
#endif

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
extern int composite_get_udc_state(char *udc_state);

char usb_conf_buf[128];

char* usb_composite_get_udc_state(void)
{
	char conf_buf[128] = {'\0',};
	int ret = 0;

	ret = composite_get_udc_state(conf_buf);
	strcpy(usb_conf_buf, conf_buf);

	return usb_conf_buf;
}    
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
static bool pm8921_charger_wake_state = false;
void pm8921_charger_prevent_suspend(void)
{
	if(!pm8921_charger_wake_state)
	{
        	wake_lock(&the_chip->work_wake_lock);
        	pm8921_charger_wake_state = true;
	}
}
void pm8921_charger_allow_suspend(void)
{
	if(pm8921_charger_wake_state)
{
        	wake_unlock(&the_chip->work_wake_lock);
        	pm8921_charger_wake_state = false;		
	}
}
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
static int get_batt_mvolts(void)
{
#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	if (max17058_uses)
		return get_max17058_voltage() / 1000;
	else
		return (the_chip->current_uvolt /1000);
#else
	return (the_chip->current_uvolt /1000);
#endif
}

static int get_batt_temp(void)
{
	return the_chip->current_temp/10;
}
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
static void bms_init_set(struct pm8921_chg_chip *chip)
{
}

static ssize_t bms_input_show_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	enable = atomic_read(&bms_input_flag);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t bms_input_store_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 scale = (u8)simple_strtoul(buf, NULL, 10);	
	atomic_set(&bms_input_flag, scale);
	return count;
}

static ssize_t bms_cutoff_show_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	int enable;
	enable = atomic_read(&bms_cutoff_flag);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t bms_cutoff_store_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 scale = (u8)simple_strtoul(buf, NULL, 10);
	atomic_set(&bms_cutoff_flag, scale);
	return count;
}

#if defined(PANTECH_USED_ALL_OTHER_WRITABLE_FILES)
static DEVICE_ATTR(setflag, S_IWUGO | S_IRUGO, bms_input_show_flag, bms_input_store_flag);
static DEVICE_ATTR(cutoff, S_IWUGO | S_IRUGO, bms_cutoff_show_flag, bms_cutoff_store_flag);
#else
static DEVICE_ATTR(setflag, 0644, bms_input_show_flag, bms_input_store_flag);
static DEVICE_ATTR(cutoff, 0644, bms_cutoff_show_flag, bms_cutoff_store_flag);
#endif /* PANTECH_USED_ALL_OTHER_WRITABLE_FILES */

static struct attribute *bms_input_attrs[] = {
	&dev_attr_setflag.attr,
	&dev_attr_cutoff.attr,
	NULL,
};

static struct attribute_group bms_input_attr_group = {
	.attrs = bms_input_attrs,
};
#endif

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
static int pm_power_get_forced_property
(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val
)
{   
	int power_supply_type = 0;

	if (psp > POWER_SUPPLY_PROP_SERIAL_NUMBER || psy->type > POWER_SUPPLY_TYPE_USB_ACA) {
		pr_err("arg Error!!!");
		return -1;
	}

	power_supply_type = psy->type;

	if (power_supply_type >= POWER_SUPPLY_TYPE_USB && power_supply_type <= POWER_SUPPLY_TYPE_USB_ACA)
		power_supply_type = POWER_SUPPLY_TYPE_USB;

	if (power_supply_write_item[power_supply_type][psp].is_forced_item == true) {
		pr_err("power_supply_type=%d, is_forced_item=%d, value=%d\n", power_supply_type, psp,power_supply_write_item[power_supply_type][psp].forced_value);
		val->intval =  power_supply_write_item[power_supply_type][psp].forced_value;
		return 0;
	}

	return -1;
}

static int pm_batt_power_set_property
(
	struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val
)
{
	int power_supply_type = 0;
	struct pm8921_chg_chip *chip;

	if (psp > POWER_SUPPLY_PROP_SERIAL_NUMBER || psy->type > POWER_SUPPLY_TYPE_USB_ACA) {
		pr_err("arg Error!!!");
		return 0;
	}

	power_supply_type = psy->type;

	if (power_supply_type >= POWER_SUPPLY_TYPE_USB && power_supply_type <= POWER_SUPPLY_TYPE_USB_ACA)
		power_supply_type = POWER_SUPPLY_TYPE_USB;

	if (power_supply_type == POWER_SUPPLY_TYPE_BATTERY)
		chip = container_of(psy, struct pm8921_chg_chip,batt_psy);       
	else if (power_supply_type = POWER_SUPPLY_TYPE_MAINS)
		chip = container_of(psy, struct pm8921_chg_chip,dc_psy);       
	else if (power_supply_type == POWER_SUPPLY_TYPE_USB)
		chip = container_of(psy, struct pm8921_chg_chip,usb_psy);
	else {
		pr_err("chip Error!!!");
		return 0;
	}

	pr_err("power_supply_type=%d, is_forced_item=%d, value=%d\n",power_supply_type, psp,val->intval);

	power_supply_write_item[power_supply_type][psp].is_forced_item = true;
	power_supply_write_item[power_supply_type][psp].forced_value = val->intval;

	if (power_supply_type == POWER_SUPPLY_TYPE_BATTERY)
		power_supply_changed(&chip->batt_psy);
	else if (power_supply_type == POWER_SUPPLY_TYPE_MAINS)
		power_supply_changed(&chip->dc_psy);
	else if (power_supply_type == POWER_SUPPLY_TYPE_USB)
		power_supply_changed(&chip->usb_psy);

	return 0;
}
#endif /* PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST */

#if defined(CONFIG_PANTECH_CHARGER)
static int pm_chg_masked_read(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8* val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_debug("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= mask;
        *val = reg; 

	return 0;
}
#endif

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
	if (rc) {
		pr_err("pm8xxx_writeb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}
#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err, ret = 0;

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;
	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err;

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}

	/* return the lower 4 bits */
	return temp & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
}

#if !defined(CONFIG_PANTECH_CHARGER)
static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#endif

#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#define PM8921_CHG_VDDMAX_MIN	3400
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm8xxx_writeb(chip->dev->parent, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
	if (is_usb_chg_plugged_in(chip)) {
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}

#define PM8921_CHG_VDDSAFE_MIN	3400
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8917_USB_UVD_MIN_MV	3850
#define PM8917_USB_UVD_MAX_MV	4350
#define PM8917_USB_UVD_STEP_MV	100
#define PM8917_USB_UVD_MASK	0x7
static int pm_chg_uvd_threshold_set(struct pm8921_chg_chip *chip, int thresh_mv)
{
	u8 temp;

	if (thresh_mv < PM8917_USB_UVD_MIN_MV
			|| thresh_mv > PM8917_USB_UVD_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", thresh_mv);
		return -EINVAL;
	}
	temp = (thresh_mv - PM8917_USB_UVD_MIN_MV) / PM8917_USB_UVD_STEP_MV;
	return pm_chg_masked_write(chip, OVP_USB_UVD,
				PM8917_USB_UVD_MASK, temp);
}

#define PM8921_CHG_IBATMAX_MIN	325
#define PM8921_CHG_IBATMAX_MAX	2000
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F
static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

#if defined(CONFIG_PANTECH_CHARGER)
	ibatmax_set_cur = chg_current;
#endif

	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

/* USB Trim tables */
static int usb_trim_8038_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x9,
	0x0,
	-0xD,
	0x0,
	-0x10,
	-0x11,
	0x0,
	0x0,
	-0x25,
	0x0,
	-0x28,
	0x0,
	-0x32,
	0x0
};

static int usb_trim_8917_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	0xA,
	0xC,
	0x10,
	0x10,
	0x13,
	0x14,
	0x13,
	0x16,
	0x1A,
	0x1D,
	0x1D,
	0x21,
	0x24,
	0x26
};

/* Maximum USB  setting table */
static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define REG_SBI_CONFIG		0x04F
#define PAGE3_ENABLE_MASK	0x6
#define USB_OVP_TRIM_MASK	0x3F
#define USB_OVP_TRIM_MIN	0x00
#define REG_USB_OVP_TRIM_ORIG_LSB	0x10A
#define REG_USB_OVP_TRIM_ORIG_MSB	0x09C
static int pm_chg_usb_trim(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, sbi_config, msb, lsb;
	s8 trim;
	int rc = 0;
	static u8 usb_trim_reg_orig = 0xFF;

	/* No trim data for PM8921 */
	if (!chip->usb_trim_table)
		return 0;

	if (usb_trim_reg_orig == 0xFF) {
		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_MSB, &msb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_LSB, &lsb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		msb = msb >> 5;
		lsb = lsb >> 5;
		usb_trim_reg_orig = msb << 3 | lsb;
	}

	/* use the original trim value */
	trim = usb_trim_reg_orig;

	trim += chip->usb_trim_table[index];
	if (trim < 0)
		trim = 0;

	pr_err("trim_orig %d write 0x%x index=%d value 0x%x to USB_OVP_TRIM\n",
		usb_trim_reg_orig, trim, index, chip->usb_trim_table[index]);

	rc = pm8xxx_readb(chip->dev->parent, REG_SBI_CONFIG, &sbi_config);
	if (rc) {
		pr_err("error = %d reading sbi config reg\n", rc);
		return rc;
	}

	temp = sbi_config | PAGE3_ENABLE_MASK;
	rc = pm8xxx_writeb(chip->dev->parent, REG_SBI_CONFIG, temp);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}

	rc = pm_chg_masked_write(chip, USB_OVP_TRIM, USB_OVP_TRIM_MASK, trim);
	if (rc) {
		pr_err("error = %d writing USB_OVP_TRIM\n", rc);
		return rc;
	}

	rc = pm8xxx_writeb(chip->dev->parent, REG_SBI_CONFIG, sbi_config);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}
	return rc;
}

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)
static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, fineres;
        u8 reg_val=0;
	int rc;


#if defined(PANTECH_CHARGER_FACTORY_BOOT)
	if (chip->cable_type == FACTORY_CABLE) {
		if (pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ))
			reg_val = 7;
		else
			reg_val = 4;
	}
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	iusbmax_set_cur = usb_ma_table[reg_val].usb_ma;
#endif

	reg_val = usb_ma_table[index].value >> 1;
	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[index].value;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc) {
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
				return rc;
		}
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
	}
	}

	rc = pm_chg_usb_trim(chip, index);
	if (rc)
			pr_err("unable to set usb trim rc = %d\n", rc);

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x3F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}

static void disable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* set the input voltage disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void enable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* unset the input voltage disable bit */
	temp &= 0xFE;
	/* set the write bit */
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
	int64_t rc;

	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (is_dummy_battery)
		return 1;
#endif /* FEATURE_PANTECH_BATTERY_DUMMY */

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;
#endif

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

#if defined(CONFIG_PANTECH_BMS_BATTERY_TYPE)
static int is_between(int left, int right, int value)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;

	return 0;
}

static int charger_set_battery_data(struct pm8921_chg_chip *chip)
{
	int64_t battery_id;

	battery_id = read_battery_id(chip);

	if (battery_id < 0) {
		pr_debug("cannot read battery id err = %lld\n", battery_id);
		return battery_id;
	}

	pr_err("pm8921-charger: battery id (%lld)\n", battery_id);

	#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (is_between(BATTERY_ID_DUMMY_MIN, BATTERY_ID_DUMMY_MAX, battery_id)) {
		if(!is_dummy_battery) {
			is_dummy_battery = true;
			pr_err("battery is dummy (%lld)!!!\n", battery_id);
		}
	} else {
		if (is_dummy_battery) {
			is_dummy_battery = false;
			pr_err("battery isn't dummy (%lld)!!!\n", battery_id);
		}
	}
	#endif

	#if defined(CONFIG_MACH_MSM8960_VEGAPVW)
	if ((is_between(BATTERY_ID_LG_STD_MIN, BATTERY_ID_LG_STD_MAX, battery_id)) ||
		(is_between(BATTERY_ID_SS_STD_MIN, BATTERY_ID_SS_STD_MAX, battery_id)))
	#elif defined (CONFIG_MACH_MSM8960_MAGNUS)
	if ((is_between(BATTERY_ID_SY_STD_MIN, BATTERY_ID_SY_STD_MAX, battery_id)) ||
		(is_between(BATTERY_ID_SS_STD_MIN, BATTERY_ID_SS_STD_MAX, battery_id)))
	#else
	if (is_between(BATTERY_ID_STD_MIN, BATTERY_ID_STD_MAX, battery_id))
	#endif		
	{
		chip->max_voltage_mv	= CHG_MAX_VOLTAGE_STD; 
		chip->min_voltage_mv	= CHG_MIN_VOLTAGE_STD; 
		chip->resume_voltage_delta = CHG_RESUME_VOLTAGE_STD;
		chip->term_current	= CHG_TERM_CURRENT_STD;
		chip->cool_bat_voltage = CHG_COOL_BAT_VOLTAGE_STD;
		chip->warm_bat_voltage = CHG_WARM_BAT_VOLTAGE_STD;
		chip->safety_time = CHG_SAFETY_TIME_STD;
		chip->ttrkl_time = CHG_TTRKL_TIME_STD;

		return 0;
	} else if (is_between(BATTERY_ID_EXT_MIN, BATTERY_ID_EXT_MAX, battery_id)) {
		chip->max_voltage_mv	= CHG_MAX_VOLTAGE_EXT;
		chip->min_voltage_mv	= CHG_MIN_VOLTAGE_EXT;
		chip->resume_voltage_delta = CHG_RESUME_VOLTAGE_EXT;
		chip->term_current	= CHG_TERM_CURRENT_EXT;
		chip->cool_bat_voltage = CHG_COOL_BAT_VOLTAGE_EXT;
		chip->warm_bat_voltage = CHG_WARM_BAT_VOLTAGE_EXT;
		chip->safety_time = CHG_SAFETY_TIME_EXT;      
		chip->ttrkl_time = CHG_TTRKL_TIME_EXT;

		return 0;
	} else {
		#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
		if (!is_dummy_battery)
		#endif

		pr_warn("invalid battery id, pantech standard battery assumed\n");
		chip->max_voltage_mv	= CHG_MAX_VOLTAGE_STD; 
		chip->min_voltage_mv	= CHG_MIN_VOLTAGE_STD;
		chip->resume_voltage_delta = CHG_RESUME_VOLTAGE_STD;
		chip->term_current	= CHG_TERM_CURRENT_STD;
		chip->cool_bat_voltage = CHG_COOL_BAT_VOLTAGE_STD;
		chip->warm_bat_voltage = CHG_WARM_BAT_VOLTAGE_STD;
		chip->safety_time = CHG_SAFETY_TIME_STD;       
		chip->ttrkl_time = CHG_TTRKL_TIME_STD;
		
		return 0;
	}
}
#endif //CONFIG_PANTECH_BMS_BATTERY_TYPE

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
static void wireless_full_sign(struct pm8921_chg_chip *chip, bool ctrl)
{
    if(ctrl){
        if(!chip->wireless_recharge_loop)         
            gpio_direction_output(W_CHG_FULL, 1);
        chip->wireless_recharge_loop = true; 
    }    
    else{     
        if(chip->wireless_recharge_loop)         
            gpio_direction_output(W_CHG_FULL, 0);
        chip->wireless_recharge_loop = false;
    }    
}
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
void set_stop_otg_chg(bool disabled)
{
	struct pm8921_chg_chip *chip = the_chip;

	if (chip) {
		pr_debug("%s charging_disabled = %d, disabled = %d\n", 
				__func__, charging_disabled, disabled);
		charging_disabled = (int) disabled;
		pm_chg_usb_suspend_enable(chip, !!disabled);

		if(disabled){
			pm_chg_iusbmax_set(the_chip, 0);
			mdelay(20);			
			pm8921_chg_disable_irq(chip, USBIN_VALID_IRQ);
#if defined(CONFIG_PANTECH_CHARGER)
#else
			pm8921_chg_disable_irq(chip, USBIN_OV_IRQ);
			pm8921_chg_disable_irq(chip, USBIN_UV_IRQ);
#endif
		}else{
			mdelay(20);			
			pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
#if defined(CONFIG_PANTECH_CHARGER)
#else
			pm8921_chg_enable_irq(chip, USBIN_OV_IRQ);
			pm8921_chg_enable_irq(chip, USBIN_UV_IRQ);
#endif
		}

	}
}
EXPORT_SYMBOL_GPL(set_stop_otg_chg);
void set_charger_otg_mode(bool value)
{
	atomic_set(&g_otg_mode_enabled, !!value);
}
EXPORT_SYMBOL_GPL(set_charger_otg_mode);
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
static void bms_notify(struct work_struct *work)
{
	#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
	int end = -1;
	#endif

	static int soc = -1, old_soc = -1;
	struct pm8921_chg_chip *chip = container_of(work,
		struct pm8921_chg_chip, bms_notify.work); 
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	#if defined(CONFIG_PANTECH_BMS_TEST)
	int enable;
	static int first_enable = 1;
	#endif

	pm8921_charger_prevent_suspend();

	if (chip->bms_notify.batt_notifier_init == true) {
		pm8921_bms_charging_began();
		chip->bms_notify.batt_notifier_init = false;
	}

	if (chip->bms_notify.batt_notifier_state_change == true) {
		if (n->is_charging)
			pm8921_bms_charging_began();
		else {
			pm8921_bms_charging_end(n->is_battery_full);
			n->is_battery_full = 0;
		}

		chip->bms_notify.batt_notifier_state_change = false;
	} else if (chip->bms_notify.batt_notifier_periodic_update == true) {
		#if defined(CONFIG_PANTECH_PMIC_MAX17058)
		if (max17058_uses)
			soc = pm8921_bms_get_percent(max17058_uses);
		else
			soc = pm8921_bms_get_percent_charge();
		#else
		soc = pm8921_bms_get_percent_charge();
		soc = pm8921_bms_get_percent();
		pr_debug("battery SOC [%d]  : Calculate SOC [%d] \n", pm8921_bms_get_percent_charge(), soc);							
		#endif

		#if defined(CONFIG_PANTECH_BMS_TEST)
		enable = atomic_read(&bms_input_flag);
		if (enable) {             
			if ((soc != old_soc) || first_enable) {
				chip->soc = soc;

				get_prop_battery_uvolts(chip);
				get_prop_batt_temp(chip);

				if (!soc)
					input_report_rel(bms_input_dev, REL_RX, -1);
				else
					input_report_rel(bms_input_dev, REL_RX, soc);

				input_report_rel(bms_input_dev, REL_RY, get_batt_mvolts());                
				input_report_rel(bms_input_dev, REL_RZ, get_batt_temp());		
				input_sync(bms_input_dev);

				if (first_enable)
					first_enable = 0;
			}
		} else {
			first_enable = 1;
		}
		#endif

		#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
		if (chip->is_bat_warm || chip->is_bat_cool) {
			int old_batt_charge_done = 0;
			end = is_charging_finished(chip);
			
			if (end == CHG_NOT_IN_PROGRESS ||end == CHG_FINISHED) {
				if (is_usb_chg_plugged_in(chip) ||is_dc_chg_plugged_in(chip)) {
					int vbat_meas_uv = get_prop_battery_uvolts(chip);
					
					if (vbat_meas_uv >= 3800000) {
						pr_err("battery charge is done in abnormal temp\n");
						old_batt_charge_done= 1;
					}
				}
			}

			chip->batt_charge_done_warm_cool = old_batt_charge_done;		
		} else {
			if (chip->batt_charge_done_warm_cool)
				chip->batt_charge_done_warm_cool =0;
		}
		#endif

		if (!chip->is_early_suspend)
			power_supply_changed(&chip->batt_psy);
		else if (soc <= NOTIFY_THR_IN_SLEEP && soc != old_soc)
			power_supply_changed(&chip->batt_psy);

		#if defined(CONFIG_PANTECH_PMIC_MAX17058)
		if (max17058_uses)
			old_soc = soc;
		else
			old_soc = soc;
		#else
			old_soc = soc;
		#endif

		schedule_delayed_work(&chip->update_heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies(chip->update_time)));

		chip->bms_notify.batt_notifier_periodic_update = false;
	}       

	#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	if (pm8921_bms_get_percent(max17058_uses) <= WIRELESS_RECHARGE_PERCENTS)
		wireless_full_sign(chip, false);
	#else
	if (pm8921_bms_get_percent() <= WIRELESS_RECHARGE_PERCENTS)
		wireless_full_sign(chip, false);
	#endif // #if defined(CONFIG_PANTECH_PMIC_MAX17058)
	#endif // #if defined(CONFIG_PANTECH_CHARGER_WIRELESS)

	#if  defined (CONFIG_MACH_MSM8960_SIRIUSLTE)
	get_max17058_debug_info();
	#endif

	pm8921_charger_allow_suspend();
}
#else // #if defined(CONFIG_PANTECH_BMS_UPDATE)
static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}
#endif // #if defined(CONFIG_PANTECH_BMS_UPDATE)

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);

	if (chip->bms_notify.is_charging ^ new_is_charging) {
		chip->bms_notify.is_charging = new_is_charging;

#if defined(CONFIG_PANTECH_BMS_UPDATE)
		chip->bms_notify.batt_notifier_state_change = true;
#endif

		schedule_work(&(chip->bms_notify.work));
	}
}

#if defined(CONFIG_PANTECH_CHARGER)
static enum power_supply_property pm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	POWER_SUPPLY_PROP_WIRELESS,
#endif
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	POWER_SUPPLY_PROP_DTH,
#endif
};

static char *pm_power_supplied_to[] = {
	"battery",
};

#define USB_WALL_THRESHOLD_MA	500
static int pm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;
#if defined(CONFIG_PANTECH_CHARGER)
	int status; 
#endif

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	if (pm_power_get_forced_property(psy, psp, val) >= 0)
		return 0;
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pm_chg_iusbmax_get(the_chip, &current_max);
		val->intval = current_max;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
#ifdef CONFIG_PANTECH_SMB_CHARGER
#else
		if (charging_disabled)
			return 0;
#endif

#if defined(CONFIG_PANTECH_CHARGER)
		status = pm_chg_get_rt_status(the_chip, BATT_REMOVED_IRQ);

		if (status && 
			(smem_id_vendor1_ptr->power_on_mode == CHG_PM_OFFLINE_NORMAL_BOOT_MODE))
			return 0;
#endif

#if defined(CONFIG_PANTECH_CHARGER)
		/* USB charging */
		if (psy->type == POWER_SUPPLY_TYPE_USB ||
				psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
				psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
				psy->type == POWER_SUPPLY_TYPE_USB_ACA) {
			#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
			if (is_usb_chg_plugged_in(the_chip)) {
				if ((chg_usb_type == USB_SDP_CHARGER) && (the_chip->cable_type != FACTORY_CABLE)) {
					if (!the_chip->cable_usb_update_delay_decision)
						val->intval = 1;
					else if((the_chip->cable_usb_update_delay_decision) && (strcmp(usb_composite_get_udc_state(), "CONNECTED") == 0))                         
						val->intval = 1;
					else
						val->intval = 0; 
				} else
					val->intval = 0;
			}
			#else
			if(is_usb_chg_plugged_in(the_chip) && 
				(chg_usb_type == USB_SDP_CHARGER) &&
				(the_chip->cable_type != FACTORY_CABLE))
				val->intval = 1;
			#endif
			else
				val->intval = 0;

			pr_debug("[USB] chg_usb_type = %d, cable_type = %d\n", chg_usb_type, the_chip->cable_type);
			return 0;
		}
#else
		/* USB charging */
		if (psy->type == POWER_SUPPLY_TYPE_USB ||
				psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
				psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
				psy->type == POWER_SUPPLY_TYPE_USB_ACA) {
			val->intval = is_usb_chg_plugged_in(the_chip);
			return 0;
		}
#endif

#if defined(CONFIG_PANTECH_CHARGER)
		/* DC charging */
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
			if((true == the_smb347->is_dth_detect) && (false == the_smb347->is_otg_detect))
			{
				val->intval = 1;
				return 0;
			}
#endif
#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
			if (is_dc_chg_plugged_in(the_chip))
				val->intval = 1;
			else if (is_usb_chg_plugged_in(the_chip)) {
				if ((chg_usb_type > USB_SDP_CHARGER) || (the_chip->cable_type == FACTORY_CABLE))
					val->intval = 1;
				else if((chg_usb_type == USB_SDP_CHARGER) &&
						(the_chip->cable_usb_update_delay_decision) &&
						(strcmp(usb_composite_get_udc_state(), "CONNECTED") != 0))                 
					val->intval = 1;
				else
					val->intval = 0;
			}
#else
			if (is_dc_chg_plugged_in(the_chip) || 
					(is_usb_chg_plugged_in(the_chip) && (chg_usb_type > USB_SDP_CHARGER || the_chip->cable_type == FACTORY_CABLE)))
				val->intval = 1;
#endif                                               
			else
				val->intval = 0;

			pr_debug("[MAINS] chg_usb_type = %d, cable_type = %d\n", chg_usb_type, the_chip->cable_type);

			return 0;
		}
#else
		/* DC charging */
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			/* external charger is connected */
			if (the_chip->dc_present || is_ext_charging(the_chip)) {
				val->intval = 1;
				return 0;
			}
			/* USB with max current greater than 500 mA connected */
			pm_chg_iusbmax_get(the_chip, &current_max);
			if (current_max > USB_WALL_THRESHOLD_MA)
				val->intval = is_usb_chg_plugged_in(the_chip);
			return 0;
		}
#endif
		pr_err("Unkown POWER_SUPPLY_TYPE %d\n", psy->type);
		break;
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        case POWER_SUPPLY_PROP_WIRELESS:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {          
                    if(is_dc_chg_plugged_in(the_chip) && !is_usb_chg_plugged_in(the_chip))
                        val->intval = 1;
                    else
                        val->intval = 0;
                    //pr_info("[pm_power_get_property] chg_usb_type = %d, cable_type = %d, cable_usb_update_delay_decision = %d\n", chg_usb_type, chip->cable_type, chip->cable_usb_update_delay_decision);
               }
		if (psy->type == POWER_SUPPLY_TYPE_USB ||
			psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
			psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
			psy->type == POWER_SUPPLY_TYPE_USB_ACA) {
                    if(is_dc_chg_plugged_in(the_chip) && !is_usb_chg_plugged_in(the_chip))
                        val->intval = 1;
                    else
                        val->intval = 0;
                    //pr_info("[pm_power_get_property] chg_usb_type = %d, cable_type = %d, cable_usb_update_delay_decision = %d\n", chg_usb_type, chip->cable_type, chip->cable_usb_update_delay_decision);
               }
	        break;
#endif    
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	case POWER_SUPPLY_PROP_DTH:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if(the_smb347->is_dth_detect)
				val->intval = 1;
			else
				val->intval = 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB ||
				psy->type == POWER_SUPPLY_TYPE_USB_DCP ||
				psy->type == POWER_SUPPLY_TYPE_USB_CDP ||
				psy->type == POWER_SUPPLY_TYPE_USB_ACA) {
			if(the_smb347->is_dth_detect)
				val->intval = 1;
			else
				val->intval = 0;
		}
		break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}
#else // CONFIG_PANTECH_CHARGER
static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

#define USB_WALL_THRESHOLD_MA	500
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;

		if (the_chip->has_dc_supply) {
			val->intval = 1;
			return 0;
		}

		if (charging_disabled)
			return 0;

		/* check external charger first before the dc path */
		if (is_ext_charging(the_chip)) {
			val->intval = 1;
			return 0;
		}

		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
			return 0;
		}

		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}

		/* USB with max current greater than 500 mA connected */
		if (usb_target_ma > USB_WALL_THRESHOLD_MA)
			val->intval = is_usb_chg_plugged_in(the_chip);
			return 0;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int switch_usb_to_charge_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (!chip->host_mode)
		return 0;

	/* enable usbin valid comparator and remove force usb ovp fet off */
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0xB2);
	if (rc < 0) {
		pr_err("Failed to write 0xB2 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 0;

	return 0;
}

static int switch_usb_to_host_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->host_mode)
		return 0;

	/* disable usbin valid comparator and force usb ovp fet off */
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0xB3);
	if (rc < 0) {
		pr_err("Failed to write 0xB3 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 1;

	return 0;
}

static int pm_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		if (val->intval == POWER_SUPPLY_SCOPE_SYSTEM)
			return switch_usb_to_host_mode(the_chip);
		if (val->intval == POWER_SUPPLY_SCOPE_DEVICE)
			return switch_usb_to_charge_mode(the_chip);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		return pm8921_set_usb_power_supply_type(val->intval);
	default:
		return -EINVAL;
	}
	return 0;
}

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;

		/*
		 * if drawing any current from usb is disabled behave
		 * as if no usb cable is connected
		 */
		if (pm_is_chg_charge_dis(the_chip))
			return 0;

		/* USB charging */
		if (usb_target_ma < USB_WALL_THRESHOLD_MA)
			val->intval = is_usb_chg_plugged_in(the_chip);
		else
		    return 0;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		if (the_chip->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif // CONFIG_PANTECH_CHARGER


static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ENERGY_FULL,
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);

#if defined(CONFIG_PANTECH_BMS_TEST)
	chip->current_uvolt = (int)(result.physical);
#endif

	return (int)result.physical;
}

#if !defined(CONFIG_PANTECH_BMS_UPDATE)
static unsigned int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
	unsigned int current_voltage_uv = get_prop_battery_uvolts(chip);
	unsigned int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
}
#endif

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
}

#if defined(CONFIG_PANTECH_BMS_UPDATE)
static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	int percent_soc = pm8921_bms_get_percent(max17058_uses);
#else
	int percent_soc = pm8921_bms_get_percent();
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
	if (atomic_read(&bms_input_flag))
		percent_soc = chip->soc;
#endif

#if defined(PANTECH_CHARGER_FACTORY_BOOT)
	if ((percent_soc <= 0) && (chip->cable_type == FACTORY_CABLE))
		percent_soc = 1;
#endif

#if defined(CONFIG_PANTECH_BMS_TEST)
	if ((percent_soc <= 0) && (atomic_read(&bms_cutoff_flag) == 0))
		percent_soc = 1;
#endif

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
	if (percent_soc >= 100)
		chip->err_charge_done = true;
#endif

	pr_err("percent_soc : %d\n", percent_soc);

	return percent_soc;
}
#else
static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;

	if (!get_prop_batt_present(chip))
		percent_soc = voltage_based_capacity(chip);
	else
		percent_soc = pm8921_bms_get_percent_charge();

	if (percent_soc == -ENXIO)
		percent_soc = voltage_based_capacity(chip);

	if (percent_soc <= 10)
		pr_warn_ratelimited("low battery charge = %d%%\n",
						percent_soc);

	if (chip->recent_reported_soc == (chip->resume_charge_percent + 1)
			&& percent_soc == chip->resume_charge_percent) {
		pr_debug("soc fell below %d. charging enabled.\n",
						chip->resume_charge_percent);
		if (chip->is_bat_warm)
			pr_warn_ratelimited("battery is warm = %d, do not resume charging at %d%%.\n",
					chip->is_bat_warm,
					chip->resume_charge_percent);
		else if (chip->is_bat_cool)
			pr_warn_ratelimited("battery is cool = %d, do not resume charging at %d%%.\n",
					chip->is_bat_cool,
					chip->resume_charge_percent);
		else
			pm_chg_vbatdet_set(the_chip, PM8921_CHG_VBATDET_MAX);
	}

	chip->recent_reported_soc = percent_soc;
	return percent_soc;
}
#endif

static int get_prop_batt_current(struct pm8921_chg_chip *chip)
{
	int result_ua, rc;

	rc = pm8921_bms_get_battery_current(&result_ua);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(&result_ua);
	}

	if (rc) {
		pr_err("unable to get batt current rc = %d\n", rc);
		return rc;
	} else {
		return result_ua;
	}
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
	int temp;

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (is_dummy_battery)
		return POWER_SUPPLY_HEALTH_GOOD;
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	temp = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ) && pm_chg_get_rt_status(chip,CHG_GONE_IRQ) &&
		pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ) && pm_chg_get_rt_status(chip, DCIN_OV_IRQ) &&
		!pm_chg_get_rt_status(chip, DCIN_UV_IRQ);

	if (temp)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
#endif

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_COLD;

#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
	if (chip->batt_charge_done_warm_cool)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
#endif

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (is_ext_trickle_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (is_ext_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;

	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;

	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

#if defined(FEATURE_PANTECH_BATTERY_STATUS)
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	if ((is_dc_chg_plugged_in(chip) > 0) || (is_usb_chg_plugged_in(chip) > 0) || the_smb347->is_dth_detect) {
#else
	if (is_dc_chg_plugged_in(chip) || is_usb_chg_plugged_in(chip)) {
#endif
		batt_state = POWER_SUPPLY_STATUS_CHARGING;

		#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
		if (atomic_read(&g_otg_mode_enabled))
			return POWER_SUPPLY_STATUS_DISCHARGING;
		#endif
	} else
		batt_state = POWER_SUPPLY_STATUS_DISCHARGING;

	if (pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ) || 
		pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ)) {
		pr_err("battery temperature is too hot or cold\n");
		#if !defined(CONFIG_PANTECH_SMB_CHARGER)
		batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
		#endif
	} else {
		int ov;
		ov = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ) &&
				pm_chg_get_rt_status(chip,CHG_GONE_IRQ) &&
				pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ) &&
				pm_chg_get_rt_status(chip, DCIN_OV_IRQ) &&
				!pm_chg_get_rt_status(chip, DCIN_UV_IRQ);

		if (ov)
			batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	#if defined(PANTECH_CHARGER_TIME_LIMITATION)
	if (chip->err_type != ERR_NONE)
		batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	#endif

	#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	if (pm8921_bms_get_percent(max17058_uses) >= 100)   
	#else
	if (pm8921_bms_get_percent() >= 100)        
	#endif
		batt_state =  POWER_SUPPLY_STATUS_FULL;
#endif

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
	if ((fsm_state == FSM_STATE_TRKL_CHG_8) || (fsm_state == FSM_STATE_FAST_CHG_7))
		chip->err_fsm_type = fsm_state;
#endif

	return batt_state;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int get_prop_batt_temp(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (is_dummy_battery) {
		pr_err("Dummy battery recognized\n");
		return 300;
	}
#endif

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);

#if defined(CONFIG_PANTECH_BMS_TEST)
	chip->current_temp = (int)(result.physical);
	pr_err("battery temperature = %d\n", chip->current_temp/10);
#endif

	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

#if defined(TEMPORARY_FIX_FOR_BATT_TEMP)
	return 300;
#endif
	return (int)result.physical;
}

static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	if (pm_power_get_forced_property(psy, psp, val) >= 0)
		return 0;
#endif
    
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#if defined(CONFIG_PANTECH_PMIC_MAX17058)
		if (max17058_uses)
			val->intval = get_max17058_voltage();
		else
		val->intval = get_prop_battery_uvolts(chip);
#else
		val->intval = get_prop_battery_uvolts(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (is_dummy_battery) {
		val->intval = 30;
		pr_err("Dummy battery recognized\n");
	}
#endif

		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = get_prop_batt_fcc(chip) * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#if defined(FEATURE_ANDROID_PANTECH_USB_OTG_MODE) || defined(FEATURE_ANDROID_PANTECH_USB_SMB_OTG_MODE)
int get_cable_id_adc_value(void)
{
	struct pm8xxx_adc_chan_result result;
	int rc, try_max = 0;

	do{
		rc = pm8xxx_adc_mpp_config_read(PM8XXX_AMUX_MPP_3, ADC_MPP_1_AMUX6, &result);
		if(rc == -EINVAL)
			return -EINVAL;
		try_max++;
	}while(rc && (try_max < 20));

	if(!rc){
		return result.physical;
	}else{
		return 0;
	}
}
#endif

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
static int init_charger(struct pm8921_chg_chip *chip)
{
	int ret = 0;

	chip->err_type = ERR_NONE;
	chip->err_fsm_type = FSM_STATE_OFF_0;
	chip->err_charge_done = false;

	ret = pm_chg_failed_clear(chip, 1);

	if (ret)
		pr_err("Failed to write CHG_FAILED_CLEAR bit\n");

	if (chip->safety_time > PM8921_CHG_TCHG_MAX) {       
		safety_time_remain = chip->safety_time - PM8921_CHG_TCHG_MAX;
		ret |= pm_chg_tchg_max_set(chip, PM8921_CHG_TCHG_MAX);
	} else {    
		safety_time_remain = 0;
		ret |= pm_chg_tchg_max_set(chip, chip->safety_time);
	}

	if (ret)
		pr_err("Failed to set max time to %d minutes ret=%d\n", chip->safety_time, ret);

	return ret;
}
#endif

#if defined(CONFIG_PANTECH_CHARGER)
static unsigned int init_iusb_ibat(struct pm8921_chg_chip *chip)
{
	pm_chg_iusbmax_set(chip, 0);
	pm_chg_ibatmax_set(chip, DEFAULT_IBAT_IMAX);

	return 0;
}

static int64_t get_cable_id_raw(void)
{
	int rc;
	int try_max = 0;

	struct pm8xxx_adc_chan_result result;
	
	do {
		rc = pm8xxx_adc_mpp_config_read(PM8XXX_AMUX_MPP_3, ADC_MPP_1_AMUX6, &result);
	} while (rc && (try_max < 20));

	if (!rc)
		return result.physical;
	else
		return 0;
}

static uint get_cable_type(struct pm8921_chg_chip *chip)
{
	static uint cable_type = INVALID_CABLE; 
	int is_usb, is_dc;
	int64_t adc_val;    

	is_dc  = is_dc_chg_plugged_in(chip);
	is_usb = is_usb_chg_plugged_in(chip);

	adc_val = get_cable_id_raw();

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
		if (is_usb && adc_val) {
#else
	if (adc_val) {
		if (is_usb) {
#endif          
			if ((adc_val >= FACT_CABLE_MIN) && (adc_val <= FACT_CABLE_MAX))
				cable_type = FACTORY_CABLE;
			#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
			else if (chg_usb_type == USB_SDP_CHARGER) {
				if (!chip->cable_usb_update_delay_decision)
					cable_type = STANDARD_CABLE;
				else {
					if (strcmp(usb_composite_get_udc_state(), "CONNECTED") == 0)                        
						cable_type = STANDARD_CABLE;
					else
						cable_type = UNKNOWN_CABLE;
				}
			}                
			#else
			else if (chg_usb_type == USB_SDP_CHARGER)
				cable_type = STANDARD_CABLE;
			#endif                 
			else if (chg_usb_type >= USB_DCP_CHARGER && chg_usb_type <= USB_ACA_DOCK_CHARGER)
				cable_type = TA_CABLE;
			else
				cable_type = UNKNOWN_CABLE;                              
		} 
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
		else if (is_dc)
			cable_type = WIRELESS_CABLE;  
#endif
		else
			cable_type = NO_CABLE;

#ifdef CONFIG_PANTECH_SMB_CHARGER_DTH
	if(the_smb347->is_dth_detect)
		cable_type = DTH_CABLE;
	if(the_smb347->is_otg_detect)
		cable_type = NO_CABLE;
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
	if (atomic_read(&g_otg_mode_enabled)) {
		printk(KERN_ERR "[%s]NO_CABLE\n",__func__);
		cable_type = NO_CABLE;
	}
#endif
#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	}    
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
	chip->cable_adc = adc_val;
#endif
	chip->cable_type = cable_type;

	pr_err("is_usb : %d, is_dc : %d, USB_IN adc : %lld, chg_usb_type : %d\n", is_usb, is_dc, adc_val, chg_usb_type);

	return cable_type;
}

unsigned int set_cable_imax(struct pm8921_chg_chip *chip, uint cable_type)
{
	unsigned int ibat_max;
	unsigned int chg_current;
#ifdef CONFIG_PANTECH_SMB_CHARGER
	static int flag = true;
	static uint old_cable_type = NO_CABLE;
	pr_debug("[PMIC]flag=%d,cable_type=%d\n",flag,cable_type);

	if(old_cable_type != cable_type || cable_type == NO_CABLE)
		flag = true;
	old_cable_type = cable_type;
#endif

	switch (cable_type) {
		case STANDARD_CABLE:
#ifdef CONFIG_PANTECH_SMB_CHARGER
			if(flag)
			{
				smb347_set_cable_imax_usb();
				flag = false;
			}
#endif
			usb_target_ma = STANDARD_IUSB_IMAX;
			ibat_max = STANDARD_IBAT_IMAX;                
			break;

		case TA_CABLE:
#ifdef CONFIG_PANTECH_SMB_CHARGER
			if(flag)
			{
				if(smb347_is_trickle())
					smb347_set_cable_imax_usb();
				else
					smb347_set_cable_imax_ta();
				flag = false;
			}
#endif
			usb_target_ma = TA_IUSB_IMAX;
			ibat_max = TA_IBAT_IMAX;    
			break;

		case FACTORY_CABLE:
			usb_target_ma = FACTORY_IUSB_IMAX; 
			ibat_max = FACTORY_IBAT_IMAX;    
			break;
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
		case WIRELESS_CABLE:
            	        usb_target_ma = 0; 
			//usb_target_ma = WIRELESS_IUSB_IMAX; 
			ibat_max = WIRELESS_IBAT_IMAX;    
			break;                
#endif
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
		case DTH_CABLE:
			if(flag)
			{
				smb347_set_cable_imax_dth();
				flag = false;
			}
			ibat_max = TA_IBAT_IMAX;    
			break;
#endif
		case UNKNOWN_CABLE:
#ifdef CONFIG_PANTECH_SMB_CHARGER
			if(flag)
			{
				smb347_set_cable_imax_unkown();
				flag = false;
			}
#endif
			usb_target_ma = UNKNOWN_IUSB_IMAX; 
			ibat_max = UNKNOWN_IBAT_IMAX;    
			break;             

		default:
			usb_target_ma = DEFAULT_IUSB_IMAX;
			ibat_max = DEFAULT_IBAT_IMAX;    
			break;
	}

#if defined(T_EF45K) || defined(T_EF46L) || defined(T_EF47S)
	if (!smem_id_vendor1_ptr->power_on_mode) {
		usb_target_ma = TA_IUSB_IMAX;
	}
#endif

	//set IBAT
	chg_current = ibat_max;
	
	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);
	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);
	
	pm_chg_ibatmax_set(the_chip, chg_current);

	chip->cable_iusb_max = usb_target_ma;
	chip->cable_ibat_max = ibat_max;        

	return usb_target_ma;
}     
#endif // #if defined(CONFIG_PANTECH_CHARGER)

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;
static DEFINE_SPINLOCK(vbus_lock);

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

/* assumes vbus_lock is held */
static void __pm8921_charger_vbus_draw(unsigned int mA)
{
#if defined(CONFIG_PANTECH_CHARGER)
	int i = 0;
	int rc;
#else
	int i, rc;
#endif
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

#if defined(CONFIG_PANTECH_CHARGER)
	if (mA > the_chip->max_bat_chg_current)
		mA = the_chip->max_bat_chg_current;
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
	if (atomic_read(&g_otg_mode_enabled)){
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_debug("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_debug("fail to set suspend bit rc=%d\n", rc);
#if defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&the_chip->dc_psy);
	power_supply_changed(&the_chip->usb_psy);
#endif	
		return;
	}
#endif

	if (usb_max_current && mA > usb_max_current) {
		pr_debug("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}

	if (mA <= 2) {
#if defined(PANTECH_CHARGER_FACTORY_BOOT)
		if(the_chip->cable_type != FACTORY_CABLE)
#endif                    
		{
		usb_chg_current = 0;

		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}

		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
		}
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		/* Check if IUSB_FINE_RES is available */
		if ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		if (rc)
                        pr_err("unable to set iusb to %d rc = %d\n", i, rc);                
	}

#if defined(CONFIG_PANTECH_CHARGER)
	usb_chg_current = usb_ma_table[i].usb_ma;
#endif // #if defined(CONFIG_PANTECH_CHARGER)
}

/* USB calls these to tell us how much max usb current the system can draw */
#if defined(CONFIG_PANTECH_CHARGER)
void pm8921_charger_vbus_draw(unsigned int mA, unsigned int chg_type)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);
	pr_debug("usb_target_ma=%d mA, chg_type : %d\n", usb_target_ma, chg_type);

#if !defined(CONFIG_PANTECH_CHARGER)
	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	if (chg_type <= USB_SDP_CHARGER)
		usb_target_ma = 0;
	else
		usb_target_ma = TA_IUSB_IMAX;
#else
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;
#endif

	spin_lock_irqsave(&vbus_lock, flags);

#if defined(CONFIG_PANTECH_CHARGER)
	if (the_chip) {
#if defined(CONFIG_ANDROID_PANTECH_USB_OTG_CHARGER_SUSPEND)
		if(atomic_read(&g_otg_mode_enabled)){
			chg_usb_type = USB_INVALID_CHARGER;
			set_cable_imax(the_chip, NO_CABLE);
			printk("[jeanclad] %s : set_cable_imax = NO_CABLE\n", __func__);
		}
		else
#endif
		if (is_usb_chg_plugged_in(the_chip)) {
			if (mA > 0 && mA <= 2) {
				if (!is_usb_chg_plugged_in(the_chip))
					chg_usb_type = USB_INVALID_CHARGER;
			}
			else
				chg_usb_type = chg_type;
		}
		else
			chg_usb_type = USB_INVALID_CHARGER;
	} else
		chg_usb_type = chg_type;
#endif

	if (the_chip) {
#if defined(CONFIG_PANTECH_CHARGER)
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
#else
		if (mA > USB_WALL_THRESHOLD_MA)
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		else
			__pm8921_charger_vbus_draw(mA);
#endif
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		if (mA > USB_WALL_THRESHOLD_MA)
			usb_chg_current = USB_WALL_THRESHOLD_MA;
		else
			usb_chg_current = mA;
	}
	spin_unlock_irqrestore(&vbus_lock, flags);

#if defined(CONFIG_PANTECH_CHARGER)
	queue_work(the_chip->cable_update_wq, &the_chip->update_cable_work);
#endif

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
	if(the_chip->cable_usb_update_delay)
		__cancel_delayed_work(&the_chip->update_cable_work_delay);

	the_chip->cable_usb_update_delay_decision = false;
	the_chip->cable_usb_update_delay = true;

	schedule_delayed_work(&the_chip->update_cable_work_delay,
		round_jiffies_relative(msecs_to_jiffies(the_chip->update_cable_time_delay)));
#endif
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);
#else
void pm8921_charger_vbus_draw(unsigned int mA)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);

	if (!the_chip) {
		pr_err("chip not yet initalized\n");
		return;
	}

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (!get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		if (!the_chip->has_dc_supply) {
			pr_err("rejected: no other power source connected\n");
			return;
		}
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	spin_lock_irqsave(&vbus_lock, flags);
	if (the_chip) {
		if (mA > USB_WALL_THRESHOLD_MA)
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		else
			__pm8921_charger_vbus_draw(mA);
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		if (mA > USB_WALL_THRESHOLD_MA)
			usb_chg_current = USB_WALL_THRESHOLD_MA;
		else
			usb_chg_current = mA;
	}
	spin_unlock_irqrestore(&vbus_lock, flags);
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);
#endif

int pm8921_charger_enable(bool enable)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	enable = !!enable;
	rc = pm_chg_auto_enable(the_chip, enable);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(pm8921_charger_enable);

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);
/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm8xxx_writeb(the_chip->dev->parent,
			 CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8917_set_under_voltage_detection_threshold(int mv)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_uvd_threshold_set(the_chip, mv);
}
EXPORT_SYMBOL(pm8917_set_under_voltage_detection_threshold);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");
	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB)
		return -EINVAL;

	the_chip->usb_psy.type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_temp(the_chip);
}

static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present;

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {

#if defined(CONFIG_PANTECH_CHARGER)
		if (!usb_present) {
			pr_debug("usb_present : FALSE\n");
			#if defined(PANTECH_CHARGER_TIME_LIMITATION)
			init_charger(chip);
			#endif
			init_iusb_ibat(chip);
			chg_usb_type = USB_INVALID_CHARGER;
		}
#endif
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;

#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
	if (!usb_present && chip->batt_charge_done_warm_cool)
		chip->batt_charge_done_warm_cool = false;
#endif

//++ p11309 - 2012.08.01 for fast remove noti
#if !defined(CONFIG_PANTECH_CHARGER)
		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
#endif
//-- p11309

		pm8921_bms_calibrate_hkadc();
	}
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	} else {
		/* USB unplugged reset target current */
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	}
	enable_input_voltage_regulation(chip);
	bms_notify_check(chip);

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        if (usb_present)
            wireless_full_sign(chip, false);
#endif
}

#if defined(CONFIG_PANTECH_CHARGER)
int pm8921_power_supply_changed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	power_supply_changed(&the_chip->batt_psy);
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
int smb347_dth_power_supply_changed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	queue_work(the_chip->cable_update_wq, &the_chip->update_cable_work);

	if(the_smb347->is_dth_detect == false)
	{
		if (is_usb_chg_plugged_in(the_chip)) 
			handle_usb_insertion_removal(the_chip);
	}
	pm8921_power_supply_changed();
	return 0;
}

int smb347_otg_power_supply_changed(void)
{
	uint cable_type = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	cable_type = get_cable_type(the_chip);
	set_cable_imax(the_chip, cable_type);

	pm8921_power_supply_changed();
	return 0;
}
#endif
#endif

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
static void pm_chg_iusbmax_set_ma(struct pm8921_chg_chip *chip, int ma)
{
        int rc, i =0;

        for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
            if (usb_ma_table[i].usb_ma <= ma)
            	break;
        }
        if (i < 0)
            i = 0;
        rc = pm_chg_iusbmax_set(chip, i);
        if (rc) {
            pr_info("unable to set iusb to %d rc = %d\n", i, rc);
        }
}

static void handle_dc_removal_insertion(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int usb_present;

	usb_present = is_usb_chg_plugged_in(chip);
	dc_present = is_dc_chg_plugged_in(chip);
          
	if (chip->dc_present ^ dc_present) {
		chip->dc_present = dc_present;
        
                if(!usb_present){
                        pm_chg_failed_clear(chip, 1);
                    
#ifdef PANTECH_CHARGER_TIME_LIMITATION
                        init_charger(chip);
#endif                    
                        init_iusb_ibat(chip);
                        if(chip->dc_present)
													queue_work(the_chip->cable_update_wq, &the_chip->update_cable_work);
                        else
                            chip->cable_type = NO_CABLE;

            		pm8921_bms_calibrate_hkadc();
}

#ifdef PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL	
		if(!dc_present && chip->batt_charge_done_warm_cool)			
			chip->batt_charge_done_warm_cool = false;		
#endif /* PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL */

//++ p11309 - 2012.08.01 for fast remove noti
#if !defined(CONFIG_PANTECH_CHARGER)
		power_supply_changed(&chip->dc_psy);
		power_supply_changed(&chip->batt_psy);
#endif
//-- p11309
        } 
        if(dc_present){
             if(!usb_present){
                    usb_target_ma = 0;
                    pm_chg_iusbmax_set_ma(chip, 700); //WIRELESS_IUSB_IMAX
                    pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
            }                
	}
        else{
            if(!usb_present){
		/* USB unplugged reset target current */
		usb_target_ma = 0;
                pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
            }    
        }
        enable_input_voltage_regulation(chip);
	bms_notify_check(chip);
}
#endif

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}
#endif    

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        handle_dc_removal_insertion(chip);
#else
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
#endif
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	int vbat_ov;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}
#endif

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(the_chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)        
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}
	pm8921_disable_source_current(true); /* Force BATFET=ON */
#endif
	vbat_ov = pm_chg_get_rt_status(chip, VBAT_OV_IRQ);
	if (vbat_ov) {
		pr_warn("%s. battery over voltage.\n", __func__);
		return;
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        handle_dc_removal_insertion(chip);
#else
	schedule_delayed_work(&chip->unplug_check_work,
	round_jiffies_relative(msecs_to_jiffies
		(UNPLUG_CHECK_WAIT_PERIOD_MS)));
	pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
#endif
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Start BMS */
	schedule_delayed_work(&chip->eoc_work, delay);
	wake_lock(&chip->eoc_wake_lock);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void turn_off_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from OVP_TEST rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x OVP_TEST rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

static int param_open_ovp_counter = 10;
module_param(param_open_ovp_counter, int, 0644);

#define USB_ACTIVE_BIT BIT(5)
#define DC_ACTIVE_BIT BIT(6)
static int is_active_chg_plugged_in(struct pm8921_chg_chip *chip,
						u8 active_chg_mask)
{
	if (active_chg_mask & USB_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	else if (active_chg_mask & DC_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	else
		return 0;
}

#define WRITE_BANK_4		0xC0
#define OVP_DEBOUNCE_TIME 0x06
static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg_gone = 0, active_chg_plugged_in = 0;
	int count = 0;
	u8 active_mask = 0;
	u16 ovpreg, ovptestreg;

	if (is_usb_chg_plugged_in(chip) &&
		(chip->active_path & USB_ACTIVE_BIT)) {
		ovpreg = USB_OVP_CONTROL;
		ovptestreg = USB_OVP_TEST;
		active_mask = USB_ACTIVE_BIT;
	} else if (is_dc_chg_plugged_in(chip) &&
		(chip->active_path & DC_ACTIVE_BIT)) {
		ovpreg = DC_OVP_CONTROL;
		ovptestreg = DC_OVP_TEST;
		active_mask = DC_ACTIVE_BIT;
	} else {
		return;
	}

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		active_chg_plugged_in
			= is_active_chg_plugged_in(chip, active_mask);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_debug("OVP FET count = %d chg_gone=%d, active_valid = %d\n",
					count, chg_gone, active_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && active_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_ovp_fet(chip, ovptestreg);

			msleep(20);

			turn_on_ovp_fet(chip, ovptestreg);
		} else {
			break;
		}
	}
	pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x2);
	pr_debug("Exit count=%d chg_gone=%d, active_valid=%d\n",
		count, chg_gone, active_chg_plugged_in);
	return;
}

static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		while (!the_chip->iusb_fine_res && i > 0
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES))
			i--;
		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/* AICL only for wall-chargers */
	if (is_usb_chg_plugged_in(chip) &&
		usb_target_ma > USB_WALL_THRESHOLD_MA) {
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
	} else {
		handle_usb_insertion_removal(chip);
	}
}

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
static void pm_dth_irq_detect(struct work_struct *work)
{
	int is_dth_detect;

	is_dth_detect =  gpio_get_value(PM8921_GPIO_PM_TO_SYS(35));
	the_smb347->is_dth_detect = is_dth_detect;

	pr_err("DTH Detect is called : %d\n",is_dth_detect);

	switch_set_state(&dth_switch_dev, is_dth_detect);
	smb347_dth_power_supply_changed();

	return;
}
static irqreturn_t pm_dth_detect_irq_handler(int irq, void *data)
{
	if(gpio_get_value(PM8921_GPIO_PM_TO_SYS(35))) 
	{
		if (the_chip->smb_wake_is == false)
		{
			pr_err("wak lock!!!!!!!!!!!!!!!!!!!!!!\n");
			the_chip->smb_wake_is = true;
			wake_lock(&the_chip->eoc_wake_lock);
		}
	}
	else 
	{
		schedule_delayed_work(&the_chip->smb_wake_unlock_work, 
				round_jiffies_relative(msecs_to_jiffies(3000)));
	}
	schedule_delayed_work(&the_smb347->dth_detect_work, round_jiffies_relative(msecs_to_jiffies
        	(100)));
	return IRQ_HANDLED;
}
#endif

#define VIN_MIN_COLLAPSE_CHECK_MS	50
static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	if (is_usb_chg_plugged_in(the_chip)) {
		if (the_chip->smb_wake_is == false)
		{
			pr_err("wak lock!!!!!!!!!!!!!!!!!!!!!!\n");
			the_chip->smb_wake_is = true;
			wake_lock(&the_chip->eoc_wake_lock);
		}
	}
	else 
	{
		schedule_delayed_work(&the_chip->smb_wake_unlock_work, 
				round_jiffies_relative(msecs_to_jiffies(3000)));
	}
#endif

	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
	    handle_usb_insertion_removal(data);
	return IRQ_HANDLED;
}

#if !defined(CONFIG_PANTECH_CHARGER)
static irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	pr_err("USB OverVoltage\n");
	return IRQ_HANDLED;
}
#endif

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
	pr_debug("battery present=%d", status);
	power_supply_changed(&chip->batt_psy);

#if defined(PANTECH_CHARGER_FACTORY_BOOT)
	if (chip->cable_type == FACTORY_CABLE) {
		if (!status)
			pm_chg_iusbmax_set(chip, 7);
		else
			pm_chg_iusbmax_set(chip, 4);
	}                 
#endif

	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);

	if (high_transition) {
		/* enable auto charging */
		pm_chg_auto_enable(chip, !charging_disabled);
		pr_info("batt fell below resume voltage %s\n",
			charging_disabled ? "" : "charger enabled");
	}
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif

	return IRQ_HANDLED;
}

#if !defined(CONFIG_PANTECH_CHARGER)
static irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	pr_err("USB UnderVoltage\n");
	return IRQ_HANDLED;
}
#endif

static irqreturn_t vbat_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        if(is_dc_chg_plugged_in(chip)){
            wireless_full_sign(chip, true);
        }     
#else
	handle_stop_ext_chg(chip);
#endif	

//++ p11309 - 2012.07.31 for charging LEDs
#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif
//-- p11309

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
	#if defined(PANTECH_CHARGER_MONITOR_TEST)
	if(chip->pm_chg_test) {
	ret = pm_chg_failed_clear(chip, 1);

	if (ret)
		pr_err("Failed to write CHG_FAILED_CLEAR bit\n");

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	return IRQ_HANDLED;
}
	#endif

	if (!chip->err_charge_done && 
		(chip->err_fsm_type == FSM_STATE_TRKL_CHG_8 || chip->err_fsm_type == FSM_STATE_FAST_CHG_7)) {
		if (safety_time_remain > 0 && chip->err_fsm_type == FSM_STATE_FAST_CHG_7) {
			ret = pm_chg_failed_clear(chip, 1);

			if (ret)
				pr_err("Failed to write CHG_FAILED_CLEAR bit\n");

			if (safety_time_remain > PM8921_CHG_TCHG_MAX) {
				ret = pm_chg_tchg_max_set(chip, PM8921_CHG_TCHG_MAX);
				safety_time_remain = safety_time_remain - PM8921_CHG_TCHG_MAX;
			} else {
				ret = pm_chg_tchg_max_set(chip, safety_time_remain);
				safety_time_remain = 0;
			}

			if (ret) {
				pr_err("Failed to set max time to %d minutes rc=%d\n",
					chip->safety_time, ret);
			}
		} else
				chip->err_type = ERR_TIMER_EXPIRED;
	}else {
		ret = pm_chg_failed_clear(chip, 1);

		if (ret)
			pr_err("Failed to write CHG_FAILED_CLEAR bit\n");
	} 
#else
	ret = pm_chg_failed_clear(chip, 1);
	if (ret)
		pr_err("Failed to write CHG_FAILED_CLEAR bit\n");
#endif

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);

#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static int param_vin_disable_counter = 5;
module_param(param_vin_disable_counter, int, 0644);

#if !defined(CONFIG_PANTECH_SMB_CHARGER)
static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip,
							int count, int usb_ma)
{
#if defined(CONFIG_PANTECH_CHARGER)
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
    int dc_chg_plugged_in;
    dc_chg_plugged_in = is_dc_chg_plugged_in(chip);
    if(dc_chg_plugged_in)
        pm_chg_iusbmax_set_ma(chip, 500);
    else
#endif
    pm8921_charger_vbus_draw(500, chg_usb_type);
#else
	if (usb_ma)
		__pm8921_charger_vbus_draw(500);
#endif	
	pr_debug("count = %d iusb=500mA\n", count);
	disable_input_voltage_regulation(chip);
	pr_debug("count = %d disable_input_regulation\n", count);

	msleep(20);

	pr_debug("count = %d end sleep 20ms chg_gone=%d, usb_valid = %d\n",
				count,
				pm_chg_get_rt_status(chip, CHG_GONE_IRQ),
				is_usb_chg_plugged_in(chip));
	pr_debug("count = %d restoring input regulation and usb_ma = %d\n",
		 count, usb_ma);
	enable_input_voltage_regulation(chip);
#if defined(CONFIG_PANTECH_CHARGER)
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
    if(dc_chg_plugged_in)
        pm_chg_iusbmax_set_ma(chip, 700); //WIRELESS_IUSB_IMAX
    else
#endif        
    pm8921_charger_vbus_draw(usb_ma, chg_usb_type);
#else		
		__pm8921_charger_vbus_draw(usb_ma);
#endif	
}
#endif

#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US 200
#define VIN_MIN_INCREASE_MV 100
#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
#define BTM_ISR_LOCK_WAIT_PERIOD_US 300
#endif
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop, active_path;
	int rc, ibat, active_chg_plugged_in, usb_ma;
	int chg_gone = 0;
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS) /* JB BUILD FIXED */
	int dc_chg_plugged_in = 0;
#endif
	reg_loop = 0;

	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS1, &active_path);
	if (rc) {
		pr_err("Failed to read PBL_ACCESS1 rc=%d\n", rc);
		return;
	}
	chip->active_path = active_path;

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg_plugged_in = %d\n",
			active_path, active_chg_plugged_in);
	if (active_path & USB_ACTIVE_BIT) {
		pr_debug("USB charger active\n");

		pm_chg_iusbmax_get(chip, &usb_ma);
		if (usb_ma == 500 && !usb_target_ma) {
			pr_debug("Stopping Unplug Check Worker USB == 500mA\n");
			disable_input_voltage_regulation(chip);
			return;
		}

		if (usb_ma <= 100) {
			pr_debug(
				"Unenumerated or suspended usb_ma = %d skip\n",
				usb_ma);
			goto check_again_later;
		}
	} else if (active_path & DC_ACTIVE_BIT) {
		pr_debug("DC charger active\n");
		/* Some board designs are not prone to reverse boost on DC
		 * charging path */
		if (!chip->dc_unplug_check)
			return;
	} else {
		/* No charger active */
		if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			pr_debug(
			"Stop: chg removed reg_loop = %d, fsm = %d ibat = %d\n",
				pm_chg_get_regulation_loop(chip),
				pm_chg_get_fsm_state(chip),
				get_prop_batt_current(chip)
				);
			return;
		} else {
			goto check_again_later;
		}
	}

	if (active_path & USB_ACTIVE_BIT) {
		reg_loop = pm_chg_get_regulation_loop(chip);
		pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);
		if ((reg_loop & VIN_ACTIVE_BIT) &&
			(usb_ma > USB_WALL_THRESHOLD_MA)
			&& !charging_disabled) {
			decrease_usb_ma_value(&usb_ma);
			usb_target_ma = usb_ma;
			/* end AICL here */
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
				usb_ma, usb_target_ma);
		}
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	ibat = get_prop_batt_current(chip);
	if (reg_loop & VIN_ACTIVE_BIT) {
		pr_debug("ibat = %d fsm = %d reg_loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
		#if !defined(CONFIG_PANTECH_SMB_CHARGER)
		if (ibat > 0) {
			int count = 0;

			while (count++ < param_vin_disable_counter
					&& active_chg_plugged_in == 1) {
				if (active_path & USB_ACTIVE_BIT)
					attempt_reverse_boost_fix(chip,
								count, usb_ma);
				else
					attempt_reverse_boost_fix(chip,
								count, 0);
				/* after reverse boost fix check if the active
				 * charger was detected as removed */
				active_chg_plugged_in
					= is_active_chg_plugged_in(chip,
						active_path);
				pr_debug("active_chg_plugged_in = %d\n",
						active_chg_plugged_in);
			}
		}
		#endif
	}

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg = %d\n",
			active_path, active_chg_plugged_in);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && active_chg_plugged_in == 1) {
		pr_debug("chg_gone=%d, active_chg_plugged_in = %d\n",
					chg_gone, active_chg_plugged_in);
		unplug_ovp_fet_open(chip);
	}

	if (!(reg_loop & VIN_ACTIVE_BIT) && (active_path & USB_ACTIVE_BIT)
		&& !charging_disabled) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
		} else {
			usb_target_ma = usb_ma;
		}
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	if (dc_chg_plugged_in){
    	    if (pm8921_bms_get_percent(max17058_uses) >= 100 && pm_chg_get_fsm_state(chip)== FSM_STATE_ON_CHG_HIGHI_1)
                wireless_full_sign(chip, true);
       }
#else
	if (dc_chg_plugged_in){
    	    if (pm8921_bms_get_percent() >= 100 && pm_chg_get_fsm_state(chip)== FSM_STATE_ON_CHG_HIGHI_1)
                wireless_full_sign(chip, true);
       }
    
#endif // #if defined(CONFIG_PANTECH_PMIC_MAX17058)
#endif // #if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
    
check_again_later:
	/* schedule to check again later */
	schedule_delayed_work(&chip->unplug_check_work,
		      round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
}

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	schedule_work(&chip->unplug_check_work.work);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;
#if defined(CONFIG_PANTECH_CHARGER)
	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	if (status) {
		if (smem_id_vendor1_ptr->power_on_mode == CHG_PM_OFFLINE_NORMAL_BOOT_MODE) {
			power_supply_changed(&chip->dc_psy);
			power_supply_changed(&chip->usb_psy);
			power_supply_changed(&chip->batt_psy);
			return IRQ_HANDLED;
		} else if (!((smem_id_vendor1_ptr->power_on_mode == CHG_PM_ONLINE_CABLE_IN_BOOT_MODE) ||
			(chip->cable_type == FACTORY_CABLE))) {
			//arch_reset(0xC9, 0);
			msm_restart(0xC9, 0);
			return IRQ_HANDLED;
		} else {
			pr_debug("battery present=%d state=%d", !status, pm_chg_get_fsm_state(data));
			handle_stop_ext_chg(chip);
			power_supply_changed(&chip->batt_psy);
			return IRQ_HANDLED;
		}
	} else {
		power_supply_changed(&chip->batt_psy);
		return IRQ_HANDLED;
	}
#else
	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_debug("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
#endif
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	handle_stop_ext_chg(chip);
#endif
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);

#if defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->dc_psy);
#endif    

#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	handle_stop_ext_chg(chip);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)  
	handle_stop_ext_chg(chip);
#endif
	power_supply_changed(&chip->batt_psy);
#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));

#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
#endif
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_err("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	if (bat_temp_ok)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);
#endif    
#ifdef CONFIG_PANTECH_SMB_CHARGER
	the_smb347->bat_temp_ok = bat_temp_ok;

	pr_err("the_smb347->cable_type : %d\n",the_smb347->cable_type);

	if(TA_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_ta();
	else if(STANDARD_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_usb();
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	else if(DTH_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_dth();
#endif
#endif

	power_supply_changed(&chip->batt_psy);
#if !defined(CONFIG_PANTECH_CHARGER)
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
#endif
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	if (chip->ext_psy)
		power_supply_set_online(chip->ext_psy, dc_present);

#if !defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	chip->dc_present = dc_present;
#endif

	if (dc_present)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);

	return IRQ_HANDLED;
}

#if !defined(CONFIG_PANTECH_CHARGER)
static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}
#endif

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#define LOW_SOC_HEARTBEAT_MS	20000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);

#if defined(CONFIG_PANTECH_CHARGER)
	u8  temp;
	int reg_val1, reg_val2;

	pm_chg_masked_read(chip, PBL_ACCESS2, PM8921_CHG_IUSB_MASK, &temp);
	reg_val1 = temp >> 2;
	pm_chg_masked_read(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, &temp);
	reg_val2 = (temp * PM8921_CHG_I_STEP_MA) + PM8921_CHG_I_MIN_MA;
	
#else
	pm_chg_failed_clear(chip, 1);
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
        chip->bms_notify.batt_notifier_periodic_update = true;
        schedule_work(&(chip->bms_notify.work));       
#else
	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
#endif
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);

#define PM8921_CHG_VDDMAX_RES_MV	10
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip,
						int vbat_batt_terminal_uv)
{
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
		pr_debug("Exiting as rconn_mohm is 0\n");
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	adj_vdd_max_mv = DIV_ROUND_UP(adj_vdd_max_mv, PM8921_CHG_VDDMAX_RES_MV)
					* PM8921_CHG_VDDMAX_RES_MV;

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;
	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

#if defined(CONFIG_PANTECH_CHARGER)
static void update_cable(struct work_struct *work)
{
	uint cable_type = 0;
	uint cable_current;

	struct pm8921_chg_chip *chip = container_of(work,
		struct pm8921_chg_chip,
		update_cable_work);

	cable_type = get_cable_type(chip);
	cable_current = set_cable_imax(chip, cable_type);

#if defined(PANTECH_CHARGER_SMB_EOC)
	chip->smb_charging_cable_type = cable_type;
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	if (cable_type != NO_CABLE)
	{
#if defined(PANTECH_CHARGER_SMB_EOC)
		schedule_delayed_work(&chip->smb_eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (SMB_EOC_CHECK_PERIOD_MS)));
#endif
	}
#endif

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
                btm_init(chip);
#endif

	//if (cable_type != STANDARD_CABLE) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
			(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		enable_input_voltage_regulation(chip);
	//}

	pr_err("cable_type=%d, iusb_imax=%d, ibat_imax=%d\n", cable_type, iusbmax_set_cur, ibatmax_set_cur);  
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	the_smb347->cable_type = cable_type;
#endif
	power_supply_changed(&chip->dc_psy);
	power_supply_changed(&chip->usb_psy);
//++ p11309 - 2012.07.31 for charging LEDs
	power_supply_changed(&chip->batt_psy);
//-- p11309
}
#endif

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
static void update_cable_delay(struct work_struct *work)
{
	uint cable_type, cable_current;

	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_cable_work_delay);

	if (chip->cable_usb_update_delay) {
		chip->cable_usb_update_delay = false;  
		chip->cable_usb_update_delay_decision = true;

		cable_type = get_cable_type(chip);
		cable_current = set_cable_imax(chip, cable_type);
#if defined(PANTECH_CHARGER_SMB_EOC)		
		chip->smb_charging_cable_type = cable_type;
#endif
		//if (cable_type != STANDARD_CABLE) {        
			schedule_delayed_work(&chip->unplug_check_work,
				round_jiffies_relative(msecs_to_jiffies
					(UNPLUG_CHECK_WAIT_PERIOD_MS)));
			enable_input_voltage_regulation(chip);
		//}

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	the_smb347->cable_type = cable_type;
#endif
		power_supply_changed(&chip->dc_psy);
		power_supply_changed(&chip->usb_psy); 
//++ p11309 - 2012.07.31 for charging LEDs
		power_supply_changed(&chip->batt_psy);
//-- p11309
		pr_err("cable_type = %d, iusb_imax = %d, ibat_imax = %d\n", cable_type, iusbmax_set_cur, ibatmax_set_cur);       
	}          
}
#endif

#if !defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};
#endif

#define VBAT_TOLERANCE_MV	70
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip,
			int vbat_batt_terminal_uv, int ichg_meas_ma)
{
	int vbat_programmed, iterm_programmed, vbat_intended;
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_batt_terminal_uv=%d\n",
			 vbat_programmed, vbat_batt_terminal_uv);

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

		if (chip->is_bat_cool)
			vbat_intended = chip->cool_bat_voltage;
		else if (chip->is_bat_warm)
			vbat_intended = chip->warm_bat_voltage;
		else
			vbat_intended = chip->max_voltage_mv;

		if (vbat_batt_terminal_uv / 1000 < vbat_intended) {
			pr_debug("terminal_uv:%d < vbat_intended:%d.\n",
							vbat_batt_terminal_uv,
							vbat_intended);
			return CHG_IN_PROGRESS;
		}

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

	return CHG_FINISHED;
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;
	int vbat_meas_uv, vbat_meas_mv;
	int ichg_meas_ua, ichg_meas_ma;
	int vbat_batt_terminal_uv;

#if !defined(CONFIG_PANTECH_CHARGER)
	pm_chg_failed_clear(chip, 1);
#endif

	pm8921_bms_get_simultaneous_battery_voltage_and_current(
					&ichg_meas_ua,	&vbat_meas_uv);
	vbat_meas_mv = vbat_meas_uv / 1000;
	/* rconn_mohm is in milliOhms */
	ichg_meas_ma = ichg_meas_ua / 1000;
	vbat_batt_terminal_uv = vbat_meas_uv
					+ ichg_meas_ma
					* the_chip->rconn_mohm;

	end = is_charging_finished(chip, vbat_batt_terminal_uv, ichg_meas_ma);

	if (end == CHG_NOT_IN_PROGRESS) {
		count = 0;
		wake_unlock(&chip->eoc_wake_lock);
		return;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");
		/* set the vbatdet back, in case it was changed
		 * to trigger charging */
		if (chip->is_bat_cool) {
			pm_chg_vbatdet_set(the_chip,
				the_chip->cool_bat_voltage
				- the_chip->resume_voltage_delta);
		} else if (chip->is_bat_warm) {
			pm_chg_vbatdet_set(the_chip,
				the_chip->warm_bat_voltage
				- the_chip->resume_voltage_delta);
		} else {
			pm_chg_vbatdet_set(the_chip,
				the_chip->max_voltage_mv
				- the_chip->resume_voltage_delta);
		}

		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;

#if defined(PANTECH_CHARGER_TIME_LIMITATION)
		if(chip->bms_notify.is_battery_full)
			chip->err_charge_done = true;
#endif

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
                if(is_dc_chg_plugged_in(chip) && chip->bms_notify.is_battery_full){
                    wireless_full_sign(chip, true);
                }
#endif

		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
		wake_unlock(&chip->eoc_wake_lock);
	} else {
		adjust_vdd_max_for_fastchg(chip, vbat_batt_terminal_uv);
		pr_debug("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
}

static void btm_configure_work(struct work_struct *work)
{
	int rc;

	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc=%d", rc);
}

DECLARE_WORK(btm_config_work, btm_configure_work);

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
#if defined(CONFIG_PANTECH_CHARGER)
	unsigned int chg_current = chip->cable_ibat_max;            
#else
	unsigned int chg_current = chip->max_bat_chg_current;
#endif   

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

	#if !defined(PANTECH_CHARGER_THERMAL_MITIGATION_DISABLE)
	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);
	#endif

	pm_chg_ibatmax_set(the_chip, chg_current);
}

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
static void btm_isr_lock(struct pm8921_chg_chip *chip)
{
        static bool is_lock = false;
	int dc_present;
	int usb_present;

	usb_present = is_usb_chg_plugged_in(chip);
	dc_present = is_dc_chg_plugged_in(chip);
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
        if((usb_present || dc_present || the_smb347->is_dth_detect) && (chip->is_bat_cool || chip->is_bat_warm)){
#else
        if((usb_present || dc_present) && (chip->is_bat_cool || chip->is_bat_warm)){
#endif
            if(!is_lock)
                wake_lock(&chip->btm_isr_lock);
            is_lock = true;
        }
        else{
            if(is_lock)
                wake_unlock(&chip->btm_isr_lock);
            is_lock = false;
        }
}

static void btm_init(struct pm8921_chg_chip *chip)
{
	int rc, current_temp;
	struct pm8xxx_adc_chan_result result;
        int warm_temp_thr, cool_temp_thr;

        cool_temp_thr = chip->cool_temp_dc + 50;
        warm_temp_thr = chip->warm_temp_dc - 50;

        // check temp when insering the charger
        if(chip->is_bat_cool ||chip->is_bat_warm){
	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);    
	}
            else{
	current_temp = (int)(result.physical);

            if(current_temp >= cool_temp_thr && current_temp <= warm_temp_thr){
                    chip->is_bat_cool = 0;
                    chip->is_bat_warm = 0;
                
    		btm_config.low_thr_temp = chip->cool_temp_dc;
                btm_config.high_thr_temp = chip->warm_temp_dc;

#if defined(CONFIG_PANTECH_SMB_CHARGER)
		the_smb347->is_bat_cool = 0;
		the_smb347->is_bat_warm = 0;
		the_smb347->enable_aicl_off_on = true;

		if(TA_CABLE == the_chip->cable_type)
			smb347_set_cable_imax_ta();
		else if(STANDARD_CABLE == the_chip->cable_type)
			smb347_set_cable_imax_usb();
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
		else if(DTH_CABLE == the_chip->cable_type)
			smb347_set_cable_imax_dth();
#endif
#endif
            
        		set_appropriate_battery_current(chip);
    		pm_chg_vddmax_set(chip, chip->max_voltage_mv);
    		pm_chg_vbatdet_set(chip,
    			chip->max_voltage_mv
    			- chip->resume_voltage_delta);

                    schedule_work(&btm_config_work);
            }
        }
}
}

static void btm_isr_lock_worker(struct work_struct *work)
{
	int dc_present;
	int usb_present;
    
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip,
				btm_isr_lock_work);

	usb_present = is_usb_chg_plugged_in(chip);
	dc_present = is_dc_chg_plugged_in(chip);

        // wakelock (charging & warm & cool)
        btm_isr_lock(chip);

        // polling temp when the charger is connected
        btm_init(chip);
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH) 
        if(usb_present || dc_present || the_smb347->is_dth_detect){
#else
        if(usb_present || dc_present){
#endif
            if(!delayed_work_pending(&chip->btm_isr_lock_work))
                schedule_delayed_work(&chip->btm_isr_lock_work, round_jiffies_relative(usecs_to_jiffies(BTM_ISR_LOCK_WAIT_PERIOD_US)));
        }
        else
            chip->btm_isr_lock_work_is = false; 
}
#endif
#if defined(PANTECH_CHARGER_SMB_EOC)
static void smb_eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, smb_eoc_work);
	int battery_capacity;
	u8 smb_reg;
	static bool flag = true;

	smb347_read_reg(0x3F, &smb_reg);
	battery_capacity = get_prop_batt_capacity(chip);

	pr_err("smb battery capacity=%d,0x3F=%.2X\n", battery_capacity, smb_reg);
	
	if (battery_capacity == 100)
	{
		power_supply_changed(&chip->batt_psy);
	}
	else
	{
		if(!delayed_work_pending(&chip->smb_eoc_work))
		{
			if(chip->smb_charging_cable_type != NO_CABLE)
			{
				if((smb347_is_trickle() == false) && (chip->smb_charging_cable_type == TA_CABLE) && (flag == true))
				{
					set_cable_imax(chip, chip->smb_charging_cable_type);	
					flag = false;
					pr_err("no trickle setting.\n");
				}
				pr_err("smb_eoc_worker schedule, cable_type:%d\n",chip->smb_charging_cable_type);
				schedule_delayed_work(&chip->smb_eoc_work, round_jiffies_relative(msecs_to_jiffies(SMB_EOC_CHECK_PERIOD_MS)));
			}
			else
			{
				flag = true;
				pr_err("smb_eoc_worker end, flag reset.\n");
			}
		}
	}
}
#endif
#if defined(CONFIG_PANTECH_SMB_CHARGER)
static void smb_wake_unlock_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, smb_wake_unlock_work);

#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	if(!is_usb_chg_plugged_in(the_chip) && !gpio_get_value(PM8921_GPIO_PM_TO_SYS(35)))
#else
	if(!is_usb_chg_plugged_in(the_chip))
#endif
	{
		if (chip->smb_wake_is == true)
		{
			pr_err("wak unlock!!!!!!!!!!!!!!!!!!!!!!\n");
			chip->smb_wake_is = false;
			wake_unlock(&chip->eoc_wake_lock);
		}
	}
}
#endif

#define TEMP_HYSTERISIS_DEGC 2
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter) {
		btm_config.low_thr_temp =
			the_chip->cool_temp_dc + TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.low_thr_temp = the_chip->cool_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
#if defined(CONFIG_PANTECH_SMB_CHARGER)
		the_smb347->enable_aicl_off_on = true;
#endif
	}
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	the_smb347->is_bat_cool = enter;
	the_smb347->bat_temp_ok = pm_chg_get_rt_status(the_chip, BAT_TEMP_OK_IRQ);

	pr_err("the_smb347->cable_type : %d\n",the_smb347->cable_type);

	if(TA_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_ta();
	else if(STANDARD_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_usb();
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	else if(DTH_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_dth();
#endif
#endif
	schedule_work(&btm_config_work);

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
        btm_isr_lock(the_chip);
#endif
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter) {
		btm_config.high_thr_temp =
			the_chip->warm_temp_dc - TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.high_thr_temp = the_chip->warm_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
#if defined(CONFIG_PANTECH_SMB_CHARGER)
		the_smb347->enable_aicl_off_on = true;
#endif
	}
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	the_smb347->is_bat_warm = enter;
	the_smb347->bat_temp_ok = pm_chg_get_rt_status(the_chip, BAT_TEMP_OK_IRQ);

	pr_err("the_smb347->cable_type : %d\n",the_smb347->cable_type);

	if(TA_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_ta();
	else if(STANDARD_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_usb();
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	else if(DTH_CABLE == the_chip->cable_type)
		smb347_set_cable_imax_dth();
#endif
#endif
	schedule_work(&btm_config_work);
#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
        btm_isr_lock(the_chip);
#endif
}

static int configure_btm(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->warm_temp_dc != INT_MIN)
		btm_config.btm_warm_fn = battery_warm;
	else
		btm_config.btm_warm_fn = NULL;

	if (chip->cool_temp_dc != INT_MIN)
		btm_config.btm_cool_fn = battery_cool;
	else
		btm_config.btm_cool_fn = NULL;

	btm_config.low_thr_temp = chip->cool_temp_dc;
	btm_config.high_thr_temp = chip->warm_temp_dc;
	btm_config.interval = chip->temp_check_period;
	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc = %d\n", rc);
	rc = pm8xxx_adc_btm_start();
	if (rc)
		pr_err("failed to start btm rc = %d\n", rc);

	return rc;
}

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}

	set_appropriate_battery_current(chip);
	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
#if defined(CONFIG_PANTECH_CHARGER)
			pm8921_charger_vbus_draw(usb_max_current, chg_usb_type);
#else
			pm8921_charger_vbus_draw(usb_max_current);
#endif        
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	unsigned long flags;
	int fsm_state;

	chip->dc_present = !!is_dc_chg_plugged_in(chip);
	chip->usb_present = !!is_usb_chg_plugged_in(chip);
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	the_smb347->is_dth_detect = gpio_get_value(PM8921_GPIO_PM_TO_SYS(35));
#endif

	notify_usb_of_the_plugin_event(chip->usb_present);
	if (chip->usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	else if (chip->dc_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	}
#endif   

#if defined(CONFIG_PANTECH_CHARGER)
#if defined(CONFIG_PANTECH_SMB_CHARGER_DTH)
	if (chip->usb_present ||chip->dc_present || the_smb347->is_dth_detect) {
#else
	if (chip->usb_present ||chip->dc_present) {
#endif

		if (chip->usb_present)
			chg_usb_type = USB_SDP_CHARGER;

		queue_work(chip->cable_update_wq, &chip->update_cable_work);

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
		chip->cable_usb_update_delay = true;
		schedule_delayed_work(&chip->update_cable_work_delay,
			round_jiffies_relative(msecs_to_jiffies(chip->update_cable_time_delay)));
#endif
	}
#endif

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
#if !defined(CONFIG_PANTECH_CHARGER)
	pm8921_chg_enable_irq(chip, USBIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_UV_IRQ);
#endif
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

	spin_lock_irqsave(&vbus_lock, flags);
	if (usb_chg_current) {
		/* reissue a vbus draw call */
		__pm8921_charger_vbus_draw(usb_chg_current);
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	}
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        else if(is_dc_chg_plugged_in(chip)){
            handle_start_ext_chg(chip);
        }    
#endif   
	spin_unlock_irqrestore(&vbus_lock, flags);

	fsm_state = pm_chg_get_fsm_state(chip);
#if defined(CONFIG_PANTECH_CHARGER)
	spin_lock_irqsave(&vbus_lock, flags);
	if (fsm_state == FSM_STATE_FAST_CHG_7)
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	spin_unlock_irqrestore(&vbus_lock, flags);
#endif
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
#if defined(CONFIG_PANTECH_BMS_UPDATE)
                chip->bms_notify.batt_notifier_init = true;
                schedule_work(&(chip->bms_notify.work));
#else
		pm8921_bms_charging_began();
#endif
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);

	/* Determine which USB trim column to use */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		chip->usb_trim_table = usb_trim_8917_table;
	else if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8038)
		chip->usb_trim_table = usb_trim_8038_table;
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
#ifdef CONFIG_PANTECH_MHL_CABLE_DETECT
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING|IRQF_SHARED,
						usbin_valid_irq_handler),
#else	
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
#endif						
#if !defined(CONFIG_PANTECH_CHARGER)
	CHG_IRQ(USBIN_OV_IRQ, IRQF_TRIGGER_RISING, usbin_ov_irq_handler),
#endif
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
#if !defined(CONFIG_PANTECH_CHARGER)
	CHG_IRQ(USBIN_UV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							usbin_uv_irq_handler),
#endif						
	CHG_IRQ(VBAT_OV_IRQ, IRQF_TRIGGER_RISING, vbat_ov_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

static void pm8921_chg_force_19p2mhz_clk(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD5;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	udelay(183);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
	udelay(32);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

static void pm8921_chg_set_hw_clk_switching(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define BOOT_TIMER_EN_BIT	BIT(1)
#define BOOT_DONE_MASK		(BOOT_DONE_BIT | BOOT_TIMER_EN_BIT)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define SAFE_CURRENT_MA		1500
#define PM_SUB_REV		0x001
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	int rc;
	int vdd_safe;
	u8 subrev;

	/* forcing 19p2mhz before accessing any charger registers */
	pm8921_chg_force_19p2mhz_clk(chip);

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_MASK, BOOT_DONE_MASK);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_ibatsafe_set(chip, SAFE_CURRENT_MA);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}

#if defined(CONFIG_PANTECH_CHARGER)
	rc = pm_chg_ibatmax_set(chip, DEFAULT_IBAT_IMAX);
#else
	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
#endif

	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

#if defined(CONFIG_PANTECH_CHARGER)
#if defined (CONFIG_MACH_MSM8960_EF45K) || defined (CONFIG_MACH_MSM8960_EF47S) || defined (CONFIG_MACH_MSM8960_EF46L)
    if(smem_id_vendor1_ptr->power_on_mode == CHG_PM_OFFLINE_NORMAL_BOOT_MODE)
        rc = pm_chg_iusbmax_set(chip, 7);    
    else
#endif 
        rc = pm_chg_iusbmax_set(chip, 2);
   
#else
	rc = pm_chg_iusbmax_set(chip, 0);
#endif
	if (rc) {
		pr_err("Failed to set usb max to %d rc=%d\n", 0, rc);
		return rc;
	}

	if (chip->safety_time != 0) {
#if defined(CONFIG_PANTECH_CHARGER)
		if (chip->safety_time > PM8921_CHG_TCHG_MAX) {
			safety_time_remain = chip->safety_time - PM8921_CHG_TCHG_MAX;
			rc = pm_chg_tchg_max_set(chip, PM8921_CHG_TCHG_MAX);
		} else {
			safety_time_remain = 0;
		rc = pm_chg_tchg_max_set(chip, chip->safety_time);
		}
#else
		rc = pm_chg_tchg_max_set(chip, chip->safety_time);
#endif
		if (rc) {
			pr_err("Failed to set max time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CLOCK_CTRL, 0x15);
	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0) {
		rc = pm8xxx_readb(chip->dev->parent, PM_SUB_REV, &subrev);
		if (rc) {
			pr_err("read failed: addr=%03X, rc=%d\n",
				PM_SUB_REV, rc);
			return rc;
		}
		/* Check if die 3.0.1 is present */
		if (subrev == 0x1)
			pm8xxx_writeb(chip->dev->parent,
				CHG_BUCK_CTRL_TEST3, 0xA4);
		else
			pm8xxx_writeb(chip->dev->parent,
				CHG_BUCK_CTRL_TEST3, 0xAC);
	}

	/* Enable isub_fine resolution AICL for PM8917 */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917) {
		chip->iusb_fine_res = true;
		if (chip->uvd_voltage_mv)
			rc = pm_chg_uvd_threshold_set(chip,
					chip->uvd_voltage_mv);
			if (rc) {
				pr_err("Failed to set UVD threshold %drc=%d\n",
						chip->uvd_voltage_mv, rc);
			return rc;
		}
	}

	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);

	rc = pm_chg_charge_dis(chip, charging_disabled);
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	rc = gpio_request(W_CHG_FULL, "w_chg_full");
	if (rc) {
		return rc;
	}
	wireless_full_sign(chip, false);

	rc = gpio_request(USB_CHG_DET, "usb_chg_det");
	if (rc) {
		return rc;
	}
	gpio_direction_output(USB_CHG_DET, 0);
#endif

	return 0;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm8xxx_writeb(the_chip->dev->parent, addr, temp);
	if (ret) {
		pr_err("pm8xxx_writeb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

#if defined(CONFIG_PM)
#if defined(CONFIG_PANTECH_BMS_UPDATE)
static int pm8921_charger_sleep_time(int soc)
{
	int time = 0;
	int enable;
    
	if (soc > 5) {
		time = (soc - 5) * SLEEP_HALF_HOUR;		
#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
		if (the_chip->wireless_recharge_loop)
			time = SLEEP_FIVE_MINUTE; 
#endif
	} else
		time = SLEEP_THREE_MINUTE;
        
#if defined(CONFIG_PANTECH_BMS_TEST)
	enable = atomic_read(&bms_input_flag);

	if (enable)
		time = SLEEP_THREE_MINUTE;
#endif

	return time;
}
#endif

static int pm8921_charger_suspend_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#if defined(CONFIG_PANTECH_CHARGER)
	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, VREF_BATT_THERM_FORCE_ON);
#else
	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
#endif
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);
	pm8921_chg_set_hw_clk_switching(chip);
	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	pm8921_chg_force_19p2mhz_clk(chip);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
	return 0;
}

static int pm8921_charger_resume(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_configure(&btm_config);
		if (rc)
			pr_err("couldn't reconfigure btm rc=%d\n", rc);

		rc = pm8xxx_adc_btm_start();
		if (rc)
			pr_err("couldn't restart btm rc=%d\n", rc);
	}
	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}

#if defined(CONFIG_PANTECH_BMS_UPDATE)
	schedule_delayed_work(&chip->update_heartbeat_work,
		round_jiffies_relative(msecs_to_jiffies(0)));
#endif

	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	int rc;
#if defined(CONFIG_PANTECH_BMS_UPDATE)
	int time;
#endif
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_end();
		if (rc)
			pr_err("Failed to disable BTM on suspend rc=%d\n", rc);
	}

	if (is_usb_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
	else if (is_dc_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}
#endif    

#if defined(CONFIG_PANTECH_BMS_UPDATE)
#if defined(PANTECH_CHARGER_BMS_WAKELOCK_FIX)
        suspend_status = true;
#endif

        cancel_delayed_work(&chip->update_heartbeat_work);	
#if defined(CONFIG_PANTECH_SMB_CHARGER)
        cancel_delayed_work(&chip->smb_wake_unlock_work);	
#endif

#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	time=pm8921_charger_sleep_time(pm8921_bms_get_percent(max17058_uses));
#else
	time=pm8921_charger_sleep_time(pm8921_bms_get_percent());
#endif
	msm_pm_set_max_sleep_time((int64_t)((int64_t) time * NSEC_PER_SEC)); 
#endif
    
	return 0;
}
#else
#define pm8921_charger_resume NULL
#define pm8921_charger_suspend NULL
#define pm8921_charger_suspend_noirq NULL
#define pm8921_charger_resume_noirq NULL
#endif // #if defined(CONFIG_PM)

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void pm8921_charger_pm_early_suspend(struct early_suspend *h)
{
	the_chip->is_early_suspend = true;
}

static void pm8921_charger_pm_late_resume(struct early_suspend *h)
{
	the_chip->is_early_suspend = false;
	power_supply_changed(&the_chip->batt_psy);
}
#else
#define pm8921_charger_pm_early_suspend NULL
#define pm8921_charger_pm_late_resume NULL
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
static char *str_cable_type[] = {
		"NO_CABLE", "STANDARD_CABLE", "FACTORY_CABLE", "TA_CABLE", "WIRELESS_CABLE", "DTH_CABLE",
		"UNKNOWN_CABLE", "INVALID_CABLE",
};

static int proc_debug_pm_chg_get_fsm_state(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	int fsm_state;

	if (!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	fsm_state = pm_chg_get_fsm_state(the_chip);
	*eof = 1;

	return sprintf(page, "%d\n", fsm_state);
}

static int proc_debug_pm_chg_get_I_USBMax(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	u8 usbin_max_reg;
	int usbin_max = 0;
	*eof = 1;  
#else
	*eof = 1;  
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	if(smb347_is_charging() != true)
		return sprintf(page, "%d\n", usbin_max);

	smb347_read_reg(INPUT_CURRENT_LIMIT_REG_0x01, &usbin_max_reg);
	
	if(false == the_smb347->is_dth_detect)
		usbin_max_reg &= USBIN_INPUT_CURRENT_LIMIT_MASK;
	else
	{
		usbin_max_reg &= DC_INPUT_CURRENT_LIMIT_MASK;
		usbin_max_reg = usbin_max_reg >> 4;
	}

	switch(usbin_max_reg)
	{
		case 0:
			usbin_max = 300;
			break;
		case 1:
			usbin_max = 500;
			break;
		case 2:
			usbin_max = 700;
			break;
		case 3:
			usbin_max = 900;
			break;
		case 4:
			usbin_max = 1200;
			break;
		case 5:
			usbin_max = 1500;
			break;
		case 6:
			usbin_max = 1800;
			break;
		case 7:
			usbin_max = 2000;
			break;
		case 8:
			usbin_max = 2200;
			break;
		case 9:
			usbin_max = 2500;
			break;
		default:
			usbin_max = 0;
			break;
	}
	return sprintf(page, "%d\n", usbin_max);
#else
	return sprintf(page, "%d\n", iusbmax_set_cur);
#endif
}

static int proc_debug_pm_chg_get_I_BattMax(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	int fl_v;
	int i_batt_max = 0;
#else
	/*u8 chg_stat;*/
	*eof = 1;  
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	//return sprintf(page, "%u\n", the_smb347->is_bat_warm);
	smb347_get_fast_chg_current(&fl_v);
	switch(fl_v)
	{
		case 0:
			i_batt_max = 700;
			break;
		case 1:
			i_batt_max = 900;
			break;
		case 2:
			i_batt_max = 1200;
			break;
		case 3:
			i_batt_max = 1500;
			break;
		case 4:
			i_batt_max = 1800;
			break;
		case 5:
			i_batt_max = 2000;
			break;
		case 6:
			i_batt_max = 2200;
			break;
		case 7:
			i_batt_max = 2500;
			break;
		default:
			i_batt_max = 0;
			break;
	}

	return sprintf(page, "%d\n", i_batt_max);
#else
	return sprintf(page, "%u\n", ibatmax_set_cur);
#endif

}

static int proc_debug_pm_chg_get_I_BattCurrent(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
#if defined(CONFIG_PANTECH_SMB_CHARGER)
	int is_charging;

	if(smb347_is_charging() == true)
		is_charging = 1;
	else
		is_charging = 0;

	*eof = 1;  


	return sprintf(page, "%d\n", is_charging);
#else
	int ichg_meas_ma;

	if (!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	ichg_meas_ma = (get_prop_batt_current(the_chip)) / 1000;
	*eof = 1;  

	
	return sprintf(page, "%d\n", ichg_meas_ma);
#endif
}

static int proc_debug_pm_chg_get_BattID(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	int64_t rc;

	if (!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	rc = read_battery_id(the_chip);
	*eof = 1;  

	return sprintf(page, "%lld\n", rc);
}

static int proc_debug_pm_chg_get_CableID(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	if (!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	*eof = 1;  

	return sprintf(page, "%lld\n", the_chip->cable_adc);
}

static int proc_debug_pm_chg_get_CableType(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	if (!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	*eof = 1;  

	return sprintf(page, "%s\n", str_cable_type[the_chip->cable_type]);
}

static int proc_debug_pm_chg_get_pm_chg_test(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	if(!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	*eof = 1;  
	return sprintf(page, "%d\n", the_chip->pm_chg_test);
}

#if defined(CONFIG_PANTECH_SMB_CHARGER)
#define AICL_RESULTS_MASK SMB347_MASK(4, 0)
static int proc_debug_pm_chg_get_I_AICL_result(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	u8 reg = 0;
	int i_aicl_curr = 0;
	
	if(!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	smb347_read_reg(STATUS_E_REG_0x3F, &reg);

	switch(reg & AICL_RESULTS_MASK)
	{
		case 0:
			i_aicl_curr = 300;
			break;
		case 1:
			i_aicl_curr = 500;
			break;
		case 2:
			i_aicl_curr = 700;
			break;
		case 3:
			i_aicl_curr = 900;
			break;
		case 4:
			i_aicl_curr = 1200;
			break;
		case 5:
			i_aicl_curr = 1500;
			break;
		case 6:
			i_aicl_curr = 1800;
			break;
		case 7:
			i_aicl_curr = 2000;
			break;
		case 8:
			i_aicl_curr = 2200;
			break;
		case 9: case 10: case 11: case 12: case 13: case 14: case 15:
			i_aicl_curr = 2500;
			break;
		default:
			i_aicl_curr = 0;
			break;
	}
	
	if(reg == 0)
		i_aicl_curr = 0;

	*eof = 1;  
	return sprintf(page, "%d\n", i_aicl_curr);
}

static int proc_debug_pm_chg_get_AICL_status(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	u8 reg;
	bool status;

	if(!the_chip) {
		pr_err("the_chip is NULL\n");
		return 0;
	}

	smb347_read_reg(STATUS_E_REG_0x3F, &reg);
	status = test_bit(4,(void *)&reg);

	*eof = 1;  
	return sprintf(page, "%d\n", status);
}
#endif
#endif

#if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST) || defined(PANTECH_CHARGER_MONITOR_TEST)
static int pm8921_charger_test_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long pm8921_charger_test_misc_ioctl(struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc;
	uint32_t n;
	pr_err("cmd = [%x]\n",cmd);

	switch (cmd) {
#if defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST)
	case PM8921_CHARER_TEST_SET_CHARING_CTL:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc) {
			if (n)
			{
				charging_disabled = 1;
			}
			else
			{
				charging_disabled = 0;
			}

			if (the_chip) {
				pm_chg_auto_enable(the_chip, !charging_disabled);
				pm_chg_charge_dis(the_chip, charging_disabled);
			}

			pr_err("SET_CHARING_CTL charging_disabled[%d]\n",charging_disabled);
		}

		break;
		
	case PM8921_CHARER_TEST_GET_CHARING_CTL:
		if (copy_to_user((void *)arg, &charging_disabled, sizeof(int)))
			rc = -EINVAL;
		else
			rc = 0;

		break;
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
	case PM8921_CHARGER_TEST_SET_PM_CHG_TEST:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc) {
			if (n){
				the_chip->pm_chg_test = true;
#ifdef CONFIG_PANTECH_SMB_CHARGER
				smb347_charging_enable(0);
				smb347_is_charging();
#endif
			}
			else{
				the_chip->pm_chg_test = false;
#ifdef CONFIG_PANTECH_SMB_CHARGER
				smb347_charging_enable(1);
				smb347_is_charging();
#endif
			}
			
			
			pr_err("SET_PM_CHG_TEST pm_chg_test [%d]\n",the_chip->pm_chg_test);
		}	
		break;
#endif
		
	default:
		rc = -EINVAL;
	}

	return rc;
}


static int pm8921_charger_battery_test_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations pm8921_charger_test_dev_fops = {
	.owner = THIS_MODULE,
	.open = pm8921_charger_test_misc_open,
	.unlocked_ioctl	= pm8921_charger_test_misc_ioctl,
	.release	= pm8921_charger_battery_test_misc_release,
};

struct miscdevice pm8921_charger_test_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "pm8921-charger",
	.fops	= &pm8921_charger_test_dev_fops,
};

int pm8921_charger_battery_charging_test_init(void)
{
	return misc_register(&pm8921_charger_test_misc_device);
}
#endif //defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST) || defined(PANTECH_CHARGER_MONITOR_TEST)

#if defined(PANTECH_CHARGER_MONITOR_TEST)
void pm8921_charger_test_charger_monitor_init(struct pm8921_chg_chip *chip)
{
	struct proc_dir_entry *ent;

	if (!chip)
		return;

	chip->pm_chg_test = false;
	chip->cable_adc = 0;

	pm8921_charger_dir = proc_mkdir(PM8921_CHARGER_DEV_NAME, NULL);
	if (!pm8921_charger_dir) {
		pr_err("Unable to create /proc/%s directory\n",PM8921_CHARGER_DEV_NAME);
		return;
	}

	ent = create_proc_entry("fsm_state", 0, pm8921_charger_dir);

	if (!ent) {
		pr_err("Unable to create /proc/fsm_state entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_fsm_state;

	ent = create_proc_entry("cable_type", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/cable_type entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_CableType;

	ent = create_proc_entry("cable_id", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/cable_id entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_CableID;

	ent = create_proc_entry("batt_id", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/batt_id entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_BattID;

	ent = create_proc_entry("i_usbmax", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/i_usbmax entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_I_USBMax;

	ent = create_proc_entry("i_battmax", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/i_battmax entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_I_BattMax;

	ent = create_proc_entry("i_battcurr", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/i_battcurr entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_I_BattCurrent;
#if defined(CONFIG_PANTECH_SMB_CHARGER) /* JB BUILD FIXED */
	ent = create_proc_entry("i_aicl_result", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/i_aicl_result entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_I_AICL_result;

	ent = create_proc_entry("aicl_status", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/aicl_status entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_AICL_status;
#endif
	ent = create_proc_entry("pm_chg_test", 0, pm8921_charger_dir);
	if (!ent) {
		pr_err("Unable to create /proc/pm_chg_test entry\n");
		return;
	}
	ent->read_proc = proc_debug_pm_chg_get_pm_chg_test;      
}
#endif

static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;

	#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	int i,j;
	#endif /* PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST */

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->safety_time = pdata->safety_time;
	chip->ttrkl_time = pdata->ttrkl_time;
	chip->update_time = pdata->update_time;
	chip->max_voltage_mv = pdata->max_voltage;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->uvd_voltage_mv = pdata->uvd_thresh_voltage;
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
	chip->resume_charge_percent = pdata->resume_charge_percent;
	chip->term_current = pdata->term_current;
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;

	chip->temp_check_period = pdata->temp_check_period;
	chip->dc_unplug_check = pdata->dc_unplug_check;
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
	/* Assign to corresponding module parameter */
	usb_max_current = pdata->usb_max_current;
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
	chip->keep_btm_on_suspend = pdata->keep_btm_on_suspend;
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
	chip->vin_min = pdata->vin_min;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
	chip->rconn_mohm = pdata->rconn_mohm;
	chip->led_src_config = pdata->led_src_config;
	chip->has_dc_supply = pdata->has_dc_supply;

#if defined(CONFIG_PANTECH_CHARGER)
	chip->err_type = ERR_NONE;
	chip->err_fsm_type = FSM_STATE_OFF_0;
	chip->err_charge_done = false;
#endif

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
        chip->btm_isr_lock_work_is = false; 
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER)
	chip->smb_wake_is = false;
#endif

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
	chip->update_cable_time_delay = 90000; // 90sec     
	chip->cable_usb_update_delay = false;
	chip->cable_usb_update_delay_decision = false;
#endif

#if defined(PANTECH_BATT_CHARGE_DONE_IN_WARM_COOL)
	chip->batt_charge_done_warm_cool = 0;
#endif

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        chip->ext_charging = false;
        chip->wireless_recharge_loop = false;
#endif

#if defined(CONFIG_PANTECH_PMIC)
	chg_pm_read_proc_reset_info();
	pr_err("smem_id_vendor1_ptr->power_on_mode 0x%x\n", smem_id_vendor1_ptr->power_on_mode);

#if defined(PANTECH_CHARGER_FACTORY_BOOT)
	if ((smem_id_vendor1_ptr->factory_cable_adc >= (FACT_CABLE_MIN/1000)) &&
		(smem_id_vendor1_ptr->factory_cable_adc <= (FACT_CABLE_MAX/1000))) {
		pr_err("Factory Cable Boot\n");
		chip->cable_type = FACTORY_CABLE;
	}
#endif

#if defined(FEATURE_PANTECH_BATTERY_DUMMY)
	if (smem_id_vendor1_ptr->battery_id == OEM_PM_BATTERY_TYPE_DUMMY) {
		is_dummy_battery = true;
		pr_err("the dummy battery is insered !!!\n");
	}
#endif
#endif // #if defined(CONFIG_PANTECH_PMIC)

#ifdef CONFIG_PANTECH_SMB_CHARGER
	if (i2c_add_driver(&smb347_i2c_driver))
		pr_err("Can't add smb347 i2c drv\n");
#endif
#if defined(CONFIG_PANTECH_PMIC_MAX17058)
	#if defined(T_EF44S)
	#if(BOARD_VER >= WS20)
	max17058_uses = 1;
	#else
	max17058_uses = 0;
	#endif
	#elif defined(T_STARQ)
	if ((smem_id_vendor1_ptr->hw_rev == 6) || (smem_id_vendor1_ptr->hw_rev == 8))
		max17058_uses = 1;
	else
		max17058_uses = 0;
	#elif defined(T_OSCAR)
	if (smem_id_vendor1_ptr->hw_rev >= 5)
		max17058_uses = 1;
	else
		max17058_uses = 0;
	#elif defined(T_MAGNUS)
	max17058_uses = 1;
	#elif defined(T_VEGAPVW)
	if(smem_id_vendor1_ptr->hw_rev >= 5)
		max17058_uses = 1;
	#elif defined(T_SIRIUSLTE)
	max17058_uses = 1;
	#else
	max17058_uses = 0;
	#endif

	if (max17058_uses)
		if (i2c_add_driver(&max17058_i2c_driver))
			pr_err("Can't add max17058 i2c drv\n");
#endif // #if defined(CONFIG_PANTECH_PMIC_MAX17058)

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	for (i = 0; i <= POWER_SUPPLY_TYPE_USB ; i++)
		for(j=0; j <= POWER_SUPPLY_PROP_SERIAL_NUMBER ; j++)
			power_supply_write_item[i][j].is_forced_item = false;
#endif

#if defined(CONFIG_PANTECH_BMS_BATTERY_TYPE)
	charger_set_battery_data(chip);
#endif
	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

#if defined(CONFIG_PANTECH_CHARGER)
	chip->usb_psy.name = "usb",
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	chip->usb_psy.supplied_to = pm_power_supplied_to,
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->usb_psy.properties = pm_power_props,
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props),
	chip->usb_psy.get_property = pm_power_get_property,
	
	#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	chip->usb_psy.set_property = pm_batt_power_set_property,
	#endif

	chip->dc_psy.name = "ac",
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS,
	chip->dc_psy.supplied_to = pm_power_supplied_to,
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->dc_psy.properties = pm_power_props,
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props),
	chip->dc_psy.get_property = pm_power_get_property,
	
	#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	chip->dc_psy.set_property = pm_batt_power_set_property,
	#endif
#else
	chip->usb_psy.name = "usb",
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	chip->usb_psy.supplied_to = pm_power_supplied_to,
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->usb_psy.properties = pm_power_props_usb,
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb),
	chip->usb_psy.get_property = pm_power_get_property_usb,
	chip->usb_psy.set_property = pm_power_set_property_usb,

	chip->dc_psy.name = "pm8921-dc",
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS,
	chip->dc_psy.supplied_to = pm_power_supplied_to,
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->dc_psy.properties = pm_power_props_mains,
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains),
	chip->dc_psy.get_property = pm_power_get_property_mains,
#endif

	chip->batt_psy.name = "battery",
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY,
	chip->batt_psy.properties = msm_batt_power_props,
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props),
	chip->batt_psy.get_property = pm_batt_power_get_property,

#if defined(PANTECH_POWER_SUPPLY_SYSFS_WRITE_TEST)
	chip->batt_psy.set_property = pm_batt_power_set_property,
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	chip->batt_psy.external_power_changed = NULL,
#else
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed,
#endif

#if defined(CONFIG_PANTECH_CHARGER)
	the_chip = chip;
#endif

	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}

	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto unregister_usb;
	}

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}

	platform_set_drvdata(pdev, chip);

#if !defined(CONFIG_PANTECH_CHARGER)
	the_chip = chip;
#endif

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
	wake_lock_init(&chip->btm_isr_lock, WAKE_LOCK_SUSPEND, "pm8921_btm_isr_lock");
#endif
	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);

#if defined(PANTECH_CHARGER_BTM_ISR_FIX)
	INIT_DELAYED_WORK(&chip->btm_isr_lock_work, btm_isr_lock_worker);
#endif
#if defined(PANTECH_CHARGER_SMB_EOC)
	INIT_DELAYED_WORK(&chip->smb_eoc_work, smb_eoc_worker);
	INIT_DELAYED_WORK(&chip->smb_wake_unlock_work, smb_wake_unlock_worker);
       chip->smb_charging_cable_type = NO_CABLE;
#endif
	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_batt;
	}

	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
#if !defined(CONFIG_PANTECH_CHARGER)
	enable_irq_wake(chip->pmic_chg_irq[USBIN_OV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[USBIN_UV_IRQ]);
#endif
	enable_irq_wake(chip->pmic_chg_irq[BAT_TEMP_OK_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);
#if defined(CONFIG_PANTECH_CHARGER)
	enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_OV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_UV_IRQ]);
#endif    
	/*
	 * if both the cool_temp_dc and warm_temp_dc are invalid device doesnt
	 * care for jeita compliance
	 */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)) {
		rc = configure_btm(chip);
		if (rc) {
			pr_err("couldn't register with btm rc=%d\n", rc);
			goto free_irq;
		}
	}

	create_debugfs_entries(chip);

#if defined(CONFIG_PANTECH_BMS_TEST)
	atomic_set(&bms_input_flag, 0);
	atomic_set(&bms_cutoff_flag, 1);

	bms_input_attr_dev = platform_device_register_simple("bms_app_attr", -1, NULL, 0);

	if (!bms_input_attr_dev) {
		pr_debug("BMS: Unable to register platform_device_register_simple device\n");
		rc = -ENXIO;
		goto free_irq;
	}

	rc = sysfs_create_group(&bms_input_attr_dev->dev.kobj, &bms_input_attr_group);	
	if (rc) {
		pr_debug("[BMS] failed: sysfs_create_group  [ERROR]\n");	
		goto unregister_input_attr;
	} 

	bms_input_dev = input_allocate_device();

	if (!bms_input_dev) {
		pr_debug("BMS: Unable to input_allocate_device \n");
		rc = -ENXIO;
		goto remove_sysfs;
	}

	set_bit(EV_REL, bms_input_dev->evbit);

	input_set_capability(bms_input_dev, EV_REL, REL_RX);	// SOC
	input_set_capability(bms_input_dev, EV_REL, REL_RY); 	// Volt
	input_set_capability(bms_input_dev, EV_REL, REL_RZ);    // TEMP

	bms_input_dev->name="bms_app";

	rc = input_register_device(bms_input_dev);
	if (rc) {
		pr_debug("BMS: Unable to register input_register_device device\n");
		goto free_input_device;
	}

	// set init values
	bms_init_set(chip);
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
	chip->bms_notify.batt_notifier_init = false;
	chip->bms_notify.batt_notifier_state_change = false;
	chip->bms_notify.batt_notifier_periodic_update = false;
	wake_lock_init(&chip->work_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_charger");
#endif

	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

#if defined(CONFIG_PANTECH_CHARGER)
	INIT_WORK(&chip->update_cable_work, update_cable);
	chip->cable_update_wq = create_singlethread_workqueue("cable_update");
	if (!chip->cable_update_wq)
		return -ENOMEM;
#endif

#if defined(PANTECH_CHARGER_INFO_ABNORMAL)
	INIT_DELAYED_WORK(&chip->update_cable_work_delay,
						update_cable_delay);
#endif

	/* determine what state the charger is in */
	determine_initial_state(chip);

#if defined(CONFIG_PANTECH_CHARGER)
	if (pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ)) {
		if (smem_id_vendor1_ptr->power_on_mode != CHG_PM_ONLINE_CABLE_IN_BOOT_MODE)
			if (chip->cable_type != FACTORY_CABLE)
				//arch_reset(0xC9, 0);
				msm_restart(0xC9, 0);
	}
#endif

#if defined(PANTECH_CHARGER_MONITOR_TEST)
	pm8921_charger_test_charger_monitor_init(chip);
#endif
	
#if defined(PANTECH_CHARGER_MONITOR_TEST) || defined(PANTECH_BATTERY_CHARING_DISCHARING_TEST)
	pm8921_charger_battery_charging_test_init();
#endif

	if (chip->update_time) {
		INIT_DELAYED_WORK(&chip->update_heartbeat_work,
							update_heartbeat);
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));
	}

#if defined(CONFIG_PANTECH_BMS_UPDATE)
#if defined(CONFIG_HAS_EARLYSUSPEND)
	chip->is_early_suspend = false;
	chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	chip->early_suspend.suspend = pm8921_charger_pm_early_suspend;
	chip->early_suspend.resume = pm8921_charger_pm_late_resume;
	register_early_suspend(&chip->early_suspend);
#endif    
#endif
   
	return 0;

#if defined(CONFIG_PANTECH_BMS_TEST)
free_input_device:
        input_free_device(bms_input_dev);
remove_sysfs:
        sysfs_remove_group(&bms_input_attr_dev->dev.kobj, &bms_input_attr_group);
unregister_input_attr:
        platform_device_unregister(bms_input_attr_dev);
#endif
free_irq:
	free_irqs(chip);
unregister_batt:
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

#if defined(CONFIG_PANTECH_BMS_TEST)
	input_unregister_device(bms_input_dev);
	input_free_device(bms_input_dev);
	sysfs_remove_group(&bms_input_attr_dev->dev.kobj, &bms_input_attr_group);
	platform_device_unregister(bms_input_attr_dev);
#endif

#if defined(CONFIG_PANTECH_BMS_UPDATE)
#if defined(CONFIG_HAS_EARLYSUSPEND)
	if (chip->early_suspend.suspend == pm8921_charger_pm_early_suspend)
		unregister_early_suspend(&chip->early_suspend);
#endif
#endif

#if defined(CONFIG_PANTECH_CHARGER_WIRELESS)
        gpio_set_value_cansleep(W_CHG_FULL, 0);
        gpio_set_value_cansleep(USB_CHG_DET, 0);
        gpio_free(W_CHG_FULL);
        gpio_free(USB_CHG_DET);
#endif

#if defined(CONFIG_PANTECH_SMB_CHARGER) && defined(CONFIG_MACH_MSM8960_SIRIUSLTE)
	gpio_free(SMB_CHG_SUSP);
#endif

	free_irqs(chip);
	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}

#if defined(CONFIG_PM)
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
#endif

static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
			.pm	= &pm8921_pm_ops,
#endif
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);
