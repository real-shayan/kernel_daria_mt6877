// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/regmap.h>


#ifndef CONFIG_MTK_CLASS
#define CONFIG_MTK_CLASS
#endif

#ifndef CONFIG_MTK_CHARGER_V4P19
#define CONFIG_MTK_CHARGER_V4P19
#endif

#ifdef CONFIG_MTK_CLASS
//#include "charger_class.h"
#include <mt-plat/v1/charger_class.h>
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_MTK_CHARGER_V4P19
#include "mtk_charger_intf.h"
#endif /*CONFIG_MTK_CHARGER_V4P19*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
#include "dvchg_class.h"
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

#define SC8571_DRV_VERSION              "1.0.1_G"

enum {
    SC8571_STANDALONG = 0,
    SC8571_MASTER,
    SC8571_SLAVE,
};

static const char* sc8571_psy_name[] = {
    [SC8571_STANDALONG] = "sc-cp-standalone",
    [SC8571_MASTER] = "sc-cp-master",
    [SC8571_SLAVE] = "sc-cp-slave",
};

static const char* sc8571_irq_name[] = {
    [SC8571_STANDALONG] = "sc8571-standalone-irq",
    [SC8571_MASTER] = "sc8571-master-irq",
    [SC8571_SLAVE] = "sc8571-slave-irq",
};

static int sc8571_mode_data[] = {
    [SC8571_STANDALONG] = SC8571_STANDALONG,
    [SC8571_MASTER] = SC8571_MASTER,
    [SC8571_SLAVE] = SC8571_SLAVE,
};

enum {
    ADC_IBUS,
    ADC_VBUS,
    ADC_VAC1,
    ADC_VAC2,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_TSBUS,
    ADC_TSBAT,
    ADC_TDIE,
    ADC_MAX_NUM,
}SC8571_ADC_CH;

static const u32 sc8571_adc_accuracy_tbl[ADC_MAX_NUM] = {
	150000,	/* IBUS */
	35000,	/* VBUS */
	35000,	/* VAC1 */
    35000,	/* VAC2 */
	20000,	/* VOUT */
	20000,	/* VBAT */
	200000,	/* IBAT */
    0,	/* TSBUS */
    0,	/* TSBAT */
	4,	/* TDIE */
};

static const int sc8571_adc_m[] = 
    {25, 625, 625, 625, 25, 25, 3125, 9766, 9766, 5};

static const int sc8571_adc_l[] = 
    {10, 100, 100, 100, 10, 10, 1000, 100000, 100000, 10};

enum sc8571_notify {
    SC8571_NOTIFY_OTHER = 0,
	SC8571_NOTIFY_IBUSUCPF,
	SC8571_NOTIFY_VBUSOVPALM,
	SC8571_NOTIFY_VBATOVPALM,
	SC8571_NOTIFY_IBUSOCP,
	SC8571_NOTIFY_VBUSOVP,
	SC8571_NOTIFY_IBATOCP,
	SC8571_NOTIFY_VBATOVP,
	SC8571_NOTIFY_VOUTOVP,
	SC8571_NOTIFY_VDROVP,
};

enum sc8571_error_stata {
    ERROR_VBUS_HIGH = 0,
	ERROR_VBUS_LOW,
	ERROR_VBUS_OVP,
	ERROR_IBUS_OCP,
	ERROR_VBAT_OVP,
	ERROR_IBAT_OCP,
};

struct flag_bit {
    int notify;
    int mask;
    char *name;
};

struct intr_flag {
    int reg;
    int len;
    struct flag_bit bit[8];
};

static struct intr_flag cp_intr_flag[] = {
    { .reg = 0x18, .len = 7, .bit = {
                {.mask = BIT(0), .name = "vbus ovp alm flag", .notify = SC8571_NOTIFY_VBUSOVPALM},
                {.mask = BIT(1), .name = "vbus ovp flag", .notify = SC8571_NOTIFY_VBUSOVP},
                {.mask = BIT(3), .name = "ibat ocp alm flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "ibat ocp flag", .notify = SC8571_NOTIFY_IBATOCP},
                {.mask = BIT(5), .name = "vout ovp flag", .notify = SC8571_NOTIFY_VOUTOVP},
                {.mask = BIT(6), .name = "vbat ovp alm flag", .notify = SC8571_NOTIFY_VBATOVPALM},
                {.mask = BIT(7), .name = "vbat ovp flag", .notify = SC8571_NOTIFY_VBATOVP},
                },
    },
    { .reg = 0x19, .len = 3, .bit = {
                {.mask = BIT(2), .name = "pin diag fall flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "ibus ucp fall flag", .notify = SC8571_NOTIFY_IBUSUCPF},
                {.mask = BIT(7), .name = "ibus ocp flag", .notify = SC8571_NOTIFY_IBUSOCP},
                },
    },
    { .reg = 0x1a, .len = 8, .bit = {
                {.mask = BIT(0), .name = "acrb2 config flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "acrb1 config flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "vbus present flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "vac2 insert flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vac1 insert flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "vat insert flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "vac2 ovp flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(7), .name = "vac1 ovp flag", .notify = SC8571_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1b, .len = 7, .bit = {
                {.mask = BIT(0), .name = "wd timeout flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(1), .name = "tdie alm flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(2), .name = "tshut flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(3), .name = "tsbat flt flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "tsbus flt flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(5), .name = "tsbus tsbat alm flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(6), .name = "ss timeout flag", .notify = SC8571_NOTIFY_OTHER},
                },
    },
    { .reg = 0x1c, .len = 2, .bit = {
                {.mask = BIT(3), .name = "vbus errorlo flag", .notify = SC8571_NOTIFY_OTHER},
                {.mask = BIT(4), .name = "vbus errorhi flag", .notify = SC8571_NOTIFY_OTHER},
                },
    },
};
    

/************************************************************************/
#define SC8571_DEVICE_ID                0x41

#define SC8571_VBAT_OVP_MIN             3840
#define SC8571_VBAT_OVP_MAX             5110
#define SC8571_VBAT_OVP_STEP            10

#define SC8571_IBAT_OCP_MIN             0
#define SC8571_IBAT_OCP_MAX             12700
#define SC8571_IBAT_OCP_STEP            10

#define SC8571_VBUS_OVP_MIN             7000
#define SC8571_VBUS_OVP_MAX             13350
#define SC8571_VBUS_OVP_STEP            50

#define SC8571_IBUS_OCP_MIN             1000
#define SC8571_IBUS_OCP_MAX             8000
#define SC8571_IBUS_OCP_STEP            250

#define SC8571_REG18                    0x18
#define SC8571_REG25                    0x25
#define SC8571_REGMAX                   0x42

enum sc8571_reg_range {
    SC8571_VBAT_OVP,
    SC8571_VBAT_OVP_ALM,
    SC8571_IBAT_OCP,
    SC8571_IBAT_OCP_ALM,
    SC8571_VBUS_OVP,
    SC8571_VBUS_OVP_ALM,
    SC8571_IBUS_OCP,
};

struct reg_range {
    u32 min;
    u32 max;
    u32 step;
    u32 offset;
    const u32 *table;
    u16 num_table;
    bool round_up;
};

#define SC8571_CHG_RANGE(_min, _max, _step, _offset, _ru) \
{ \
    .min = _min, \
    .max = _max, \
    .step = _step, \
    .offset = _offset, \
    .round_up = _ru, \
}

#define SC8571_CHG_RANGE_T(_table, _ru) \
    { .table = _table, .num_table = ARRAY_SIZE(_table), .round_up = _ru, }


static const struct reg_range sc8571_reg_range[] = {
    [SC8571_VBAT_OVP]      = SC8571_CHG_RANGE(7000, 9540, 20, 7000, true),
    [SC8571_VBAT_OVP_ALM]  = SC8571_CHG_RANGE(7000, 9540, 20, 7000, true),
    [SC8571_IBAT_OCP]      = SC8571_CHG_RANGE(0, 12700, 100, 0, true),
    [SC8571_IBAT_OCP_ALM]      = SC8571_CHG_RANGE(0, 12700, 100, 0, true),
    [SC8571_VBUS_OVP]      = SC8571_CHG_RANGE(14000, 26700, 100, 14000, true),
    [SC8571_VBUS_OVP_ALM]  = SC8571_CHG_RANGE(14000, 26700, 100, 14000, true),
    [SC8571_IBUS_OCP]      = SC8571_CHG_RANGE(1000, 8000, 250, 1000, true),
};


enum sc8571_fields {
    VBAT_OVP_DIS, VBAT_OVP, /*reg00*/
    VBAT_OVP_ALM_DIS, VBAT_OVP_ALM, /*reg01*/
    IBAT_OCP_DIS, IBAT_OCP, /*reg02*/
    IBAT_OCP_ALM_DIS, IBAT_OCP_ALM, /*reg03*/
    IBUS_UCP_DIS, VBUS_IN_RANGE_DIS, /*reg05*/
    VBUS_PD_EN, VBUS_OVP, /*reg06*/
    VBUS_OVP_ALM_DIS, VBUS_OVP_ALM, /*reg07*/
    IBUS_OCP_DIS, IBUS_OCP, /*reg08*/
    TSHUT_DIS, TDIE_ALM_DIS, TSBUS_FLT_DIS, TSBAT_FLT_DIS, /*reg0a*/
    TDIE_ALM, /*reg0b*/
    TSBUS_FLT, /*reg0c*/
    TSBAT_FLT, /*reg0d*/
    VAC1_OVP, VAC2_OVP, VAC1_PD_EN, VAC2_PD_EN, /*reg0e*/
    REG_RST, OTG_EN, CHG_EN, MODE, ACDRV1_STAT, ACDRV2_STAT, /*reg0f*/
    FSW_SET, WD_TIMEOUT, WD_TIMEOUT_DIS, /*reg10*/
    SET_IBAT_SNS_RES, SS_TIMEOUT, IBUS_UCP_FALL_DG, /*reg11*/
    VOUT_OVP_DIS, VOUT_OVP, MS, /*reg12*/
    CP_SWITCHING_STAT,VBUS_ERRORHI_STAT,VBUS_ERRORLO_STAT, /*17*/
    DEVICE_ID, /*reg22*/
    ADC_EN, /*reg23*/
    ACDRV_MANUAL_EN, ACDRV1_EN, ACDRV2_EN, /*reg40*/
    SS_TIMEOUT_DIS, VBUS_OVP_DIS, PMID2OUT_OVP_DIS, PMID2OUT_UVP_DIS, /*reg41*/
    PMID2OUT_UVP, PMID2OUT_OVP, PMID2OUT_UVP_FLAG, PMID2OUT_OVP_FLAG, /*reg42*/
    F_MAX_FIELDS,
};


//REGISTER
static const struct reg_field sc8571_reg_fields[] = {
    /*reg00*/
    [VBAT_OVP_DIS] = REG_FIELD(0x00, 7, 7),
    [VBAT_OVP] = REG_FIELD(0x00, 0, 6),
    /*reg01*/
    [VBAT_OVP_ALM_DIS] = REG_FIELD(0x01, 7, 7),
    [VBAT_OVP_ALM] = REG_FIELD(0x01, 0, 6),
    /*reg02*/
    [IBAT_OCP_DIS] = REG_FIELD(0x02, 7, 7),
    [IBAT_OCP] = REG_FIELD(0x02, 0, 6),
    /*reg03*/
    [IBAT_OCP_ALM_DIS] = REG_FIELD(0x03, 7, 7),
    [IBAT_OCP_ALM] = REG_FIELD(0x03, 0, 6),
    /*reg05*/
    [IBUS_UCP_DIS] = REG_FIELD(0x05, 7, 7),
    [VBUS_IN_RANGE_DIS] = REG_FIELD(0x05, 2, 2),
    /*reg06*/
    [VBUS_PD_EN] = REG_FIELD(0x06, 7, 7),
    [VBUS_OVP] = REG_FIELD(0x06, 0, 6),
    /*reg07*/
    [VBUS_OVP_ALM_DIS] = REG_FIELD(0x07, 7, 7),
    [VBUS_OVP_ALM] = REG_FIELD(0x07, 0, 6),
    /*reg08*/
    [IBUS_OCP_DIS] = REG_FIELD(0x08, 7, 7),
    [IBUS_OCP] = REG_FIELD(0x08, 0, 4),
    /*reg0A*/
    [TSHUT_DIS] = REG_FIELD(0x0A, 7, 7),
    [TDIE_ALM_DIS] = REG_FIELD(0x0A, 4, 4),
    [TSBUS_FLT_DIS] = REG_FIELD(0x0A, 3, 3),
    [TSBAT_FLT_DIS] = REG_FIELD(0x0A, 2, 2),
    /*reg0B*/
    [TDIE_ALM] = REG_FIELD(0x0B, 0, 7),
    /*reg0C*/
    [TSBUS_FLT] = REG_FIELD(0x0C, 0, 7),
    /*reg0D*/
    [TSBAT_FLT] = REG_FIELD(0x0D, 0, 7),
    /*reg0E*/
    [VAC1_OVP] = REG_FIELD(0x0E, 5, 7),
    [VAC2_OVP] = REG_FIELD(0x0E, 2, 4),
    [VAC1_PD_EN] = REG_FIELD(0x0E, 1, 1),
    [VAC2_PD_EN] = REG_FIELD(0x0E, 0, 0),
    /*reg0F*/
    [REG_RST] = REG_FIELD(0x0F, 7, 7),
    [OTG_EN] = REG_FIELD(0x0F, 5, 5),
    [CHG_EN] = REG_FIELD(0x0F, 4, 4),
    [MODE] = REG_FIELD(0x0F, 3, 3),
    [ACDRV1_STAT] = REG_FIELD(0x0F, 1, 1),
    [ACDRV2_STAT] = REG_FIELD(0x0F, 0, 0),
    /*reg10*/
    [FSW_SET] = REG_FIELD(0x10, 5, 7),
    [WD_TIMEOUT] = REG_FIELD(0x10, 3, 4),
    [WD_TIMEOUT_DIS] = REG_FIELD(0x10, 2, 2),
    /*reg11*/
    [SET_IBAT_SNS_RES] = REG_FIELD(0x11, 7, 7),
    [SS_TIMEOUT] = REG_FIELD(0x11, 4, 6),
    [IBUS_UCP_FALL_DG] = REG_FIELD(0x11, 2, 3),
    /*reg12*/
    [VOUT_OVP_DIS] = REG_FIELD(0x12, 7, 7),
    [VOUT_OVP] = REG_FIELD(0x12, 5, 6),
    [MS] = REG_FIELD(0x12, 0, 1),
    /*reg17*/
    [CP_SWITCHING_STAT] = REG_FIELD(0x17, 6, 6),
    [VBUS_ERRORHI_STAT] = REG_FIELD(0x17, 4, 4),
    [VBUS_ERRORLO_STAT] = REG_FIELD(0x17, 3, 3),
    /*reg22*/
    [DEVICE_ID] = REG_FIELD(0x22, 0, 7),
    /*reg23*/
    [ADC_EN] = REG_FIELD(0x23, 7, 7),
    /*reg40*/
    [ACDRV_MANUAL_EN] = REG_FIELD(0x40, 6, 6),
    [ACDRV1_EN] = REG_FIELD(0x40, 5, 5),
    [ACDRV2_EN] = REG_FIELD(0x40, 4, 4),
    /*reg41*/
    [SS_TIMEOUT_DIS] = REG_FIELD(0x41, 7, 7),
    [VBUS_OVP_DIS] = REG_FIELD(0x41, 6, 6),
    [PMID2OUT_OVP_DIS] = REG_FIELD(0x41, 5, 5),
    [PMID2OUT_UVP_DIS] = REG_FIELD(0x41, 4, 4),
    /*reg42*/
    [PMID2OUT_UVP] = REG_FIELD(0x42, 5, 7),
    [PMID2OUT_OVP] = REG_FIELD(0x42, 2, 4),
    [PMID2OUT_UVP_FLAG] = REG_FIELD(0x42, 1, 1),
    [PMID2OUT_OVP_FLAG] = REG_FIELD(0x42, 0, 0),
};

static const struct regmap_config sc8571_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    
    .max_register = SC8571_REGMAX,
};

/************************************************************************/

struct sc8571_cfg_e {
    int vbat_ovp_dis;
    int vbat_ovp;
    int vbat_ovp_alm_dis;
    int vbat_ovp_alm;
    int ibat_ocp_dis;
    int ibat_ocp;
    int ibat_ocp_alm_dis;
    int ibat_ocp_alm;
    int ibus_ucp_dis;
    int vbus_in_range_dis;
    int vbus_pd_en;
    int vbus_ovp;
    int vbus_ovp_alm_dis;
    int vbus_ovp_alm;
    int ibus_ocp_dis;
    int ibus_ocp;
    int tshut_dis;
    int tsbus_flt_dis;
    int tsbat_flt_dis;
    int tdie_alm;
    int tsbus_flt;
    int tsbat_flt;
    int vac1_ovp;
    int vac2_ovp;
    int vac1_pd_en;
    int vac2_pd_en;
    int fsw_set;
    int wd_timeout;
    int wd_timeout_dis;
    int ibat_sns_r;
    int ss_timeout;
    int ibus_ucp_fall_dg;
    int vout_ovp_dis;
    int vout_ovp;
    int ss_timeout_dis;
    int vbus_ovp_dis;
    int pmid2out_ovp_dis;
    int pmid2out_ovp;
    int pmid2out_uvp_dis;
    int pmid2out_uvp;
};

struct sc8571_chip {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;
    struct regmap_field *rmap_fields[F_MAX_FIELDS];

    struct sc8571_cfg_e cfg;
    int irq_gpio;
    int irq;

    int mode;

    bool charge_enabled;
    int usb_present;
    int vbus_volt;
    int ibus_curr;
    int vbat_volt;
    int ibat_curr;
    int die_temp;

#ifdef CONFIG_MTK_CLASS
    struct charger_device *chg_dev;
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    struct dvchg_dev *charger_pump;
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

    const char *chg_dev_name;

    struct power_supply_desc psy_desc;
    struct power_supply_config psy_cfg;
    struct power_supply *psy;
};

#ifdef CONFIG_MTK_CLASS
static const struct charger_properties sc8571_chg_props = {
	.alias_name = "sc8571_chg",
};
#endif /*CONFIG_MTK_CLASS*/


/********************COMMON API***********************/
__maybe_unused static u8 val2reg(enum sc8571_reg_range id, u32 val)
{
    int i;
    u8 reg;
    const struct reg_range *range= &sc8571_reg_range[id];

    if (!range)
        return val;

    if (range->table) {
        if (val <= range->table[0])
            return 0;
        for (i = 1; i < range->num_table - 1; i++) {
            if (val == range->table[i])
                return i;
            if (val > range->table[i] &&
                val < range->table[i + 1])
                return range->round_up ? i + 1 : i;
        }
        return range->num_table - 1;
    }
    if (val <= range->min)
        reg = 0;
    else if (val >= range->max)
        reg = (range->max - range->offset) / range->step;
    else if (range->round_up)
        reg = (val - range->offset) / range->step + 1;
    else
        reg = (val - range->offset) / range->step;
    return reg;
}

__maybe_unused static u32 reg2val(enum sc8571_reg_range id, u8 reg)
{
    const struct reg_range *range= &sc8571_reg_range[id];
    if (!range)
        return reg;
    return range->table ? range->table[reg] :
                  range->offset + range->step * reg;
}
/*********************************************************/
static int sc8571_field_read(struct sc8571_chip *sc,
                enum sc8571_fields field_id, int *val)
{
    int ret;

    ret = regmap_field_read(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8571 read field %d fail: %d\n", field_id, ret);
    }
    
    return ret;
}

static int sc8571_field_write(struct sc8571_chip *sc,
                enum sc8571_fields field_id, int val)
{
    int ret;
    
    ret = regmap_field_write(sc->rmap_fields[field_id], val);
    if (ret < 0) {
        dev_err(sc->dev, "sc8571 read field %d fail: %d\n", field_id, ret);
    }
    
    return ret;
}

static int sc8571_read_block(struct sc8571_chip *sc,
                int reg, uint8_t *val, int len)
{
    int ret;

    ret = regmap_bulk_read(sc->regmap, reg, val, len);
    if (ret < 0) {
        dev_err(sc->dev, "sc8571 read %02x block failed %d\n", reg, ret);
    }

    return ret;
}

/*******************************************************/
__maybe_unused static int sc8571_detect_device(struct sc8571_chip *sc)
{
    int ret;
    int val;

    ret = sc8571_field_read(sc, DEVICE_ID, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail(%d)\n", __func__, ret);
        return ret;
    }

    if (val != SC8571_DEVICE_ID) {
        dev_err(sc->dev, "%s not find SC8571, ID = 0x%02x\n", __func__, ret);
        return -EINVAL;
    }

    return ret;
}

__maybe_unused static int sc8571_reg_reset(struct sc8571_chip *sc)
{
    return sc8571_field_write(sc, REG_RST, 1);
}

__maybe_unused static int sc8571_dump_reg(struct sc8571_chip *sc)
{
    int ret;
    int i;
    int val;

    for (i = 0; i <= SC8571_REGMAX; i++) {
        ret = regmap_read(sc->regmap, i, &val);
        dev_err(sc->dev, "%s reg[0x%02x] = 0x%02x\n", 
                __func__, i, val);
    }

    return ret;
}

__maybe_unused static int sc8571_enable_charge(struct sc8571_chip *sc, bool en)
{
    int ret;
    dev_info(sc->dev,"%s:%d",__func__,en);

    ret = sc8571_field_write(sc, CHG_EN, !!en);

    return ret;
}


__maybe_unused static int sc8571_check_charge_enabled(struct sc8571_chip *sc, bool *enabled)
{
    int ret, val;

    ret = sc8571_field_read(sc, CP_SWITCHING_STAT, &val);

    *enabled = (bool)val;

    dev_info(sc->dev,"%s:%d", __func__, val);

    return ret;
}

__maybe_unused static int sc8571_get_status(struct sc8571_chip *sc, uint32_t *status)
{
    int ret, val;
    *status = 0;

    ret = sc8571_field_read(sc, VBUS_ERRORHI_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORHI_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_HIGH);

    ret = sc8571_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if (ret < 0) {
        dev_err(sc->dev, "%s fail to read VBUS_ERRORLO_STAT(%d)\n", __func__, ret);
        return ret;
    }
    if (val != 0)
        *status |= BIT(ERROR_VBUS_LOW);

    return ret;
}

__maybe_unused static int sc8571_enable_adc(struct sc8571_chip *sc, bool en)
{
    //dev_info(sc->dev,"%s:%d", __func__, en);
    return sc8571_field_write(sc, ADC_EN, !!en);
}

static int sc8571_get_adc_data(struct sc8571_chip *sc, 
            int channel, int *result)
{
    uint8_t val[2] = {0};
    int ret;

    if(channel >= ADC_MAX_NUM) 
        return -EINVAL;

    sc8571_enable_adc(sc, true);
    msleep(50);

    ret = sc8571_read_block(sc, SC8571_REG25 + (channel << 1), val, 2);
    if (ret < 0) {
        return ret;
    }
	
	*result = (val[1] | (val[0] << 8)) * sc8571_adc_m[channel] / sc8571_adc_l[channel];
	

    dev_info(sc->dev,"%s %d %d", __func__, channel, *result);
	

    sc8571_enable_adc(sc, false);

    return ret;
}

__maybe_unused static int sc8571_set_busovp_th(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_VBUS_OVP, threshold);

    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, VBUS_OVP, reg_val);
}

__maybe_unused static int sc8571_set_busocp_th(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_IBUS_OCP, threshold);
    
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, IBUS_OCP, reg_val);
}

__maybe_unused static int sc8571_set_batovp_th(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_VBAT_OVP, threshold);
    
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, VBAT_OVP, reg_val);
}

__maybe_unused static int sc8571_set_batocp_th(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_IBAT_OCP, threshold);
    
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, IBAT_OCP, reg_val);
}

__maybe_unused static int sc8571_set_vbusovp_alarm(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_VBUS_OVP_ALM, threshold);
    
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, VBUS_OVP_ALM, reg_val);
}  

__maybe_unused static int sc8571_set_vbatovp_alarm(struct sc8571_chip *sc, int threshold)
{
    int reg_val = val2reg(SC8571_VBAT_OVP_ALM, threshold);
    
    dev_info(sc->dev,"%s:%d-%d", __func__, threshold, reg_val);

    return sc8571_field_write(sc, VBAT_OVP_ALM, reg_val);
}  

__maybe_unused static int sc8571_disable_vbatovp_alarm(struct sc8571_chip *sc, bool en)
{
    int ret;
    dev_info(sc->dev,"%s:%d",__func__,en);

    ret = sc8571_field_write(sc, VBAT_OVP_ALM_DIS, !!en);

    return ret;
}

__maybe_unused static int sc8571_disable_vbusovp_alarm(struct sc8571_chip *sc, bool en)
{
    int ret;
    dev_info(sc->dev,"%s:%d",__func__,en);

    ret = sc8571_field_write(sc, VBUS_OVP_ALM_DIS, !!en);

    return ret;
}


__maybe_unused static int sc8571_is_vbuslowerr(struct sc8571_chip *sc, bool *err)
{
    int ret;
    int val;

    ret = sc8571_field_read(sc, VBUS_ERRORLO_STAT, &val);
    if(ret < 0) {
        return ret;
    }

    dev_info(sc->dev,"%s:%d",__func__,val);

    *err = (bool)val;

    return ret;
}

__maybe_unused static int sc8571_init_device(struct sc8571_chip *sc)
{
    int ret = 0;
    int i;
    struct {
        enum sc8571_fields field_id;
        int conv_data;
    } props[] = {
        {VBAT_OVP_DIS, sc->cfg.vbat_ovp_dis},
        {VBAT_OVP, sc->cfg.vbat_ovp},
        {VBAT_OVP_ALM_DIS, sc->cfg.vbat_ovp_alm_dis},
        {VBAT_OVP_ALM, sc->cfg.vbat_ovp_alm},
        {IBAT_OCP_DIS, sc->cfg.ibat_ocp_dis},
        {IBAT_OCP, sc->cfg.ibat_ocp},
        {IBAT_OCP_ALM_DIS, sc->cfg.ibat_ocp_alm_dis},
        {IBAT_OCP_ALM, sc->cfg.ibat_ocp_alm},
        {IBUS_UCP_DIS, sc->cfg.ibus_ucp_dis},
        {VBUS_IN_RANGE_DIS, sc->cfg.vbus_in_range_dis},
        {VBUS_PD_EN, sc->cfg.vbus_pd_en},
        {VBUS_OVP, sc->cfg.vbus_ovp},
        {VBUS_OVP_ALM_DIS, sc->cfg.vbus_ovp_alm_dis},
        {VBUS_OVP_ALM, sc->cfg.vbus_ovp_alm},
        {IBUS_OCP_DIS, sc->cfg.ibus_ocp_dis},
        {IBUS_OCP, sc->cfg.ibus_ocp},
        {TSHUT_DIS, sc->cfg.tshut_dis},
        {TSBUS_FLT_DIS, sc->cfg.tsbus_flt_dis},
        {TSBAT_FLT_DIS, sc->cfg.tsbat_flt_dis},
        {TDIE_ALM, sc->cfg.tdie_alm},
        {TSBUS_FLT, sc->cfg.tsbus_flt},
        {TSBAT_FLT, sc->cfg.tsbat_flt},
        {VAC1_OVP, sc->cfg.vac1_ovp},
        {VAC2_OVP, sc->cfg.vac2_ovp},
        {VAC1_PD_EN, sc->cfg.vac1_pd_en},
        {VAC2_PD_EN, sc->cfg.vac2_pd_en},
        {FSW_SET, sc->cfg.fsw_set},
        {WD_TIMEOUT, sc->cfg.wd_timeout},
        {WD_TIMEOUT_DIS, sc->cfg.wd_timeout_dis},
        {SET_IBAT_SNS_RES, sc->cfg.ibat_sns_r},
        {SS_TIMEOUT_DIS, sc->cfg.ss_timeout_dis},
        {SS_TIMEOUT, sc->cfg.ss_timeout},
        {IBUS_UCP_FALL_DG, sc->cfg.ibus_ucp_fall_dg},
        {VOUT_OVP_DIS, sc->cfg.vout_ovp_dis},
        {VOUT_OVP, sc->cfg.vout_ovp},
        {VBUS_OVP_DIS, sc->cfg.vbus_ovp_dis},
        {PMID2OUT_OVP_DIS, sc->cfg.pmid2out_ovp_dis},
        {PMID2OUT_OVP, sc->cfg.pmid2out_ovp},
        {PMID2OUT_UVP_DIS, sc->cfg.pmid2out_uvp_dis},
        {PMID2OUT_UVP, sc->cfg.pmid2out_uvp},
        
    };

    ret = sc8571_reg_reset(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s Failed to reset registers(%d)\n", __func__, ret);
    }
    msleep(10);

    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = sc8571_field_write(sc, props[i].field_id, props[i].conv_data);
    }

    if (sc->mode == SC8571_SLAVE) {
        ret = sc8571_field_write(sc, VBUS_IN_RANGE_DIS, 1);
        if (ret < 0) {
            dev_err(sc->dev, "%s Failed to disable vbus in range(%d)\n", __func__, ret);
        }
    }

    sc8571_dump_reg(sc);

    return ret;
}


/*********************mtk charger interface start**********************************/
#ifdef CONFIG_MTK_CLASS
static inline int to_sc8571_adc(enum adc_channel chan)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		return ADC_VBUS;
	case ADC_CHANNEL_VBAT:
		return ADC_VBAT;
	case ADC_CHANNEL_IBUS:
		return ADC_IBUS;
	case ADC_CHANNEL_IBAT:
		return ADC_IBAT;
	case ADC_CHANNEL_TEMP_JC:
		return ADC_TDIE;
	case ADC_CHANNEL_VOUT:
		return ADC_VOUT;
	default:
		break;
	}
	return ADC_MAX_NUM;
}


static int mtk_sc8571_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8571_check_charge_enabled(sc, en);

    return ret;
}


static int mtk_sc8571_enable_chg(struct charger_device *chg_dev, bool en)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;
	
	pr_err("gezi----%s----%d",__func__,en);

    ret = sc8571_enable_charge(sc,en);

    return ret;
}


static int mtk_sc8571_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int mv;
    mv = uV / 1000;

    return sc8571_set_busovp_th(sc, mv);
}

static int mtk_sc8571_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ma;
    ma = uA / 1000;

    return sc8571_set_busocp_th(sc, ma);
}

static int mtk_sc8571_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{   
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8571_set_batovp_th(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_sc8571_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{   
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8571_set_batocp_th(sc, uA/1000);
    if (ret < 0)
        return ret;

    return ret;
}


static int mtk_sc8571_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);

    sc8571_get_adc_data(sc, to_sc8571_adc(chan), max);

    if(chan != ADC_CHANNEL_TEMP_JC) 
        *max = *max * 1000;
    
    if (min != max)
		*min = *max;

    return 0;
}

static int mtk_sc8571_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
    *min = *max = sc8571_adc_accuracy_tbl[to_sc8571_adc(chan)];
    return 0;   
}

static int mtk_sc8571_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);

    return sc8571_is_vbuslowerr(sc,err);
}

static int mtk_sc8571_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8571_set_vbatovp_alarm(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
}

static int mtk_sc8571_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    dev_err(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8571_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    int ret;

    ret = sc8571_set_vbusovp_alarm(sc, uV/1000);
    if (ret < 0)
        return ret;

    return ret;
}   

static int mtk_sc8571_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);
    dev_err(sc->dev,"%s",__func__);
    return 0;
}

static int mtk_sc8571_init_chip(struct charger_device *chg_dev)
{
    struct sc8571_chip *sc = charger_get_data(chg_dev);

    return sc8571_init_device(sc);
}

/*driver add by zhaopengge 20230727-----start*/	
static int mtk_sc8571_enable_otg(struct charger_device *chg_dev, bool en)
{
	
	struct sc8571_chip *sc = charger_get_data(chg_dev);
	
	pr_err("%s:%d",__func__,en);
	
	if(en){
/*
To enter OTG mode, the host should follow the steps below��
1. Host writes ACDRV_MANUAL_EN=1, ACDRV1_EN=0 and ACDRV2_EN=0
2. Host enables main charger working in OTG mode
3. Host set OTG_EN=1
4. Host writes ACDRV1_EN=1 or ACDRV2_EN=1 depending on the which port is desired		
*/
    
    sc8571_field_write(sc, ACDRV_MANUAL_EN,1);
	msleep(10);
	sc8571_field_write(sc, OTG_EN, 1);
	msleep(10);
	sc8571_field_write(sc, ACDRV2_EN, 1);
	
	}
	else{
/*
To exit OTG mode, the host should follow the steps below��
1. Host turns off the OTG source
2. Host writes BUS_PD_EN=1 or corresponding VAC1/2_PD_EN=1 to discharge the port which was used.
3. Wait for VBUS and VAC to be discharged
4. Host writes ACDRV1/2_EN=0
5. Host writes OTG_EN=0
*/		
	sc8571_field_write(sc, ACDRV2_EN, 0);
	sc8571_field_write(sc, OTG_EN, 0);
	sc8571_field_write(sc, ACDRV_MANUAL_EN,0);
	
	}
	
	return 0;
}
/*driver add by zhaopengge 20230727-----end*/	
static const struct charger_ops sc8571_chg_ops = {
	 .enable = mtk_sc8571_enable_chg,
	 .is_enabled = mtk_sc8571_is_chg_enabled,
	 .get_adc = mtk_sc8571_get_adc,
     .get_adc_accuracy = mtk_sc8571_get_adc_accuracy,
	 .set_vbusovp = mtk_sc8571_set_vbusovp,
	 .set_ibusocp = mtk_sc8571_set_ibusocp,
	 .set_vbatovp = mtk_sc8571_set_vbatovp,
	 .set_ibatocp = mtk_sc8571_set_ibatocp,
	 .init_chip = mtk_sc8571_init_chip,
     .is_vbuslowerr = mtk_sc8571_is_vbuslowerr,
	 .set_vbatovp_alarm = mtk_sc8571_set_vbatovp_alarm,
	 .reset_vbatovp_alarm = mtk_sc8571_reset_vbatovp_alarm,
	 .set_vbusovp_alarm = mtk_sc8571_set_vbusovp_alarm,
	 .reset_vbusovp_alarm = mtk_sc8571_reset_vbusovp_alarm,
/*driver add by zhaopengge 20230727-----start*/	
	 .enable_otg = mtk_sc8571_enable_otg,
/*driver add by zhaopengge 20230727-----end*/	
};
#endif /*CONFIG_MTK_CLASS*/
/********************mtk charger interface end*************************************************/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
static inline int sc_to_sc8571_adc(enum sc_adc_channel chan)
{
	switch (chan) {
	case SC_ADC_VBUS:
		return ADC_VBUS;
	case SC_ADC_VBAT:
		return ADC_VBAT;
	case SC_ADC_IBUS:
		return ADC_IBUS;
	case SC_ADC_IBAT:
		return ADC_IBAT;
	case SC_ADC_TDIE:
		return ADC_TDIE;
	default:
		break;
	}
	return ADC_MAX_NUM;
}


static int sc_sc8571_set_enable(struct dvchg_dev *charger_pump, bool enable)
{
    struct sc8571_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc8571_enable_charge(sc,enable);

    return ret;
}

static int sc_sc8571_get_is_enable(struct dvchg_dev *charger_pump, bool *enable)
{
    struct sc8571_chip *sc = dvchg_get_private(charger_pump);
    int ret;

    ret = sc8571_check_charge_enabled(sc, enable);

    return ret;
}

static int sc_sc8571_get_status(struct dvchg_dev *charger_pump, uint32_t *status)
{
    struct sc8571_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;

    ret = sc8571_get_status(sc, status);

    return ret;
}

static int sc_sc8571_get_adc_value(struct dvchg_dev *charger_pump, enum sc_adc_channel ch, int *value)
{
    struct sc8571_chip *sc = dvchg_get_private(charger_pump);
    int ret = 0;

    ret = sc8571_get_adc_data(sc, sc_to_sc8571_adc(ch), value);

    return ret;
}

static struct dvchg_ops sc_sc8571_dvchg_ops = {
    .set_enable = sc_sc8571_set_enable,
    .get_status = sc_sc8571_get_status,
    .get_is_enable = sc_sc8571_get_is_enable,
    .get_adc_value = sc_sc8571_get_adc_value,
};
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/

/********************creat devices note start*************************************************/
static ssize_t sc8571_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc8571_chip *sc = dev_get_drvdata(dev);
    u8 addr;
    int val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    int ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8571");
    for (addr = 0x0; addr <= SC8571_REGMAX; addr++) {
        ret = regmap_read(sc->regmap, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc8571_store_register(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc8571_chip *sc = dev_get_drvdata(dev);
    int ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= SC8571_REGMAX)
        regmap_write(sc->regmap, (unsigned char)reg, (unsigned char)val);

    return count;
}

static DEVICE_ATTR(registers, 0660, sc8571_show_registers, sc8571_store_register);

static void sc8571_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
}
/********************creat devices note end*************************************************/


/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
#ifdef CONFIG_MTK_CLASS
static inline int status_reg_to_charger(enum sc8571_notify notify)
{
	switch (notify) {
	case SC8571_NOTIFY_IBUSUCPF:
		return CHARGER_DEV_NOTIFY_IBUSUCP_FALL;
    case SC8571_NOTIFY_VBUSOVPALM:
		return CHARGER_DEV_NOTIFY_VBUSOVP_ALARM;
    case SC8571_NOTIFY_VBATOVPALM:
		return CHARGER_DEV_NOTIFY_VBATOVP_ALARM;
    case SC8571_NOTIFY_IBUSOCP:
		return CHARGER_DEV_NOTIFY_IBUSOCP;
    case SC8571_NOTIFY_VBUSOVP:
		return CHARGER_DEV_NOTIFY_VBUS_OVP;
    case SC8571_NOTIFY_IBATOCP:
		return CHARGER_DEV_NOTIFY_IBATOCP;
    case SC8571_NOTIFY_VBATOVP:
		return CHARGER_DEV_NOTIFY_BAT_OVP;
    case SC8571_NOTIFY_VOUTOVP:
		return CHARGER_DEV_NOTIFY_VOUTOVP;
	default:
        return -EINVAL;
		break;
	}
	return -EINVAL;
}
#endif /*CONFIG_MTK_CLASS*/
static void sc8571_check_fault_status(struct sc8571_chip *sc)
{
    int ret;
    u8 flag = 0;
    int i,j;
#ifdef CONFIG_MTK_CLASS
    int noti;
#endif /*CONFIG_MTK_CLASS*/

    for (i=0;i < ARRAY_SIZE(cp_intr_flag);i++) {
        ret = sc8571_read_block(sc, cp_intr_flag[i].reg, &flag, 1);
        for (j=0; j <  cp_intr_flag[i].len; j++) {
            if (flag & cp_intr_flag[i].bit[j].mask) {
                dev_err(sc->dev,"trigger :%s\n",cp_intr_flag[i].bit[j].name);
#ifdef CONFIG_MTK_CLASS
                noti = status_reg_to_charger(cp_intr_flag[i].bit[j].notify);
                if(noti >= 0) {
                    charger_dev_notify(sc->chg_dev, noti);
                }
#endif /*CONFIG_MTK_CLASS*/
            }
        }
    }
}

static irqreturn_t sc8571_irq_handler(int irq, void *data)
{
    struct sc8571_chip *sc = data;

    dev_err(sc->dev,"INT OCCURED\n");

    sc8571_check_fault_status(sc);

    power_supply_changed(sc->psy);

    return IRQ_HANDLED;
}

static int sc8571_register_interrupt(struct sc8571_chip *sc)
{
    int ret;

    if (gpio_is_valid(sc->irq_gpio)) {
        ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN,"sc8571_irq");
        if (ret) {
            dev_err(sc->dev,"failed to request sc8571_irq\n");
            return -EINVAL;
        }
        sc->irq = gpio_to_irq(sc->irq_gpio);
        if (sc->irq < 0) {
            dev_err(sc->dev,"failed to gpio_to_irq\n");
            return -EINVAL;
        }
    } else {
        dev_err(sc->dev,"irq gpio not provided\n");
        return -EINVAL;
    }

    if (sc->irq) {
        ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
                NULL, sc8571_irq_handler,
                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                sc8571_irq_name[sc->mode], sc);

        if (ret < 0) {
            dev_err(sc->dev,"request irq for irq=%d failed, ret =%d\n",
                            sc->irq, ret);
            return ret;
        }
		else {
			dev_err(sc->dev,"request irq for irq=%d success, ret =%d\n", sc->irq, ret);
		}
        enable_irq_wake(sc->irq);
    }

    return ret;
}
/********************interrupte end*************************************************/


/************************psy start**************************************/
static enum power_supply_property sc8571_charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_TEMP,
};

static int sc8571_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc8571_chip *sc = power_supply_get_drvdata(psy);
    int result;
    int ret;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8571_check_charge_enabled(sc, &sc->charge_enabled);
        val->intval = sc->charge_enabled;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc8571_get_adc_data(sc, ADC_VBUS, &result);
        if (!ret)
            sc->vbus_volt = result;
        val->intval = sc->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc8571_get_adc_data(sc, ADC_IBUS, &result);
        if (!ret)
            sc->ibus_curr = result;
        val->intval = sc->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
        ret = sc8571_get_adc_data(sc, ADC_VBAT, &result);
        if (!ret)
            sc->vbat_volt = result;
        val->intval = sc->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
        ret = sc8571_get_adc_data(sc, ADC_IBAT, &result);
        if (!ret)
            sc->ibat_curr = result;
        val->intval = sc->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        ret = sc8571_get_adc_data(sc, ADC_TDIE, &result);
        if (!ret)
            sc->die_temp = result;
        val->intval = sc->die_temp;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int sc8571_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
    struct sc8571_chip *sc = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_ONLINE:
        sc8571_enable_charge(sc, val->intval);
        dev_info(sc->dev, "POWER_SUPPLY_PROP_ONLINE: %s\n",
                val->intval ? "enable" : "disable");
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int sc8571_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    return 0;
}

static int sc8571_psy_register(struct sc8571_chip *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = sc8571_psy_name[sc->mode];

    sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
    sc->psy_desc.properties = sc8571_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc8571_charger_props);
    sc->psy_desc.get_property = sc8571_charger_get_property;
    sc->psy_desc.set_property = sc8571_charger_set_property;
    sc->psy_desc.property_is_writeable = sc8571_charger_is_writeable;


    sc->psy = devm_power_supply_register(sc->dev, 
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->psy)) {
        dev_err(sc->dev, "%s failed to register psy\n", __func__);
        return PTR_ERR(sc->psy);
    }

    dev_info(sc->dev, "%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}


/************************psy end**************************************/

static int sc8571_set_work_mode(struct sc8571_chip *sc, int mode)
{
    sc->mode = mode;

    dev_err(sc->dev,"work mode is %s\n", sc->mode == SC8571_STANDALONG 
        ? "standalone" : (sc->mode == SC8571_MASTER ? "master" : "slave"));

    return 0;
}

static int sc8571_parse_dt(struct sc8571_chip *sc, struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i;
    int ret;
    struct {
        char *name;
        int *conv_data;
    } props[] = {
        {"sc,sc8571,vbat-ovp-dis", &(sc->cfg.vbat_ovp_dis)},
        {"sc,sc8571,vbat-ovp", &(sc->cfg.vbat_ovp)},
        {"sc,sc8571,vbat-ovp-alm-dis", &(sc->cfg.vbat_ovp_alm_dis)},
        {"sc,sc8571,vbat-ovp-alm", &(sc->cfg.vbat_ovp_alm)},
        {"sc,sc8571,ibat-ocp-dis", &(sc->cfg.ibat_ocp_dis)},
        {"sc,sc8571,ibat-ocp", &(sc->cfg.ibat_ocp)},
        {"sc,sc8571,ibat-ocp-alm-dis", &(sc->cfg.ibat_ocp_alm_dis)},
        {"sc,sc8571,ibat-ocp-alm", &(sc->cfg.ibat_ocp_alm)},
        {"sc,sc8571,ibus-ucp-dis", &(sc->cfg.ibus_ucp_dis)},
        {"sc,sc8571,vbus-in-range-dis", &(sc->cfg.vbus_in_range_dis)},
        {"sc,sc8571,vbus-pd-en", &(sc->cfg.vbus_pd_en)},
        {"sc,sc8571,vbus-ovp", &(sc->cfg.vbus_ovp)},
        {"sc,sc8571,vbus-ovp-alm-dis", &(sc->cfg.vbus_ovp_alm_dis)},
        {"sc,sc8571,vbus-ovp-alm", &(sc->cfg.vbus_ovp_alm)},
        {"sc,sc8571,ibus-ocp-dis", &(sc->cfg.ibus_ocp_dis)},
        {"sc,sc8571,ibus-ocp", &(sc->cfg.ibus_ocp)},
        {"sc,sc8571,tshut-dis", &(sc->cfg.tshut_dis)},
        {"sc,sc8571,tsbus-flt-dis", &(sc->cfg.tsbus_flt_dis)},
        {"sc,sc8571,tsbat-flt-dis", &(sc->cfg.tsbat_flt_dis)},
        {"sc,sc8571,tdie-alm", &(sc->cfg.tdie_alm)},
        {"sc,sc8571,tsbus-flt", &(sc->cfg.tsbus_flt)},
        {"sc,sc8571,tsbat-flt", &(sc->cfg.tsbat_flt)},
        {"sc,sc8571,vac1-ovp", &(sc->cfg.vac1_ovp)},
        {"sc,sc8571,vac2-ovp", &(sc->cfg.vac2_ovp)},
        {"sc,sc8571,vac1-pd-en", &(sc->cfg.vac1_pd_en)},
        {"sc,sc8571,vac2-pd-en", &(sc->cfg.vac2_pd_en)},
        {"sc,sc8571,fsw-set", &(sc->cfg.fsw_set)},
        {"sc,sc8571,wd-timeout", &(sc->cfg.wd_timeout)},
        {"sc,sc8571,wd-timeout-dis", &(sc->cfg.wd_timeout_dis)},
        {"sc,sc8571,ibat-sns-r", &(sc->cfg.ibat_sns_r)},
        {"sc,sc8571,ss-timeout-dis", &(sc->cfg.ss_timeout_dis)},
        {"sc,sc8571,ss-timeout", &(sc->cfg.ss_timeout)},
        {"sc,sc8571,ibus-ucp-fall-dg", &(sc->cfg.ibus_ucp_fall_dg)},
        {"sc,sc8571,vout-ovp-dis", &(sc->cfg.vout_ovp_dis)},
        {"sc,sc8571,vout-ovp", &(sc->cfg.vout_ovp)},
        {"sc,sc8571,vbus-ovp-dis", &(sc->cfg.vbus_ovp_dis)},
        {"sc,sc8571,pmid2out-ovp-dis", &(sc->cfg.pmid2out_ovp_dis)},
        {"sc,sc8571,pmid2out-ovp", &(sc->cfg.pmid2out_ovp)},
        {"sc,sc8571,pmid2out-uvp-dis", &(sc->cfg.pmid2out_uvp_dis)},
        {"sc,sc8571,pmid2out-uvp", &(sc->cfg.pmid2out_uvp)},
    };

    /* initialize data for optional properties */
    for (i = 0; i < ARRAY_SIZE(props); i++) {
        ret = of_property_read_u32(np, props[i].name,
                        props[i].conv_data);
        if (ret < 0) {
            dev_err(sc->dev, "can not read %s \n", props[i].name);
            return ret;
        }
    }

    sc->irq_gpio = of_get_named_gpio(np, "sc8571,intr_gpio", 0);
    if (!gpio_is_valid(sc->irq_gpio)) {
        dev_err(sc->dev,"fail to valid gpio : %d\n", sc->irq_gpio);
        return -EINVAL;
    }

    if (of_property_read_string(np, "charger_name", &sc->chg_dev_name) < 0) {
		sc->chg_dev_name = "charger";
		dev_err(sc->dev, "no charger name\n");
	}

    return 0;
}

static struct of_device_id sc8571_charger_match_table[] = {
    {   .compatible = "sc,sc8571-standalone", 
        .data = &sc8571_mode_data[SC8571_STANDALONG], },
    {   .compatible = "sc,sc8571-master", 
        .data = &sc8571_mode_data[SC8571_MASTER], },
    {   .compatible = "sc,sc8571-slave", 
        .data = &sc8571_mode_data[SC8571_SLAVE], },
};

static int sc8571_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc8571_chip *sc;
    const struct of_device_id *match;
    struct device_node *node = client->dev.of_node;
    int ret, i;

    dev_err(&client->dev, "%s (%s)\n", __func__, SC8571_DRV_VERSION);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc8571_chip), GFP_KERNEL);
    if (!sc) {
        ret = -ENOMEM;
        goto err_kzalloc;
    }

    sc->dev = &client->dev;
    sc->client = client;

    sc->regmap = devm_regmap_init_i2c(client,
                            &sc8571_regmap_config);
    if (IS_ERR(sc->regmap)) {
        dev_err(sc->dev, "Failed to initialize regmap\n");
        ret = PTR_ERR(sc->regmap);
        goto err_regmap_init;
    }

    for (i = 0; i < ARRAY_SIZE(sc8571_reg_fields); i++) {
        const struct reg_field *reg_fields = sc8571_reg_fields;

        sc->rmap_fields[i] =
            devm_regmap_field_alloc(sc->dev,
                        sc->regmap,
                        reg_fields[i]);
        if (IS_ERR(sc->rmap_fields[i])) {
            dev_err(sc->dev, "cannot allocate regmap field\n");
            ret = PTR_ERR(sc->rmap_fields[i]);
            goto err_regmap_field;
        }
    }

    ret = sc8571_detect_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s detect device fail\n", __func__);
        goto err_detect_dev;
    }

    i2c_set_clientdata(client, sc);
    sc8571_create_device_node(&(client->dev));

    match = of_match_node(sc8571_charger_match_table, node);
    if (match == NULL) {
        dev_err(sc->dev, "device tree match not found!\n");
        goto err_match_node;
    }

    sc8571_set_work_mode(sc, *(int *)match->data);
    if (ret) {
        dev_err(sc->dev,"Fail to set work mode!\n");
        goto err_set_mode;
    }

    ret = sc8571_parse_dt(sc, &client->dev);
    if (ret < 0) {
        dev_err(sc->dev, "%s parse dt failed(%d)\n", __func__, ret);
        goto err_parse_dt;
    }

    ret = sc8571_init_device(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s init device failed(%d)\n", __func__, ret);
        goto err_init_device;
    }

    ret = sc8571_psy_register(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s psy register failed(%d)\n", __func__, ret);
        goto err_register_psy;
    }

    ret = sc8571_register_interrupt(sc);
    if (ret < 0) {
        dev_err(sc->dev, "%s register irq fail(%d)\n",
                    __func__, ret);
        goto err_register_irq;
    }

#ifdef CONFIG_MTK_CLASS
    sc->chg_dev = charger_device_register(sc->chg_dev_name,
					      &client->dev, sc,
					      &sc8571_chg_ops,
					      &sc8571_chg_props);
	if (IS_ERR_OR_NULL(sc->chg_dev)) {
		ret = PTR_ERR(sc->chg_dev);
		dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_mtk_charger;
	}
#endif /*CONFIG_MTK_CLASS*/

#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
    sc->charger_pump = dvchg_register("sc_dvchg",
                             sc->dev, &sc_sc8571_dvchg_ops, sc);
    if (IS_ERR_OR_NULL(sc->charger_pump)) {
		ret = PTR_ERR(sc->charger_pump);
		dev_err(sc->dev,"Fail to register charger!\n");
        goto err_register_sc_charger;
	}
#endif /* CONFIG_SOUTHCHIP_DVCHG_CLASS */
    dev_err(sc->dev, "sc8571[%s] probe successfully!\n", 
            sc->mode == SC8571_MASTER ? "master" : "slave");
    return 0;

err_register_psy:
err_register_irq:
#ifdef CONFIG_MTK_CLASS
err_register_mtk_charger:
#endif /*CONFIG_MTK_CLASS*/
#ifdef CONFIG_SOUTHCHIP_DVCHG_CLASS
err_register_sc_charger:
#endif /*CONFIG_SOUTHCHIP_DVCHG_CLASS*/
err_init_device:
    power_supply_unregister(sc->psy);
err_detect_dev:
err_match_node:
err_set_mode:
err_parse_dt:
err_regmap_init:
err_regmap_field:
    devm_kfree(&client->dev, sc);
err_kzalloc:
    dev_err(&client->dev,"sc8571 probe fail\n");
    return ret;
}


static int sc8571_charger_remove(struct i2c_client *client)
{
    struct sc8571_chip *sc = i2c_get_clientdata(client);

    power_supply_unregister(sc->psy);
    devm_kfree(&client->dev, sc);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sc8571_suspend(struct device *dev)
{
    struct sc8571_chip *sc = dev_get_drvdata(dev);

    dev_info(sc->dev, "Suspend successfully!");
    if (device_may_wakeup(dev))
        enable_irq_wake(sc->irq);
    disable_irq(sc->irq);

    return 0;
}
static int sc8571_resume(struct device *dev)
{
    struct sc8571_chip *sc = dev_get_drvdata(dev);

    dev_info(sc->dev, "Resume successfully!");
    if (device_may_wakeup(dev))
        disable_irq_wake(sc->irq);
    enable_irq(sc->irq);

    return 0;
}

static const struct dev_pm_ops sc8571_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(sc8571_suspend, sc8571_resume)
};
#endif

static struct i2c_driver sc8571_charger_driver = {
    .driver     = {
        .name   = "sc8571",
        .owner  = THIS_MODULE,
        .of_match_table = sc8571_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sc8571_pm,
#endif
    },
    .probe      = sc8571_charger_probe,
    .remove     = sc8571_charger_remove,
};

module_i2c_driver(sc8571_charger_driver);

MODULE_DESCRIPTION("SC SC8571 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("South Chip <Aiden-yu@southchip.com>");

