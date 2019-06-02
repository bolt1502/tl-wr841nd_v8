#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/proc_fs.h>
/*#include <linux/miscdevice.h>*/
#include <asm/types.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/system.h>

#include <linux/mtd/mtd.h>
#include <linux/cdev.h>
#include <linux/irqreturn.h>

#include <atheros.h>

/*
 * IOCTL Command Codes
 */

#define ATH_GPIO_IOCTL_BASE			0x01
#define ATH_GPIO_IOCTL_CMD1      	ATH_GPIO_IOCTL_BASE
#define ATH_GPIO_IOCTL_CMD2      	ATH_GPIO_IOCTL_BASE + 0x01
#define ATH_GPIO_IOCTL_CMD3      	ATH_GPIO_IOCTL_BASE + 0x02
#define ATH_GPIO_IOCTL_CMD4      	ATH_GPIO_IOCTL_BASE + 0x03
#define ATH_GPIO_IOCTL_CMD5      	ATH_GPIO_IOCTL_BASE + 0x04
#define ATH_GPIO_IOCTL_CMD6      	ATH_GPIO_IOCTL_BASE + 0x05
#define ATH_GPIO_IOCTL_MAX			ATH_GPIO_IOCTL_CMD6

#define ATH_GPIO_MAGIC 				0xB2
#define	ATH_GPIO_BTN_READ			_IOR(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD1, int)
#define	ATH_GPIO_WIFI_SW_READ		_IOR(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD2, int)
#define	ATH_GPIO_LED_WRITE			_IOW(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD3, int)
#define ATH_GPIO_USB_LED1_WRITE		_IOW(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD4, int)
#define ATH_GPIO_INET_LED_WRITE		_IOW(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD5, int)
#define ATH_GPIO_WIFI_BTN_READ		_IOW(ATH_GPIO_MAGIC, ATH_GPIO_IOCTL_CMD6, int)

#define gpio_major      				238
#define gpio_minor      				0
#if 0
/*
 * GPIO assignment
 */

#define GPIO_USB_LED0		0	/* controled by usb EHCI currently not used */
#define GPIO_USB_LED1		8	/* usb connect or not, controled by software */

/* 090108, modified by lsz */
#define RST_DFT_GPIO		11	/* reset default */
#define SYS_LED_GPIO		1	/* system led 	*/
#define JUMPSTART_GPIO		12	/* wireless jumpstart */
#define RST_HOLD_TIME		5	/* How long the user must press the button before Router rst */


#define TRICOLOR_LED_GREEN_PIN  0   /* jump start LED */

#define OFF     (!0)             /* jump start LED's value when off */
#define ON      0               /* jump start LED'S value when on */
#endif
/*
 * GPIO assignment
 */

#define ATH_GPIO_MIN		0

/* reset default button, default is GPIO17 */
#define RST_DFT_GPIO		CONFIG_GPIO_RESET_FAC_BIT
/* How long the user must press the button before Router rst, default is 5 */
#define RST_HOLD_TIME		CONFIG_GPIO_FAC_RST_HOLD_TIME

/* system LED, default is GPIO14 	*/
#define SYS_LED_GPIO		CONFIG_GPIO_READY_STATUS_BIT
/* system LED's value when off, default is 1 */
#define SYS_LED_OFF         (!CONFIG_GPIO_READY_STATUS_ON)

/* QSS default button, default is None, we use RST&QSS MUX button */
/*#define JUMPSTART_GPIO		CONFIG_GPIO_JUMPSTART_SW_BIT*/
/* QSS LED, default is GPIO15 */
#define TRICOLOR_LED_GREEN_PIN  CONFIG_GPIO_JUMPSTART_LED_BIT
/* jump start LED's value when off */
#define OFF     (!CONFIG_GPIO_JUMPSTART_LED_ON)
/* jump start LED'S value when on, default is 0 */
#define ON      CONFIG_GPIO_JUMPSTART_LED_ON

/* WiFi switch, default is GPIO16 */
#define WIFI_RADIO_SW_GPIO	CONFIG_GPIO_WIFI_SWITCH_BIT

/* Work Mode switch for 803, GPIO 14 and 16 */
#define WORKMODE_SWITCH_1ST_GPIO	CONFIG_GPIO_WORKMODE_SWITCH_1ST_BIT
#define WORKMODE_SWITCH_2ND_GPIO	CONFIG_GPIO_WORKMODE_SWITCH_2ND_BIT

/* Internet LED for 803, default is GPIO11 */
#define INET_LED_GPIO		CONFIG_GPIO_INTERNET_LED_BIT
/* Internet LED's vaule when on, default is 0 */
#define INET_LED_ON			CONFIG_GPIO_INTERNET_LED_ON
/* Internet LED's vaule when off, default is 1 */
#define	INET_LED_OFF		(!CONFIG_GPIO_INTERNET_LED_ON)


#ifdef CONFIG_GPIO_USB_LED_BIT
/* USB LED, default is GPIO11 */
#define AP_USB_LED_GPIO     CONFIG_GPIO_USB_LED_BIT
/* USB LED's value when off */
#define USB_LED_OFF         (!CONFIG_GPIO_USB_LED_ON)
/* USB LED's value when on, default is 0 */
#define USB_LED_ON          CONFIG_GPIO_USB_LED_ON
/* usb power switch, default is 4 */
#define USB_POWER_SW_GPIO	CONFIG_GPIO_USB_SWITCHFOR3G_BIT
#define USB_POWER_ON		1
#define USB_POWER_OFF		(!USB_POWER_ON)
#else
#undef AP_USB_LED_GPIO
#endif


static int counter = 0;
//int jiff_when_press = 0;
static int bBlockWps = 1;
static struct timer_list rst_timer;

/* local var for wifi-switch */
static struct timer_list wifi_button_timer;
static int ignore_wifibutton = 1;
static int wifi_button_flag = 0;

int g_usbLedBlinkCountDown = 1;

/* control params for reset button reuse, by zjg, 13Apr10 */
static int l_bMultiUseResetButton		=	0;
static int l_bWaitForQss				= 	1;

/* some models use wifi button for wps */
static int l_bMultiUseWifiButton		=	0;

EXPORT_SYMBOL(g_usbLedBlinkCountDown);

/*
 * GPIO interrupt stuff
 */
typedef enum {
	INT_TYPE_EDGE,
	INT_TYPE_LEVEL,
} ath_gpio_int_type_t;

typedef enum {
	INT_POL_ACTIVE_LOW,
	INT_POL_ACTIVE_HIGH,
} ath_gpio_int_pol_t;

/* 
** Simple Config stuff
*/
#if 0
#if !defined(IRQ_NONE)
#define IRQ_NONE
#define IRQ_HANDLED
#endif /* !defined(IRQ_NONE) */
#endif


/*changed by hujia.*/
typedef irqreturn_t(*sc_callback_t)(int, void *, struct pt_regs *, void *);
#if 0
/*
 * Multiple Simple Config callback support
 * For multiple radio scenarios, we need to post the button push to
 * all radios at the same time.  However, there is only 1 button, so
 * we only have one set of GPIO callback pointers.
 *
 * Creating a structure that contains each callback, tagged with the
 * name of the device registering the callback.  The unregister routine
 * will need to determine which element to "unregister", so the device
 * name will have to be passed to unregister also
 */

typedef struct {
	char		*name;
	sc_callback_t	registered_cb;
	void		*cb_arg1;
	void		*cb_arg2;
} multi_callback_t;

/*
 * Specific instance of the callback structure
 */
static multi_callback_t sccallback[2];
#endif
static sc_callback_t registered_cb = NULL;
static void *cb_arg;
/*add by hujia.*/
static void *cb_pushtime;
/*end add.*/
//static int ignore_pushbutton = 1;
static struct proc_dir_entry *simple_config_entry = NULL;
static struct proc_dir_entry *simulate_push_button_entry = NULL;
static struct proc_dir_entry *tricolor_led_entry = NULL;
/* added by zjg, 12Apr10 */
static struct proc_dir_entry *multi_use_reset_button_entry = NULL;

static struct proc_dir_entry *multi_use_wifi_button_entry = NULL;

/* ZJin 100317: for 3g usb led blink feature, use procfs simple config. */
static struct proc_dir_entry *usb_led_blink_entry = NULL;

/*added by  ZQQ<10.06.02 for usb power*/
static struct proc_dir_entry *usb_power_entry = NULL;

static struct proc_dir_entry *workmode_entry = NULL;

static struct proc_dir_entry *wifi_button_entry = NULL;

void ath_gpio_config_int(int gpio, 
				ath_gpio_int_type_t type, 
				ath_gpio_int_pol_t polarity)
{
	u32 val;

	/*
	 * allow edge sensitive/rising edge too
	 */
	if (type == INT_TYPE_LEVEL) {
		/* level sensitive */
		ath_reg_rmw_set(ATH_GPIO_INT_TYPE, (1 << gpio));
	} else {
		/* edge triggered */
		val = ath_reg_rd(ATH_GPIO_INT_TYPE);
		val &= ~(1 << gpio);
		ath_reg_wr(ATH_GPIO_INT_TYPE, val);
	}

	if (polarity == INT_POL_ACTIVE_HIGH) {
		ath_reg_rmw_set(ATH_GPIO_INT_POLARITY, (1 << gpio));
	} else {
		val = ath_reg_rd(ATH_GPIO_INT_POLARITY);
		val &= ~(1 << gpio);
		ath_reg_wr(ATH_GPIO_INT_POLARITY, val);
	}

	ath_reg_rmw_set(ATH_GPIO_INT_ENABLE, (1 << gpio));
}

void ath_gpio_config_output(int gpio)
{
#ifdef CONFIG_MACH_AR934x
	ath_reg_rmw_clear(ATH_GPIO_OE, (1 << gpio));
#else
	ath_reg_rmw_set(ATH_GPIO_OE, (1 << gpio));
#endif
}
EXPORT_SYMBOL(ath_gpio_config_output);

void ath_gpio_config_input(int gpio)
{
#ifdef CONFIG_MACH_AR934x
	ath_reg_rmw_set(ATH_GPIO_OE, (1 << gpio));
#else
	ath_reg_rmw_clear(ATH_GPIO_OE, (1 << gpio));
#endif
}

void ath_gpio_out_val(int gpio, int val)
{
	if (val & 0x1) {
		ath_reg_rmw_set(ATH_GPIO_OUT, (1 << gpio));
	} else {
		ath_reg_rmw_clear(ATH_GPIO_OUT, (1 << gpio));
	}
}

EXPORT_SYMBOL(ath_gpio_out_val);

int ath_gpio_in_val(int gpio)
{
	return ((1 << gpio) & (ath_reg_rd(ATH_GPIO_IN)));
}

static void
ath_gpio_intr_enable(unsigned int irq)
{
	ath_reg_rmw_set(ATH_GPIO_INT_MASK,
				(1 << (irq - ATH_GPIO_IRQ_BASE)));
}

static void
ath_gpio_intr_disable(unsigned int irq)
{
	ath_reg_rmw_clear(ATH_GPIO_INT_MASK,
				(1 << (irq - ATH_GPIO_IRQ_BASE)));
}

static unsigned int
ath_gpio_intr_startup(unsigned int irq)
{
	ath_gpio_intr_enable(irq);
	return 0;
}

static void
ath_gpio_intr_shutdown(unsigned int irq)
{
	ath_gpio_intr_disable(irq);
}

static void
ath_gpio_intr_ack(unsigned int irq)
{
	ath_gpio_intr_disable(irq);
}

static void
ath_gpio_intr_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		ath_gpio_intr_enable(irq);
}

static int
ath_gpio_intr_set_affinity(unsigned int irq, const struct cpumask *dest)
{
	/*
	 * Only 1 CPU; ignore affinity request
	 */
	return 0;
}

struct irq_chip /* hw_interrupt_type */ ath_gpio_intr_controller = {
	.name = "ATH GPIO",
	.startup = ath_gpio_intr_startup,
	.shutdown = ath_gpio_intr_shutdown,
	.enable = ath_gpio_intr_enable,
	.disable = ath_gpio_intr_disable,
	.ack = ath_gpio_intr_ack,
	.end = ath_gpio_intr_end,
	.eoi = ath_gpio_intr_end,
	.set_affinity = ath_gpio_intr_set_affinity,
};

void ath_gpio_irq_init(int irq_base)
{
	int i;

	for (i = irq_base; i < irq_base + ATH_GPIO_IRQ_COUNT; i++) {
		irq_desc[i].status = IRQ_DISABLED;
		irq_desc[i].action = NULL;
		irq_desc[i].depth = 1;
		//irq_desc[i].chip = &ath_gpio_intr_controller;
		set_irq_chip_and_handler(i, &ath_gpio_intr_controller,
					 handle_percpu_irq);
	}
}

/*
 *  USB GPIO control
 */

void ap_usb_led_on(void)
{
#ifdef AP_USB_LED_GPIO
	if (AP_USB_LED_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_out_val(AP_USB_LED_GPIO, USB_LED_ON);
	}
#endif
}

EXPORT_SYMBOL(ap_usb_led_on);

void ap_usb_led_off(void)
{
#ifdef AP_USB_LED_GPIO
	if (AP_USB_LED_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_out_val(AP_USB_LED_GPIO, USB_LED_OFF);
	}
#endif
}

EXPORT_SYMBOL(ap_usb_led_off);


/*changed by hujia.*/
/*void register_simple_config_callback (void *callback, void *arg)*/
void register_simple_config_callback (void *callback, void *arg, void *arg2)
{
    registered_cb = (sc_callback_t) callback;
    cb_arg = arg;
    /* add by hujia.*/
    cb_pushtime=arg2;
    /*end add.*/
}
EXPORT_SYMBOL(register_simple_config_callback);

void unregister_simple_config_callback (void)
{
    registered_cb = NULL;
    cb_arg = NULL;
}
EXPORT_SYMBOL(unregister_simple_config_callback);

#if 0
/*
 * Irq for front panel SW jumpstart switch
 * Connected to XSCALE through GPIO4
 */
irqreturn_t jumpstart_irq(int cpl, void *dev_id, struct pt_regs *regs)
{
    if (ignore_pushbutton) {
        ath_gpio_config_int (JUMPSTART_GPIO,INT_TYPE_LEVEL,
                                INT_POL_ACTIVE_HIGH);
            ignore_pushbutton = 0;
            return IRQ_HANDLED;
    }

    ath_gpio_config_int (JUMPSTART_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);
    ignore_pushbutton = 1;

    printk ("Jumpstart button pressed.\n");

	/*changed by hujia.*/
    if (registered_cb ) {
        /*return registered_cb (cpl, cb_arg, regs);*/
		return registered_cb (cpl, cb_arg, regs, cb_pushtime);
    }
    return IRQ_HANDLED;
}
#endif

static int ignore_rstbutton = 1;
	
/* irq handler for reset button */
irqreturn_t rst_irq(int cpl, void *dev_id, struct pt_regs *regs)
{
    if (ignore_rstbutton)
	{
		ath_gpio_config_int(RST_DFT_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_HIGH);
		ignore_rstbutton = 0;

		mod_timer(&rst_timer, jiffies + RST_HOLD_TIME * HZ);

		return IRQ_HANDLED;
	}

	ath_gpio_config_int (RST_DFT_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);
	ignore_rstbutton = 1;

	printk("Reset button pressed.\n");

	/* mark reset status, by zjg, 12Apr10 */
	if (registered_cb && !bBlockWps && l_bMultiUseResetButton && l_bWaitForQss) 
	{
		/*return registered_cb (cpl, cb_arg, regs);*/
		return registered_cb(cpl, cb_arg, regs, cb_pushtime);
	}

	return IRQ_HANDLED;
}

void check_rst(unsigned long nothing)
{
	if (!ignore_rstbutton)
	{
		printk ("restoring factory default...\n");
		counter ++;

		/* to mark reset status, forbid QSS, added by zjg, 12Apr10 */
		if (l_bMultiUseResetButton)
		{
			l_bWaitForQss	= 0;
		}
    }
}


/* irq handler for wifi switch */
static irqreturn_t wifi_sw_irq(int cpl, void *dev_id, struct pt_regs *regs)
{
    if (ignore_wifibutton)
	{
		ath_gpio_config_int (WIFI_RADIO_SW_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_HIGH);
		ignore_wifibutton = 0;

		mod_timer(&wifi_button_timer, jiffies + 1 * HZ);	/* 1sec */

		if (l_bMultiUseWifiButton)
		{
			l_bWaitForQss = 1;
		}
		
		return IRQ_HANDLED;
    }

	ath_gpio_config_int (WIFI_RADIO_SW_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);
	ignore_wifibutton = 1;

	printk("WIFI button pressed.\n");

	if (registered_cb && !bBlockWps && l_bMultiUseWifiButton && l_bWaitForQss) 
	{
		return registered_cb(cpl, cb_arg, regs, cb_pushtime);
	}

	return IRQ_HANDLED;
}

static void wifi_sw_check(unsigned long nothing)
{
	/* if user keep push button more than 2s, 
	ignore_wifibutton will keep 0, 
	or ignore_wifibutton will be 1 */
	if (!ignore_wifibutton)
	{
		printk ("Switch Wi-Fi...\n");
    	wifi_button_flag++;

		if (l_bMultiUseWifiButton)
		{
			l_bWaitForQss	= 0;
		}
    }
}

static int multi_use_reset_button_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return sprintf (page, "%d\n", l_bMultiUseResetButton);
}

static int multi_use_reset_button_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	/* only admit "0" or "1" */
	if ((val < 0) || (val > 1))
		return -EINVAL;	

	l_bMultiUseResetButton = val;
	
	return count;

}

static int multi_use_wifi_button_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return sprintf (page, "%d\n", l_bMultiUseWifiButton);
}

static int multi_use_wifi_button_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	/* only admit "0" or "1" */
	if ((val < 0) || (val > 1))
		return -EINVAL;	

	l_bMultiUseWifiButton = val;
	
	return count;

}

static int push_button_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return 0;
}

static int push_button_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    if (registered_cb) {
		/*changed by hujia.*/
		/* registered_cb (0, cb_arg, 0);*/
		registered_cb (0, cb_arg, 0,NULL);
    }
    return count;
}

static int usb_led_blink_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return sprintf (page, "%d\n", g_usbLedBlinkCountDown);
}

static int usb_led_blink_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	if ((val < 0) || (val > 1))
		return -EINVAL;

	g_usbLedBlinkCountDown = val;

	return count;
}

/*added by ZQQ,10.06.02*/
static int usb_power_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return 0;
}

static int usb_power_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val = 0;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	if ((val < 0) || (val > 1))
		return -EINVAL;

	printk("%s %d: write gpio:value = %d\r\n",__FUNCTION__,__LINE__,val);
	
	if (USB_POWER_ON == val)
	{
		ath_gpio_out_val(USB_POWER_SW_GPIO, USB_POWER_ON);
	}
	else
	{
		ath_gpio_out_val(USB_POWER_SW_GPIO, USB_POWER_OFF);
	}
	
	return count;
}

static int workmode_proc_read(char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
	u_int32_t ret = 0;
	
#if (WORKMODE_SWITCH_1ST_GPIO >= ATH_GPIO_MIN) && (WORKMODE_SWITCH_2ND_GPIO >= ATH_GPIO_MIN)
	u_int32_t reg_value1 = 0;
	u_int32_t reg_value2 = 0;

	reg_value1 = ath_gpio_in_val(WORKMODE_SWITCH_1ST_GPIO);
	reg_value2 = ath_gpio_in_val(WORKMODE_SWITCH_2ND_GPIO);

	reg_value1 = reg_value1 >> WORKMODE_SWITCH_1ST_GPIO;
	reg_value2 = reg_value2 >> WORKMODE_SWITCH_2ND_GPIO;

	ret = (reg_value2 << 1) | reg_value1;
#endif
	
    return sprintf (page, "%d\n", ret);
}

static int workmode_proc_write(struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val = 0;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	if ((val < 0) || (val > 1))
		return -EINVAL;

	printk("%s %d: write value = %d\r\n",__FUNCTION__,__LINE__,val);
	
	return count;
}

static int wifi_button_proc_read(char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
	u_int32_t ret = 0;

#if WIFI_RADIO_SW_GPIO >= ATH_GPIO_MIN
	u_int32_t reg_value1 = 0;

	reg_value1 = ath_gpio_in_val(WIFI_RADIO_SW_GPIO);

	reg_value1 = reg_value1 >> WIFI_RADIO_SW_GPIO;

	ret = reg_value1;
#endif
	
    return sprintf (page, "%d\n", ret);
}

static int wifi_button_proc_write(struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val = 0;

	if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

	if ((val < 0) || (val > 1))
		return -EINVAL;

	printk("%s %d: write value = %d\r\n",__FUNCTION__,__LINE__,val);
	
	return count;
}


typedef enum {
        LED_STATE_OFF   =       0,
        LED_STATE_GREEN =       1,
        LED_STATE_YELLOW =      2,
        LED_STATE_ORANGE =      3,
        LED_STATE_MAX =         4
} led_state_e;

static led_state_e gpio_tricolorled = LED_STATE_OFF;

static int gpio_tricolor_led_read (char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
    return sprintf (page, "%d\n", gpio_tricolorled);
}

static int gpio_tricolor_led_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val, green_led_onoff = 0, yellow_led_onoff = 0;

    if (sscanf(buf, "%d", &val) != 1)
        return -EINVAL;

    if (val >= LED_STATE_MAX)
        return -EINVAL;

    if (val == gpio_tricolorled)
    return count;

    switch (val) {
        case LED_STATE_OFF :
                green_led_onoff = OFF;   /* both LEDs OFF */
                yellow_led_onoff = OFF;
                break;

        case LED_STATE_GREEN:
                green_led_onoff = ON;    /* green ON, Yellow OFF */
                yellow_led_onoff = OFF;
                break;

        case LED_STATE_YELLOW:
                green_led_onoff = OFF;   /* green OFF, Yellow ON */
                yellow_led_onoff = ON;
                break;

        case LED_STATE_ORANGE:
                green_led_onoff = ON;    /* both LEDs ON */
                yellow_led_onoff = ON;
                break;
	}

    ath_gpio_out_val (TRICOLOR_LED_GREEN_PIN, green_led_onoff);
    //ath_gpio_out_val (TRICOLOR_LED_YELLOW_PIN, yellow_led_onoff);
    gpio_tricolorled = val;

    return count;
}


static int create_simple_config_led_proc_entry (void)
{
    if (simple_config_entry != NULL) {
        printk ("Already have a proc entry for /proc/simple_config!\n");
        return -ENOENT;
    }

    simple_config_entry = proc_mkdir("simple_config", NULL);
    if (!simple_config_entry)
        return -ENOENT;

    simulate_push_button_entry = create_proc_entry ("push_button", 0644,
                                                      simple_config_entry);
    if (!simulate_push_button_entry)
        return -ENOENT;

    simulate_push_button_entry->write_proc = push_button_write;
    simulate_push_button_entry->read_proc = push_button_read;

	/* added by zjg, 12Apr10 */
	multi_use_reset_button_entry = create_proc_entry ("multi_use_reset_button", 0644,
                                                      simple_config_entry);
    if (!multi_use_reset_button_entry)
        return -ENOENT;

    multi_use_reset_button_entry->write_proc	= multi_use_reset_button_write;
    multi_use_reset_button_entry->read_proc 	= multi_use_reset_button_read;
	/* end added */

	/* added by zjin for multi-use wifi button, 27Apr2012 */
	multi_use_wifi_button_entry = create_proc_entry ("multi_use_wifi_button", 0644,
                                                      simple_config_entry);
    if (!multi_use_wifi_button_entry)
        return -ENOENT;

    multi_use_wifi_button_entry->write_proc	= multi_use_wifi_button_write;
    multi_use_wifi_button_entry->read_proc 	= multi_use_wifi_button_read;
	/* end added */

    tricolor_led_entry = create_proc_entry ("tricolor_led", 0644,
                                            simple_config_entry);
    if (!tricolor_led_entry)
        return -ENOENT;

    tricolor_led_entry->write_proc = gpio_tricolor_led_write;
    tricolor_led_entry->read_proc = gpio_tricolor_led_read;

	/* for usb led blink */
	usb_led_blink_entry = create_proc_entry ("usb_blink", 0666,
                                                      simple_config_entry);
	if (!usb_led_blink_entry)
		return -ENOENT;
	
    usb_led_blink_entry->write_proc = usb_led_blink_write;
    usb_led_blink_entry->read_proc = usb_led_blink_read;
	
	/*added by ZQQ, 10.06.02 for usb power*/
	usb_power_entry = create_proc_entry("usb_power", 0666, simple_config_entry);
	if(!usb_power_entry)
		return -ENOENT;

	usb_power_entry->write_proc = usb_power_write;
	usb_power_entry->read_proc = usb_power_read;
	/*end added*/

	/* workmode switch entry */
	workmode_entry = create_proc_entry("workmode", 0666, simple_config_entry);
	if(!workmode_entry)
		return -ENOENT;
	workmode_entry->write_proc = workmode_proc_write;
	workmode_entry->read_proc = workmode_proc_read;

	/* wifi switch entry */
	wifi_button_entry = create_proc_entry("wifi_button", 0666, simple_config_entry);
	if(!wifi_button_entry)
		return -ENOENT;
	wifi_button_entry->write_proc = wifi_button_proc_write;
	wifi_button_entry->read_proc = wifi_button_proc_read;

    /* configure gpio as outputs */
    ath_gpio_config_output (TRICOLOR_LED_GREEN_PIN); 
    //ath_gpio_config_output (TRICOLOR_LED_YELLOW_PIN); 

    /* switch off the led */
	/* TRICOLOR_LED_GREEN_PIN is poll up, so ON is OFF modified by tiger 09/07/15 */
    ath_gpio_out_val(TRICOLOR_LED_GREEN_PIN, ON);
    //ath_gpio_out_val(TRICOLOR_LED_YELLOW_PIN, OFF);

    return 0;
}



/******* begin ioctl stuff **********/
#ifdef CONFIG_GPIO_DEBUG
void print_gpio_regs(char* prefix)
{
	printk("\n-------------------------%s---------------------------\n", prefix);
	printk("ATH_GPIO_OE:%#X\n", ath_reg_rd(ATH_GPIO_OE));
	printk("ATH_GPIO_IN:%#X\n", ath_reg_rd(ATH_GPIO_IN));
	printk("ATH_GPIO_OUT:%#X\n", ath_reg_rd(ATH_GPIO_OUT));
	printk("ATH_GPIO_SET:%#X\n", ath_reg_rd(ATH_GPIO_SET));
	printk("ATH_GPIO_CLEAR:%#X\n", ath_reg_rd(ATH_GPIO_CLEAR));
	printk("ATH_GPIO_INT_ENABLE:%#X\n", ath_reg_rd(ATH_GPIO_INT_ENABLE));
	printk("ATH_GPIO_INT_TYPE:%#X\n", ath_reg_rd(ATH_GPIO_INT_TYPE));
	printk("ATH_GPIO_INT_POLARITY:%#X\n", ath_reg_rd(ATH_GPIO_INT_POLARITY));
	printk("ATH_GPIO_INT_PENDING:%#X\n", ath_reg_rd(ATH_GPIO_INT_PENDING));
	printk("ATH_GPIO_INT_MASK:%#X\n", ath_reg_rd(ATH_GPIO_INT_MASK));
	printk("\n-------------------------------------------------------\n");
	}
#endif

/* ioctl for reset default detection and system led switch*/
int ath_gpio_ioctl(struct inode *inode, struct file *file,  unsigned int cmd, unsigned long arg)
{
//	int i;
	int* argp = (int *)arg;

	if (_IOC_TYPE(cmd) != ATH_GPIO_MAGIC ||
		_IOC_NR(cmd) < ATH_GPIO_IOCTL_BASE ||
		_IOC_NR(cmd) > ATH_GPIO_IOCTL_MAX)
	{
		printk("type:%d nr:%d\n", _IOC_TYPE(cmd), _IOC_NR(cmd));
		printk("ath_gpio_ioctl:unknown command\n");
		return -1;
	}

	switch (cmd)
	{
	case ATH_GPIO_BTN_READ:
		*argp = counter;
		counter = 0;
		break;
			
	case ATH_GPIO_WIFI_SW_READ:
		
	#if WIFI_RADIO_SW_GPIO >= ATH_GPIO_MIN
		*argp = (ath_gpio_in_val(WIFI_RADIO_SW_GPIO)) >> WIFI_RADIO_SW_GPIO;
	#endif
		break;

	#ifdef CONFIG_GPIO_DEBUG
		print_gpio_regs("");
	#endif
			
		break;
			
	case ATH_GPIO_LED_WRITE:
		if (unlikely(bBlockWps))
			bBlockWps = 0;

		if (SYS_LED_GPIO >= ATH_GPIO_MIN)
		{
			ath_gpio_out_val(SYS_LED_GPIO, *argp);
		}
		break;

	case ATH_GPIO_USB_LED1_WRITE:
		if (AP_USB_LED_GPIO >= ATH_GPIO_MIN)
		{
			ath_gpio_out_val(AP_USB_LED_GPIO, *argp);
		}
		break;

	case ATH_GPIO_INET_LED_WRITE:
		if (INET_LED_GPIO >= ATH_GPIO_MIN)
		{
			ath_gpio_out_val(INET_LED_GPIO, *argp);
		}
		break;

	case ATH_GPIO_WIFI_BTN_READ:
		*argp = wifi_button_flag;
		wifi_button_flag = 0;
		break;

	default:
		printk("command not supported\n");
		return -1;
	}


	return 0;
}


int ath_gpio_open (struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	int devnum = minor; //>> 1;
	struct mtd_info *mtd;

	if ((filp->f_mode & 2) && (minor & 1))
{
		printk("You can't open the RO devices RW!\n");
		return -EACCES;
}

	mtd = get_mtd_device(NULL, devnum);   
	if (!mtd)
{
		printk("Can not open mtd!\n");
		return -ENODEV;	
	}
	filp->private_data = mtd;
	return 0;

}

/* struct for cdev */
struct file_operations gpio_device_op =
{
	.owner = THIS_MODULE,
	.ioctl = ath_gpio_ioctl,
	.open = ath_gpio_open,
};

/* struct for ioctl */
static struct cdev gpio_device_cdev =
{
	.owner  = THIS_MODULE,
	.ops	= &gpio_device_op,
};
/******* end  ioctl stuff **********/

#if 0
static void ath_config_eth_led(void)
{
	uint32_t reg_value;

	/*WAN, GPIO19, DS3*/
	ath_gpio_config_output(19);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION4, 0xff<<24);
	reg_value = ath_reg_rd(ATH_GPIO_OUT_FUNCTION4);
	reg_value = reg_value | (41 << 24);
	ath_reg_wr(ATH_GPIO_OUT_FUNCTION4, reg_value);
	printk("ATH_GPIO_OUT_FUNCTION4: 0x%x\n", ath_reg_rd(ATH_GPIO_OUT_FUNCTION4));

	/* LAN1, GPIO20, DS4 */
	ath_gpio_config_output(20);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION5, 0xff<<0);
	reg_value = ath_reg_rd(ATH_GPIO_OUT_FUNCTION5);
	reg_value = reg_value | (42 << 0);
	ath_reg_wr(ATH_GPIO_OUT_FUNCTION5, reg_value);
	/* it's ok */

	/* LAN2, GPIO21, DS5 */
	ath_gpio_config_output(21);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION5, 0xff<<8);
	reg_value = ath_reg_rd(ATH_GPIO_OUT_FUNCTION5);
	reg_value = reg_value | (43 << 8);
	ath_reg_wr(ATH_GPIO_OUT_FUNCTION5, reg_value);
	printk("ATH_GPIO_OUT_FUNCTION5: 0x%x\n", ath_reg_rd(ATH_GPIO_OUT_FUNCTION5));

	/* LAN3, GPIO12, DS6 */
	ath_gpio_config_output(12);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION3, 0xff<<0);
	reg_value = ath_reg_rd(ATH_GPIO_OUT_FUNCTION3);
	reg_value = reg_value | (44 << 0);
	ath_reg_wr(ATH_GPIO_OUT_FUNCTION3, reg_value);
	/* it's ok */
	

	/* LAN4, GPIO18, DS7 */
	ath_gpio_config_output(18);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION4, 0xff<<16);
	reg_value = ath_reg_rd(ATH_GPIO_OUT_FUNCTION4);
	reg_value = reg_value | (45 << 16);
	ath_reg_wr(ATH_GPIO_OUT_FUNCTION4, reg_value);
	printk("ATH_GPIO_OUT_FUNCTION4: 0x%x\n", ath_reg_rd(ATH_GPIO_OUT_FUNCTION4));


}
#endif

int __init ath_simple_config_init(void)
{
    int req;

	/* restore factory default and system led */
	dev_t dev;
    int rt;
    int ath_gpio_major = gpio_major;
    int ath_gpio_minor = gpio_minor;

	init_timer(&rst_timer);
	rst_timer.function = check_rst;

	init_timer(&wifi_button_timer);
	wifi_button_timer.function = wifi_sw_check;

	/* config gpio15, 14, 11, 4, 16, 17 as normal gpio function */
	/* gpio4 */
	ath_reg_rmw_clear(ATH_GPIO_FUNCTIONS, 1<<6);
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION1, 0xff);
	/* gpio11 */
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION2, 0xff<<24);
	/* gpio14 */
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION3, 0xff<<16);
	/* gpio15 */
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION3, 0xff<<24);
	/* gpio16 */
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION4, 0xff<<0);
	/* gpio17 */
	ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION4, 0xff<<8);

	if (INET_LED_GPIO >= ATH_GPIO_MIN)
	{
		/* gpio18 */
		ath_reg_rmw_clear(ATH_GPIO_OUT_FUNCTION4, 0xff<<16);
	}

#if 0
	/* This is NECESSARY, lsz 090109 */
	ath_gpio_config_input(JUMPSTART_GPIO);

    /* configure JUMPSTART_GPIO as level triggered interrupt */
    ath_gpio_config_int (JUMPSTART_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);

    req = request_irq (ATH_GPIO_IRQn(JUMPSTART_GPIO), jumpstart_irq, 0,
                       "SW_JUMPSTART", NULL);
    if (req != 0)
	{
        printk (KERN_ERR "unable to request IRQ for SWJUMPSTART GPIO (error %d)\n", req);
    }
#endif
    create_simple_config_led_proc_entry ();

	ath_gpio_config_input(RST_DFT_GPIO);

	/* configure GPIO RST_DFT_GPIO as level triggered interrupt */
    ath_gpio_config_int (RST_DFT_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);

    rt = request_irq (ATH_GPIO_IRQn(RST_DFT_GPIO), rst_irq, 0,
                       "RESTORE_FACTORY_DEFAULT", NULL);
    if (rt != 0)
	{
        printk (KERN_ERR "unable to request IRQ for RESTORE_FACTORY_DEFAULT GPIO (error %d)\n", rt);
    }

	/* wifi switch! */
	if (WIFI_RADIO_SW_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_config_input(WIFI_RADIO_SW_GPIO);

		/* configure GPIO WIFI_RADIO_SW_GPIO as level triggered interrupt */
	    ath_gpio_config_int (WIFI_RADIO_SW_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);

	    req = request_irq (ATH_GPIO_IRQn(WIFI_RADIO_SW_GPIO), wifi_sw_irq, 0,
	                       "WIFI_RADIO_SWITCH", NULL);
	    if (req != 0)
		{
	        printk (KERN_ERR "unable to request IRQ for WIFI_RADIO_SWITCH GPIO (error %d)\n", req);
	    }
	}

	/* Work mode switchs!
	 * 14	16
	 	L	H	3G
	 	H	L	Router/APC Router
	 	H	H	AP
	 */
	 if ((WORKMODE_SWITCH_1ST_GPIO >= ATH_GPIO_MIN) 
	 	&& (WORKMODE_SWITCH_2ND_GPIO >= ATH_GPIO_MIN))
	 {
		ath_gpio_config_input(WORKMODE_SWITCH_1ST_GPIO);
#if 0
		/* configure GPIO WORKMODE_SWITCH_1ST_GPIO as level triggered interrupt */
	    ath_gpio_config_int (WORKMODE_SWITCH_1ST_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);

	    req = request_irq (ATH_GPIO_IRQn(WORKMODE_SWITCH_1ST_GPIO), workmode_sw_2nd_irq, 0,
	                       "WORKMODE_SWITCH_1ST", NULL);
	    if (req != 0)
		{
	        printk (KERN_ERR "unable to request IRQ for WORKMODE_SWITCH_1ST GPIO (error %d)\n", req);
	    }
#endif

		/* step2 gpio */
		ath_gpio_config_input(WORKMODE_SWITCH_2ND_GPIO);
#if 0
		/* configure GPIO WORKMODE_SWITCH_2ND_GPIO as level triggered interrupt */
	    ath_gpio_config_int (WORKMODE_SWITCH_2ND_GPIO, INT_TYPE_LEVEL, INT_POL_ACTIVE_LOW);

	    req = request_irq (ATH_GPIO_IRQn(WORKMODE_SWITCH_2ND_GPIO), workmode_sw_2nd_irq, 0,
	                       "WORKMODE_SWITCH_2ND", NULL);
	    if (req != 0)
		{
	        printk (KERN_ERR "unable to request IRQ for WORKMODE_SWITCH_2ND GPIO (error %d)\n", req);
	    }
#endif
	 }
	
	/* Create char device for gpio */
    if (ath_gpio_major)
	{
        dev = MKDEV(ath_gpio_major, ath_gpio_minor);
        rt = register_chrdev_region(dev, 1, "ar7240_gpio_chrdev");
    }
	else
	{
        rt = alloc_chrdev_region(&dev, ath_gpio_minor, 1, "ar7240_gpio_chrdev");
        ath_gpio_major = MAJOR(dev);
    }

    if (rt < 0)
	{
        printk(KERN_WARNING "ar7240_gpio_chrdev : can`t get major %d\n", ath_gpio_major);
        return rt;
    }

    cdev_init (&gpio_device_cdev, &gpio_device_op);
    rt = cdev_add(&gpio_device_cdev, dev, 1);
	
    if (rt < 0) 
		printk(KERN_NOTICE "Error %d adding ar7240_gpio_chrdev ", rt);

	/* Configure GPIOs default status */
	if (AP_USB_LED_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_config_output(AP_USB_LED_GPIO);
		ath_gpio_out_val(AP_USB_LED_GPIO, USB_LED_OFF);
	}

	if (SYS_LED_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_config_output(SYS_LED_GPIO);
		ath_gpio_out_val(SYS_LED_GPIO, SYS_LED_OFF);
	}

	if (INET_LED_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_config_output(INET_LED_GPIO);
		ath_gpio_out_val(INET_LED_GPIO, INET_LED_OFF);
	}

	if (USB_POWER_SW_GPIO >= ATH_GPIO_MIN)
	{
		ath_gpio_config_output(USB_POWER_SW_GPIO);
		ath_gpio_out_val(USB_POWER_SW_GPIO, 1);
	}
	
	ath_gpio_out_val (TRICOLOR_LED_GREEN_PIN, OFF);

	//ath_config_eth_led();

    return 0;
}

subsys_initcall(ath_simple_config_init);
