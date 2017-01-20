#ifndef __AW2013_LED_H_
#define __AW2013_LED_H_

#define 	AW2013_NAME             "AW2013_LED"
#define 	AW_I2C_ADDR     	(0x45)
#define 	AW2013_I2C_MAX_LOOP 	50
#define  	SHAWN_I2C_TEST
#define 	DEFINE_TIMER
#define 	REGISTER_CDEV
#define 	DEVICE_NAME             "aw2013_cdev"
#define     AW2013_IRQ_MODE       (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)
#define     aw2013_msg(...)    do {printk("[AW2013_LED]: "__VA_ARGS__);} while(0)


int color = 0;
int led_no = 0;
int bright_range;
bool timer_flag = 0;

static __u32 twi_id = 1;
struct timer_list aw2013_timer;

struct AW2013_led_data {
	
	struct input_dev	*input_dev;
	struct AW2013_event 	*event;
	struct work_struct 	work;
	struct workqueue_struct *queue;
	struct timer_list touch_timer;
	struct timer_list charge_detect;
	int (*power)(struct AW2013_led_data * ts, int on);	
	int irq;
};
struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
	.name = NULL,
	.int_number = 0,
};

struct i2c_dev{
	struct list_head list;	
	struct i2c_adapter *adap;
	struct device *dev;
};

#ifdef  REGISTER_CDEV
struct aw2013_dev {
    struct cdev         cdev;
    dev_t               devno;
    struct class*       class;
};

static struct aw2013_dev* aw2013_devp = NULL;

#endif
static struct i2c_client *this_client;
static struct AW2013_led_data *aw2013_work;
#endif
