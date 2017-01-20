/**************************************************************************
*  aw2013_led.c
*  AW2013 zicoo sample code version 1.0
*  Create Date : 2015/12/07
*  Modify Date : 
*  Create by   : shawn
**************************************************************************/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/earlysuspend.h>
#include <linux/input/mt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/init-input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include "aw2013_led.h"
  
static char *brelight_para = "brelight_para";
//aw2013 function info
struct brelight_func_info {
	int  brelight_used;
  char *module_name;
  int  brelight_int_pin;
};

 struct brelight_func_info  aw2013_info;

static int aw2013_pm_get_res(void)
{
	script_item_value_type_e type;
	script_item_u val; 
    struct gpio_config  *gpio_p = NULL;
	//aw2013_info.brelight_int_pin = -1;

	type = script_get_item(brelight_para, "brelight_used", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		aw2013_msg("failed to fetch aw2013 configuration!\n");
		return -1;
	}
	if (!val.val) {
		aw2013_msg("no aw2013 used in configuration!\n");
		return -1;
	}
	aw2013_info.brelight_used = val.val;
	
    type = script_get_item(brelight_para, "brelight_int_pin", &val);
    if (SCIRPT_ITEM_VALUE_TYPE_PIO!=type)
        aw2013_msg("get brelight_int_pin gpio failed\n");
    else {
        gpio_p = &val.gpio;
        aw2013_info.brelight_int_pin = gpio_p->gpio;
        aw2013_msg("brelight_int_pin:%d\n",aw2013_info.brelight_int_pin); 
    }

	return 0;
}

int AW2013_led_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = client->adapter;
    aw2013_msg("adapter->nr=%d\n",adapter->nr);

    if(twi_id == adapter->nr)
    {
        aw2013_msg("%s: Detected chip %s at adapter %d, address 0x%02x\n",
                __func__, AW2013_NAME, i2c_adapter_id(adapter), client->addr);
        printk("aw2013 used twi_id%d\n",twi_id);
        strlcpy(info->type, AW2013_NAME, I2C_NAME_SIZE);

    }else{
        return -ENODEV;
    }
    aw2013_msg("[%s]:I2C detect ok!\n",__func__);
    return 0;
}


static int i2c_write_bytes(struct i2c_client *client, char * buf ,int count)
{
	unsigned char ret;

	ret = i2c_master_send(this_client, buf, count);

	if (ret != count) 
	{
		aw2013_msg("test_i2c_write failed  %s \r\n", __func__);
		dev_err(&this_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
        return -EINVAL;
	}
    return 0;
}


unsigned char i2c_read_bytes(unsigned char regaddr) 
{
	unsigned char rdbuf[1], wrbuf[1], ret, i;

	wrbuf[0] = regaddr;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++) 
	{
		ret = i2c_master_send(this_client, wrbuf, 1);
		if (ret == 1)
			break;
	}

	ret = i2c_master_recv(this_client, rdbuf, 1);

	if (ret != 1)
	{
		aw2013_msg("i2c_read_bytes failed  %s \r\n", __func__);
		dev_err(&this_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
	}
	return rdbuf[0];
}



static u32 AW2013_i2c_write(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}


static u32 AW2013_reg_read(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}


static int aw2013_get_gpio_val(u32 gpio)
{
    u32 err; 
    err = __gpio_get_value(gpio);
    if(err < 0){
        aw2013_msg("failed get gpio val!\n");
        return -EINVAL;
    }
    return err;
}


static const unsigned short normal_i2c[2] = {AW_I2C_ADDR,I2C_CLIENT_END};


/*LED no fade/flash*/
static void light_on(u32 led_no)
{
    char buf[2];
    buf[0]=0x01;
	buf[1]=0x01;
	i2c_write_bytes(this_client,buf,2);

	buf[0]=0x31+led_no;
	buf[1]=0x03;
	i2c_write_bytes(this_client,buf,2);

    buf[0]=0x34+led_no;
	buf[1]=bright_range;
	i2c_write_bytes(this_client,buf,2);


	buf[0]=0x30;
	buf[1]=1<<led_no;
	i2c_write_bytes(this_client,buf,2);

}

/*LED fade on/off*/
void light_fade(unsigned int led_no)
{
	char buf[2];
	i2c_read_bytes(0x55);
	buf[0]=0x01;
	buf[1]=0x01;
	i2c_write_bytes(this_client,buf,2);

	buf[0]=0x31+led_no;
	buf[1]=0x77;
	i2c_write_bytes(this_client,buf,2);
	
    buf[0]=0x34+led_no;
	buf[1]=bright_range;
	i2c_write_bytes(this_client,buf,2);

	buf[0]=0x37+led_no*3;
	buf[1]=0x42;
	i2c_write_bytes(this_client,buf,2);

	buf[0]=0x38+led_no*3;
	buf[1]=0x42;
	i2c_write_bytes(this_client,buf,2);

	buf[0]=0x39+led_no*3;
	buf[1]=0x00;
	i2c_write_bytes(this_client,buf,2);
	
    buf[0]=0x30;
	buf[1]=1<<led_no;
	i2c_write_bytes(this_client,buf,2);	
}

/*light off*/
static void light_off(unsigned int id)
{
    char buf[2];
    buf[0]=0x30;
    buf[1]=0x00;
    i2c_write_bytes(this_client,buf,2);
}

/*LED keep lightflash*/
static void light_flash(void)
{
    light_on(led_no);
    msleep(25);
    light_off(led_no);
}

/*
 *color:  RED_LED=0; GREEN_LED=1; BLUE_LED=2
 *led_no: light_on=0,light_flash=1,light_fade=2
*/
static long aw2013_ioctl (u32 color, u32 led_no)
{
    switch(color){
        case 0:
            switch(led_no){
                case 0:
                    msleep(25);
                    light_on(0);
                    break;
                case 2:
                    msleep(25);
                    light_fade(0);						
                    break;
            }
            break;
        case 1:
            switch(led_no){	
                case 0:
                    msleep(25);
                    light_on(1);
                    break;
                case 2:
                    msleep(25);
                    light_fade(1);						
                    break;	
            }	
            break;
        case 2: 
            switch(led_no){
                case 0:
                    msleep(25);
                    light_on(2);
                    break;
                case 2:
                    msleep(25);
                    light_fade(2);
                    break;			
            }
            break;
    }
    return 0;
}

static void aw2013_suspend(struct AW2013_led_data *led)
{
    return;
}


static void aw2013_resume(struct i2c_client *client)
{
	return;
}

#ifdef REGISTER_CDEV
static int aw2013_open (struct inode *inode, struct file *filp)
{
    int err;
    char buf[2];
    buf[0]=0x01;
	buf[1]=0x01;
    err = i2c_write_bytes(this_client,buf,2);
    if(err){
        aw2013_msg("i2c test fail!!!\n");
        return -EINVAL;
    }
	return 0;
}


static int aw2013_read (struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    int err;
    aw2013_msg("aw2013 send data to usr!\n");

    err = aw2013_get_gpio_val(aw2013_info.brelight_int_pin);
    if(err){
        aw2013_msg("failed send data to user");
        return -EFAULT;
    }
    return 0;
}



static int aw2013_write (struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    char buf1[4] = {0};
    int err = -1;

    err = copy_from_user(buf1, buf, size);
    //aw2013_msg("received: %02x,%02x,%02x\n", buf1[0], buf1[1], buf1[2]);
    if(0 == err){
        bright_range = buf1[2];
        if(1 == buf1[1]){
            led_no = buf1[0];
            if(0 == timer_flag){
                timer_flag--;
                add_timer(&aw2013_timer);
            }
        }else{
            if(1 == timer_flag){
                timer_flag--;
                del_timer(&aw2013_timer);
            }
                aw2013_ioctl(buf1[0], buf1[1]);
        }
    }
    else {
        aw2013_msg("copy_from_user failed!\n");
        return -EINVAL;
    }
    return 0;
}

static int aw2013_release (struct inode *inode, struct file *filp)
{
	return 0;

}

static const struct file_operations aw2013_fops = {
    .owner   = THIS_MODULE,
    .open    = aw2013_open,
    .read    = aw2013_read,
    .write   = aw2013_write,
	.unlocked_ioctl	 = aw2013_ioctl,
	.release = aw2013_release
};
#endif



static void timer_work(unsigned int data)
{
    schedule_work(&aw2013_work->work);
    mod_timer(&aw2013_timer,jiffies + HZ/10);
    return;
}

static int AW2013_led_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;

#ifdef  REGISTER_CDEV
    dev_t devno;
    int   error;

    error = alloc_chrdev_region(&devno, 0u, 1u, DEVICE_NAME);
    if (error) {
        aw2013_msg("ERROR: alloc_chrdev_regin failed.\n");
        return error;
    }
    aw2013_msg("alloc_chrdev_regin, major = %d, minor = %d.\n", MAJOR(devno), MINOR(devno));

    aw2013_devp = kmalloc(sizeof(struct aw2013_dev), GFP_KERNEL);
    if (NULL == aw2013_devp) {
        aw2013_msg("ERROR: kmallloc failed.\n");
        error = -ENOMEM;
        goto error_unregister_chrdev_region;
    }
    memset(aw2013_devp, 0, sizeof(struct aw2013_dev));

    cdev_init(&aw2013_devp->cdev, &aw2013_fops);
    aw2013_devp->cdev.owner = THIS_MODULE;
    error = cdev_add(&aw2013_devp->cdev, devno, 1u);
    if (error) {
        aw2013_msg("ERROR: cdev_add failed.\n");
        goto error_kfree;
    }

    memcpy(&aw2013_devp->devno, &devno, sizeof(dev_t));

    aw2013_devp->class = class_create(THIS_MODULE, DEVICE_NAME);
    if(IS_ERR(aw2013_devp->class)) {
        aw2013_msg("ERROR: class_create failed.\n");
        error = -EFAULT;
        goto error_cdev_del;
    }
    device_create(aw2013_devp->class, NULL, devno, NULL, DEVICE_NAME);
    aw2013_msg("device name = %s success.\n", DEVICE_NAME);

#endif


    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }

    aw2013_work = kzalloc(sizeof(*aw2013_work), GFP_KERNEL);
    if (!aw2013_work){
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    this_client = client;

    this_client->addr = client->addr;
    aw2013_msg("client addr = %x\n", this_client->addr);


    i2c_set_clientdata(client, aw2013_work);

#ifdef DEFINE_TIMER
    aw2013_msg("====INIT_TIMER_WORK=== %d\n",__LINE__);
    aw2013_work->queue = create_singlethread_workqueue("AW2013_QUEUE");
    if (!aw2013_work->queue) {
        aw2013_msg("creat_single_thread failed\n");
        err = -ESRCH;
        goto exit_create_singlethread;
    }
    INIT_WORK(&aw2013_work->work, light_flash);
    /*timer*/
    setup_timer(&aw2013_timer, timer_work, (unsigned long)"Timer_Out!");
    aw2013_timer.expires = jiffies + HZ/10;
#endif   
    return 0;

exit_create_singlethread:
    aw2013_msg("==singlethread error =\n");
    i2c_set_clientdata(this_client, NULL);
    kfree(aw2013_work);
exit_alloc_data_failed:
exit_check_functionality_failed:

error_cdev_del:
    cdev_del(&aw2013_devp->cdev);
error_kfree:
    kfree(aw2013_devp);
    aw2013_devp = NULL;
error_unregister_chrdev_region:
    unregister_chrdev_region(devno, 1u);


    return err;
}

static int __devexit AW2013_led_remove(struct i2c_client *client)
{
    light_off(0);
    light_off(1);
    light_off(2);
#ifdef DEFINE_TIMER
    del_timer(&aw2013_timer);
	cancel_work_sync(&aw2013_work->work);
	destroy_workqueue(aw2013_work->queue);
#endif	

    i2c_set_clientdata(this_client, NULL);
    kfree(aw2013_work->input_dev);

#ifdef REGISTER_CDEV
    device_destroy(aw2013_devp->class, aw2013_devp->devno);
    class_destroy (aw2013_devp->class);
    cdev_del(&aw2013_devp->cdev);
    unregister_chrdev_region(aw2013_devp->devno, 1u);
    kfree(aw2013_devp);
    aw2013_devp = NULL;
#endif
    aw2013_msg("AW2013_led_remove ok!\n");
    return 0;
}

static const struct i2c_device_id AW2013_led_id[] = {
	{ AW2013_NAME, 0 },
	{}
};

static struct i2c_driver AW2013_led_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= AW2013_led_probe,
	.remove		= __devexit_p(AW2013_led_remove),
	.id_table	= AW2013_led_id,
	.driver	= {
		.name	= AW2013_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= normal_i2c,
	.detect		= AW2013_led_detect,
	.suspend  =  aw2013_suspend,
	.resume   =  aw2013_resume,
};


static int __init AW2013_led_init(void)
{
    int ret = -1;
    aw2013_msg("AW2013_led_init!\n");
    ret = i2c_add_driver(&AW2013_led_driver);
    if(ret < 0)
        aw2013_msg("add_driver's ret_val = %d %d\n",ret,__LINE__);
    return ret;	
}	

static void __exit AW2013_led_exit(void)
{
    aw2013_msg("AW2013_led_exit\n");
    i2c_del_driver(&AW2013_led_driver);
}

module_init(AW2013_led_init);
module_exit(AW2013_led_exit);

MODULE_AUTHOR("<lx@zicoo.cn>");
MODULE_DESCRIPTION("Zicoo Breathe Led Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:breath_led");
