#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include "bsc-slave.h"

#define DRIVER_AUTHOR	"Cedric Dumondelle <cedric.dumondelle@live.fr>"
#define DRIVER_DESC	"i2c slave controller for BCM2835"


#define IRQ_DESC	"i2c-slave-irq"
#define DEVICE_NAME	"i2c-slave"
#define DRV_NAME	"i2c-slave-controller"
#define BUFFER_SIZE	PAGE_SIZE
#define IRQ_NUMBER	73

#define DEVICE_SLV_ADD	0x33

struct circ_buff {
	size_t size;
	unsigned long volatile head;
	unsigned long volatile tail;
}

struct bcm2835_i2c_slave_struct {
       void __iomem	*base;
       int		irq;
       struct cdev	cdev;
       dev_t		dev_number;
       struct circ_buff	rx_buff;
       struct circ_buff tx_buff;	
} *i2c_slave_dev;

static struct file_operations i2c_slave_fops = {
        .owner          =  THIS_MODULE,
        .open           =  i2c_slave_open,
        .release        =  i2c_slave_release,
        .read           =  i2c_slave_read,
        .write          =  i2c_slave_write,
        .unlocked_ioctl =  i2c_slave_ioctl,
};

int __init bcm2835_i2c_slave_init(void) {

}

void __exit bcm2708_i2c_slave_cleanup(void) {

}

module_init(bcm2835_i2c_slave_init);
module_exit(bcm2835_i2c_slave_exit);

/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("chdrv:" DRV_NAME);

