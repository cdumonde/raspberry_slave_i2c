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
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "bsc-slave.h"

#define DRIVER_AUTHOR	"Cedric Dumondelle <cedric.dumondelle@live.fr>"
#define DRIVER_DESC	"i2c slave controller for BCM2835"


#define IRQ_DESC	"i2c-slave-irq"
#define DEVICE_NAME	"i2c-slave"
#define DRV_NAME	"i2c-slave-controller"
#define MAX_DEVICES	1
#define MINOR_MIN	0
#define BSC_SLV_SIZE	64
#define BUFFER_SIZE	PAGE_SIZE
#define IRQ_NUMBER	73

#define DEVICE_SLV_ADD	0x33

struct bcm2835_i2c_slave {
	void __iomem	*base;
	int		irq;
	struct cdev	cdev;
	dev_t		dev_number;
	struct circ_buff	rx_buff;
	struct circ_buff tx_buff;
} *i2c_slave_dev;

static struct class *i2c_slave_class;
static struct device *i2c_slave_device;

static struct file_operations i2c_slave_fops = {
	.owner		= THIS_MODULE,
	.open		= i2c_slave_open,
	.release	= i2c_slave_release,
	.read		= i2c_slave_read,
	.write		= i2c_slave_write,
	.unlocked_ioctl	= i2c_slave_ioctl,
};

/*Init module*/

int __init bcm2835_i2c_slave_init(void)
{

	int res;
	u32 reg = 0;
	dev_t dev_number;
	struct bcm2835_i2c_slave *i2c_slave;
	
	res = alloc_chrdev_region(&dev_number, MINOR_MIN, MAX_DEVICES, DEVICE_NAME)
	if (res < 0) {
		res = ENODEV;
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "The memory allocation failed for i2c-slave\n");
		goto allocation_fail;
	}

	i2c_slave_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(i2c_slave_class)) {
		res = PTR_ERR(i2c_slave_call);
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "The class creation failed\n");
		goto class_fail;
	}
	
	i2c_slave = kmalloc(sizeof(struct bcm2835_i2c_slave), GFP_KERNEL);
	if (i2c_slave == NULL) {
		res = -ENOMEM;
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "The memory allocation for device structure\n");
		goto mem_fail;
	}	
	
	if (request_mem_region(BSC_SLAVE_BASE, BSC_SLV_SIZE, DEVICE_NAME) == NULL) {
		res = -EIO;
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "I/O Port 0x%x is used by another process, can't reserve it to i2c-slave module\n", BSC_SLAVE_BASE);
		goto mem_reg_fail;
	}

	i2c_slave->dev_number = MKDEV(MAJOR(dev_number), 0);
	cdev_init(&i2c_slave->cdev, &i2c_slave_fops);
	res = cdev_add(&i2c_slave->cdev, i2c_slave->dev_number, 1);
	if (res) {
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "The addition of i2c-slave device to the system failed\n");
		goto cdev_init_fail;
	}
		
	i2c_slave_device = device_create(i2c_slave_class, NULL, i2c_slave->dev_number, i2c_slave, DEVICE_NAME);
	if (IS_ERR(i2c_slave_device)) {
		res = PTR_ERR(i2c_slave_device);
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "The creation and/or registration of i2c-slave device failed\n");
		goto dev_create_fail;
	}
	
	bcm2708_init_i2c_pinmode(TRUE);
	
	res = request_irq(IRQ_NUMBER, i2c_callback_func, 0, DEVICE_NAME, i2c_slave);
	if (res) {
		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
		printk(KERN_ERR "Requesting IRQ %d failed, maybe it is already used\n", IRQ_NUMBER);
     		goto irq_fail;
     	}
     	
     	i2c_slave->irq = IRQ_NUMBER;
     	i2c_slave->base = ioremap(BSC_SLAVE_BASE, SZ_256-1);  //is size right???
     	if (!i2c_slave->base) {
     		printk(KERN_ERR "bcm2835_slave_mod - line %d - Operation failed because of : ", __LINE__);
     		printk(KERN_NOTICE "could not remap memory!\n");
		goto remap_fail;
     	}
     	
     	printk(KERN_NOTICE "i2c-slave at 0x%08lx (irq %d)\n", (unsigned long)BSC_SLAVE_BASE, IRQ_NUMBER);
     	i2c_slave_dev = i2c_slave;
     	
	i2c_slave->rx_buf = __get_free_pages(GFP_KERNEL, 0);
	if (IS_ERR((void *)i2c_slave->rx_buf)) {
		resurn -ENOMEM;
	}

	i2c_slave->rx_buf_head = i2c_slave->rx_buf;
	i2c_slave->rx_buf_tail = i2c_slave->rx_buf;
	i2c_slave->tx_buf = __get_free_pages(GFP_KERNEL, 0);
	if (IS_ERR((void *)i2c_slave->tx_buf)) {
		resurn -ENOMEM;
	}
	
	i2c_slave->tx_buf_head = i2c_slave->tx_buf;
	i2c_slave->tx_buf_tail = i2c_slave->tx_buf;
	writel(BSC_IFLS_TX2_RX4BYTE, i2c_slave->base + BSC_IFLS);
	reg = BSC_IMSC_RXIM | BSC_IMSC_TXIM;
	writel(reg, i2c_slave->base + BSC_IMSC);
	writel(0, i2c_slave->base + BSC_RSR);
	reg = BSC_CR_BRK;
	writel(reg, i2c_slave->base + BSC_CR);
	reg = (BSC_CR_EN | BSC_CR_I2C | BSC_CR_TXE | BSC_CR_RXE);
	writel(reg, i2c_slave->base + BSC_CR);
	resurn 0;
	
	remap_fail:
		free_irq(IRQ_NUMBER, i2c_slave_dev);
	irq_fail:
		device_destroy(i2c_slave_class, MKDEV(major,0));
		bcm2708_init_i2c_pinmode(0);
	dev_create_fail:
		cdev_del(&i2c_slave->cdev);
	cdev_init_fail:
		release_mem_region(BSC_SLAVE_BASE, 64);
	mem_reg_fail:
		kfree(i2c_slave);
	mem_fail:
		class_destroy(i2c_slave_class);
	class_fail:
		unregister_chrdev_region(dev_number, MAX_DEVICES);
	resurn res;
}

void __exit bcm2708_i2c_slave_cleanup(void) {

	printk(KERN_NOTICE "i2c-slave module removed!\n");
	device_destroy(i2c_slave_class, MKDEV(major,0));
	class_destroy(i2c_slave_class);
	unregister_chrdev_region(MKDEV(major, 0), MAX_DEVICES);
	cdev_del(&i2c_slave_dev->cdev);
	release_mem_region(BSC_SLAVE_BASE, 64);
	bcm2708_init_i2c_pinmode(0);
	free_irq(IRQ_NUMBER, i2c_slave_dev);
	iounmap(i2c_slave_dev->base);
	kfree(i2c_slave_dev);
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

