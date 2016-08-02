#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/circ_buf.h>
#include <asm/uaccess.h>
#include "bcm2835_slave_mod.h"

#define I2C_WAIT_LOOP_COUNT	200
#define SLV_ADDRESS		0x42
#define DRV_NAME		"bcm2835_i2c_slave"
#define DEVICE_NAME		"i2c_slave"
#define BUFFER_SIZE		PAGE_SIZE
#define READL( __bcm2835_i2c_slave__, __reg__ ) readl( __bcm2835_i2c_slave__->base + __reg__)
#define WRITEL( __bcm2835_i2c_slave__, __reg__ , __val__) writel( __val__, __bcm2835_i2c_slave__->base + __reg__)
#define INCREMENT_BUFF_CURSOR( __buffer_cursor__, __val__) (__buffer_cursor__ = (__buffer_cursor__ + __val__) & (BUFFER_SIZE - 1))

#define DEBUG 1

static unsigned int slave_add = SLV_ADDRESS;
module_param(slave_add, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(slave_add, "The I2C Slave address");

static struct class *i2c_slave_class;

struct bcm2835_i2c_slave {

	spinlock_t lock;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	void __iomem *base;
	int irq;
	struct cdev cdev;
	struct device *dev;
	int major;
	unsigned int slave_address;
	
	struct completion done;
	
	struct circ_buf rx_buff;
	struct circ_buf tx_buff;
	wait_queue_head_t in_queue;
	wait_queue_head_t out_queue;
};

static inline void bcm2835_bsc_slave_reset(struct bcm2835_i2c_slave *bi)
{
	WRITEL(bi, BSC_CR, 0);
	WRITEL(bi, BSC_SLV, 0);
}

static inline void bcm2835_bsc_slave_fifo_drain(struct bcm2835_i2c_slave *bi)
{
	spin_lock(&bi->rx_lock);
	while (!(READL(bi, BSC_FR) & BSC_FR_RXFE)) {
		if(CIRC_SPACE(bi->rx_buff.head, bi->rx_buff.tail, BUFFER_SIZE)) {
			bi->rx_buff.buf[bi->rx_buff.head] = READL(bi, BSC_DR) & BSC_DR_DATA_MASK;
			smp_wmb();
			INCREMENT_BUFF_CURSOR(bi->rx_buff.head, 1);
		} else {
			(void)READL(bi, BSC_DR);
		}
	}
	spin_unlock(&bi->rx_lock);
}

static inline void bcm2835_bsc_slave_fifo_fill(struct bcm2835_i2c_slave  *bi)
{
	spin_lock(&bi->tx_lock);
	while (!(READL(bi, BSC_FR) & BSC_FR_TXFF) && (CIRC_CNT(bi->tx_buff.head, bi->tx_buff.tail, BUFFER_SIZE) >= 1)) {
		WRITEL(bi, BSC_DR, bi->tx_buff.buf[bi->tx_buff.tail]);
		smp_rmb();
		printk(KERN_INFO "data stored : %c\n", bi->tx_buff.buf[bi->tx_buff.tail]);
		INCREMENT_BUFF_CURSOR(bi->tx_buff.tail, 1);
		}
	spin_unlock(&bi->tx_lock);
}

static ssize_t i2c_slave_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct bcm2835_i2c_slave  *bi = file->private_data;
	int res = -EFAULT;
	int length;
	size_t effective_count;
	
	wait_event_interruptible(bi->out_queue, CIRC_CNT(bi->rx_buff.head, bi->rx_buff.tail, BUFFER_SIZE));
	spin_lock(&bi->rx_lock);
	length = CIRC_CNT_TO_END(bi->rx_buff.head, bi->rx_buff.tail, BUFFER_SIZE);
	effective_count = CIRC_CNT(bi->rx_buff.head, bi->rx_buff.tail, BUFFER_SIZE);
	if(effective_count > count) {
		effective_count = count;
	}	
	length = min(length, effective_count);
	if(copy_to_user(buf, &(bi->rx_buff.buf[bi->rx_buff.tail]), length)) {
		goto unlock_rx_buffer;
	}
	res = length;
	if (effective_count > length) {
		if(copy_to_user(buf + length, &(bi->rx_buff.buf[0]), effective_count - length) == 0) {
			res += effective_count - length;
		}		
	}
	INCREMENT_BUFF_CURSOR(bi->rx_buff.tail, res);
unlock_rx_buffer:
	spin_unlock(&bi->rx_lock);
	return res;
}

static ssize_t i2c_slave_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct bcm2835_i2c_slave  *bi = file->private_data;
	unsigned int reg;
	int res = -EFAULT;
	int length;
	size_t effective_count;
	
	wait_event_interruptible(bi->in_queue, CIRC_SPACE(bi->tx_buff.head, bi->tx_buff.tail, BUFFER_SIZE));
	spin_lock(&bi->tx_lock);
	length = CIRC_SPACE_TO_END(bi->tx_buff.head, bi->tx_buff.tail, BUFFER_SIZE);
	effective_count = CIRC_SPACE(bi->tx_buff.head, bi->tx_buff.tail, BUFFER_SIZE);
	if(effective_count >= count) {
		effective_count = count;
	}
	length = min(length , effective_count);
	if(copy_from_user(&bi->tx_buff.buf[bi->tx_buff.head], buf, length)) {
		goto unlock_tx_buffer;
	}
	res = length;
	if (effective_count > length) {
		if(copy_from_user(&bi->tx_buff.buf[0], buf + length, effective_count - length) == 0) {
			res += effective_count - length;
		}
	}	
	INCREMENT_BUFF_CURSOR(bi->tx_buff.head, res);
	reg = READL(bi, BSC_IMSC);
	reg |= BSC_IMSC_TXIM;
	WRITEL(bi, BSC_IMSC, reg);
unlock_tx_buffer:
	spin_unlock(&bi->tx_lock);
	return res;
}

static int i2c_slave_open(struct inode *inode, struct file *file) 
{
	struct bcm2835_i2c_slave  *bi;
	bi = container_of(inode->i_cdev, struct bcm2835_i2c_slave, cdev);
	file->private_data = bi;
	return 0;
}

int i2c_slave_release(struct inode *inode, struct file *file)
{
	struct bcm2835_i2c_slave *bi;
	bi = file->private_data;
	return 0;
}

static inline int bcm2835_bsc_slave_setup(struct bcm2835_i2c_slave *bi)
{
	unsigned int c = BSC_CR_I2C | BSC_CR_RXE | BSC_CR_TXE | BSC_CR_EN;
	
	WRITEL(bi, BSC_IFLS, ((BSC_IFLS_ONE_EIGHTS << 3) & BSC_IFLS_RXIFLSEL) | (BSC_IFLS_ONE_EIGHTS & BSC_IFLS_TXIFLSEL));
	WRITEL(bi, BSC_IMSC, BSC_IMSC_RXIM | BSC_IMSC_TXIM);
	WRITEL(bi, BSC_RSR, 0);
	WRITEL(bi, BSC_SLV, bi->slave_address);
	WRITEL(bi, BSC_CR, c);
	return 0;
}

static irqreturn_t bcm2835_i2c_slave_interrupt(int irq, void *dev_id)
{
	unsigned int reg;
	struct bcm2835_i2c_slave *bi = dev_id;

	spin_lock(&bi->lock);
   	reg = READL(bi, BSC_MIS);
 	WRITEL(bi, BSC_RSR, 0);

   	if(reg & BSC_MIS_RXMIS) {
		printk(KERN_INFO "RX interrupt %d\n", reg & BSC_MIS_TXMIS);
		bcm2835_bsc_slave_fifo_drain(bi);
		wake_up_interruptible(&bi->out_queue);
	} if(reg & BSC_MIS_TXMIS) {
		printk(KERN_INFO "TX interrupt\n");
		if(bi->tx_buff.head == bi->tx_buff.tail) {
			reg = READL(bi, BSC_IMSC);
			reg &= ~(BSC_IMSC_TXIM);
			WRITEL(bi, BSC_IMSC, reg);
		} else {
			bcm2835_bsc_slave_fifo_fill(bi);
			wake_up_interruptible(&bi->in_queue);
		} 
	}
	spin_unlock(&bi->lock);
	return IRQ_HANDLED;
}

static long i2c_slave_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct bcm2835_i2c_slave  *bi = file->private_data;

	switch (cmd) {
		case I2C_READ_AVAILABLE : 
			return (READL(bi, BSC_MIS) & BSC_MIS_RXMIS);
		default : 
			return -ENOTTY;			
	}
	return 0;
}/*---------------------------------------------ATTRIBUTES---------------------------------------------------------------*/
static ssize_t address_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcm2835_i2c_slave *bi = dev_get_drvdata(dev->parent);
	return sprintf(buf, "0x%x\n", bi->slave_address);
}
static ssize_t address_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long tmp;
	int ret;
	struct bcm2835_i2c_slave *bi = dev_get_drvdata(dev->parent);
	ret = kstrtol(buf, 0, &tmp);
	if(ret < 0 || tmp > 0x7F) {
		printk(KERN_INFO "Invalid address :  Address must be passed under hexadecimal form\n0x00 -- 0x7F");
		return -EINVAL;
	} else {		
		bi->slave_address = tmp;
		WRITEL(bi, BSC_SLV, bi->slave_address);
	}
	return size;
}

static DEVICE_ATTR_RW(address);
#ifdef DEBUG
static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int reg;
	ssize_t ret = 0;
	struct bcm2835_i2c_slave *bi = dev_get_drvdata(dev->parent);
	spin_lock(&bi->lock);
	reg = READL(bi, BSC_SLV);
	ret += sprintf(buf + ret, "Slave Address: 0x%x\n", reg);

	reg = READL(bi, BSC_CR);
	ret += sprintf(buf + ret, "Control Register value: 0x%x\n", reg);
	
	ret += sprintf(buf + ret, "Break %s (I2C TX functions %s)\n", (reg & BSC_CR_BRK) ? "Enabled" : "Disabled", (reg & BSC_CR_BRK) ? "disabled" : "enabled");
	ret += sprintf(buf + ret, "Receive Mode %s (Only affects SPI?)\n", (reg & BSC_CR_RXE) ? "Enabled" : "Disabled");
	ret += sprintf(buf + ret, "Transmit mode %s\n", (reg & BSC_CR_TXE) ? "enabled" : "disabled");
	ret += sprintf(buf + ret, "I2C Mode %s\n", (reg & BSC_CR_I2C) ? "enabled" : "disabled");
	ret += sprintf(buf + ret, "SPI Mode %s\n", (reg & BSC_CR_SPI) ? "enabled" : "disabled");
	ret += sprintf(buf + ret, "Device %s\n", (reg & BSC_CR_EN) ? "Enabled" : "Disabled");
	
	reg = READL(bi, BSC_FR);
	ret += sprintf(buf + ret, "FR: 0x%x\n", reg);

	ret += sprintf(buf + ret, "RX FIFO Level: 0x%x\n", (reg & 0xf800) / 2048);
	ret += sprintf(buf + ret, "TX FIFO Level: 0x%x\n", (reg & 0x7c0) / 64);
	
	if ((reg & BSC_FR_TXFE) != 0)
		ret += sprintf(buf + ret, "TX Fifo Empty\n");
	else if ((reg & BSC_FR_TXFF) != 0)
		ret += sprintf(buf + ret, "TX Fifo full\n");
	if ((reg & BSC_FR_RXFE) != 0)
		ret += sprintf(buf + ret, "RX Fifo Empty\n");
	else if ((reg & BSC_FR_RXFF) != 0)
		ret += sprintf(buf + ret, "RX Fifo full\n");
		
	ret += sprintf(buf + ret, "Transmit %s.\n", (reg & BSC_CR_EN) ? "operation in progress" : "inactive");

	reg = READL(bi, BSC_IFLS);
	ret += sprintf(buf + ret, "IFLS: 0x%x\n", reg);

	ret += sprintf(buf + ret, "RX FIFO Interrupt trigger: 0x%x\n", (reg & 0x0038) / 8);
	ret += sprintf(buf + ret, "TX FIFO interrupt trigger: 0x%x\n", (reg & 0x7));
	spin_unlock(&bi->lock);
	
	return ret;	
}

static DEVICE_ATTR_RO(debug);
static struct attribute *i2c_slave_attrs[ ] = {
	&dev_attr_debug.attr,
	&dev_attr_address.attr,
	NULL,
};
#else
static struct attribute *i2c_slave_attrs[ ] = {
	&dev_attr_address.attr,
	NULL,
};
#endif
ATTRIBUTE_GROUPS(i2c_slave);
/*-------------------------------------------------------------------------------------------------------------------------------*/

static const struct file_operations i2c_slave_fops = {
	.owner = THIS_MODULE,
	.open = i2c_slave_open,
	.release = i2c_slave_release,
	.read = i2c_slave_read,
	.write = i2c_slave_write,
	.llseek = no_llseek,
	.unlocked_ioctl = i2c_slave_ioctl,
};

static int bcm2835_i2c_slave_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int irq, err = -ENOMEM;
	struct bcm2835_i2c_slave *bi;
	dev_t dev_number;

	printk(KERN_INFO "%s\n", pdev->name);
	if (pdev->dev.of_node) {
		pdev->id = of_alias_get_id(pdev->dev.of_node, "i2cslv");
		if (pdev->id < 0) {
			dev_err(&pdev->dev, "alias is missing\n");
			return -EINVAL;
		}
	}
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "could not get IO memory\n");
		return -ENXIO;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "could not get IRQ\n");
	}
	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (!bi)
		goto out_free_bi;
	platform_set_drvdata(pdev, bi);

	err = alloc_chrdev_region(&dev_number, 0, 1, DEVICE_NAME);
 	bi->major = MAJOR(dev_number);
  	if(err < 0) {
    		dev_err(&pdev->dev, "alloc_chrdev_region failed for i2cslave : %d\n", err);
    		return -ENODEV;
 	}
  	i2c_slave_class = class_create(THIS_MODULE, DEVICE_NAME);
  	if(IS_ERR(i2c_slave_class)) {
     		err = PTR_ERR(i2c_slave_class);
     		goto out_delete_cdev;
  	}
	i2c_slave_class->dev_groups = i2c_slave_groups;
  	cdev_init(&bi->cdev, &i2c_slave_fops);
  	err = cdev_add(&bi->cdev, dev_number, 1);
  	if(err){
     		goto out_delete_cdev;
  	}
	bi->dev = device_create(i2c_slave_class, &pdev->dev, dev_number, NULL, DEVICE_NAME);
	if(IS_ERR(bi->dev)) {
		printk(KERN_NOTICE "could not create device in sysfs!\n");
		goto out_unalloc_region;
	}
	spin_lock_init(&bi->lock);
	spin_lock_init(&bi->rx_lock);
	spin_lock_init(&bi->tx_lock);
	init_completion(&bi->done);

	bi->base = ioremap(regs->start, resource_size(regs));
	if (!bi->base) {
		dev_err(&pdev->dev, "could not remap memory\n");
		goto out_iounmap;
	}

	bi->irq = irq;
	bi->slave_address = slave_add;

	err = request_irq(irq, bcm2835_i2c_slave_interrupt, IRQF_SHARED,
			dev_name(&pdev->dev), bi);
	if (err) {
		dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
		goto out_iounmap;
	}
	
	bi->rx_buff.buf = kmalloc(BUFFER_SIZE*sizeof(char), GFP_KERNEL);
	if(bi->rx_buff.buf == NULL)
		goto out_rx_alloc_mem;
	bi->tx_buff.buf = kmalloc(BUFFER_SIZE*sizeof(char), GFP_KERNEL);
	if(bi->tx_buff.buf == NULL)
		goto out_tx_alloc_mem;
	bi->rx_buff.head = bi->rx_buff.tail = 0;
	bi->tx_buff.head = bi->tx_buff.tail = 0;
	
	init_waitqueue_head(&bi->in_queue);
	init_waitqueue_head(&bi->out_queue);

	bcm2835_bsc_slave_reset(bi);
	bcm2835_bsc_slave_setup(bi);

	dev_info(&pdev->dev, "BSC%d Controller at 0x%08lx (irq %d)\n",
		pdev->id, (unsigned long)regs->start, irq);
	return 0;
	
out_tx_alloc_mem:
	dev_err(&pdev->dev, "could not allocate TX buffer memory\n");
	kfree(bi->rx_buff.buf);
out_rx_alloc_mem:
	dev_err(&pdev->dev, "could not allocate RX buffer memory\n");
	free_irq(bi->irq, bi);
out_iounmap:
	iounmap(bi->base);
out_delete_cdev:
	cdev_del(&bi->cdev);
out_unalloc_region:
	unregister_chrdev_region(dev_number, 1);
out_free_bi:
	kfree(bi);
	return err;
}

static int bcm2835_i2c_slave_remove(struct platform_device *pdev)
{
	struct bcm2835_i2c_slave *bi = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	
	kfree(bi->rx_buff.buf);
	kfree(bi->tx_buff.buf);
	device_destroy(i2c_slave_class, MKDEV(bi->major,0));
	class_destroy(i2c_slave_class);
	
	unregister_chrdev_region(MKDEV(bi->major, 0), 1);
	cdev_del(&bi->cdev);

	free_irq(bi->irq, bi);
	iounmap(bi->base);
	kfree(bi);
	
	printk(KERN_NOTICE "i2c-slave module removed!\n");
	
	return 0;
}

static const struct of_device_id bcm2835_i2c_slave_of_match[] = {
        { .compatible = "brcm,bcm2835-i2c-slave" },
        {},
};
MODULE_DEVICE_TABLE(of, bcm2835_i2c_slave_of_match);

static struct platform_driver bcm2835_i2c_slave_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = bcm2835_i2c_slave_of_match,
	},
	.probe		= bcm2835_i2c_slave_probe,
	.remove		= bcm2835_i2c_slave_remove,
};

static int __init bcm2835_i2c_slave_init(void)
{	
	return platform_driver_register(&bcm2835_i2c_slave_driver);
}

static void __exit bcm2835_i2c_slave_exit(void)
{
	platform_driver_unregister(&bcm2835_i2c_slave_driver);
}

module_init(bcm2835_i2c_slave_init);
module_exit(bcm2835_i2c_slave_exit);

MODULE_DESCRIPTION("BSC slave driver for Broadcom BCM2835");
MODULE_AUTHOR("Floreal Morandat <florealm@gmail.com>");
MODULE_AUTHOR("Cedric Dumondelle <cedric.dumondelle@live.fr>");
MODULE_LICENSE("GPL v2");
