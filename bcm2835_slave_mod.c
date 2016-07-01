#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/circ_buf.h>
#include "bcm2835_slave_mod.h"

#define I2C_WAIT_LOOP_COUNT	200
#define SLV_ADDRESS		0x42
#define DRV_NAME		"bcm2835_i2c_slave"

#define READ( __bcm2835_i2c_slave__, __reg__ ) readl( __bcm2835_i2c_slave__->base + __reg__)
#define WRITE( __bcm2835_i2c_slave__, __reg__ , __val__) writel( __val__, __bcm2835_i2c_slave__->base + __reg__)

#define DEBUG 1

static unsigned int baudrate;
module_param(baudrate, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(baudrate, "The I2C baudrate");

static unsigned int slave_add = SLV_ADDRESS;
module_param(slave_add, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(slave_add, "The I2C Slave address");

static bool combined = false;
module_param(combined, bool, 0644);
MODULE_PARM_DESC(combined, "Use combined transactions");

struct bcm2835_i2c_slave {

	spinlock_t lock;
	void __iomem *base;
	int irq;
	struct clk *clk;
	struct device *dev;

	struct completion done;

	struct i2c_msg *msg;	
	int pos;
	int nmsgs;
	bool error;
};

static inline void debug_bsc_slave_register(struct bcm2835_i2c_slave *bi)
{
	u32 reg;
	reg = READ(bi, BSC_SLV);
	printk( KERN_INFO "Slave Address: 0x%x\n", reg);

	reg = READ(bi, BSC_CR);
	printk( KERN_INFO "Control Register value: 0x%x\n", reg);
	
	printk( KERN_INFO "Break %s (I2C TX functions %s)\n", (reg & BSC_CR_BRK) ? "Enabled" : "Disabled", (reg & BSC_CR_BRK) ? "disabled" : "enabled");
	printk( KERN_INFO "Receive Mode %s (Only affects SPI?)\n", (reg & BSC_CR_RXE) ? "Enabled" : "Disabled");
	printk( KERN_INFO "Transmit mode %s\n", (reg & BSC_CR_TXE) ? "enabled" : "disabled");
	printk( KERN_INFO "I2C Mode %s\n", (reg & BSC_CR_I2C) ? "enabled" : "disabled");
	printk( KERN_INFO "SPI Mode %s\n", (reg & BSC_CR_SPI) ? "enabled" : "disabled");
	printk( KERN_INFO "Device %s\n", (reg & BSC_CR_EN) ? "Enabled" : "Disabled");
	
	reg = READ(bi, BSC_FR);
	printk( KERN_INFO "FR: 0x%x\n", reg);

	printk( KERN_INFO "RX FIFO Level: 0x%x\n", (reg & 0xf800) / 2048);
	printk( KERN_INFO "TX FIFO Level: 0x%x\n", (reg & 0x7c0) / 64);
	
	if ((reg & BSC_FR_TXFE) != 0)
		printk( KERN_INFO "TX Fifo Empty\n");
	else if ((reg & BSC_FR_TXFF) != 0)
		printk( KERN_INFO "TX Fifo full\n");
	if ((reg & BSC_FR_RXFE) != 0)
		printk( KERN_INFO "RX Fifo Empty\n");
	else if ((reg & BSC_FR_RXFF) != 0)
		printk( KERN_INFO "RX Fifo full\n");
		
	printk( KERN_INFO "Transmit %s.\n", (reg & BSC_CR_EN) ? "operation in progress" : "inactive");

	reg = READ(bi, BSC_IFLS);
	printk( KERN_INFO "IFLS: 0x%x\n", reg);

	printk( KERN_INFO "RX FIFO Interrupt trigger: 0x%x\n", (reg & 0x0038) / 8);
	printk( KERN_INFO "TX FIFO interrupt trigger: 0x%x\n", (reg & 0x7));

	reg = READ(bi, BSC_IMSC);
	printk( KERN_INFO "IMSC: 0x%x\n", reg);

	reg = READ(bi, BSC_RIS);
	printk( KERN_INFO "RIS: 0x%x\n", reg);

	reg = READ(bi, BSC_MIS);
	printk( KERN_INFO "MIS: 0x%x\n", reg);

	reg = READ(bi, BSC_ICR);
	printk( KERN_INFO "ICR: 0x%x\n", reg);
}

static inline void bcm2835_bsc_slave_reset(struct bcm2835_i2c_slave *bi)
{
	WRITE(bi, BSC_CR, 0);
	WRITE(bi, BSC_SLV, 0);
}

static inline void bcm2835_bsc_slave_fifo_drain(struct bcm2835_i2c_slave *bi)
{
	while (!(READ(bi, BSC_FR) & BSC_FR_RXFE) && (bi->pos < bi->msg->len))
		bi->msg->buf[bi->pos++] = READ(bi, BSC_DR) & BSC_DR_DATA_MASK;
}

static inline void bcm2835_bsc_slave_fifo_fill(struct bcm2835_i2c_slave  *bi)
{
	while (!(READ(bi, BSC_FR) & BSC_FR_TXFE) && (bi->pos < bi->msg->len))
		WRITE(bi, BSC_DR, bi->msg->buf[bi->pos++]);
}

static inline int bcm2835_bsc_slave_setup(struct bcm2835_i2c_slave *bi)
{
	u32 c = BSC_CR_I2C | BSC_CR_RXE | BSC_CR_TXE | BSC_CR_EN;
	
	WRITE(bi, BSC_IFLS, ((BSC_IFLS_ONE_EIGHTS << 3) & BSC_IFLS_RXIFLSEL) | (BSC_IFLS_ONE_EIGHTS & BSC_IFLS_TXIFLSEL));
	WRITE(bi, BSC_IMSC, BSC_IMSC_RXIM | BSC_IMSC_TXIM);
	WRITE(bi, BSC_RSR, 0);
	WRITE(bi, BSC_SLV, slave_add);
	WRITE(bi, BSC_CR, c);	
	
	#ifdef DEBUG
	debug_bsc_slave_register(bi);
	#endif
	return 0;
}

static irqreturn_t bcm2835_i2c_slave_interrupt(int irq, void *dev_id)
{
	u32 reg;
	struct bcm2835_i2c_slave *bi = dev_id;

	spin_lock(&bi->lock);
   	reg = READ(bi, BSC_MIS);      //interrupt status reg
                                                     // clear error register
 	WRITE(bi, BSC_RSR, 0);
	printk( KERN_INFO "irq mis en place\n");
   	if(reg & BSC_MIS_RXMIS) {
		printk( KERN_INFO "interruption detectee sur RX\n");
		bcm2835_bsc_slave_fifo_drain(bi);    		
   	}

	if(reg & BSC_MIS_TXMIS){
		printk( KERN_INFO "interruption detectee sur TX\n");
		if(!bi->nmsgs || !bi->msg)
		{
			reg = READ(bi, BSC_IMSC);
			reg &= ~(BSC_IMSC_TXIM);
			WRITE(bi, BSC_IMSC, reg);
		}
		else
 			bcm2835_bsc_slave_fifo_fill(bi); 
  	}
	spin_unlock(&bi->lock);

   return IRQ_HANDLED;
}
static int bcm2835_i2c_slave_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int irq, err = -ENOMEM;
	struct clk *clk;
	struct bcm2835_i2c_slave *bi;
	unsigned long bus_hz;
	u32 cdiv, clk_tout;
	u32 baud;
	
	baud = CONFIG_I2C_BCM2708_BAUDRATE;

	printk(KERN_INFO "%s\n", pdev->name);
	if (pdev->dev.of_node) {
		u32 bus_clk_rate;
		pdev->id = of_alias_get_id(pdev->dev.of_node, "i2cslv");
		if (pdev->id < 0) {
			dev_err(&pdev->dev, "alias is missing\n");
			return -EINVAL;
		}
		if (!of_property_read_u32(pdev->dev.of_node,
					"clock-frequency", &bus_clk_rate))
			baud = bus_clk_rate;
		else
			dev_warn(&pdev->dev,
				"Could not read clock-frequency property\n");
	}

	if (baudrate)
		baud = baudrate;
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "could not get IO memory\n");
		return -ENXIO;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "could not get IRQ\n");
		//return irq;
	}
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "could not find clk: %ld\n", PTR_ERR(clk));
	}
	err = clk_prepare_enable(clk);
	if (err) {
		dev_err(&pdev->dev, "could not enable clk: %d\n", err);
		goto out_clk_put;
	}
	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (!bi)
		goto out_clk_disable;
	platform_set_drvdata(pdev, bi);

	spin_lock_init(&bi->lock);
	init_completion(&bi->done);

	bi->base = ioremap(regs->start, resource_size(regs));
	if (!bi->base) {
		dev_err(&pdev->dev, "could not remap memory\n");
		goto out_free_bi;
	}

	bi->irq = irq;
	bi->clk = clk;

	err = request_irq(irq, bcm2835_i2c_slave_interrupt, IRQF_SHARED,
			dev_name(&pdev->dev), bi);
	if (err) {
		dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
		goto out_iounmap;
	}

	bcm2835_bsc_slave_reset(bi);
	bcm2835_bsc_slave_setup(bi);
	
	bus_hz = clk_get_rate(bi->clk);
	cdiv = bus_hz / baud;
	if (cdiv > 0xffff) {
		cdiv = 0xffff;
		baud = bus_hz / cdiv;
	}

	clk_tout = 35/1000*baud; //35ms timeout as per SMBus specs.
	if (clk_tout > 0xffff)
		clk_tout = 0xffff;

	dev_info(&pdev->dev, "BSC%d Controller at 0x%08lx (irq %d) (baudrate %d)\n",
		pdev->id, (unsigned long)regs->start, irq, baud);
	printk(KERN_INFO "it's ok\n");
	return 0;

out_free_irq:
	free_irq(bi->irq, bi);
out_iounmap:
	iounmap(bi->base);
out_free_bi:
	kfree(bi);
out_clk_disable:
	clk_disable_unprepare(clk);
out_clk_put:
	clk_put(clk);
	return err;
}

static int bcm2835_i2c_slave_remove(struct platform_device *pdev)
{
	struct bcm2835_i2c_slave *bi = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	
	free_irq(bi->irq, bi);
	iounmap(bi->base);
	clk_disable_unprepare(bi->clk);
	clk_put(bi->clk);
	kfree(bi);

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
MODULE_AUTHOR("Cedric Dumondelle <cedric.dumondelle@live.fr>");
MODULE_LICENSE("GPL v2");
