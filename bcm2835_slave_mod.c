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
#include "bcm2835_slave_mod.h"

#define I2C_WAIT_LOOP_COUNT	200
#define SLV_ADDRESS			0x33
#define DRV_NAME		"bcm2835_i2c_slave"

static unsigned int baudrate;
module_param(baudrate, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(baudrate, "The I2C baudrate");

static bool combined = false;
module_param(combined, bool, 0644);
MODULE_PARM_DESC(combined, "Use combined transactions");

struct bcm2835_i2c_slave {
	struct i2c_adapter adapter;

	spinlock_t lock;
	void __iomem *base;
	int irq;
	struct clk *clk;
	u32 cdiv;
	u32 clk_tout;

	struct completion done;

	struct i2c_msg *msg;
	int pos;
	int nmsgs;
	bool error;
};

static inline u32 bcm2835_rd(struct bcm2835_i2c_slave  *bi, unsigned reg)
{
	return readl(bi->base + reg);
}

static inline void bcm2835_wr(struct bcm2835_i2c_slave  *bi, unsigned reg, u32 val)
{
	writel(val, bi->base + reg);
}

static inline void bcm2835_bsc_slave_reset(struct bcm2835_i2c_slave  *bi)
{
	bcm2835_wr(bi, BSC_CR, 0);
	bcm2835_wr(bi, BSC_SLV, 0);
}

static inline void bcm2835_bsc_slave_fifo_drain(struct bcm2835_i2c_slave  *bi)
{
	while (!(bcm2835_rd(bi, BSC_FR) & BSC_FR_RXFE) && (bi->pos < bi->msg->len))
		bi->msg->buf[bi->pos++] = bcm2835_rd(bi, BSC_DR) & BSC_DR_DATA_MASK;
}

static inline void bcm2835_bsc_slave_fifo_fill(struct bcm2835_i2c_slave  *bi)
{
	while (!(bcm2835_rd(bi, BSC_FR) & BSC_FR_TXFE) && (bi->pos < bi->msg->len))
		bcm2835_wr(bi, BSC_DR, bi->msg->buf[bi->pos++]);
}

static inline int bcm2835_bsc_slave_setup(struct bcm2835_i2c_slave  *bi)
{
	int reg;
	u32 cdiv, clk_tout;
	u32 c = BSC_CR_I2C | BSC_CR_RXE | BSC_CR_TXE | BSC_CR_EN;
	cdiv = bi->cdiv;
	clk_tout = bi->clk_tout;
	

	bcm2835_wr(bi, BSC_IFLS, 0);//TODO 
	bcm2835_wr(bi, BSC_IMSC, BSC_IMSC_RXIM | BSC_IMSC_TXIM);
	bcm2835_wr(bi, BSC_RSR, 0);
	bcm2835_wr(bi, BSC_SLV, SLV_ADDRESS);
	bcm2835_wr(bi, BSC_CR, c);
	


	//DEBUG
	reg = bcm2835_rd(bi, BSC_SLV);
	printk( KERN_INFO "Slave Address: 0x%x\n", reg);

	reg = bcm2835_rd(bi, BSC_CR);
	printk( KERN_INFO "CR: 0x%x\n", reg);

	if ((reg & BSC_CR_BRK) != 0)
		printk( KERN_INFO "Break Enabled (I2C TX functions disabled)\n");
	else
		printk( KERN_INFO "Break Disabled (I2C TX Fucntions enabled)\n");

	if ((reg & BSC_CR_RXE) != 0)
		printk( KERN_INFO "Receive Mode Enabled (Only affects SPI?)\n");
	else
		printk( KERN_INFO "Receive Mode Disabled (Only affects SPI?)\n");

	if ((reg & BSC_CR_TXE) != 0)
		printk( KERN_INFO "Transmit mode enabled\n");
	else
		printk( KERN_INFO "Transmit mode disabled\n");

	if ((reg & BSC_CR_I2C) != 0)
		printk( KERN_INFO "I2C Mode enabled\n");
	else
		printk( KERN_INFO "I2C mode disabled\n");

	if ((reg & BSC_CR_SPI) != 0)
		printk( KERN_INFO "SPI Mode enabled\n");
	else
		printk( KERN_INFO "SPI mode disabled\n");

	if ((reg & BSC_CR_EN) != 0)
		printk( KERN_INFO "Device Enabled\n");
	else
		printk( KERN_INFO "Device Disabled\n");

	reg = bcm2835_rd(bi, BSC_FR);
	printk( KERN_INFO "FR: 0x%x\n", reg);

	printk( KERN_INFO "RX FIFO Level: 0x%x\n", (reg & 0xf800) / 2048);
	printk( KERN_INFO "TX FIFO Level: 0x%x\n", (reg & 0x7c0) / 64);
	
	if ((reg & BSC_FR_TXFE) != 0)
		printk( KERN_INFO "TX Fifo Empty\n");
	if ((reg & BSC_FR_RXFF) != 0)
		printk( KERN_INFO "RX Fifo full\n");
	if ((reg & BSC_FR_TXFF) != 0)
		printk( KERN_INFO "TX Fifo full\n");
	if ((reg & BSC_FR_RXFE) != 0)
		printk( KERN_INFO "RX Fifo Empty\n");
	if ((reg & BSC_FR_TXBUSY) != 0)
		printk( KERN_INFO "Transmit operation in progress.\n");
	else
		printk( KERN_INFO "Transmit inactive.\n");

	reg = bcm2835_rd(bi, BSC_IFLS);
	printk( KERN_INFO "IFLS: 0x%x\n", reg);

	printk( KERN_INFO "RX FIFO Interrupt trigger: 0x%x\n", (reg & 0x0038) / 8);
	printk( KERN_INFO "TX FIFO interrupt trigger: 0x%x\n", (reg & 0x7));

	reg = bcm2835_rd(bi, BSC_IMSC);
	printk( KERN_INFO "IMSC: 0x%x\n", reg);

	reg = bcm2835_rd(bi, BSC_RIS);
	printk( KERN_INFO "RIS: 0x%x\n", reg);

	reg = bcm2835_rd(bi, BSC_MIS);
	printk( KERN_INFO "MIS: 0x%x\n", reg);

	reg = bcm2835_rd(bi, BSC_ICR);
	printk( KERN_INFO "ICR: 0x%x\n", reg);
	return 0;
}

static irqreturn_t bcm2835_i2c_slave_interrupt(int irq, void *dev_id)
{
	u32 reg, stat_reg;
	struct bcm2835_i2c_slave *i2c_slave = dev_id;
   	int tx_value_count;

	spin_lock(&i2c_slave->lock);
   	stat_reg = readl(i2c_slave->base + BSC_MIS);      //interrupt status reg
                                                     // clear error register
 	writel(0, i2c_slave->base + BSC_RSR);
	printk( KERN_INFO "irq mis en place\n");
   	if(stat_reg & BSC_MIS_RXMIS){
	printk( KERN_INFO "interruption detectee sur RX\n");
                                                //while RX_FIFO not empty
    		while( !((reg = readl(i2c_slave->base + BSC_FR)) & BSC_FR_RXFE) ){
			printk( KERN_INFO "remplissage du buffer\n");
			reg = readl(i2c_slave->base + BSC_DR);       //read RX_FIFO
  		}
   	}


	if(stat_reg & BSC_MIS_TXMIS){

		printk( KERN_INFO "interruption detectee sur TX\n");
 		tx_value_count = 0;

     		if(tx_value_count == 0){                        //no data to send

          		reg = readl(i2c_slave->base + BSC_IMSC);
          		reg &= ~(BSC_IMSC_TXIM);                   //disable TX interruts
          		writel(reg ,i2c_slave->base + BSC_IMSC);
     		}

     		reg = readl(i2c_slave->base + BSC_FR);
                                               //while space in TX FIFO
	 	while (((!(reg & BSC_FR_TXFF)) && (tx_value_count != 0))){
		printk( KERN_INFO "remplissage du buffer\n");
		 reg = readl(i2c_slave->base + BSC_FR);
	 	}
 
  	}
	spin_unlock(&i2c_slave->lock);

   return IRQ_HANDLED;
}

static int bcm2835_i2c_slave_master_xfer(struct i2c_adapter *adap,
	struct i2c_msg *msgs, int num)
{
	struct bcm2835_i2c_slave *bi = adap->algo_data;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bi->lock, flags);

	reinit_completion(&bi->done);
	bi->msg = msgs;
	bi->pos = 0;
	bi->nmsgs = num;
	bi->error = false;
	ret = 5;
	//ret = bcm2835_bsc_setup(bi);

	spin_unlock_irqrestore(&bi->lock, flags);

	/* check the result of the setup */
	if (ret < 0)
	{
		dev_err(&adap->dev, "transfer setup timed out\n");
		goto error_timeout;
	}

	ret = wait_for_completion_timeout(&bi->done, adap->timeout);
	if (ret == 0) {
		dev_err(&adap->dev, "transfer timed out\n");
		goto error_timeout;
	}

	ret = bi->error ? -EIO : num;
	return ret;

error_timeout:
	spin_lock_irqsave(&bi->lock, flags);
	//bcm2835_bsc_slave_reset(bi);
	bi->msg = 0; /* to inform the interrupt handler that there's nothing else to be done */
	bi->nmsgs = 0;
	spin_unlock_irqrestore(&bi->lock, flags);
	return -ETIMEDOUT;
}

static u32 bcm2835_i2c_slave_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | /*I2C_FUNC_10BIT_ADDR |*/ I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm bcm2835_i2c_slave_algorithm = {
	.master_xfer = bcm2835_i2c_slave_master_xfer,
	.functionality = bcm2835_i2c_slave_functionality,
};

static int bcm2835_i2c_slave_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int irq, err = -ENOMEM;
	struct clk *clk;
	struct bcm2835_i2c_slave *bi;
	struct i2c_adapter *adap;
	unsigned long bus_hz;
	u32 cdiv, clk_tout;
	u32 baud;
	
	baud = CONFIG_I2C_BCM2708_BAUDRATE;

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
		//return PTR_ERR(clk);
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
	adap = &bi->adapter;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_DDC;
	adap->algo = &bcm2835_i2c_slave_algorithm;
	adap->algo_data = bi;
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	strlcpy(adap->name, dev_name(&pdev->dev), sizeof(adap->name));
	adap->dev.of_node = pdev->dev.of_node;

	switch (pdev->id) {
	case 0:
		adap->class = I2C_CLASS_HWMON;
		break;
	case 1:
		adap->class = I2C_CLASS_DDC;
		break;
	case 2:
		adap->class = I2C_CLASS_DDC;
		break;
	default:
		dev_err(&pdev->dev, "can only bind to BSC 0, 1 or 2\n");
		err = -ENXIO;
		goto out_free_bi;
	}

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
	err = i2c_add_numbered_adapter(adap);
	if (err < 0) {
		dev_err(&pdev->dev, "could not add I2C adapter: %d\n", err);
		goto out_free_irq;
	}

	bus_hz = clk_get_rate(bi->clk);
	cdiv = bus_hz / baud;
	if (cdiv > 0xffff) {
		cdiv = 0xffff;
		baud = bus_hz / cdiv;
	}

	clk_tout = 35/1000*baud; //35ms timeout as per SMBus specs.
	if (clk_tout > 0xffff)
		clk_tout = 0xffff;
	
	bi->cdiv = cdiv;
	bi->clk_tout = clk_tout;

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
//////////////////////////////////////////////////////////////////////
static int bcm2835_i2c_slave_remove(struct platform_device *pdev)
{
	struct bcm2835_i2c_slave *bi = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	i2c_del_adapter(&bi->adapter);
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
	printk(KERN_INFO "\n OK \n");
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
