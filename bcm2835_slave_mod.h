/*IOCTL define*/
#define I2C_READ_AVAILABLE	0x00000001

/*Address register base*/
#define I2C_SPI_FIFOS_BASE	0x3F204000	// BSC/SPI FIFOS registers base addresses

/*FIFO register mask*/
#define CLEAR_TX_FIFO		0x00000010	// Masked TX bit of CLEAR FIFO register
#define CLEAR_RX_FIFO		0x00000020	// Masked RX bit of CLEAR FIFO register

/*I2C_SLAVE register offsets*/
#define BSC_DR			0x00000000	// Data register
#define BSC_RSR			0x00000004	// Operation status register and error clear register
#define BSC_SLV			0x00000008	// I2C/SPI slave address register
#define BSC_CR			0x0000000C	// Control register (used to configure i2c/spi operation)
#define BSC_FR			0x00000010	// Flag register
#define BSC_IFLS		0x00000014	// Interrupt FIFO level select register 
#define BSC_IMSC		0x00000018	// Interrupt mask set clear register
#define BSC_RIS			0x0000001C	// Raw interrupt status register
#define BSC_MIS			0x00000020	// Masked interrupt status register
#define BSC_ICR			0x00000024	// Interrupt clear register
#define BSC_DMACR		0x00000028	// DMA control register (not supported)
#define	BSC_TDR			0x0000002C	// FIFO test data register
#define BSC_GPUSTAT		0x00000030	// GPU status register
#define	BSC_HCTRL		0x00000034	// Host control register
#define	BSC_I2C_DEBUG		0x00000038	// I2C debug register
#define BSC_SPI_DEBUG		0x0000003C	// SPI debug register

/*Data register mask*/
#define BSC_DR_RXFLEVEL		0xF8000000	// The current level of the RX FIFO
#define BSC_DR_TXFLEVEL		0x07C00000	// The current level of the TX FIFO
#define BSC_DR_RXBUSY		0x00200000	// Set when receive operation is in operation
#define BSC_DR_TXFE		0x00100000	// Set when TX FIFO is empty
#define BSC_DR_RXFF		0x00080000	// Set when RX FIFO is full
#define BSC_DR_TXFF		0x00040000	// Set when TX FIFO is full
#define BSC_DR_RXFE		0x00020000	// Set when RX FIFO is empty
#define BSC_DR_TXBUSY		0x00010000	// Set when transmit operation is in operation
#define BSC_DR_UE		0x00000200	// Set when TX FIFO is empty and I2C master attempt to read
#define BSC_DR_OE		0x00000100	// Set when RX FIFO is full and a new data is received
#define BSC_DR_DATA_MASK	0x000000FF	// Received and transfered data register

/*RSR mask*/
#define BSC_RSR_UE		0x00000002	// Set when TX FIFO is empty and I2C master attempt to read (Same as BSC_DR_UE)
#define BSC_RSR_OE		0x00000001	// Set when RX FIFO is full and a new data is received (Same as BSC_DR_OE)

/*Control register mask*/
#define BSC_CR_INV_TXF		0x00002000	// Change the default reset value of TX flags
#define BSC_CR_HOSTCTRLEN	0x00001000	// Allow the host control
#define BSC_CR_TESTFIFO		0x00000800	// Allow the TEST FIFO
#define BSC_CR_INV_RXF		0x00000400	// Change the default reset value of RX flags
#define BSC_CR_RXE		0x00000200	// Enable receive mode
#define BSC_CR_TXE		0x00000100	// Enable transmit mode
#define BSC_CR_BRK		0x00000080	// Stop operation and clear FIFO
#define BSC_CR_ENCTRL		0x00000040	// Enable Control, ordinary I2C protocol when unset
#define BSC_CR_ENSTAT		0x00000020	// Enable Status, ordinary I2C protocol when unset
#define BSC_CR_CPOL		0x00000010	// Clock polarity
#define BSC_CR_CPHA		0x00000008	// Clock phase
#define BSC_CR_I2C		0x00000004	// Enable I2C mode
#define BSC_CR_SPI		0x00000002	// Enable SPI mode
#define BSC_CR_EN		0x00000001	// Enable I2C/SPI slave

/*Flag register mask*/
#define BSC_FR_RXFLEVEL		0x0000F800	// Return the current level of RX FIFO
#define BSC_FR_TXFLEVEL		0x000007C0	// Return the current level of TX FIFO
#define BSC_FR_RXBUSY		0x00000020	// Set when receive operation is in operation
#define BSC_FR_TXFE		0x00000010	// Set when TX FIFO is empty
#define BSC_FR_RXFF		0x00000008	// Set when RX FIFO is full
#define BSC_FR_TXFF		0x00000004	// Set when TX FIFO is full
#define BSC_FR_RXFE		0x00000002	// Set when RX FIFO is empty
#define BSC_FR_TXBUSY		0x00000001	// Set when transmit operation is in operation

/*Interrupt FIFO level mask*/
#define BSC_IFLS_RXIFLSEL	0x00000038	// Mask to select RX interrupt FIFO level
#define BSC_IFLS_TXIFLSEL	0x00000007	// Mask to select TX interrupt FIFO level

/*Interrupt FIFO level value*/
#define BSC_IFLS_ONE_EIGHTS	0x00000000	// Interrupt is set when fifo gets 1/8 full
#define BSC_IFLS_ONE_QUART	0x00000009	// Interrupt is set when fifo gets 1/4 full
#define BSC_IFLS_ONE_HALF	0x00000012	// Interrupt is set when fifo gets 1/2 full
#define BSC_IFLS_THREE_QUART	0x0000001B	// Interrupt is set when fifo gets 3/4 full
#define BSC_IFLS_SEVEN_EIGHTS	0x00000024	// Interrupt is set when fifo gets 7/8 full

/*Interrupt mask set/clear register mask*/
#define BSC_IMSC_OEIM		0x00000008	// Overrun error interrupt mask
#define BSC_IMSC_BEIM		0x00000004	// Break error interrupt mask
#define BSC_IMSC_TXIM		0x00000002	// Transmit interrupt mask
#define BSC_IMSC_RXIM		0x00000001	// Receive interrupt mask

/*Masked interrupt status register mask*/
#define BSC_MIS_OEMIS		0x00000008	// Overrun error masked interrupt status
#define BSC_MIS_BEMIS		0x00000004	// Break error masked interrupt status
#define BSC_MIS_TXMIS		0x00000002	// Transmit masked interrupt status
#define BSC_MIS_RXMIS		0x00000001	// Receive masked interrupt status

/*Interrupt clear register mask*/
#define BSC_ICR_OEIC		0x00000008	// Overrun error interrupt clear
#define BSC_ICR_BEIC		0x00000004	// Break error interrupt clear
#define BSC_ICR_TXIC		0x00000002	// Transmit interrupt clear
#define BSC_ICR_RXIC		0x00000001	// Receive interrupt clear

/*Test data register mask*/
#define BSC_TDR_DATA		0x0000000F	// Test data register, written into the RX FIFO or read out of the TX FIFO

/*GPU status register mask*/
#define BSC_GPUSTAT_DATA	0x00000007

/*Host control register mask*/
#define BSC_HCTRL_DATA		0x000000FF

/*I2C debug register mask*/
#define BSC_DEBUG1_DATA		0x03FFFFFF

/*SPI debug register mask*/
#define BSC_DEBUG2_DATA		0x00FFFFFF
