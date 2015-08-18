
#ifndef XGENE_I2C_H_
#define XGENE_I2C_H_

/* Parameters */
#define	XGENE_I2C_BUS_SPEED				100000	/* Bus speed */
#define	XGENE_I2C_MAX_STANDARD_SPEED	100000
#define XGENE_I2C_MAX_FAST_SPEED		400000
#define	XGENE_I2C_TIMEOUT				20		/* ms */
#define	XGENE_I2C_STAT_WAIT_RET			5		/* clk tics	*/
#define	XGENE_I2C_STOP_TIMEOUT			10
#define	XGENE_I2C_TX_RX_TIMEOUT			40
#define XGENE_I2C_TX_RX_TIMEOUT_RST		160
#define	XGENE_I2C_DELAY_IF_BUSY			1000	/* ms */
#define	XGENE_I2C_DELAY_AFT_START		800		/* ms */
#define	XGENE_I2C_REG_RID				0		/* Resource ID */
#define	XGENE_I2C_CLOCK_PRESC			1000	/* Clock prescaler */

/* Mask */
#define MASK_8_BITS						0xFF

/* Registers */
#define	XGENE_I2C_CON					0x00	/* Control register */
#define	XGENE_I2C_CON_MM				0x01	/* Master mode			*/
#define	XGENE_I2C_CON_MSM_S				0x02	/* Max speed mode - standard	*/
#define	XGENE_I2C_CON_MSM_F				0x04	/* Max speed mode - fast	*/
#define	XGENE_I2C_CON_10BAS				0x08	/* 10 bit address slave		*/
#define	XGENE_I2C_CON_10BAM				0x10	/* 10 bit address master	*/
#define	XGENE_I2C_CON_REST_EN			0x20	/* Restart command enable	*/
#define	XGENE_I2C_CON_SD				0x40	/* Slave disable		*/

#define	XGENE_I2C_TAR					0x04    /* Target adress register */
#define	XGENE_I2C_TAR_START				0x400
#define	XGENE_I2C_TAR_SPECIAL			0x800
#define	GET_SLAVE_ADDRESS(x)			(((x) >> 1) & 0x7F)

#define	XGENE_I2C_CLR_I					0x40	/* Clear interrupts register */

#define	XGENE_I2C_DATA_CMD 				0x10	/* Data to read/write register */
#define	READ_DIRECTION					0x100
#define	WRITE_DIRECTION					0x0
#define	CHECK_CMD_READ(x)				((x) & 0x1)

#define	XGENE_I2C_STATUS 				0x70	/* Status register */
#define	XGENE_I2C_STATUS_ACTIVITY		0x01

#define	XGENE_I2C_ENABLE				0x6C
#define	XGENE_I2C_ENABLE_EN				0x1

#define	XGENE_I2C_RAW_INTR				0x34	/* Raw interrupt status register */
#define	XGENE_I2C_RAW_INTR_TX_ABRT		0x40
#define	XGENE_I2C_RAW_INTR_STOP			0x200
#define	XGENE_I2C_RAW_INTR_START		0x400

#define	XGENE_I2C_TXFLR					0x74 	/* Number of data in transmit FIFO */
#define	XGENE_I2C_RXFLR					0x78 	/* Number of data in receive FIFO */

#define	XGENE_I2C_TX_TL					0x3C	/* Transmit FIFO threshold register */
#define	XGENE_I2C_RX_TL					0x38	/* Receive FIFO threshold register */

#define	XGENE_I2C_COMP_P1				0xF4	/* Component parameter register */
#define	XGENE_I2C_RX_LENGTH(x)			((((x) >> 8) & 0xFF) + 1)
#define	XGENE_I2C_TX_LENGTH(x)			((((x) >> 16) & 0xFF) + 1)

#define	XGENE_I2C_SS_SCL_H				0x14
#define	XGENE_I2C_SS_SCL_L				0x18
#define	XGENE_I2C_FS_SCL_H				0x1C
#define	XGENE_I2C_FS_SCL_L				0x20
#define	XGENE_I2C_HS_SCL_H				0x24
#define	XGENE_I2C_HS_SCL_L				0x28
#define	XGENE_I2C_SCL_HCNT(x,y,z)		((((x) * ((y) + (z)) + 500000) / 1000000) - 3)
#define	XGENE_I2C_SCL_LCNT(x,y,z)		((((x) * ((y) + (z)) + 500000) / 1000000) - 1)

#define	XGENE_I2C_TX_ABRT_SRC			0x80	/* Abort transmission */
#define	XGENE_I2C_TX_ABRT_SRC_NOACK7	0x01
#define	XGENE_I2C_TX_ABRT_SRC_SBTACK 	0x80
#define	XGENE_I2C_TX_ABRT_SRC_ARBLOST	0x1000

#define	XGENE_I2C_INTR_MASK				0x30	/* Interrupt mask register	*/
#define	XGENE_I2C_INTR_MASK_DIS			0x00

typedef struct xgene_i2c_softc {
		device_t			iicbus;
		device_t			dev;
		struct resource		*mem_res;
		struct resource		*irq_res;
		struct mtx			mutex;
		void				*intr_hdl;
	} i2c_softc_t;

struct xgene_i2c_conf {
		uint32_t endianness;
		uint32_t i2c_clk;
		uint32_t sclk_t;
	};

enum {
	I2C_SS = 0,
	I2C_FS,
	I2C_HS,
};

enum {
	I2C_SCL_HIGH = 0,
	I2C_SCL_LOW,
	I2C_SCL_TF,
};

static uint32_t xgene_i2c_scl[4][3] = {
	[I2C_SS] =	{4000, 4700, 300},
	[I2C_FS] =	{ 600, 1300, 300},
};

static uint32_t xgene_i2c_sclk_t[] = {
	[I2C_SS] = 10,
	[I2C_FS] = 3,
	[I2C_HS] = 1,
};

struct xgene_i2c_conf i2c_conf;

#endif /* XGENE_I2C_H_ */
