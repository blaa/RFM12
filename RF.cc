/***********************************
 * (C) 2009 by Tomasz bla Fortuna <bla@thera.be>.
 * License: GPLv3+ (See Docs/LICENSE)
 *
 * Desc: RF12 Radio module interface for AVR.
 *
 * Configures RFM12 module and allows basic send/recv operations.
 *
 * v1.1
 ********************/

#include <inttypes.h>
#include <util/crc16.h>

/* Changes sets of pin configuration and SPI speed.
 * + Some trivial things in Comm testcases.
 * My controller (Master==1) had only a LCD, while
 * slave a two-directional UART.
 * Allows sharing of this file between two different devices.
 */
#define RF_MASTER	0
#define RF_DEBUG	1

/***
 * AVR port configuration
 ***/
#define RF_PORT		PORTB
#define RF_PIN		PINB
#define RF_DDR		DDRB
#define RF_SS		PB4
#define RF_SCK		PB7
#define RF_MOSI		PB5
#define RF_MISO		PB6

#if RF_MASTER == 1
#	define RF_IRQ_PIN	PINB
#	define RF_IRQ_PORT	PORTB
#	define RF_IRQ_DDR	DDRB
#	define RF_IRQ_MASK	(1<<PB2)

#	define RF_IRQ_vect	INT2_vect	/* Select external interrupt */
#	define RF_IRQ_CONFIG()	RF_IRQ_CONFIG_(2)
#	define RF_IRQ_ON()	RF_IRQ_ON_(2)
#	define RF_IRQ_OFF()	RF_IRQ_OFF_(2)

#else

#	define RF_IRQ_PIN	PIND
#	define RF_IRQ_PORT	PORTD
#	define RF_IRQ_DDR	DDRD
#	define RF_IRQ_MASK	(1<<PD2)

#	define RF_IRQ_vect	INT0_vect
#	define RF_IRQ_CONFIG()	RF_IRQ_CONFIG_(0)
#	define RF_IRQ_ON()	RF_IRQ_ON_(0)
#	define RF_IRQ_OFF()	RF_IRQ_OFF_(0)

#endif

#define RF_SS_LOW()	do { RF_PORT &= (unsigned char)~(1<<RF_SS); } while (0)
#define RF_SS_HIGH()	do { RF_PORT |= 1<<RF_SS; } while (0)


/* Enable RF interrupt (ATmega644) on falling edge
 * of specified pin */
#define RF_IRQ_CONFIG_(num)				\
	do {						\
		RF_IRQ_DDR &= ~RF_IRQ_MASK;		\
		RF_IRQ_PORT |= RF_IRQ_MASK;		\
	/*		EICRA &= ~(1<<ISC00);	*/	\
	} while (0)

#define RF_IRQ_ON_(num)					\
	do {						\
		EIMSK |= (1<<(INT ## num));		\
	} while (0)

#define RF_IRQ_OFF_(num)				\
	do {						\
		EIMSK &= ~(1<<(INT ## num));		\
	} while (0)

/***
 * Set of RFM12 Configuration commands
 ***/
#include "RF_CFG.h"

/* Global config */
#define RF12_CONFIG	RF12_CFG_CMD(433, 12.0, RF12_EL | RF12_EF)

/* Power management */
#define RF12_PM_DEF	RF12_PM_CMD(RF12_EX | RF12_DC | RF12_ES | RF12_EBB)
#define RF12_PM_ECO	RF12_PM_CMD(RF12_DC)
#define RF12_PM_TX	RF12_PM_CMD(RF12_ET | RF12_ES | RF12_EX | RF12_DC)
#define RF12_PM_RX	RF12_PM_CMD(RF12_ER | RF12_EBB | RF12_ES | RF12_EX | RF12_DC)

/* This selects the communication channel
 * Argument should be between 96 and 3903 (0x0060 and 0x0F3F) */
//#define RF12_FQ		RF12_FQ_CMD(0x0640) /* 10 * 1 * (43 + 0x0640/4000) = 434Mhz */
#define RF12_FQ		RF12_FQ_CMD(0x0190) /* 10 * 1 * (43 + 0x0190/4000) = 431Mhz */

/* Select data rate */
//#define RF12_DR		RF12_DR_CMD(0x0047) /* 4.8kbps */
//#define RF12_DR		RF12_DR_CMD(0x0021) /* 10kbps */
//#define RF12_DR		RF12_DR_CMD(0x0010) /* 20kbps */
#define RF12_DR		RF12_DR_CMD(0x0005) /* 50kbps */
//#define RF12_DR		RF12_DR_CMD(0x0003) /* 85kbps */


#define RF12_RXCTL	RF12_RXCTL_CMD(ALWAYS, 134, 0, n103, RF12_VDI) /* Also try ALWAYS/FAST */
/* Clock recovery; CAL - start in fast more, switch to slow mode; CML - fast mode */
#define RF12_FILTER	RF12_FILTER_CMD(4, RF12_CAL | RF12_DIG)
//#define RF12_FILTER	RF12_FILTER_CMD(4, RF12_CML | RF12_DIG)

#define RF12_FIFO_OFF	RF12_FIFO_CMD(8, RF12_DRESET | RF12_FSYNC)
#define RF12_FIFO_ON	RF12_FIFO_CMD(8, RF12_DRESET | RF12_FSYNC | RF12_FF)

#define RF12_AFC	RF12_AFC_CMD(ATRECV, NORESTR, RF12_OE | RF12_EN) /* Also try ATRECV/ATPWR/INDEP */

#define RF12_TXCTL	RF12_TXCTL_CMD(0x05, 0) /* 90kHz */
#define RF12_WAKE	RF12_WAKE_CMD(0, 0)	/* Don't use */
#define RF12_DUTY	RF12_DUTY_CMD(0, 0)	/* Don't use */
//#define RF12_BATT	RF12_BATT_CMD(1_66, 0) /* from Datasheet */
#define RF12_BATT	RF12_BATT_CMD(10, 0) /* From library */

namespace RF {
	enum RF_Mode {TX, RX, DEF, ECO} CurMode;

	static uint16_t SC_tmp;
	static inline uint16_t SendCommand(const uint16_t Cmd)
	{
		RF_SS_LOW();

		SPDR = Cmd>>8;
		while (!(SPSR & (1<<SPIF)));
		SC_tmp = SPDR << 8;

		SPDR = Cmd & 0x00FF;
		while (!(SPSR & (1<<SPIF)));
		SC_tmp |= SPDR;

		RF_SS_HIGH();

		return SC_tmp;
	}

	static inline void VSendCommand(const uint16_t Cmd)
	{
		RF_SS_LOW();

		SPDR = Cmd>>8;
		while (!(SPSR & (1<<SPIF)));

		SPDR = Cmd & 0x00FF;

		/* Clear SPIF flag */
		while (!(SPSR & (1<<SPIF)));
		SC_tmp |= SPDR;

		RF_SS_HIGH();
	}

	static inline void Mode(enum RF_Mode Mode)
	{
		CurMode = Mode;
		switch (Mode) {
		case TX:
			VSendCommand(RF12_PM_TX);
			return;
		case RX:
			VSendCommand(RF12_PM_RX);
			VSendCommand(RF12_FIFO_OFF);
			VSendCommand(RF12_FIFO_ON);
			return;
		case DEF:
			VSendCommand(RF12_PM_DEF);
			return;
		case ECO:
		default:
			VSendCommand(RF12_PM_ECO);
			return;
		}
	}

	static inline void Transmit(uint8_t Byte)
	{
		VSendCommand(RF12_TXWR_CMD(Byte));
	}

	static inline uint16_t Receive()
	{
		while (RF_IRQ_PIN & RF_IRQ_MASK);
		return SendCommand(RF12_RXRD_CMD());
	}

	static inline void FIFOReset(void)
	{
		VSendCommand(RF12_FIFO_OFF);
		VSendCommand(RF12_FIFO_ON);
	}

	static inline void Init(void)
	{
		uint8_t tmp;
		const uint16_t Config[] = {
			RF12_CONFIG,
			RF12_PM_DEF,
			RF12_FQ,
			RF12_DR,

			RF12_RXCTL,
			RF12_FILTER,
			RF12_FIFO_OFF,
			RF12_AFC,

			RF12_TXCTL,
			RF12_WAKE,
			RF12_DUTY,
			RF12_BATT
		};

		/* Configure SPI */
		RF_PORT |= (1<<RF_SS);
		RF_DDR |= (1<<RF_MOSI) | (1<<RF_SCK) | (1<<RF_SS);

		/* Configure IRQ Port */
		RF_IRQ_DDR &= (uint8_t)~RF_IRQ_MASK;
		RF_IRQ_PORT |= RF_IRQ_MASK;

		/* SPI Enable; Master;
		 * fck / 16 seems ok for robot, while
		 * fck / 32 seems ok for transmitter */
#if RF_MASTER == 1
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1);
		SPSR |= (1<<SPI2X);
#else
		SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
//		SPSR |= (1<<SPI2X); /* Double the speed */
#endif
		tmp = SPSR; /* Clear SPIF */
		tmp = SPDR;

		for (tmp = 0; tmp < sizeof(Config)/sizeof(*Config); tmp++) {
/*			printf("RF: Cmd=0x%04X, num=%d\n", Config[tmp], tmp); */
			VSendCommand(Config[tmp]);
		}
		VSendCommand(0x00); /* Read status */
	}

#if RF_DEBUG
#if RF_MASTER
	void Status(void)
	{
		uint16_t tmp;
		char cnt = 0;

		tmp = SendCommand(0x0000);

		printf("RF Status = %04X\n", tmp);
		if (RF12_S_RGIT(tmp)) cnt += printf("RGIT/FFIT ");
		if (RF12_S_POR(tmp))  cnt += printf("POR ");
		if (cnt+9 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_RGUR(tmp))  cnt += printf("RGUR/FFOV ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_WKUP(tmp))  cnt += printf("WKUP ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_EXT(tmp))  cnt += printf("EXT ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_LBD(tmp))  cnt += printf("LBD ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_FFEM(tmp))  cnt += printf("FFEM ");
		if (cnt+8 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_RSSI(tmp))  cnt += printf("RSSI/ATS ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_DQD(tmp))  cnt += printf("DQD ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_CRL(tmp))  cnt += printf("CRL ");
		if (cnt+4 >= 20) printf("\n"), cnt = 0;
		if (RF12_S_ATGL(tmp))  cnt += printf("ATGL ");

		LCD::Refresh();
	}
#else
	void Status(void)
	{
		uint16_t tmp;
		char cnt = 0;

		if (!(RF_IRQ_PIN & RF_IRQ_MASK))
			printf("Incomming IRQ\n");

		/* Don't call in interrupts */
		tmp = SendCommand(0x0000);

		printf("RF Status = %04X ", tmp);
		if (RF12_S_RGIT(tmp)) cnt += printf("RGIT/FFIT ");
		if (RF12_S_POR(tmp))  cnt += printf("POR ");
		if (RF12_S_RGUR(tmp))  cnt += printf("RGUR/FFOV ");
		if (RF12_S_WKUP(tmp))  cnt += printf("WKUP ");
		if (RF12_S_EXT(tmp))  cnt += printf("EXT ");
		if (RF12_S_LBD(tmp))  cnt += printf("LBD ");
		if (RF12_S_FFEM(tmp))  cnt += printf("FFEM ");
		if (RF12_S_RSSI(tmp))  cnt += printf("RSSI/ATS ");
		if (RF12_S_DQD(tmp))  cnt += printf("DQD ");
		if (RF12_S_CRL(tmp))  cnt += printf("CRL ");
		if (RF12_S_ATGL(tmp))  cnt += printf("ATGL ");
		printf("\n");
	}

#endif /* RF_MASTER */
#endif /* RF_DEBUG */
}
