/***********************************
 * (C) 2009 by Tomasz bla Fortuna <bla@thera.be>.
 * License: GPLv3+ (See Docs/LICENSE)
 *
 * Desc: High level send/receive for RFM12 modules.
 *
 * Requires RF.cc module included. Provides interrupt-driven
 * TX/RX functionality with support for CRC checks and fast-drop
 * of invalid packets using a control byte.
 * It sends packets containing up to 256 bytes of data (might be changed).
 * Each packet contains data length, control byte and CRC and is
 * encapsulated in a frame starting with synchronization bytes for RFM
 * Final frame looks like this:
 * AA AA 2D D4 LENGTH, CONTROL, DATA, CRC
 *
 * TX/RX/CRC/Control byte might be freely compiled-in or not.
 *
 * At least v1.2
 * Code 85 cols wide.
 *
 * TX Example: (Take a look on testcases at the bottom also)
 * char *Buff = Comm::TXGetBuff();
 * int Length = sprintf(Buff, "Some data");
 * Comm::TXInit(Length);
 * Comm::TXWait();
 *
 * RX Example:
 * char *Buff; Comm::len_t Length;
 * Comm::RXInit();
 * Comm::RXWait(); // Block until something is read (Lookout for RXRETRY option) 
 * Buff = Comm::RXGetPacket(&Length);
 ********************/

/***
 * Comm functionality configuration
 ***/

/* Enable transmitter functions */
#define COMM_TX		1
/* Enable receiver functions */
#define COMM_RX		1

/* Use CRC16 CCITT to maintain data integrity */
#define COMM_CRC	1

/* Control byte
 *
 * This serves as an early-frame drop function 
 * + creates a place for some higher level data bits 
 * It won't protect data integrity inself.
 * Control byte has 4 bits free to use, and 4 duplicating
 * the length field.
 */
#define COMM_CTR	1

/* Shall we retry TX on buffer underrun? Not well tested - beware. */
#define COMM_TXRETRY	1

/* Shall we listen for another frame after we failed to correctly receive one?
 * After the RFM sees synchronization pattern (2D D4) it starts to send data to us.
 * If the data seems incorrect we can either turn RFM off or restart receiving.
 *
 * Turning this on when CTR and CRC is turned off mostly doesn't have any sense
 * (But there's still probability that length field will be incorrect - say equal 
 * to 0)
 */
#define COMM_RXRETRY	1

/* Enable statistics for either RX, TX or both */
#define COMM_STATS_TX	1
#define COMM_STATS_RX	1

/* Debug (printfs) */
#define COMM_DEBUG	0

/* Enable testcase compilation - requires stats compiled */
#define COMM_TESTCASES	1



/* Internal helper */
#define COMM_ANY_STATS	(COMM_STATS_RX || COMM_STATS_TX)

namespace Comm {
/*** Tranport layer configuration ***/

	/* Type of size field: uint8_t for <= 255, uint16_t for <= 65536.
	 * Changing this also requires some changes to the interrupt handling.
	 * */
	typedef uint8_t		len_t;

	/* CRC type */
#if COMM_CRC
	typedef uint16_t	crc_t;
	const crc_t CRCInit	= 0xFFFF;
#endif

 	/* Control byte type */
#if COMM_CTR
	typedef uint8_t		ctr_t;
#endif

	/* Maximal size of data which can be transfered in one packet.
	 * See CHECK0 */
	const int MaxMesgSize	= 256;

	/* Define size of additional packet bytes - without synchronization data */
#if COMM_CTR
#	define COMM_HEADSIZE	(sizeof(len_t) + sizeof(ctr_t))
#else
#	define COMM_HEADSIZE	(sizeof(len_t))
#endif /* CTR */

#if COMM_CRC
#	define COMM_TAILSIZE	(sizeof(crc_t))
#else
#	define COMM_TAILSIZE	(0)
#endif /* CRC */

#define COMM_PACKETSIZE (COMM_HEADSIZE + COMM_TAILSIZE)

	/* Frame synchronization data */
#if COMM_TX
#	define SynchData {0xAA, 0xAA, 0x2D, 0xD4}
	const uint8_t SynchSize = 4;
#endif /* TX */

	/* Structure of a packet */
	typedef struct {
		/* Message length */
		len_t Length;

		/* Packet type description */
#if COMM_CTR
		union {
			ctr_t Raw;
			struct {
				uint8_t Control : 4;
				uint8_t Config : 4;
			} C;
		} Type;
#endif /* CTR */

		/* Message body + 16 bit CRC at the end */
		char Mesg[MaxMesgSize + COMM_TAILSIZE];
	} packet_t;

#if COMM_TX
	/* Structure of a frame (frame = synch + packet) */
	typedef union {
		/* Raw frame */
		uint8_t Raw[SynchSize + sizeof(packet_t)];
		struct {
			/* Constant frame synchronization pattern */
			uint8_t Synch[SynchSize];
			/* Packet body */
			packet_t Packet;
		} C;
	} frame_t;
#endif /* TX */
	/** Working mode
	 * Simplified:
	 *
	 * MI - Idle
	 * MT - Transmitting frame
	 * Mt - Idle mode, Frame transmitted

	 * Mr - RX mode, nothing in
	 * MR - RX mode, something comming!
	 * MX - Idle mode, got a frame.
	 *
	 */

	enum Mode {
		MI=0,
#if COMM_RX
		Mr=0x01,
		MR=0x02,
		MX=0x04,
#endif /* RX */

#if COMM_TX
		Mt=0x10,
		MT=0x20,
#endif /* TX */
	};


	/*** State ***/
	static volatile struct {
		/* Buffers */
#if COMM_TX
		frame_t SendBuff;
		volatile uint8_t *SendCur, *SendEnd;
#endif /* TX */

#if COMM_RX
		packet_t RecvBuff;
		volatile uint8_t *RecvCur, *RecvEnd;
#endif /* RX */

		/* Mode of operation */
		enum Mode Mode;
		uint16_t Status;

		/* Stats */
#if COMM_ANY_STATS
		uint32_t PacketsTX;
		uint32_t PacketsRX;
		uint16_t CtrErr;	/* RGUR error, lenght range error,
					   or control byte error */
#endif

#if COMM_CRC
#if COMM_ANY_STATS
		uint16_t CRCErr;	/* CRC error */
#endif
		/* Temporary used in RX and TX */
		crc_t CRC;
#endif /* CRC */

#if COMM_TX /* Initialize constant synchronization data for TX mode */
	} State = {
		{SynchData}
	};
#else
	} State;
#endif /* TX */

	/***
	 * General functions
	 ***/
	/* Enter idle mode */
	static inline void Idle(void)
	{
		RF_IRQ_OFF();
		RF::Mode(RF::ECO);
		State.Mode = MI;
	}

	/** Initialize communication module */
	static inline void Init(void)
	{
		RF::Init();
		RF_IRQ_CONFIG();
		Idle();
	}

#if COMM_RX
	/***
	 * RX functions
	 ***/

	/** Initialize receiving */
	static inline void RXInit()
	{
#if COMM_TX
		/* Ensure the interrupt is off while we configure RFM */
		RF_IRQ_OFF();
#endif
		/* Swap modes and/or reset the FIFO */
		if (RF::CurMode != RF::RX)
			RF::Mode(RF::RX);

		RF::VSendCommand(0x0000); /* Clear Status (FFOV for e.g.) */
		State.Mode = Mr;
#if COMM_CRC
		State.CRC = CRCInit;
#endif /* CRC */
		State.RecvCur = (uint8_t *)&State.RecvBuff;
		/* We hit this when we know the frame length and control byte */
		State.RecvEnd = State.RecvCur + COMM_HEADSIZE - 1;
		RF_IRQ_ON();
	}

	/** Wait indefinetely for either an correct packet (RXRETRY==1)
	 * or for any packet receive trial.
	 */
	static inline void RXWait(void)
	{
		for (;;) {
			/* Should be Interrupt safe */
			if (State.Mode == MI)
				break;
			if (State.Mode == MX)
				break;
		}
	}

	/** Check if TX is ready */
	static inline char RXReady(void)
	{
		return (State.Mode == MX) || (State.Mode == MI);
	}

	/** Return RX buffer */
	static inline char *RXGetBuff(void)
	{
		return (char *)State.RecvBuff.Mesg;
	}


	/** Return received packet.
	 *
	 * \brief Check if we have anything to return. If not Length equals 0
	 * and we return correct buffer address. If we have something in
	 * buffer we calculate it's CRC and return if it's correct,
	 * otherwise we return -1, NULL.
	 */
	static inline char *RXGetPacket(len_t *Length)
	{
		if (State.Mode != MX) {
			*Length = 0;
			return NULL;
		}
		*Length = State.RecvBuff.Length;
		return (char *)State.RecvBuff.Mesg;
	}


#if COMM_CTR
	/** Returns Config nibble from received packet */
	static inline uint8_t RXGetConfig()
	{
		return State.RecvBuff.Type.C.Config;
	}
#endif

#endif /* RX */

#if COMM_TX
	/***
	 * TX functions
	 ***/

	/** Wait until the TX buffer frees */
	static inline void TXWait(void)
	{
		while (State.Mode == MT);
	}

	/** Check if TX is ready */
	static inline char TXReady(void)
	{
		return (State.Mode == Mt);
	}

	/** Get address to the TX buffer; */
	static inline char *TXGetBuff(void)
	{
		return (char *)State.SendBuff.C.Packet.Mesg;
	}

#if COMM_CTR
	/** Set config bits in packet header */
	static inline void TXConfig(uint8_t Cfg)
	{
		/* Copy 4 LSB bits into Config field */
		State.SendBuff.C.Packet.Type.C.Config = Cfg;
	}
#endif

	/**
	 * \brief
	 *   Initializes transmission of "Length" number
	 *   of bytes - sets control bits, calculates CRC
	 *   and enables interrupts.
	 *
	 * \param Length
	 *   Number of prepared bytes in TX buffer.
	 */
	static void TXInit(len_t Length)
	{
#if COMM_RX
		/* Ensure the interrupt is off while we configure RFM */
		RF_IRQ_OFF();
#endif

		/* Turn on transmitter fast so the receiver might synchronize */
		if (RF::CurMode != RF::TX)
			RF::Mode(RF::TX);

		State.SendBuff.C.Packet.Length = Length;
		/* Set to high nibble of low byte of length */
#if COMM_CTR
		State.SendBuff.C.Packet.Type.C.Control = ~Length;
#endif /* CTR */
		State.SendCur = State.SendBuff.Raw + 1;
		/* +1 - This will send a dummy byte in the end 
		 * just not to shut down TX too early. */
		State.SendEnd = State.SendCur + SynchSize + Length + COMM_PACKETSIZE + 1;

		/* Swap mode */
		State.Mode = MT;

		/* Start sending now. It could begin with two synchronization 
		 * bytes, and only then ask for our byte... But not if we
		 * have TX constantly on, so pass it this AA bytes.
		 * We have minimum 5 bytes before we reach for CRC,
		 * there should be time to calculate it.
		 */
		RF::Transmit(*State.SendBuff.Raw);
		RF::VSendCommand(0x0000); /* Clear Status (RGUR for e.g.) */
		RF_IRQ_ON();

#if COMM_CRC
		{
			static uint16_t i;
			static volatile uint8_t *Byte;
			Byte = State.SendBuff.Raw + SynchSize;
			
			State.CRC = CRCInit;
			/* Control bits set, start calculating CRC */
			i = Length + COMM_HEADSIZE;
			do {
				State.CRC = _crc_ccitt_update(State.CRC, *Byte);
				Byte++;
			} while (--i);
			/* Store CRC at the end of the message */
			*Byte = (uint8_t)(State.CRC & 0x00FF);
			Byte++;
			*Byte = (uint8_t)(State.CRC >> 8);
		}
#endif /* CRC */
	}

	/** Initialize RFM so it will start sending synchronization bytes already
	 * 
	 * Might be required only when interleaving TX/RX modes.
	 */
	static inline void TXPreInit(void)
	{
		State.Mode = MI;
		RF_IRQ_OFF();
		RF::Mode(RF::TX);
	}
#endif /* TX */


	/** Interrupt handling all communication
	 *
	 * Interrupt might be caused depending on minor mode (TX/RX)
	 * because of variety of events:
	 * 1) TX
	 *   - RGIT - register ready to input next byte
	 *   - RGUR - TX register under run
	 * 2) RX
	 *   - FFIT - Data ready to receive
	 *   - FFOV - FIFO overflow
	 * 3) Other
	 *   - WKUP, Supply voltage too low, Power On Reset.
	 *
	 * Currently we handle RGUR/FFOV and allow other
	 * interrupts to break our frame (while in RX)
	 * This might be solved by checking if FFIT is set in status.
	 *
	 */

	ISR(RF_IRQ_vect)
	{
		/*** Read status - leaving place for reading FIFO ***/
		RF_SS_LOW();
		SPDR = 0x00;
		while (!(SPSR & (1<<SPIF)));
		State.Status = SPDR;
		/* Start reading second byte */
		SPDR = 0x00;
		State.Status <<= 8;

		/*** Handle errors ***/
		if (RF12_S_RGUR(State.Status)) {
			/* Either the TX buffer was underruned,
			 * or RX buffer overrunned. Omit the rest of frame */
			if (COMM_DEBUG)
				printf("RGURERR!\n");

			while (!(SPSR & (1<<SPIF))); /* Read second byte */
			State.Status |= SPDR;
			RF_SS_HIGH();
#if COMM_TX
#if COMM_RX
			if (State.Mode == MT)
#endif /* RX */
			{
#if COMM_STATS_TX
				State.CtrErr++;
#endif /* STATS */

				if (COMM_TXRETRY) {
					/* TODO: Debug this. */
					State.SendCur = State.SendBuff.Raw + 1;
					RF::Transmit(*State.SendBuff.Raw);
				} else {
					State.SendCur = State.SendEnd = NULL;
					State.Mode = MI;
					RF::Mode(RF::DEF);
					RF_IRQ_OFF();
				}
				return;
			}
#endif /* TX */

#if COMM_RX
#if COMM_STATS_RX
				State.CtrErr++;
#endif /* STATS */

			/* RX mode - Reset receiver; we will wait 
			 * for another frame. */
			goto ResetRX;
#endif /* RX */
			return;
		} /* RGUR CHECK */

		/*** Handle TX ***/
#if COMM_TX
#if COMM_RX
		if (State.Mode == MT)
#endif /* RX */
		{
			while (!(SPSR & (1<<SPIF))); /* Read second byte */
			State.Status |= SPDR;
			RF_SS_HIGH();

			/* RGIT. Send next byte */
			if (State.SendCur == State.SendEnd) {
				/* Dummy byte already sent; shutdown transmitter */
				State.Mode = Mt;
#if COMM_STATS_TX
				State.PacketsTX++;
#endif
				/* We must leave TX on, so receiver will be 
				 * able to synchronize to our clock fast enough.
				 * Documentation states something different,
				 * but I had plenty of packets dropped on 
				 * the synchronization bytes when TX was being
				 * shutdown.
				 */
  				/* RF::Mode(RF::DEF); */

				/* Close our ear on incoming RGURs */
				RF_IRQ_OFF();
			} else {
				RF::Transmit(*State.SendCur);
				State.SendCur++;
			}
			return;
		}
#endif /* TX */

#if COMM_RX
		/*** Handle RX ***/

		/* We are in either RX mode:
		 * Mr - reading head, not knowing length of package yet
		 * MR - receiving packet body
		 */

		/* Wait for the second status byte */
		while (!(SPSR & (1<<SPIF)));
		State.Status |= SPDR;

		/* Discard 2. status byte and read FIFO */
		SPDR = 0x00;
		while (!(SPSR & (1<<SPIF)));
		RF_SS_HIGH();

		/* Store byte and calculate CRC */
		*State.RecvCur = SPDR;
#if COMM_CRC
		State.CRC = _crc_ccitt_update(State.CRC, SPDR);
#endif /* CRC */
		/* Handle end of header and end of body */
		if (State.RecvCur == State.RecvEnd) {
			if (State.Mode == Mr) {
				/* Mode == Mr; reading header of the package */

				/* We know length and have received the control byte */
#if COMM_CTR
				if (State.RecvBuff.Type.C.Control != 
				    ((~State.RecvBuff.Length) & 0x0F)) {
#if COMM_STATS_RX
					State.CtrErr++;
#endif /* STATS */
					goto ResetRX;
				}
#endif /* CTR */

				/* CHECK0: If maxsize of len_t is greater than
				 * MaxMesgSize - we should swap those ifs  
				 */
				/* if (State.RecvBuff.Length < 1 ||
				   State.RecvBuff.Length > MaxMesgSize) { */
				if (State.RecvBuff.Length == 0) {
#if COMM_STATS_RX
					State.CtrErr++;
#endif /* STATS */
					goto ResetRX;
				}

				/* Seems ok - replace RecvEnd position. */
				State.RecvEnd = State.RecvCur + 
					State.RecvBuff.Length + COMM_TAILSIZE;
				State.Mode = MR;
			} else {
				/* Mode == MR; reading body of packet */

#if COMM_CRC
				/* Check if received correctly */
				if (State.CRC == 0x0000) {
#endif /* CRC */
					/* CRC correct; Frame received! */
					RF::Mode(RF::DEF);
					RF_IRQ_OFF();
#if COMM_STATS_RX
					State.PacketsRX++;
#endif /* STATS */
					State.Mode = MX;
					return;
#if COMM_CRC
				}

				/* CRC Error */
#if COMM_STATS_RX
				State.CRCErr++;
#endif
				goto ResetRX;
#endif /* CRC */
			}
		}
		State.RecvCur++;
		return;

		/* Reset the RX machinery */
	ResetRX:
		if (COMM_RXRETRY) {
			RF::FIFOReset();
			State.Mode = Mr;
#if COMM_CRC
			State.CRC = CRCInit;
#endif /* CRC */
			State.RecvCur = (uint8_t *)&State.RecvBuff;
			State.RecvEnd = State.RecvCur + COMM_HEADSIZE - 1;
		} else {
			State.Mode = MI;
			RF::Mode(RF::DEF);
			State.RecvCur = State.RecvEnd = NULL;
			RF_IRQ_OFF();
		}
#endif /* RX */
	}


/*************************
 * Testcases / Examples
 * 
 * For most of them you should be able to use stdout and stdin some 
 * uart or LCD.
 ************************/

#if COMM_RX
	static inline void Testcase_RX()
	{

		unsigned int c;
		char *Buff;
		Comm::len_t Length;

		/* Stabilize hardware */
		Util::SDelay(1);

		/* Initialize Comm module */
		Comm::Init();
		sei();
		c = 0;
		for (;;)
		{
			/* Start receiving */
			Comm::RXInit();
			/* Wait for frame */
			Comm::RXWait();
			Buff = Comm::RXGetPacket(&Length);
			Buff[Length] = '\0';
			printf("Got; MODE=%02X; Len=%u MSG=%s\n", Comm::State.Mode, Length, Buff);
			c++;
#if COMM_CRC
			printf("RX: %lu Err: %u/%u\n",
			       Comm::State.PacketsRX, 
			       Comm::State.CtrErr,
			       Comm::State.CRCErr);
#else
			printf("RX: %lu Err: %u\n",
			       Comm::State.PacketsRX,
			       Comm::State.CtrErr);
#endif /* CRC */
#if RF_MASTER
			LCD::Refresh();
			LCD::ClearScreen();
#endif
		}
	}

#if RF_MASTER
	/* Simulate a terminal 
	 * Data received via RF are shown on LCD.
	 * ~ clears screen, when screen is full it's cleared.
	 * Statistics shown on the last line.
	 */
	static inline void Testcase_UART_RX()
	{
		unsigned int c;
		Comm::len_t Length, i;
		char *Buff;
		char x, y;

		/* Stabilize hardware */
		Util::SDelay(1);

		/* Initialize Comm module */
		Comm::Init();
		sei();
		c = 0;
	        putchar(0x01); /* Enable overwrite mode */
		LCD::ClearScreen();
		printf("Terminal running\n");
		LCD::Refresh();
		for (;;)
		{
			/* Start receiving */
			Comm::RXInit();
			/* Wait for frame */
			Comm::RXWait();
			Buff = RXGetPacket(&Length);

			for (i=0; i < Length; i++) {
				x = LCD::State.x;
				y = LCD::State.y;

				if (Buff[i] == '~') {
					LCD::ClearScreen();
					x=y=0;
				} else if (Buff[i] == '\r' || Buff[i] == '\n') {
					y++;
					x = 0;
				} else if (Buff[i] == 0x08) { /* Backspace */
					if (x>0) {
						x--;
						LCD::GotoXY(x,y);
						putchar(' ');
					}
				} else {
					putchar(Buff[i]);
					x++;
				}
				if (x==21) {
					x=0;
					y++;
				}

				if (y==7) {
					LCD::ClearScreen();
					y = x = 0;
				}
			}
			LCD::GotoXY(0,7);
#if COMM_CRC
			printf("RX%lu Err:%u/%u",
			       Comm::State.PacketsRX, Comm::State.CtrErr, 
			       Comm::State.CRCErr);
#else
			printf("RX%lu Err:%u",
			       Comm::State.PacketsRX, Comm::State.CtrErr);
#endif
			LCD::GotoXY(x, y); 
			LCD::Refresh(); 
		}
	}

#endif /* RF_MASTER */
#endif /* RX */

#if COMM_TX
	static inline void Testcase_TX(void)
	{
		/* We should be able to use printf() - some uart or something */
		char *Buff;
		unsigned long i=0;
		int Length;

		/* Stabilize hardware */
		Util::SDelay(1);

		/* Start Comm module */
		Comm::Init();

		/* Initialize buffer */
		Buff = Comm::TXGetBuff();
		Length = 0x13;
		strncpy(Buff,
			"\x60\x61\x62\x63\x64\x65\x66\x67\x68\x69"
			"\x6a\x6b\x6c\x6d\x6e\x6f\x70\x71\x72\x73", Length);

		sei();
		for (;;)
		{
			i++;
			/* Start transmitting */
			Comm::TXInit(Length);
			/* Wait until finish */
			Comm::TXWait();

			if (i % 100 == 0) {
				/* Periodically display debug */

				printf("PTx=%lu\n", Comm::State.PacketsTX);
				RF::Status();
#if RF_MASTER
				LCD::Refresh();
				LCD::ClearScreen();
#endif /* RF_MASTER  */
			}
		}
	}

	/* Transmit in packets what you get via UART 
	 * Warning: getchar() non-blocking */
	static inline void Testcase_UART_TX(void)
	{
		char *Buff;
		int i=0;
		int Length;

		/* Stabilize hardware */
		Util::SDelay(1);

		/* Start Comm module */
		Comm::Init();

		/* Initialize buffer */
		Buff = Comm::TXGetBuff();
		Length = 1;
		sei();
		for (;;)
		{
			/* Gather data */
			Length = 0;
			while (Length < 255) {
				i = getchar();
				if (i == -1) {
					if (Length == 0)
						continue;
					else
						break;
				}
				Buff[Length++] = (unsigned char)i;
			}

			Comm::TXInit(Length);
			printf("%d\n", Length);
			Comm::TXWait();
		}
	}

	/* Send same text with counter over and over */
	static inline void Testcase_AUTO_UART_TX(void)
	{
		char *Buff;
		unsigned int i=0;
		int Length;

		/* Stabilize hardware */
		Util::SDelay(1);

		printf("Initializing Comm\n");
		/* Start Comm module */
		Comm::Init();

		/* Initialize buffer */
		printf("Initializing buffer\n");
		Buff = Comm::TXGetBuff();
		sei();
		for (;;)
		{
			Length = sprintf(Buff, "~This is PX no %u", i);
			printf("Transfering\n");
			Comm::TXInit(Length);
			Comm::TXWait();
			
			Util::SDelay(1);
			i++;
		}
	}

#endif /* TX */


#if COMM_RX && COMM_TX
	static inline void Testcase_Interleaved(void)
	{
		char *Buff;
		unsigned long i=0;
		int Length;

		/* Stabilize hardware */
		Util::SDelay(1);

		/* Start Comm module */
		Comm::Init();

		/* Initialize buffer */
		Buff = Comm::TXGetBuff();
		Length = 0x13;
		strncpy(Buff,
			"\x60\x61\x62\x63\x64\x65\x66\x67\x68\x69"
			"\x6a\x6b\x6c\x6d\x6e\x6f\x70\x71\x72\x73", Length);

		sei();
		for (;;)
		{
			i++;
			/* Start transmitting */
			TXInit(Length);
			/* Wait until finish */
			Comm::TXWait();

			/* Swap mode to RX */
			Comm::RXInit();

			/* Wait some time */
			int WaitCnt = 0;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;
			if (Comm::State.Mode != MX) _delay_ms(5), WaitCnt++;

			/* Pre initialize TX */
			Comm::TXPreInit();


#if COMM_CRC
#if !RF_MASTER
			printf("TX/RX %lu/%lu Err: %u/%u W: %d\n",
			       Comm::State.PacketsTX,
			       Comm::State.PacketsRX, 
			       Comm::State.CtrErr,
			       Comm::State.CRCErr,
			       WaitCnt);
#else
			printf("\x01TX/RX %lu/%lu  \nErr: %u/%u W:%d \n",
			       Comm::State.PacketsTX,
			       Comm::State.PacketsRX, 
			       Comm::State.CtrErr,
			       Comm::State.CRCErr,
			       WaitCnt);
#endif
#else
			printf("TX/RX: %lu Err: %u\n",
			       Comm::State.PacketsTX,
			       Comm::State.PacketsRX,
			       Comm::State.CtrErr);
#endif /* CRC */

#if RF_MASTER
			LCD::Refresh();
			LCD::ClearScreen();
#endif /* RF_MASTER  */
		}
	}
#endif /* TX + RX */


}
