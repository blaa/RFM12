/* v1.0beta part of RF/COMM set */

#ifndef _RF_CFG_H_
#define _RF_CFG_H_

/***
 * Configuration Setting Command 
 ***/
#define RF12_CFG_BASE	0x8000

/* Crystal load capacitance */
#define RF12_CLC(x)	( ((char)((x-8.5)*2.0)) & 0x0F  )

#define RF12_CFG_CMD(BAND, CLC, OPT) \
	(RF12_CFG_BASE | (RF12_B ## BAND) | RF12_CLC(CLC) | OPT)

#define RF12_EL		(1<<7)	/* Internal data register enable */
#define RF12_EF		(1<<6)	/* Enable FIFO mode */
#define RF12_B315	0	/* Frequency band selection */
#define RF12_B433	(1<<4)
#define RF12_B868	(1<<5)
#define RF12_B915	((1<<5)|(1<<4))

/***
 * Power management command 
 ***/
#define RF12_PM_BASE	0x8200
#define RF12_PM_CMD(OPT)	(RF12_PM_BASE | OPT)

#define RF12_ER		(1<<7)	/* Enable receiver */
#define RF12_EBB	(1<<6)	/* Baseband on/off */
#define RF12_ET		(1<<5)	/* Enable transmitter */
#define RF12_ES		(1<<4)	/* Synthetisizer on/off */
#define RF12_EX		(1<<3)	/* Oscillator on/off */
#define RF12_EB		(1<<2)	/* Low batt detector */
#define RF12_EW		(1<<1)	/* Wake-up timer enable */
#define RF12_DC		(1<<0)	/* Disable clock output */


/***
 * Frequency setting command 
 ***/
#define RF12_FQ_BASE	0xA000
#define RF12_FQ_CMD(F)	(RF12_FQ_BASE | F)
/* Parameter F in range 36-3903 is ORed with command 
 * Then one might calculate synthesizer band center frequency:
 * f0 = (10 * C1 * (C2 + F/4000)
 * 315_C1=1	433_C1=1	868_C1=2	915_C1=3
 * 315_C2=31	433_C2=43	868_C2=43	915_C2=30
 */

/***
 * Data rate command
 ***/
#define RF12_DR_BASE		0xC600
#define RF12_DR_CMD(csR)	(RF12_DR_BASE | csR)

/* 
 * 1 cs bit and 7 bit parameter R is ORed with command.
 * Bit rate might be calculated like this:
 * BR = 10000 / 29 / (R+1) / (1+cs*7) [kbps]
 * Therefore we must set R according to:
 * R = (10000 / 29 / (1 + cs*7) / BR) - 1
 */

/***
 * Receiver control command 
 ***/
#define RF12_RXCTL_BASE	0x9000

#define RF12_VDI	(1<<10)	/* 0 - pin 20 is interrupt input; 1 - VDI output */
#define RF12_D1		(1<<9)	/* VDI signal response time */
#define RF12_D0		(1<<8)
#define RF12_VDI_FAST	0
#define RF12_VDI_MEDIUM	RF12_D0
#define RF12_VDI_SLOW	RF12_D1
#define RF12_VDI_ALWAYS	(RF12_D1|RF12_D0)

#define RF12_I2		(1<<7)	/* Receiver baseband bandwidth select */
#define RF12_I1		(1<<6)
#define RF12_I0		(1<<5)
#define RF12_BW_400	RF12_I0)	/* [kHz] */
#define RF12_BW_340	RF12_I1
#define RF12_BW_270	(RF12_I1 | RF12_I0)
#define RF12_BW_200	RF12_I2
#define RF12_BW_134	(RF12_I2 | RF12_I0)
#define RF12_BW_67	(RF12_I2 | RF12_I1)

#define RF12_G1		(1<<4)	/* LNA gain select relative to maximum [dB] (all negative) */
#define RF12_G0		(1<<3)
#define RF12_LNA_0	0
#define RF12_LNA_n6	RF12_G0
#define RF12_LNA_n14	RF12_G1
#define RF12_LNA_n20	(RF12_G0|RF12_G1)

#define RF12_R2		(1<<2)	/* RSSI detector treshold [dBm] negative */
#define RF12_R1		(1<<1)
#define RF12_R0		(1<<0)
#define RF12_RSSI_n103	0
#define RF12_RSSI_n97	(RF12_R0)
#define RF12_RSSI_n91	(RF12_R1)
#define RF12_RSSI_n85	(RF12_R1 | RF12_R0)
#define RF12_RSSI_n79	(RF12_R2)
#define RF12_RSSI_n73	(RF12_R2 | RF12_R0)
#define RF12_RSSI_n67	(RF12_R2 | RF12_R1)
#define RF12_RSSI_n61	(RF12_R2 | RF12_R1 | RF12_R0)	/* There's error here in documentation; */

#define RF12_RXCTL_CMD(RESPONSE, BW, LNA, RSSI, OPT)	\
	(RF12_RXCTL_BASE | (RF12_VDI_ ## RESPONSE) |	\
	 (RF12_LNA_ ## LNA) |(RF12_BW_ ## BW) |		\
	 (RF12_RSSI_ ## RSSI) | OPT)


/***
 * Data filter command 
 ***/
#define RF12_FILTER_BASE	0xC228
#define RF12_FILTER_CMD(DQD, OPT)	(RF12_FILTER_BASE | (DQD & (0x07)) | OPT)

#define RF12_CAL	(1<<7)	/* Start in fast mode; switch to slow if locked */
#define RF12_CML	(1<<6)	/* 1 - fast mode, 0 - slow mode */
/* S bit: */
#define RF12_ANA	(1<<4)	/* 1 - Analog RC filter */
#define RF12_DIG	(0)	/* 0 - Digital filter */

/* F2 to F0 bits - DQD Treshold parameter */



/***
 * FIFO and reset mode command 
 ***/
#define RF12_FIFO_BASE	0xCA00

#define RF12_FIFOINT(x)	(x<<4)	/* Raise interrupt after x bits received */

#define RF12_FIFO_CMD(INTBITS, OPT) (RF12_FIFO_BASE | RF12_FIFOINT(INTBITS) | OPT)

#define RF12_FALWAYS	(1<<2)	/* FIFO fill start: 1 - always; */
#define RF12_FSYNC	(0<<2)	/* 0 - synchron pattern */

#define RF12_FF		(1<<1)	/* FIFO fill enable; stops when this bit is cleared */
#define RF12_DRESET	(1<<0)	/* 1 - Disable high sensitive reset */

/***
 * Receiver FIFO read command 
 ***/
#define RF12_RXRD_BASE	0xB000
#define RF12_RXRD_CMD()	(RF12_RXRD_BASE)

/* Enables receiver to read 8 bits of FIFO data when EF is enabled */

/***
 * AFC command 
 ***/
#define RF12_AFC_BASE	0xC400

#define RF12_NOAUTO	0
#define RF12_ATPWR	(1<<6)
#define RF12_ATRECV	(2<<6)
#define RF12_INDEP	(3<<6)

#define RF12_NORESTR	0	/* No deviation restriction */
#define RF12_RESTR1	(1<<4)	/* +15 fres to -16 fres */
#define RF12_RESTR2	(2<<4)	/* +7 fres to -8 fres */
#define RF12_RESTR3	(3<<4)	/* +3 fres to -4 fres */

#define RF12_ST		(1<<3)
#define RF12_FI		(1<<2)
#define RF12_OE		(1<<1)
#define RF12_EN		(1<<0)


#define RF12_AFC_CMD(AUTO, RESTR, OPT)					\
	(RF12_AFC_BASE | (RF12_ ## AUTO) | (RF12_ ## RESTR) | OPT )

/***
 * TX configuration control command 
 ***/
#define RF12_TXCTL_BASE	0x9800
#define RF12_TXCTL_CMD(F, PWR)	(RF12_TXCTL_BASE | (RF12_TXPWR_ ## PWR) | (F<<4))

/* Bits 8-4 form a M parameter; resulting frequency is
 * fout = f0 + (-1)^(SIGN) * (M+1) * (15kHz)
 * SIGN = MP xor FSK input
 */

/* Transmitter power [dB] relative to max */
#define RF12_TXPWR_0	0
#define RF12_TXPWR_n3	1
#define RF12_TXPWR_n6	2
#define RF12_TXPWR_n9	3
#define RF12_TXPWR_n12	4
#define RF12_TXPWR_n15	5
#define RF12_TXPWR_n18	6
#define RF12_TXPWR_n21	7


/***
 * Transmitter register write command 
 ***/
#define RF12_TXWR_BASE	0xB800
#define RF12_TXWR_CMD(BYTE)	(RF12_TXWR_BASE | ((unsigned char)BYTE))

/* Writes 8 bits into transmitter data register */

/***
 * Wake-up timer command 
 ***/
#define RF12_WAKE_BASE	0xE000

/* Bits 12 to 8 form R; 7 to 0 form M
 * wake-up time period is calculated like this:
 * M * 2^R [ms]
 * R in range 0 - 29
 */
#define RF12_WAKE_MR(M, R) (R<<8 | M)

#define RF12_WAKE_CMD(M, R)	(RF12_WAKE_BASE | RF12_WAKE_MR(M, R))

/***
 * Low duty-cycle command 
 ***/
#define RF12_DUTY_BASE	0xC800

#define RF12_GDUTY(D)	(D<<1)
/* Duty cycle is calculated like this:
 * Duty cycle = (D * 2 + 1) / M * 100%  (M from Wake_cmd))
 */
#define RF12_DUTY_CMD(D, OPT)	(RF12_DUTY_BASE | RF12_GDUTY(D) | OPT)

#define RF12_EN		(1<<0)	/* Enable low duty-cycle mode */

/***
 * Low battery + clock divider command 
 ***/
#define RF12_BATT_BASE	0xC000

#define RF12_TRESH_V(V)	(V)	/* Set treshold voltage of detector */
/* V = 2.2 + V * 0.1 [V] */
#define RF12_BATT_CMD(CLK, V)	(RF12_BATT_BASE | RF12_TRESH_V(V) | (RF12_CLK_ ## CLK))

#define RF12_CLK_1	(0<<5)	/* Clock divider configuration */
#define RF12_CLK_1_25	(1<<5)
#define RF12_CLK_1_66	(2<<5)
#define RF12_CLK_2	(3<<5)
#define RF12_CLK_2_5	(4<<5)
#define RF12_CLK_3_33	(5<<5)
#define RF12_CLK_5	(6<<5)
#define RF12_CLK_10	(7<<5)

/***
 * Status word analize 
 ***/
#define RF12_S_RGIT(SW) (SW & (1<<15))
#define RF12_S_FFIT(SW) (SW & (1<<15))
#define RF12_S_POR(SW)  (SW & (1<<14))
#define RF12_S_RGUR(SW) (SW & (1<<13))
#define RF12_S_FFOV(SW) (SW & (1<<13))
#define RF12_S_WKUP(SW) (SW & (1<<12))
#define RF12_S_EXT(SW)  (SW & (1<<11))
#define RF12_S_LBD(SW)  (SW & (1<<10))
#define RF12_S_FFEM(SW) (SW & (1<<9))
#define RF12_S_RSSI(SW) (SW & (1<<8))
#define RF12_S_ATS(SW)  (SW & (1<<8))
#define RF12_S_DQD(SW)  (SW & (1<<7))
#define RF12_S_CRL(SW)  (SW & (1<<6))
#define RF12_S_ATGL(SW) (SW & (1<<5))
#define RF12_S_OFFS(SW) (SW & 0x001F)


#endif
