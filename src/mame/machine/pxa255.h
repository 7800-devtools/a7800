// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**************************************************************************
 *
 * Intel PXA255 on-chip peripheral emulation
 *
 * Mostly-incomplete implementation by Ryan Holtz
 *
 **************************************************************************/


/*
  PXA255 DMA controller

  pg. 151 to 182, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_DMA_BASE_ADDR    (0x40000000)
#define PXA255_DCSR0            (PXA255_DMA_BASE_ADDR + 0x00000000)
#define PXA255_DCSR1            (PXA255_DMA_BASE_ADDR + 0x00000004)
#define PXA255_DCSR2            (PXA255_DMA_BASE_ADDR + 0x00000008)
#define PXA255_DCSR3            (PXA255_DMA_BASE_ADDR + 0x0000000c)
#define PXA255_DCSR4            (PXA255_DMA_BASE_ADDR + 0x00000010)
#define PXA255_DCSR5            (PXA255_DMA_BASE_ADDR + 0x00000014)
#define PXA255_DCSR6            (PXA255_DMA_BASE_ADDR + 0x00000018)
#define PXA255_DCSR7            (PXA255_DMA_BASE_ADDR + 0x0000001c)
#define PXA255_DCSR8            (PXA255_DMA_BASE_ADDR + 0x00000020)
#define PXA255_DCSR9            (PXA255_DMA_BASE_ADDR + 0x00000024)
#define PXA255_DCSR10           (PXA255_DMA_BASE_ADDR + 0x00000028)
#define PXA255_DCSR11           (PXA255_DMA_BASE_ADDR + 0x0000002c)
#define PXA255_DCSR12           (PXA255_DMA_BASE_ADDR + 0x00000030)
#define PXA255_DCSR13           (PXA255_DMA_BASE_ADDR + 0x00000034)
#define PXA255_DCSR14           (PXA255_DMA_BASE_ADDR + 0x00000038)
#define PXA255_DCSR15           (PXA255_DMA_BASE_ADDR + 0x0000003c)
	#define PXA255_DCSR_RUN         (0x80000000)
	#define PXA255_DCSR_NODESCFETCH (0x40000000)
	#define PXA255_DCSR_STOPIRQ     (0x20000000)
	#define PXA255_DCSR_REQPEND     (0x00000100)
	#define PXA255_DCSR_STOPSTATE   (0x00000008)
	#define PXA255_DCSR_ENDINTR     (0x00000004)
	#define PXA255_DCSR_STARTINTR   (0x00000002)
	#define PXA255_DCSR_BUSERRINTR  (0x00000001)
#define PXA255_DINT             (PXA255_DMA_BASE_ADDR + 0x000000f0)
#define PXA255_DRCMR0           (PXA255_DMA_BASE_ADDR + 0x00000100)
#define PXA255_DRCMR1           (PXA255_DMA_BASE_ADDR + 0x00000104)
#define PXA255_DRCMR2           (PXA255_DMA_BASE_ADDR + 0x00000108)
#define PXA255_DRCMR3           (PXA255_DMA_BASE_ADDR + 0x0000010c)
#define PXA255_DRCMR4           (PXA255_DMA_BASE_ADDR + 0x00000110)
#define PXA255_DRCMR5           (PXA255_DMA_BASE_ADDR + 0x00000114)
#define PXA255_DRCMR6           (PXA255_DMA_BASE_ADDR + 0x00000118)
#define PXA255_DRCMR7           (PXA255_DMA_BASE_ADDR + 0x0000011c)
#define PXA255_DRCMR8           (PXA255_DMA_BASE_ADDR + 0x00000120)
#define PXA255_DRCMR9           (PXA255_DMA_BASE_ADDR + 0x00000124)
#define PXA255_DRCMR10          (PXA255_DMA_BASE_ADDR + 0x00000128)
#define PXA255_DRCMR11          (PXA255_DMA_BASE_ADDR + 0x0000012c)
#define PXA255_DRCMR12          (PXA255_DMA_BASE_ADDR + 0x00000130)
#define PXA255_DRCMR13          (PXA255_DMA_BASE_ADDR + 0x00000134)
#define PXA255_DRCMR14          (PXA255_DMA_BASE_ADDR + 0x00000138)
#define PXA255_DRCMR15          (PXA255_DMA_BASE_ADDR + 0x0000013c)
#define PXA255_DRCMR16          (PXA255_DMA_BASE_ADDR + 0x00000140)
#define PXA255_DRCMR17          (PXA255_DMA_BASE_ADDR + 0x00000144)
#define PXA255_DRCMR18          (PXA255_DMA_BASE_ADDR + 0x00000148)
#define PXA255_DRCMR19          (PXA255_DMA_BASE_ADDR + 0x0000014c)
#define PXA255_DRCMR20          (PXA255_DMA_BASE_ADDR + 0x00000150)
#define PXA255_DRCMR21          (PXA255_DMA_BASE_ADDR + 0x00000154)
#define PXA255_DRCMR22          (PXA255_DMA_BASE_ADDR + 0x00000158)
#define PXA255_DRCMR23          (PXA255_DMA_BASE_ADDR + 0x0000015c)
#define PXA255_DRCMR24          (PXA255_DMA_BASE_ADDR + 0x00000160)
#define PXA255_DRCMR25          (PXA255_DMA_BASE_ADDR + 0x00000164)
#define PXA255_DRCMR26          (PXA255_DMA_BASE_ADDR + 0x00000168)
#define PXA255_DRCMR27          (PXA255_DMA_BASE_ADDR + 0x0000016c)
#define PXA255_DRCMR28          (PXA255_DMA_BASE_ADDR + 0x00000170)
#define PXA255_DRCMR29          (PXA255_DMA_BASE_ADDR + 0x00000174)
#define PXA255_DRCMR30          (PXA255_DMA_BASE_ADDR + 0x00000178)
#define PXA255_DRCMR31          (PXA255_DMA_BASE_ADDR + 0x0000017c)
#define PXA255_DRCMR32          (PXA255_DMA_BASE_ADDR + 0x00000180)
#define PXA255_DRCMR33          (PXA255_DMA_BASE_ADDR + 0x00000184)
#define PXA255_DRCMR34          (PXA255_DMA_BASE_ADDR + 0x00000188)
#define PXA255_DRCMR35          (PXA255_DMA_BASE_ADDR + 0x0000018c)
#define PXA255_DRCMR36          (PXA255_DMA_BASE_ADDR + 0x00000190)
#define PXA255_DRCMR37          (PXA255_DMA_BASE_ADDR + 0x00000194)
#define PXA255_DRCMR38          (PXA255_DMA_BASE_ADDR + 0x00000198)
#define PXA255_DRCMR39          (PXA255_DMA_BASE_ADDR + 0x0000019c)
#define PXA255_DDADR0           (PXA255_DMA_BASE_ADDR + 0x00000200)
#define PXA255_DSADR0           (PXA255_DMA_BASE_ADDR + 0x00000204)
#define PXA255_DTADR0           (PXA255_DMA_BASE_ADDR + 0x00000208)
#define PXA255_DCMD0            (PXA255_DMA_BASE_ADDR + 0x0000020c)
#define PXA255_DDADR1           (PXA255_DMA_BASE_ADDR + 0x00000210)
#define PXA255_DSADR1           (PXA255_DMA_BASE_ADDR + 0x00000214)
#define PXA255_DTADR1           (PXA255_DMA_BASE_ADDR + 0x00000218)
#define PXA255_DCMD1            (PXA255_DMA_BASE_ADDR + 0x0000021c)
#define PXA255_DDADR2           (PXA255_DMA_BASE_ADDR + 0x00000220)
#define PXA255_DSADR2           (PXA255_DMA_BASE_ADDR + 0x00000224)
#define PXA255_DTADR2           (PXA255_DMA_BASE_ADDR + 0x00000228)
#define PXA255_DCMD2            (PXA255_DMA_BASE_ADDR + 0x0000022c)
#define PXA255_DDADR3           (PXA255_DMA_BASE_ADDR + 0x00000230)
#define PXA255_DSADR3           (PXA255_DMA_BASE_ADDR + 0x00000234)
#define PXA255_DTADR3           (PXA255_DMA_BASE_ADDR + 0x00000238)
#define PXA255_DCMD3            (PXA255_DMA_BASE_ADDR + 0x0000023c)
#define PXA255_DDADR4           (PXA255_DMA_BASE_ADDR + 0x00000240)
#define PXA255_DSADR4           (PXA255_DMA_BASE_ADDR + 0x00000244)
#define PXA255_DTADR4           (PXA255_DMA_BASE_ADDR + 0x00000248)
#define PXA255_DCMD4            (PXA255_DMA_BASE_ADDR + 0x0000024c)
#define PXA255_DDADR5           (PXA255_DMA_BASE_ADDR + 0x00000250)
#define PXA255_DSADR5           (PXA255_DMA_BASE_ADDR + 0x00000254)
#define PXA255_DTADR5           (PXA255_DMA_BASE_ADDR + 0x00000258)
#define PXA255_DCMD5            (PXA255_DMA_BASE_ADDR + 0x0000025c)
#define PXA255_DDADR6           (PXA255_DMA_BASE_ADDR + 0x00000260)
#define PXA255_DSADR6           (PXA255_DMA_BASE_ADDR + 0x00000264)
#define PXA255_DTADR6           (PXA255_DMA_BASE_ADDR + 0x00000268)
#define PXA255_DCMD6            (PXA255_DMA_BASE_ADDR + 0x0000026c)
#define PXA255_DDADR7           (PXA255_DMA_BASE_ADDR + 0x00000270)
#define PXA255_DSADR7           (PXA255_DMA_BASE_ADDR + 0x00000274)
#define PXA255_DTADR7           (PXA255_DMA_BASE_ADDR + 0x00000278)
#define PXA255_DCMD7            (PXA255_DMA_BASE_ADDR + 0x0000027c)
#define PXA255_DDADR8           (PXA255_DMA_BASE_ADDR + 0x00000280)
#define PXA255_DSADR8           (PXA255_DMA_BASE_ADDR + 0x00000284)
#define PXA255_DTADR8           (PXA255_DMA_BASE_ADDR + 0x00000288)
#define PXA255_DCMD8            (PXA255_DMA_BASE_ADDR + 0x0000028c)
#define PXA255_DDADR9           (PXA255_DMA_BASE_ADDR + 0x00000290)
#define PXA255_DSADR9           (PXA255_DMA_BASE_ADDR + 0x00000294)
#define PXA255_DTADR9           (PXA255_DMA_BASE_ADDR + 0x00000298)
#define PXA255_DCMD9            (PXA255_DMA_BASE_ADDR + 0x0000029c)
#define PXA255_DDADR10          (PXA255_DMA_BASE_ADDR + 0x000002a0)
#define PXA255_DSADR10          (PXA255_DMA_BASE_ADDR + 0x000002a4)
#define PXA255_DTADR10          (PXA255_DMA_BASE_ADDR + 0x000002a8)
#define PXA255_DCMD10           (PXA255_DMA_BASE_ADDR + 0x000002ac)
#define PXA255_DDADR11          (PXA255_DMA_BASE_ADDR + 0x000002b0)
#define PXA255_DSADR11          (PXA255_DMA_BASE_ADDR + 0x000002b4)
#define PXA255_DTADR11          (PXA255_DMA_BASE_ADDR + 0x000002b8)
#define PXA255_DCMD11           (PXA255_DMA_BASE_ADDR + 0x000002bc)
#define PXA255_DDADR12          (PXA255_DMA_BASE_ADDR + 0x000002c0)
#define PXA255_DSADR12          (PXA255_DMA_BASE_ADDR + 0x000002c4)
#define PXA255_DTADR12          (PXA255_DMA_BASE_ADDR + 0x000002c8)
#define PXA255_DCMD12           (PXA255_DMA_BASE_ADDR + 0x000002cc)
#define PXA255_DDADR13          (PXA255_DMA_BASE_ADDR + 0x000002d0)
#define PXA255_DSADR13          (PXA255_DMA_BASE_ADDR + 0x000002d4)
#define PXA255_DTADR13          (PXA255_DMA_BASE_ADDR + 0x000002d8)
#define PXA255_DCMD13           (PXA255_DMA_BASE_ADDR + 0x000002dc)
#define PXA255_DDADR14          (PXA255_DMA_BASE_ADDR + 0x000002e0)
#define PXA255_DSADR14          (PXA255_DMA_BASE_ADDR + 0x000002e4)
#define PXA255_DTADR14          (PXA255_DMA_BASE_ADDR + 0x000002e8)
#define PXA255_DCMD14           (PXA255_DMA_BASE_ADDR + 0x000002ec)
#define PXA255_DDADR15          (PXA255_DMA_BASE_ADDR + 0x000002f0)
	#define PXA255_DDADR_STOP       (0x00000001)
#define PXA255_DSADR15          (PXA255_DMA_BASE_ADDR + 0x000002f4)
#define PXA255_DTADR15          (PXA255_DMA_BASE_ADDR + 0x000002f8)
#define PXA255_DCMD15           (PXA255_DMA_BASE_ADDR + 0x000002fc)
	#define PXA255_DCMD_INCSRCADDR  (0x80000000)
	#define PXA255_DCMD_INCTRGADDR  (0x40000000)
	#define PXA255_DCMD_FLOWSRC     (0x20000000)
	#define PXA255_DCMD_FLOWTRG     (0x10000000)
	#define PXA255_DCMD_STARTIRQEN  (0x00400000)
	#define PXA255_DCMD_ENDIRQEN    (0x00200000)
	#define PXA255_DCMD_ENDIAN      (0x00040000)
	#define PXA255_DCMD_SIZE        (0x00030000)
		#define PXA255_DCMD_SIZE_0  (0x00000000)
		#define PXA255_DCMD_SIZE_8  (0x00010000)
		#define PXA255_DCMD_SIZE_16 (0x00020000)
		#define PXA255_DCMD_SIZE_32 (0x00030000)
	#define PXA255_DCMD_WIDTH       (0x0000c000)
		#define PXA255_DCMD_WIDTH_0 (0x00000000)
		#define PXA255_DCMD_WIDTH_1 (0x00004000)
		#define PXA255_DCMD_WIDTH_2 (0x00008000)
		#define PXA255_DCMD_WIDTH_4 (0x0000c000)

struct PXA255_DMA_Regs
{
	uint32_t dcsr[16];

	uint32_t pad0[44];

	uint32_t dint;

	uint32_t pad1[3];

	uint32_t drcmr[40];

	uint32_t pad2[24];

	uint32_t ddadr[16];
	uint32_t dsadr[16];
	uint32_t dtadr[16];
	uint32_t dcmd[16];

	emu_timer* timer[16];
};

/*

  PXA255 Inter-Integrated-Circuit Sound (I2S) Controller

  pg. 489 to 504, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_I2S_BASE_ADDR    (0x40400000)
#define PXA255_SACR0            (PXA255_I2S_BASE_ADDR + 0x00000000)
	#define PXA255_SACR0_ENB    (0x00000001)    // Enable I2S function: 0 = Disable, 1 = Enable
	#define PXA255_SACR0_BCKD   (0x00000004)    // Input/Output direction of BITCLK: 0 = Input, 1 = Output
	#define PXA255_SACR0_RST    (0x00000008)    // Reset FIFO Logic and all registers: 0 = Not Reset, 1 = Reset is active
	#define PXA255_SACR0_EFWR   (0x00000010)    // Special-purpose FIFO Write/Read Enable: 0 = Disable, 1 = Enable
	#define PXA255_SACR0_STRF   (0x00000020)    // Select Transmit or Receive FIFO for EFWR-based special-purpose function: 0 = Xmit FIFO, 1 = Recv FIFO
	#define PXA255_SACR0_TFTH   (0x00000f00)    // Transmit FIFO interrupt or DMA threshold
	#define PXA255_SACR0_TFTH_S (8)
	#define PXA255_SACR0_RFTH   (0x0000f000)    // Receive FIFO interrupt or DMA threshold
	#define PXA255_SACR0_RFTH_S (12)
#define PXA255_SACR1            (PXA255_I2S_BASE_ADDR + 0x00000004)
	#define PXA255_SACR1_AMSL   (0x00000001)    // Alternate Mode: 0 = I2S Operation Mode, 1 = MSB-Justified Operation Mode
	#define PXA255_SACR1_DREC   (0x00000008)    // Disable Recording: 0 = Recording Function is enabled, 1 = Recording Function is disabled
	#define PXA255_SACR1_DRPL   (0x00000010)    // Disable Replaying: 0 = Replaying Function is enabled, 1 = Recording Function is disabled
	#define PXA255_SACR1_ENLBF  (0x00000020)    // Enable I2S/MSB Interface Loopback
#define PXA255_SASR0            (PXA255_I2S_BASE_ADDR + 0x0000000c)
	#define PXA255_SASR0_TNF    (0x00000001)
	#define PXA255_SASR0_RNE    (0x00000002)
	#define PXA255_SASR0_BSY    (0x00000004)
	#define PXA255_SASR0_TFS    (0x00000008)
	#define PXA255_SASR0_RFS    (0x00000010)
	#define PXA255_SASR0_TUR    (0x00000020)
	#define PXA255_SASR0_ROR    (0x00000040)
	#define PXA255_SASR0_TFL    (0x00000f00)
	#define PXA255_SASR0_RFL    (0x0000f000)
#define PXA255_SAIMR            (PXA255_I2S_BASE_ADDR + 0x00000014)
	#define PXA255_SAIMR_TFS    (0x00000008)
	#define PXA255_SAIMR_RFS    (0x00000010)
	#define PXA255_SAIMR_TUR    (0x00000020)
	#define PXA255_SAIMR_ROR    (0x00000040)
#define PXA255_SAICR            (PXA255_I2S_BASE_ADDR + 0x00000018)
	#define PXA255_SAICR_TUR    (0x00000020)
	#define PXA255_SAICR_ROR    (0x00000040)
#define PXA255_SADIV            (PXA255_I2S_BASE_ADDR + 0x00000060)
#define PXA255_SADR             (PXA255_I2S_BASE_ADDR + 0x00000080)

struct PXA255_I2S_Regs
{
	uint32_t sacr0;
	uint32_t sacr1;

	uint32_t pad0;

	uint32_t sasr0;

	uint32_t pad1;

	uint32_t saimr;
	uint32_t saicr;

	uint32_t pad2[17];

	uint32_t sadiv;

	uint32_t pad3[6];

	uint32_t sadr;
};

/*

  PXA255 OS Timer register

  pg. 138 to 142, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_OSTMR_BASE_ADDR  (0x40a00000)
#define PXA255_OSMR0            (PXA255_OSTMR_BASE_ADDR + 0x00000000)
#define PXA255_OSMR1            (PXA255_OSTMR_BASE_ADDR + 0x00000004)
#define PXA255_OSMR2            (PXA255_OSTMR_BASE_ADDR + 0x00000008)
#define PXA255_OSMR3            (PXA255_OSTMR_BASE_ADDR + 0x0000000c)
#define PXA255_OSCR             (PXA255_OSTMR_BASE_ADDR + 0x00000010)
#define PXA255_OSSR             (PXA255_OSTMR_BASE_ADDR + 0x00000014)
	#define PXA255_OSSR_M0      (0x00000001)
	#define PXA255_OSSR_M1      (0x00000002)
	#define PXA255_OSSR_M2      (0x00000004)
	#define PXA255_OSSR_M3      (0x00000008)
#define PXA255_OWER             (PXA255_OSTMR_BASE_ADDR + 0x00000018)
#define PXA255_OIER             (PXA255_OSTMR_BASE_ADDR + 0x0000001c)
	#define PXA255_OIER_E0      (0x00000001)
	#define PXA255_OIER_E1      (0x00000002)
	#define PXA255_OIER_E2      (0x00000004)
	#define PXA255_OIER_E3      (0x00000008)

struct PXA255_OSTMR_Regs
{
	uint32_t osmr[4];
	uint32_t oscr;
	uint32_t ossr;
	uint32_t ower;
	uint32_t oier;

	emu_timer* timer[4];
};

/*

  PXA255 Interrupt registers

  pg. 124 to 132, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_INTC_BASE_ADDR   (0x40d00000)
#define PXA255_ICIP             (PXA255_INTC_BASE_ADDR + 0x00000000)
#define PXA255_ICMR             (PXA255_INTC_BASE_ADDR + 0x00000004)
#define PXA255_ICLR             (PXA255_INTC_BASE_ADDR + 0x00000008)
#define PXA255_ICFP             (PXA255_INTC_BASE_ADDR + 0x0000000c)
#define PXA255_ICPR             (PXA255_INTC_BASE_ADDR + 0x00000010)
#define PXA255_ICCR             (PXA255_INTC_BASE_ADDR + 0x00000014)

#define PXA255_INT_HUART        (1 << 7)
#define PXA255_INT_GPIO0        (1 << 8)
#define PXA255_INT_GPIO1        (1 << 9)
#define PXA255_INT_GPIO84_2     (1 << 10)
#define PXA255_INT_USB          (1 << 11)
#define PXA255_INT_PMU          (1 << 12)
#define PXA255_INT_I2S          (1 << 13)
#define PXA255_INT_AC97         (1 << 14)
#define PXA255_INT_NETWORK      (1 << 16)
#define PXA255_INT_LCD          (1 << 17)
#define PXA255_INT_I2C          (1 << 18)
#define PXA255_INT_ICP          (1 << 19)
#define PXA255_INT_STUART       (1 << 20)
#define PXA255_INT_BTUART       (1 << 21)
#define PXA255_INT_FFUART       (1 << 22)
#define PXA255_INT_MMC          (1 << 23)
#define PXA255_INT_SSP          (1 << 24)
#define PXA255_INT_DMA          (1 << 25)
#define PXA255_INT_OSTIMER0     (1 << 26)
#define PXA255_INT_OSTIMER1     (1 << 27)
#define PXA255_INT_OSTIMER2     (1 << 28)
#define PXA255_INT_OSTIMER3     (1 << 29)
#define PXA255_INT_RTC_HZ       (1 << 30)
#define PXA255_INT_RTC_ALARM    (1 << 31)

struct PXA255_INTC_Regs
{
	uint32_t icip;
	uint32_t icmr;
	uint32_t iclr;
	uint32_t icfp;
	uint32_t icpr;
	uint32_t iccr;
};

/*

  PXA255 General-Purpose I/O registers

  pg. 105 to 124, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_GPIO_BASE_ADDR   (0x40e00000)
#define PXA255_GPLR0            (PXA255_GPIO_BASE_ADDR + 0x00000000)
#define PXA255_GPLR1            (PXA255_GPIO_BASE_ADDR + 0x00000004)
#define PXA255_GPLR2            (PXA255_GPIO_BASE_ADDR + 0x00000008)
#define PXA255_GPDR0            (PXA255_GPIO_BASE_ADDR + 0x0000000c)
#define PXA255_GPDR1            (PXA255_GPIO_BASE_ADDR + 0x00000010)
#define PXA255_GPDR2            (PXA255_GPIO_BASE_ADDR + 0x00000014)
#define PXA255_GPSR0            (PXA255_GPIO_BASE_ADDR + 0x00000018)
#define PXA255_GPSR1            (PXA255_GPIO_BASE_ADDR + 0x0000001c)
#define PXA255_GPSR2            (PXA255_GPIO_BASE_ADDR + 0x00000020)
#define PXA255_GPCR0            (PXA255_GPIO_BASE_ADDR + 0x00000024)
#define PXA255_GPCR1            (PXA255_GPIO_BASE_ADDR + 0x00000028)
#define PXA255_GPCR2            (PXA255_GPIO_BASE_ADDR + 0x0000002c)
#define PXA255_GRER0            (PXA255_GPIO_BASE_ADDR + 0x00000030)
#define PXA255_GRER1            (PXA255_GPIO_BASE_ADDR + 0x00000034)
#define PXA255_GRER2            (PXA255_GPIO_BASE_ADDR + 0x00000038)
#define PXA255_GFER0            (PXA255_GPIO_BASE_ADDR + 0x0000003c)
#define PXA255_GFER1            (PXA255_GPIO_BASE_ADDR + 0x00000040)
#define PXA255_GFER2            (PXA255_GPIO_BASE_ADDR + 0x00000044)
#define PXA255_GEDR0            (PXA255_GPIO_BASE_ADDR + 0x00000048)
#define PXA255_GEDR1            (PXA255_GPIO_BASE_ADDR + 0x0000004c)
#define PXA255_GEDR2            (PXA255_GPIO_BASE_ADDR + 0x00000050)
#define PXA255_GAFR0_L          (PXA255_GPIO_BASE_ADDR + 0x00000054)
#define PXA255_GAFR0_U          (PXA255_GPIO_BASE_ADDR + 0x00000058)
#define PXA255_GAFR1_L          (PXA255_GPIO_BASE_ADDR + 0x0000005c)
#define PXA255_GAFR1_U          (PXA255_GPIO_BASE_ADDR + 0x00000060)
#define PXA255_GAFR2_L          (PXA255_GPIO_BASE_ADDR + 0x00000064)
#define PXA255_GAFR2_U          (PXA255_GPIO_BASE_ADDR + 0x00000068)

struct PXA255_GPIO_Regs
{
	uint32_t gplr0; // GPIO Pin-Leve
	uint32_t gplr1;
	uint32_t gplr2;

	uint32_t gpdr0;
	uint32_t gpdr1;
	uint32_t gpdr2;

	uint32_t gpsr0;
	uint32_t gpsr1;
	uint32_t gpsr2;

	uint32_t gpcr0;
	uint32_t gpcr1;
	uint32_t gpcr2;

	uint32_t grer0;
	uint32_t grer1;
	uint32_t grer2;

	uint32_t gfer0;
	uint32_t gfer1;
	uint32_t gfer2;

	uint32_t gedr0;
	uint32_t gedr1;
	uint32_t gedr2;

	uint32_t gafr0l;
	uint32_t gafr0u;
	uint32_t gafr1l;
	uint32_t gafr1u;
	uint32_t gafr2l;
	uint32_t gafr2u;
};

/*

  PXA255 LCD Controller

  pg. 265 to 310, PXA255 Processor Developers Manual [278693-002].pdf

*/

#define PXA255_LCD_BASE_ADDR    (0x44000000)
#define PXA255_LCCR0            (PXA255_LCD_BASE_ADDR + 0x00000000)
	#define PXA255_LCCR0_OUM    (0x00200000)
	#define PXA255_LCCR0_BM     (0x00100000)
	#define PXA255_LCCR0_PDD    (0x000ff000)
	#define PXA255_LCCR0_QDM    (0x00000800)
	#define PXA255_LCCR0_DIS    (0x00000400)
	#define PXA255_LCCR0_DPD    (0x00000200)
	#define PXA255_LCCR0_PAS    (0x00000080)
	#define PXA255_LCCR0_EFM    (0x00000040)
	#define PXA255_LCCR0_IUM    (0x00000020)
	#define PXA255_LCCR0_SFM    (0x00000010)
	#define PXA255_LCCR0_LDM    (0x00000008)
	#define PXA255_LCCR0_SDS    (0x00000004)
	#define PXA255_LCCR0_CMS    (0x00000002)
	#define PXA255_LCCR0_ENB    (0x00000001)
#define PXA255_LCCR1            (PXA255_LCD_BASE_ADDR + 0x00000004)
	#define PXA255_LCCR1_PPL    (0x000003ff)
#define PXA255_LCCR2            (PXA255_LCD_BASE_ADDR + 0x00000008)
	#define PXA255_LCCR2_LPP    (0x000003ff)
#define PXA255_LCCR3            (PXA255_LCD_BASE_ADDR + 0x0000000c)
#define PXA255_FBR0             (PXA255_LCD_BASE_ADDR + 0x00000020)
	#define PXA255_FBR_BAR      (0x00000001)
	#define PXA255_FBR_BINT     (0x00000003)
#define PXA255_FBR1             (PXA255_LCD_BASE_ADDR + 0x00000024)
#define PXA255_LCSR             (PXA255_LCD_BASE_ADDR + 0x00000038)
	#define PXA255_LCSR_LDD     (0x00000001)
	#define PXA255_LCSR_SOF     (0x00000002)
	#define PXA255_LCSR_BER     (0x00000004)
	#define PXA255_LCSR_ABC     (0x00000008)
	#define PXA255_LCSR_IUL     (0x00000010)
	#define PXA255_LCSR_IUU     (0x00000020)
	#define PXA255_LCSR_OU      (0x00000040)
	#define PXA255_LCSR_QD      (0x00000080)
	#define PXA255_LCSR_EOF     (0x00000100)
	#define PXA255_LCSR_BS      (0x00000200)
	#define PXA255_LCSR_SINT    (0x00000400)
#define PXA255_LIIDR            (PXA255_LCD_BASE_ADDR + 0x0000003c)
#define PXA255_TRGBR            (PXA255_LCD_BASE_ADDR + 0x00000040)
#define PXA255_TCR              (PXA255_LCD_BASE_ADDR + 0x00000044)
#define PXA255_FDADR0           (PXA255_LCD_BASE_ADDR + 0x00000200)
#define PXA255_FSADR0           (PXA255_LCD_BASE_ADDR + 0x00000204)
#define PXA255_FIDR0            (PXA255_LCD_BASE_ADDR + 0x00000208)
#define PXA255_LDCMD0           (PXA255_LCD_BASE_ADDR + 0x0000020c)
	#define PXA255_LDCMD_EOFINT (0x00200000)
	#define PXA255_LDCMD_SOFINT (0x00400000)
	#define PXA255_LDCMD_PAL    (0x04000000)
#define PXA255_FDADR1           (PXA255_LCD_BASE_ADDR + 0x00000210)
#define PXA255_FSADR1           (PXA255_LCD_BASE_ADDR + 0x00000214)
#define PXA255_FIDR1            (PXA255_LCD_BASE_ADDR + 0x00000218)
#define PXA255_LDCMD1           (PXA255_LCD_BASE_ADDR + 0x0000021c)

struct PXA255_LCD_DMA_Regs
{
	uint32_t fdadr;
	uint32_t fsadr;
	uint32_t fidr;
	uint32_t ldcmd;
	emu_timer *eof;
};

struct PXA255_LCD_Regs
{
	uint32_t lccr0;
	uint32_t lccr1;
	uint32_t lccr2;
	uint32_t lccr3;

	uint32_t pad0[4];

	uint32_t fbr[2];

	uint32_t pad1[4];

	uint32_t lcsr;
	uint32_t liidr;
	uint32_t trgbr;
	uint32_t tcr;

	uint32_t pad2[110];

	PXA255_LCD_DMA_Regs dma[2];
};
