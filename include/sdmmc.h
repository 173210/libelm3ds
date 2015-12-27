#ifndef __SDMMC_H__
#define __SDMMC_H__

#define TRUE 1
#define FALSE 0

#include <stdint.h>

#define SDMMC_BASE				0x10006000

enum sdmmc_regs {
	REG_SDCMD				= 0x00,
	REG_SDPORTSEL			= 0x02,
	REG_SDCMDARG			= 0x04,
	REG_SDSTOP				= 0x08,
	REG_SDBLKCOUNT			= 0x0a,

	REG_SDRESP0				= 0x0c,
	REG_SDRESP1				= 0x10,
	REG_SDRESP2				= 0x14,
	REG_SDRESP3				= 0x18,

	REG_SDSTATUS			= 0x1c,

	REG_SDIRMASK			= 0x20,
	REG_SDCLKCTL			= 0x24 ,

	REG_SDBLKLEN			= 0x26,
	REG_SDOPT				= 0x28,
	REG_SDFIFO				= 0x30,

	REG_DATACTL 			= 0xd8,
	REG_SDRESET				= 0xe0,
	REG_SDPROTECTED			= 0xf6, //bit 0 determines if sd is protected or not?

	REG_DATACTL32 			= 0x100,
	REG_SDBLKLEN32			= 0x104,
	REG_SDBLKCOUNT32		= 0x108,
	REG_SDFIFO32			= 0x10C,

	REG_CLK_AND_WAIT_CTL	= 0x138,
	REG_RESET_SDIO			= 0x1e0
};

#define TMIO_STAT_CMDRESPEND    0x00000001
#define TMIO_STAT_DATAEND       0x00000004
#define TMIO_STAT_CARD_REMOVE   0x00000008
#define TMIO_STAT_CARD_INSERT   0x00000010
#define TMIO_STAT_SIGSTATE      0x00000020
#define TMIO_STAT_WRPROTECT     0x00000080
#define TMIO_STAT_CARD_REMOVE_A 0x00000100
#define TMIO_STAT_CARD_INSERT_A 0x00000200
#define TMIO_STAT_SIGSTATE_A    0x00000400
#define TMIO_STAT_CMD_IDX_ERR   0x00010000
#define TMIO_STAT_CRCFAIL       0x00020000
#define TMIO_STAT_STOPBIT_ERR   0x00040000
#define TMIO_STAT_DATATIMEOUT   0x00080000
#define TMIO_STAT_RXOVERFLOW    0x00100000
#define TMIO_STAT_TXUNDERRUN    0x00200000
#define TMIO_STAT_CMDTIMEOUT    0x00400000
#define TMIO_STAT_RXRDY         0x01000000
#define TMIO_STAT_TXRQ          0x02000000
#define TMIO_STAT_ILL_FUNC      0x20000000
#define TMIO_STAT_CMD_BUSY      0x40000000
#define TMIO_STAT_ILL_ACCESS    0x80000000

#define TMIO_MASK_ALL           0x837f031d

#define TMIO_MASK_GW            (TMIO_STAT_ILL_ACCESS | TMIO_STAT_CMDTIMEOUT | TMIO_STAT_TXUNDERRUN | TMIO_STAT_RXOVERFLOW | \
								 TMIO_STAT_DATATIMEOUT | TMIO_STAT_STOPBIT_ERR | TMIO_STAT_CRCFAIL | TMIO_STAT_CMD_IDX_ERR)

#define TMIO_MASK_READOP  (TMIO_STAT_RXRDY | TMIO_STAT_DATAEND)
#define TMIO_MASK_WRITEOP (TMIO_STAT_TXRQ | TMIO_STAT_DATAEND)

enum {
	TMIO32_ENABLE		= 0x0002,
	TMIO32_STAT_RXRDY	= 0x0100,
	TMIO32_STAT_BUSY	= 0x0200,
	TMIO32_IRQ_RXRDY	= 0x0800,
	TMIO32_IRQ_TXRQ		= 0x1000
};

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct mmcdevice {
		uint8_t* data;
		uint32_t size;
		uint32_t error;
		uint32_t stat;
		uint32_t ret[4];
		uint32_t initarg;
		uint32_t isSDHC;
		uint32_t clk;
		uint32_t SDOPT;
		uint32_t devicenumber;
		uint32_t total_size; //size in sectors of the device
		uint32_t res;
	} mmcdevice;
	
	void sdmmc_sdcard_init();
	int sdmmc_sdcard_readsector(uint32_t sector_no, uint8_t *out);
	int sdmmc_sdcard_readsectors(uint32_t sector_no, uint32_t numsectors, uint8_t *out);
	int sdmmc_sdcard_writesector(uint32_t sector_no, uint8_t *in);
	int sdmmc_sdcard_writesectors(uint32_t sector_no, uint32_t numsectors, uint8_t *in);
	
	int sdmmc_nand_readsectors(uint32_t sector_no, uint32_t numsectors, uint8_t *out);
	int sdmmc_nand_writesectors(uint32_t sector_no, uint32_t numsectors, uint8_t *in);
	
	mmcdevice *getMMCDevice(int drive);
	
	void InitSD();
	int Nand_Init();
	int SD_Init();

#ifdef __cplusplus
};
#endif

//---------------------------------------------------------------------------------
static inline uint16_t sdmmc_read16(enum sdmmc_regs reg) {
//---------------------------------------------------------------------------------
	return *(volatile uint16_t*)(SDMMC_BASE + reg);
}

//---------------------------------------------------------------------------------
static inline void sdmmc_write16(enum sdmmc_regs reg, uint16_t val) {
//---------------------------------------------------------------------------------
	*(volatile uint16_t*)(SDMMC_BASE + reg) = val;
}

//---------------------------------------------------------------------------------
static inline uint32_t sdmmc_read32(enum sdmmc_regs reg) {
//---------------------------------------------------------------------------------
	return *(volatile uint32_t*)(SDMMC_BASE + reg);
}

//---------------------------------------------------------------------------------
static inline void sdmmc_write32(enum sdmmc_regs reg, uint32_t val) {
//---------------------------------------------------------------------------------
	*(volatile uint32_t*)(SDMMC_BASE + reg) = val;
}

//---------------------------------------------------------------------------------
static inline void sdmmc_mask16(enum sdmmc_regs reg, const uint16_t clear, const uint16_t set) {
//---------------------------------------------------------------------------------
	uint16_t val = sdmmc_read16(reg);
	val &= ~clear;
	val |= set;
	sdmmc_write16(reg, val);
}

//---------------------------------------------------------------------------------
static inline void sdmmc_mask32(enum sdmmc_regs reg, const uint32_t clear, const uint32_t set) {
//---------------------------------------------------------------------------------
	uint32_t val = sdmmc_read32(reg);
	val &= ~clear;
	val |= set;
	sdmmc_write32(reg, val);
}

static inline void setckl(uint32_t data)
{
	sdmmc_mask16(REG_SDCLKCTL,0x100,0);
	sdmmc_mask16(REG_SDCLKCTL,0x2FF,data&0x2FF);
	sdmmc_mask16(REG_SDCLKCTL,0x0,0x100);
}

#endif
