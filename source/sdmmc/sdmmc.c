/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2014-2015, Normmatt
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 2, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <malloc.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>

#include "sdmmc.h"
//#include "DrawCharacter.h"

#define DATA32_SUPPORT

#define TRUE 1
#define FALSE 0

#define NO_INLINE __attribute__ ((noinline))

#ifdef __cplusplus
extern "C" {
#endif
	void waitcycles(uint32_t val);
#ifdef __cplusplus
};
#endif

struct mmcdevice handelNAND;
struct mmcdevice handelSD;

mmcdevice *getMMCDevice(int drive)
{
	if(drive==0) return &handelNAND;
	return &handelSD;
}

int geterror(struct mmcdevice *ctx)
{
	return (ctx->error << 29) >> 31;
}


void inittarget(struct mmcdevice *ctx)
{
	sdmmc_mask16(REG_SDPORTSEL,0x3,(uint16_t)ctx->devicenumber);
	setckl(ctx->clk);
	if(ctx->SDOPT == 0)
	{
		sdmmc_mask16(REG_SDOPT,0,0x8000);
	}
	else
	{
		sdmmc_mask16(REG_SDOPT,0x8000,0);
	}
	
}

void NO_INLINE sdmmc_send_command(struct mmcdevice *ctx, uint32_t cmd, uint32_t args)
{
	uint16_t flags = (cmd << 15) >> 31;
	const int readdata = cmd & 0x20000;
	const int writedata = cmd & 0x40000;
	
	if(readdata || writedata)
	{
		flags |= TMIO_STAT_DATAEND;
	}
	
	ctx->error = 0;
	while((sdmmc_read32(REG_SDSTATUS) & TMIO_STAT_CMD_BUSY)); //mmc working?
	sdmmc_write32(REG_SDIRMASK,0);
	sdmmc_write32(REG_SDSTATUS,0);
	sdmmc_write16(REG_DATACTL32,TMIO32_ENABLE | TMIO32_STAT_RXRDY);
	sdmmc_write32(REG_SDCMDARG,args);
	sdmmc_write16(REG_SDCMD,cmd &0xFFFF);
	
	uint32_t size = ctx->size;
	uint16_t *dataPtr = (uint16_t*)ctx->data;
	uint32_t *dataPtr32 = (uint32_t*)ctx->data;
	
	int useBuf = ( NULL != dataPtr );
	int useBuf32 = (useBuf && (0 == (3 & ((uint32_t)dataPtr))));
	
	while(1)
	{
		volatile uint16_t status = sdmmc_read16(REG_SDSTATUS);
#ifdef DATA32_SUPPORT
		volatile uint16_t ctl32 = sdmmc_read16(REG_DATACTL32);
		if((ctl32 & TMIO32_STAT_RXRDY))
#else
		if((status & TMIO_STAT_RXRDY))
#endif
		{
#ifdef DATA32_SUPPORT
			sdmmc_write16(REG_DATACTL32, TMIO32_ENABLE);
#else
			sdmmc_write32(REG_SDSTATUS, ~TMIO_STAT_RXRDY);
#endif

			if(readdata)
			{
				if(useBuf)
				{
					if(size > 0x1FF)
					{
						#ifdef DATA32_SUPPORT
						if(useBuf32)
						{
							for(int i = 0; i<0x200; i+=4)
							{
								*dataPtr32++ = sdmmc_read32(REG_SDFIFO32);
							}
						}
						else 
						{
						#endif
							for(int i = 0; i<0x200; i+=2)
							{
								*dataPtr++ = sdmmc_read16(REG_SDFIFO);
							}
						#ifdef DATA32_SUPPORT
						}
						#endif
						size -= 0x200;
					}
				}
			}
		}
#ifdef DATA32_SUPPORT
		if(!(ctl32 & TMIO32_STAT_BUSY))
#else
		if((status & TMIO_STAT_TXRQ))
#endif
		{
#ifdef DATA32_SUPPORT
			sdmmc_write16(REG_DATACTL32,
				TMIO32_ENABLE | TMIO32_STAT_RXRDY | TMIO32_STAT_BUSY);
#else
			sdmmc_write32(REG_SDSTATUS, ~TMIO_STAT_TXRQ);
#endif

			if(writedata)
			{
				if(useBuf)
				{
					if(size > 0x1FF)
					{
						#ifdef DATA32_SUPPORT
						for(int i = 0; i<0x200; i+=4)
						{
							sdmmc_write32(REG_SDFIFO32,*dataPtr32++);
						}
						#else
						for(int i = 0; i<0x200; i+=2)
						{
							sdmmc_write16(REG_SDFIFO,*dataPtr++);
						}
						#endif
						size -= 0x200;
					}
				}
			}
		}
		if(status & TMIO_MASK_GW)
		{
			ctx->error |= 4;
			break;
		}
		
		if(!(status & TMIO_STAT_CMD_BUSY))
		{
			if(status & TMIO_STAT_CMDRESPEND)
			{
				ctx->error |= 0x1;
			}
			if(status & TMIO_STAT_DATAEND)
			{
				ctx->error |= 0x2;
			}
			
			if((status & flags) == flags)
				break;
		}
	}
	sdmmc_write32(REG_SDSTATUS,0);
}

int NO_INLINE sdmmc_sdcard_writesectors(uint32_t sector_no, uint32_t numsectors, uint8_t *in)
{
	if(handelSD.isSDHC == 0) sector_no <<= 9;
	inittarget(&handelSD);
	sdmmc_write16(REG_SDSTOP,0x100);
#ifdef DATA32_SUPPORT
	sdmmc_write16(REG_SDBLKCOUNT32,numsectors);
	sdmmc_write16(REG_SDBLKLEN32,0x200);
#endif
	sdmmc_write16(REG_SDBLKCOUNT,numsectors);
	handelSD.data = in;
	handelSD.size = numsectors << 9;
	sdmmc_send_command(&handelSD,0x52C19,sector_no);
	return geterror(&handelSD);
}

int NO_INLINE sdmmc_sdcard_readsectors(uint32_t sector_no, uint32_t numsectors, uint8_t *out)
{
	if(handelSD.isSDHC == 0) sector_no <<= 9;
	inittarget(&handelSD);
	sdmmc_write16(REG_SDSTOP,0x100);
#ifdef DATA32_SUPPORT
	sdmmc_write16(REG_SDBLKCOUNT32,numsectors);
	sdmmc_write16(REG_SDBLKLEN32,0x200);
#endif
	sdmmc_write16(REG_SDBLKCOUNT,numsectors);
	handelSD.data = out;
	handelSD.size = numsectors << 9;
	sdmmc_send_command(&handelSD,0x33C12,sector_no);
	return geterror(&handelSD);
}



int NO_INLINE sdmmc_nand_readsectors(uint32_t sector_no, uint32_t numsectors, uint8_t *out)
{
	if(handelNAND.isSDHC == 0) sector_no <<= 9;
	inittarget(&handelNAND);
	sdmmc_write16(REG_SDSTOP,0x100);
#ifdef DATA32_SUPPORT
	sdmmc_write16(REG_SDBLKCOUNT32,numsectors);
	sdmmc_write16(REG_SDBLKLEN32,0x200);
#endif
	sdmmc_write16(REG_SDBLKCOUNT,numsectors);
	handelNAND.data = out;
	handelNAND.size = numsectors << 9;
	sdmmc_send_command(&handelNAND,0x33C12,sector_no);
	inittarget(&handelSD);
	return geterror(&handelNAND);
}

int NO_INLINE sdmmc_nand_writesectors(uint32_t sector_no, uint32_t numsectors, uint8_t *in) //experimental
{
	if(handelNAND.isSDHC == 0) sector_no <<= 9;
	inittarget(&handelNAND);
	sdmmc_write16(REG_SDSTOP,0x100);
#ifdef DATA32_SUPPORT
	sdmmc_write16(REG_SDBLKCOUNT32,numsectors);
	sdmmc_write16(REG_SDBLKLEN32,0x200);
#endif
	sdmmc_write16(REG_SDBLKCOUNT,numsectors);
	handelNAND.data = in;
	handelNAND.size = numsectors << 9;
	sdmmc_send_command(&handelNAND,0x52C19,sector_no);
	inittarget(&handelSD);
	return geterror(&handelNAND);
}

static uint32_t calcSDSize(uint8_t* csd, int type)
{
  uint32_t result=0;
  if(type == -1) type = csd[14] >> 6;
  switch(type)
  {
    case 0:
      {
        uint32_t block_len=csd[9]&0xf;
        block_len=1<<block_len;
        uint32_t mult=(csd[4]>>7)|((csd[5]&3)<<1);
        mult=1<<(mult+2);
        result=csd[8]&3;
        result=(result<<8)|csd[7];
        result=(result<<2)|(csd[6]>>6);
        result=(result+1)*mult*block_len/512;
      }
      break;
    case 1:
      result=csd[7]&0x3f;
      result=(result<<8)|csd[6];
      result=(result<<8)|csd[5];
      result=(result+1)*1024;
      break;
  }
  return result;
}

void InitSD()
{
	//NAND
	handelNAND.isSDHC = 0;
	handelNAND.SDOPT = 0;
	handelNAND.res = 0;
	handelNAND.initarg = 1;
	handelNAND.clk = 0x80;
	handelNAND.devicenumber = 1;
	
	//SD
	handelSD.isSDHC = 0;
	handelSD.SDOPT = 0;
	handelSD.res = 0;
	handelSD.initarg = 0;
	handelSD.clk = 0x80;
	handelSD.devicenumber = 0;
	
	//sdmmc_mask16(0x100,0x800,0);
	//sdmmc_mask16(0x100,0x1000,0);
	//sdmmc_mask16(0x100,0x0,0x402);
	//sdmmc_mask16(0xD8,0x22,0x2);
	//sdmmc_mask16(0x100,0x2,0);
	//sdmmc_mask16(0xD8,0x22,0);
	//sdmmc_write16(0x104,0);
	//sdmmc_write16(0x108,1);
	//sdmmc_mask16(REG_SDRESET,1,0); //not in new Version -- nintendo's code does this
	//sdmmc_mask16(REG_SDRESET,0,1); //not in new Version -- nintendo's code does this
	//sdmmc_mask16(0x20,0,0x31D);
	//sdmmc_mask16(0x22,0,0x837F);
	//sdmmc_mask16(0xFC,0,0xDB);
	//sdmmc_mask16(0xFE,0,0xDB);
	////sdmmc_write16(REG_SDCLKCTL,0x20);
	////sdmmc_write16(REG_SDOPT,0x40EE);
	////sdmmc_mask16(0x02,0x3,0);
	//sdmmc_write16(REG_SDCLKCTL,0x40);
	//sdmmc_write16(REG_SDOPT,0x40EB);
	//sdmmc_mask16(0x02,0x3,0);
	//sdmmc_write16(REG_SDBLKLEN,0x200);
	//sdmmc_write16(REG_SDSTOP,0);
	
	*(volatile uint16_t*)0x10006100 &= 0xF7FFu; //SDDATACTL32
	*(volatile uint16_t*)0x10006100 &= 0xEFFFu; //SDDATACTL32
#ifdef DATA32_SUPPORT
	*(volatile uint16_t*)0x10006100 |= 0x402u; //SDDATACTL32
#else
	*(volatile uint16_t*)0x10006100 |= 0x402u; //SDDATACTL32
#endif
	*(volatile uint16_t*)0x100060D8 = (*(volatile uint16_t*)0x100060D8 & 0xFFDD) | 2;
#ifdef DATA32_SUPPORT
	*(volatile uint16_t*)0x10006100 &= 0xFFFFu; //SDDATACTL32
	*(volatile uint16_t*)0x100060D8 &= 0xFFDFu; //SDDATACTL
	*(volatile uint16_t*)0x10006104 = 512; //SDBLKLEN32
#else
	*(volatile uint16_t*)0x10006100 &= 0xFFFDu; //SDDATACTL32
	*(volatile uint16_t*)0x100060D8 &= 0xFFDDu; //SDDATACTL
	*(volatile uint16_t*)0x10006104 = 0; //SDBLKLEN32
#endif
	*(volatile uint16_t*)0x10006108 = 1; //SDBLKCOUNT32
	*(volatile uint16_t*)0x100060E0 &= 0xFFFEu; //SDRESET
	*(volatile uint16_t*)0x100060E0 |= 1u; //SDRESET
	*(volatile uint16_t*)0x10006020 |= TMIO_MASK_ALL; //SDIR_MASK0
	*(volatile uint16_t*)0x10006022 |= TMIO_MASK_ALL>>16; //SDIR_MASK1
	*(volatile uint16_t*)0x100060FC |= 0xDBu; //SDCTL_RESERVED7
	*(volatile uint16_t*)0x100060FE |= 0xDBu; //SDCTL_RESERVED8
	*(volatile uint16_t*)0x10006002 &= 0xFFFCu; //SDPORTSEL
#ifdef DATA32_SUPPORT
	*(volatile uint16_t*)0x10006024 = 0x20;
	*(volatile uint16_t*)0x10006028 = 0x40EE;
#else
	*(volatile uint16_t*)0x10006024 = 0x40; //Nintendo sets this to 0x20
	*(volatile uint16_t*)0x10006028 = 0x40EB; //Nintendo sets this to 0x40EE
#endif
	*(volatile uint16_t*)0x10006002 &= 0xFFFCu; ////SDPORTSEL
	*(volatile uint16_t*)0x10006026 = 512; //SDBLKLEN
	*(volatile uint16_t*)0x10006008 = 0; //SDSTOP
	
	inittarget(&handelSD);
}

int Nand_Init()
{
	inittarget(&handelNAND);
	waitcycles(0xF000);
	
	sdmmc_send_command(&handelNAND,0,0);
	
	do
	{
		do
		{
			sdmmc_send_command(&handelNAND,0x10701,0x100000);
		} while ( !(handelNAND.error & 1) );
	}
	while((sdmmc_read32(REG_SDRESP0) & 0x80000000) == 0);
	
	sdmmc_send_command(&handelNAND,0x10602,0x0);
	if((handelNAND.error & 0x4))return -1;
	
	sdmmc_send_command(&handelNAND,0x10403,handelNAND.initarg << 0x10);
	if((handelNAND.error & 0x4))return -1;
	
	sdmmc_send_command(&handelNAND,0x10609,handelNAND.initarg << 0x10);
	if((handelNAND.error & 0x4))return -1;
	
	handelNAND.total_size = calcSDSize((uint8_t*)SDMMC_BASE + REG_SDRESP0,0);
	handelNAND.clk = 1;
	setckl(1);
	
	sdmmc_send_command(&handelNAND,0x10407,handelNAND.initarg << 0x10);
	if((handelNAND.error & 0x4))return -1;
	
	handelNAND.SDOPT = 1;
	
	sdmmc_send_command(&handelNAND,0x10506,0x3B70100);
	if((handelNAND.error & 0x4))return -1;
	
	sdmmc_send_command(&handelNAND,0x10506,0x3B90100);
	if((handelNAND.error & 0x4))return -1;
	
	sdmmc_send_command(&handelNAND,0x1040D,handelNAND.initarg << 0x10);
	if((handelNAND.error & 0x4))return -1;
	
	sdmmc_send_command(&handelNAND,0x10410,0x200);
	if((handelNAND.error & 0x4))return -1;
	
	handelNAND.clk |= 0x200; 
	
	inittarget(&handelSD);
	
	return 0;
}

int SD_Init()
{
	uint32_t resp;

	inittarget(&handelSD);
	waitcycles(0xF000);
	sdmmc_send_command(&handelSD,0,0);
	sdmmc_send_command(&handelSD,0x10408,0x1AA);
	uint32_t temp = (handelSD.error & 0x1) << 0x1E;

	uint32_t temp2 = 0;
	do
	{
		do
		{
			sdmmc_send_command(&handelSD,0x10437,handelSD.initarg << 0x10);
			sdmmc_send_command(&handelSD,0x10769,0x00FF8000 | temp);
			temp2 = 1;
		} while ( !(handelSD.error & 1) );

		resp = sdmmc_read32(REG_SDRESP0);
	}
	while((resp & 0x80000000) == 0);

	if(!((resp >> 30) & 1) || !temp)
		temp2 = 0;
	
	handelSD.isSDHC = temp2;
	
	sdmmc_send_command(&handelSD,0x10602,0);
	if((handelSD.error & 0x4)) return -1;
	
	sdmmc_send_command(&handelSD,0x10403,0);
	if((handelSD.error & 0x4)) return -1;
	handelSD.initarg = sdmmc_read32(REG_SDRESP0) >> 0x10;
	
	sdmmc_send_command(&handelSD,0x10609,handelSD.initarg << 0x10);
	if((handelSD.error & 0x4)) return -1;
	
	handelSD.total_size = calcSDSize((uint8_t*)SDMMC_BASE + REG_SDRESP0,-1);
	handelSD.clk = 1;
	setckl(1);
	
	sdmmc_send_command(&handelSD,0x10507,handelSD.initarg << 0x10);
	if((handelSD.error & 0x4)) return -1;
	
	sdmmc_send_command(&handelSD,0x10437,handelSD.initarg << 0x10);
	if((handelSD.error & 0x4)) return -1;
	
	handelSD.SDOPT = 1;
	sdmmc_send_command(&handelSD,0x10446,0x2);
	if((handelSD.error & 0x4)) return -1;
	
	sdmmc_send_command(&handelSD,0x1040D,handelSD.initarg << 0x10);
	if((handelSD.error & 0x4)) return -1;
	
	sdmmc_send_command(&handelSD,0x10410,0x200);
	if((handelSD.error & 0x4)) return -1;
	handelSD.clk |= 0x200;
	
	return 0;
}

void sdmmc_sdcard_init()
{
	InitSD();
	uint32_t nand_res = Nand_Init();
	uint32_t sd_res = SD_Init();
}
