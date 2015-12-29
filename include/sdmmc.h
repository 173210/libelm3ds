/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2014-2015, Normmatt, 173210
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

#ifndef __SDMMC_H__
#define __SDMMC_H__

#include <stdint.h>

enum sdmmc_dev {
	SDMMC_DEV_SDMC = 0,
	SDMMC_DEV_NAND = 1,

	SDMMC_DEV_NUM
};

#ifdef __cplusplus
extern "C" {
#endif
	void sdmmc_init();

	uint32_t sdmmc_readsectors(enum sdmmc_dev target,
		uint32_t sector_no, uint32_t numsectors, uint8_t *out);

	uint32_t sdmmc_writesectors(enum sdmmc_dev target,
		uint32_t sector_no, uint32_t numsectors, uint8_t *in);
#ifdef __cplusplus
};
#endif

#endif
