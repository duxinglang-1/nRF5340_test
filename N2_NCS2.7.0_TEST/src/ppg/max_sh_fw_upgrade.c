/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
********************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "max32674.h"
#include "max_sh_interface.h"
#include "max_sh_fw_upgrade.h"
#include "logger.h"
#include "uart.h"

//#define PPG_UPDATE_DEBUG

#ifdef SH_OTA_DATA_STORE_IN_FLASH
int32_t SH_OTA_upgrade_process(void)
{
	int32_t s32_status;
	uint8_t u8_rxbuf[3]={0};

#ifdef PPG_UPDATE_DEBUG
	LOGD("start to upgrade MAX32674 firmware");
#endif

	//hardware method to enter BL mode
	SH_rst_to_BL_mode();

	//set software work mode command
	s32_status = sh_put_in_bootloader();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set bl mode fail, %x", s32_status);
	#endif
		return s32_status;
	}

	//check MCU type
	s32_status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", s32_status);
	#endif
		return s32_status;
	}

#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	s32_status = sh_get_hub_fw_version(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("FW version is %d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#endif
	}

	//read page size
	uint16_t u16_pageSize;
	s32_status = sh_get_bootloader_pagesz(&u16_pageSize);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read page size fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("page size is %d", u16_pageSize);
	#endif
	}

	//set page number
	uint8_t u8_pageNumber;
	//SpiFlash_Read(&u8_pageNumber, PPG_ALGO_FW_ADDR+BL_PAGE_COUNT_INDEX, sizeof(u8_pageNumber));
	s32_status = sh_set_bootloader_numberofpages(u8_pageNumber);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count done %d", u8_pageNumber);
	#endif
	}

	//Set vector bytes
	uint8_t u8p_ivData[BL_AES_NONCE_SIZE] = {0};
	//SpiFlash_Read(u8p_ivData, PPG_ALGO_FW_ADDR+BL_IV_INDEX, BL_AES_NONCE_SIZE);
	s32_status = sh_set_bootloader_iv(u8p_ivData);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the vector bytes fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the vector bytes is done");
	#endif
	}

	//Set auth bytes
	uint8_t u8p_authData[BL_AES_AUTH_SIZE];
	//SpiFlash_Read(u8p_authData, PPG_ALGO_FW_ADDR+BL_AUTH_INDEX, BL_AES_AUTH_SIZE);
	s32_status = sh_set_bootloader_auth(u8p_authData);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the authentication fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the authentication is done");
	#endif
	}

	uint32_t u32_partialSize = BL_FLASH_PARTIAL_SIZE;
	s32_status = sh_set_bootloader_partial_write_size(u32_partialSize);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set partial write size fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set partial write size done %d", u32_partialSize);
	#endif
	}

	s32_status = sh_set_bootloader_erase();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erase flash fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erasing flash is done");
	#endif
	}

	s32_status = sh_set_bootloader_flashpages(PPG_ALGO_FW_ADDR, u8_pageNumber);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Write page fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("All page is flashed");
	#endif
	}

	SH_rst_to_APP_mode();

	//check MCU type
	s32_status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", s32_status);
	#endif
		return s32_status;
	}
#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	s32_status = sh_get_hub_fw_version(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("FW version is %d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#endif
	}

	return s32_status;
}

#else

int32_t SH_OTA_upgrade_process(uint8_t* u8p_FwData)
{
	int32_t s32_status;
	uint8_t u8_rxbuf[3];

#ifdef PPG_UPDATE_DEBUG
	LOGD("start to upgrade MAX32674 firmware");
#endif

	//hardware method to enter BL mode
	SH_rst_to_BL_mode();

	//set software work mode command
	s32_status = sh_put_in_bootloader();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set bl mode fail, %x", s32_status);
	#endif
		return s32_status;
	}

	//check MCU type
	s32_status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", s32_status);
	#endif
		return s32_status;
	}
#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	s32_status = sh_get_hub_fw_version(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("FW version is %d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#endif
	}

	//read page size
	uint16_t u16_pageSize;
	s32_status = sh_get_bootloader_pagesz(&u16_pageSize);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("page size is %x", u16_pageSize);
	#endif
	}

	//set page number
	uint8_t u8_pageNumber = u8p_FwData[BL_PAGE_COUNT_INDEX];
	s32_status =  sh_set_bootloader_numberofpages(u8_pageNumber);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count is done");
	#endif
	}

	//Set vector bytes
	uint8_t* u8p_ivData = &u8p_FwData[BL_IV_INDEX];
	s32_status =  sh_set_bootloader_iv(u8p_ivData);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the  vector bytes fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the  vector bytes is done");
	#endif
	}

	//Set vector bytes
	uint8_t* u8p_authData = &u8p_FwData[BL_AUTH_INDEX];
	s32_status = sh_set_bootloader_auth(u8p_authData);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the  authentication fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the authentication is done");
	#endif
	}

	s32_status =  sh_set_bootloader_erase();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erase flash fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erasing flash is done");
	#endif
	}

	s32_status = sh_set_bootloader_flashpages(u8p_FwData, u8_pageNumber);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Write page fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("All page is flashed");
	#endif
	}

	SH_rst_to_APP_mode();

	//check MCU type
	s32_status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", s32_status);
	#endif
		return s32_status;
	}
#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	s32_status = sh_get_hub_fw_version(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		return s32_status;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("FW version is %d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#endif
	}

	return s32_status;
}
#endif/*SH_OTA_DATA_STORE_IN_FLASH*/

uint8_t u8_pageNumber;

void SH_OTA_upgrade_ok(uint8_t *data, uint32_t len)
{
	uint8_t buffer[64] = {0};
	
	PPG_Disable();

	sprintf(buffer, "%s%s", COM_PPG_UPGRADE_OK, data);
	MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_OK, strlen(COM_PPG_UPGRADE_OK)+len);
}

void SH_OTA_upgrade_fail(void)
{
	PPG_Disable();

	MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_FAIL, strlen(COM_PPG_UPGRADE_FAIL));
}

void SH_OTA_upgrade_start(void)
{
	int32_t s32_status;
	uint8_t u8_rxbuf[3]={0};

#ifdef PPG_UPDATE_DEBUG
	LOGD("start to upgrade MAX32674 firmware");
#endif
	PPG_Enable();
	
	//hardware method to enter BL mode
	SH_rst_to_BL_mode();

	//set software work mode command
	s32_status = sh_put_in_bootloader();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set bl mode fail, %x", s32_status);
	#endif
		goto fail;
	}

	//check MCU type
	s32_status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", s32_status);
	#endif
		goto fail;
	}
#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	s32_status = sh_get_hub_fw_version(u8_rxbuf);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", s32_status);
	#endif
		goto fail;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("FW version is %d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#endif
	}

	//read page size
	uint16_t u16_pageSize;
	s32_status = sh_get_bootloader_pagesz(&u16_pageSize);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read page size fail %x", s32_status);
	#endif
		goto fail;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("page size is %d", u16_pageSize);
	#endif
		MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_PAGE_NUM, strlen(COM_PPG_UPGRADE_PAGE_NUM));
		return;
	}

fail:	

	SH_OTA_upgrade_fail();
}

void SH_OTA_upgrade_set_page_num(uint8_t *data, uint32_t len)
{
	//set page number
	int32_t s32_status;

	u8_pageNumber = atoi(data);
#ifdef PPG_UPDATE_DEBUG	
	LOGD("u8_pageNumber:%d", u8_pageNumber);	
#endif
	s32_status = sh_set_bootloader_numberofpages(u8_pageNumber);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count fail %x", s32_status);
	#endif
		SH_OTA_upgrade_fail();
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("set page count done %d", u8_pageNumber);
	#endif
		MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_VECTOR_BYTES, strlen(COM_PPG_UPGRADE_VECTOR_BYTES));
	}
}

void SH_OTA_upgrade_set_vector_bytes(uint8_t *data, uint32_t len)
{
	//Set vector bytes
	int32_t s32_status;

	s32_status = sh_set_bootloader_iv(data);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the vector bytes fail %x", s32_status);
	#endif
		SH_OTA_upgrade_fail();
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the vector bytes is done");
	#endif
		MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_AUTH_BYTES, strlen(COM_PPG_UPGRADE_AUTH_BYTES));
	}
}

void SH_OTA_upgrade_set_auth_bytes(uint8_t *data, uint32_t len)
{
	//Set auth bytes
	int32_t s32_status;
	
	s32_status = sh_set_bootloader_auth(data);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set the authentication fail %x", s32_status);
	#endif
		SH_OTA_upgrade_fail();
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Setting the authentication is done");
	#endif
	}

	uint32_t u32_partialSize = BL_FLASH_PARTIAL_SIZE;
	s32_status = sh_set_bootloader_partial_write_size(u32_partialSize);
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set partial write size fail %x", s32_status);
	#endif
		SH_OTA_upgrade_fail();
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Set partial write size done %d", u32_partialSize);
	#endif
	}

	s32_status = sh_set_bootloader_erase();
	if(s32_status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erase flash fail %x", s32_status);
	#endif
		SH_OTA_upgrade_fail();
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Erasing flash is done");
	#endif
		MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_FLASH_PAGE, strlen(COM_PPG_UPGRADE_FLASH_PAGE));
	}
}

void SH_OTA_upgrade_set_flash_pages(uint8_t *data, uint32_t len)
{
	int32_t status;
	uint8_t u8_rxbuf[3]={0};
	static uint32_t i=0,j=0;
    uint8_t ByteSeq[] = { 0x80, 0x04};

	status = sh_write_cmd_with_data(&ByteSeq[0], sizeof(ByteSeq), data, len, BL_PAGE_W_DLY_TIME);
	if(status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Write page %d part %d data FW fail: %x", i, j, status);
	#endif
		goto out_loop;
	}
	else
	{
		j++;
		if(j == (1+(8000/BL_FLASH_PARTIAL_SIZE)))
		{
		#ifdef PPG_UPDATE_DEBUG
			LOGD("write page %d data done!", i);
		#endif
		
			j = 0;
			i++;
			if(i == u8_pageNumber)
				goto out_loop;
		}

		MapcsSendData(UART_DATA_PPG, COM_PPG_UPGRADE_FLASH_PAGE, strlen(COM_PPG_UPGRADE_FLASH_PAGE));
		return;
	}

out_loop:

	i = 0;
	j = 0;
	if(status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Write page fail %x", status);
	#endif
		SH_OTA_upgrade_fail();
		return;
	}
	else
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("All page is flashed");
	#endif
	}

	SH_rst_to_APP_mode();

	//check MCU type
	status = sh_get_bootloader_MCU_tye(u8_rxbuf);
	if(status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("Read MCU type fail, %x", status);
	#endif
		SH_OTA_upgrade_fail();
		return;
	}
#ifdef PPG_UPDATE_DEBUG	
	LOGD("MCU type = %d", u8_rxbuf[0]);
#endif

	//check working mode and FW version
	status = sh_get_hub_fw_version(u8_rxbuf);
	if(status != SS_SUCCESS)
	{
	#ifdef PPG_UPDATE_DEBUG
		LOGD("read FW version fail %x", status);
	#endif
		SH_OTA_upgrade_fail();
		return;
	}
	else
	{
		uint8_t ver[16] = {0};

		sprintf(ver, "%d.%d.%d", u8_rxbuf[0], u8_rxbuf[1], u8_rxbuf[2]);
	#ifdef PPG_UPDATE_DEBUG	
		LOGD("FW version is %s", ver);
	#endif
		SH_OTA_upgrade_ok(ver, strlen(ver));
	}
}

