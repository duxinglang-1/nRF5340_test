/****************************************Copyright (c)************************************************
** File Name:			    logger.c
** Descriptions:			log message process source file
** Created By:				xie biao
** Created Date:			2021-10-25
** Modified Date:      		2021-10-25 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include "datetime.h"
#include "logger.h"
#include "transfer_cache.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(E2, CONFIG_LOG_DEFAULT_LEVEL);

void LOGDD(const char *fun_name, const char *fmt, ...)
{
	uint16_t str_len = 0;
	int n = 0;
	uint32_t timemap=0;
	uint8_t buf[LOG_BUFF_SIZE] = {0};
	va_list args;

#ifdef TEST_DEBUG
	timemap = (k_uptime_get()%1000);
	va_start(args, fmt);
	sprintf(buf, "[%04d-%02d-%02d %02d:%02d:%02d:%03d]..%s>>", 
						date_time.year,
						date_time.month,
						date_time.day,
						date_time.hour, 
						date_time.minute, 
						date_time.second, 
						timemap,
						fun_name
						);
	
	str_len = strlen(buf);

#if defined(WIN32)
	n = str_len + _vsnprintf(&buf[str_len], sizeof(buf), fmt, args);
#else
	n = str_len + vsnprintf(&buf[str_len], sizeof(buf), fmt, args);
#endif /* WIN32 */
	va_end(args);

	//这里N操作100的长度直接退出，打太长可能会导致重启
	if(n > LOG_BUFF_SIZE - 4)
	{
		return;
	}

	if(n > 0)
	{
		*(buf + n) = '\n';
		n++;
		
		LOG_INF("%s", buf);
	}
#endif	
}
