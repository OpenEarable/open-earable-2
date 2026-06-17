#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <ff.h>

#include "time_sync.h"

#define FAT_YEAR_MIN 1980U
#define FAT_YEAR_MAX 2107U
#define UNIX_SECONDS_PER_DAY 86400ULL
#define UNIX_SECONDS_1980_01_01 315532800ULL
#define UNIX_US_PER_SECOND 1000000ULL

#define FATFS_FALLBACK_TIME \
	(((DWORD)(2022U - FAT_YEAR_MIN) << 25) | ((DWORD)1U << 21) | ((DWORD)1U << 16))


DWORD get_fattime(void)
{

	uint64_t now_us;
	uint64_t now_seconds;

	if (!time_sync_is_synced()) {
		return FATFS_FALLBACK_TIME;
	}

	now_us = get_current_time_us();
	now_seconds = now_us / UNIX_US_PER_SECOND;


	time_t now = (time_t)now_seconds;
	struct tm calendar;

	if ((uint64_t)now != now_seconds || gmtime_r(&now, &calendar) == NULL) {
		return FATFS_FALLBACK_TIME;
	}

	uint32_t year = (uint32_t)calendar.tm_year + 1900U;

	if (year < FAT_YEAR_MIN || year > FAT_YEAR_MAX) {
		return FATFS_FALLBACK_TIME;
	}

	return (((DWORD)(year - FAT_YEAR_MIN) << 25) |
			((DWORD)(calendar.tm_mon + 1) << 21) |
			((DWORD)calendar.tm_mday << 16) |
			((DWORD)calendar.tm_hour << 11) |
			((DWORD)calendar.tm_min << 5) |
			((DWORD)(calendar.tm_sec / 2)));
}
