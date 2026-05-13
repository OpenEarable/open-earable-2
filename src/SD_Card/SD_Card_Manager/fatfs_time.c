#include <stdbool.h>
#include <stdint.h>

#include <ff.h>

#include "time_sync.h"

#define FAT_YEAR_MIN 1980U
#define FAT_YEAR_MAX 2107U
#define UNIX_SECONDS_PER_DAY 86400ULL
#define UNIX_SECONDS_1980_01_01 315532800ULL
#define UNIX_US_PER_SECOND 1000000ULL

#define FATFS_FALLBACK_TIME \
	(((DWORD)(2022U - FAT_YEAR_MIN) << 25) | ((DWORD)1U << 21) | ((DWORD)1U << 16))

struct fatfs_calendar_time {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
};

static bool is_leap_year(uint32_t year)
{
	return ((year % 4U) == 0U && (year % 100U) != 0U) || ((year % 400U) == 0U);
}

static uint32_t days_in_year(uint32_t year)
{
	return is_leap_year(year) ? 366U : 365U;
}

static uint32_t days_in_month(uint32_t year, uint32_t month)
{
	static const uint8_t month_days[] = {
		31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
	};

	if (month == 2U && is_leap_year(year)) {
		return 29U;
	}

	return month_days[month - 1U];
}

static bool unix_seconds_to_calendar(uint64_t seconds, struct fatfs_calendar_time *calendar)
{
	uint64_t days = seconds / UNIX_SECONDS_PER_DAY;
	uint32_t seconds_today = (uint32_t)(seconds % UNIX_SECONDS_PER_DAY);
	uint32_t year = 1970U;

	while (days >= days_in_year(year)) {
		days -= days_in_year(year);
		year++;
		if (year > FAT_YEAR_MAX) {
			return false;
		}
	}

	uint32_t month = 1U;
	while (days >= days_in_month(year, month)) {
		days -= days_in_month(year, month);
		month++;
	}

	calendar->year = year;
	calendar->month = month;
	calendar->day = (uint32_t)days + 1U;
	calendar->hour = seconds_today / 3600U;
	calendar->minute = (seconds_today % 3600U) / 60U;
	calendar->second = seconds_today % 60U;

	return true;
}

DWORD get_fattime(void)
{
	struct fatfs_calendar_time calendar;
	uint64_t now_us;
	uint64_t now_seconds;

	if (!time_sync_is_synced()) {
		return FATFS_FALLBACK_TIME;
	}

	now_us = get_current_time_us();
	now_seconds = now_us / UNIX_US_PER_SECOND;

	if (now_seconds < UNIX_SECONDS_1980_01_01 ||
	    !unix_seconds_to_calendar(now_seconds, &calendar) ||
	    calendar.year < FAT_YEAR_MIN ||
	    calendar.year > FAT_YEAR_MAX) {
		return FATFS_FALLBACK_TIME;
	}

	return (((DWORD)(calendar.year - FAT_YEAR_MIN) << 25) |
		((DWORD)calendar.month << 21) |
		((DWORD)calendar.day << 16) |
		((DWORD)calendar.hour << 11) |
		((DWORD)calendar.minute << 5) |
		((DWORD)(calendar.second / 2U)));
}
