#ifndef MY_ALARM
#define MY_ALARM

#include "stdint.h"
#include "stdbool.h"

#define ALARM_NOT_SET			0
//#define ALARM_SET					1

#define ALARM_NO_REPEAT		2
#define ALARM_EVERY_DAY 	3
#define ALARM_EVERY_WEEK 	4
#define ALARM_EVERY_MONTH 5

#define ALARM_ALARMS_MAX 	5



typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t day;
	uint8_t year;
	uint8_t month;
	uint8_t week_day;
	uint8_t alarm_type;
}ALARM_TypeDef;

typedef struct{
	ALARM_TypeDef alarms_data[ALARM_ALARMS_MAX];
	uint8_t alarms_count;
}ALARM_MASS_TypeDef;

ALARM_MASS_TypeDef Alarms;

#endif
