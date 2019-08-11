#ifndef MY_LOG_H__
#define MY_LOG_H__

#define MY_LOG(...) \
do { \
 	char str[64];\
 	sprintf(str, __VA_ARGS__);\
 	SEGGER_RTT_WriteString(0, str);\
 } while(0)


#endif // MY_LOG_H__