#include "LibPrintf.h"

#define LOG_LEVEL_SILENT  0
#define LOG_LEVEL_FATAL   1
#define LOG_LEVEL_ERROR   2
#define LOG_LEVEL_WARNING 3
#define LOG_LEVEL_NOTICE  4
#define LOG_LEVEL_TRACE   5
#define LOG_LEVEL_VERBOSE 6

const int logLevel=LOG_LEVEL_TRACE;
const char levelsym[]="SFEWNTV";

inline void logprint(int level,const char *fmt...) {
    va_list args;
    va_start(args,fmt);
    if (level <= logLevel) {
	printf("%c: ",levelsym[level]);
	vprintf(fmt,args);
    }
}

#define fatal(...)  logprint(LOG_LEVEL_FATAL,__VA_ARGS__)
#define error(...)  logprint(LOG_LEVEL_ERROR,__VA_ARGS__)
#define warning(...) logprint(LOG_LEVEL_WARNING,__VA_ARGS__)
#define notice(...) logprint(LOG_LEVEL_NOTICE,__VA_ARGS__)
#define trace(...) logprint(LOG_LEVEL_TRACE,__VA_ARGS__)
#define verbose(...) logprint(LOG_LEVEL_VERBOSE,__VA_ARGS__)

