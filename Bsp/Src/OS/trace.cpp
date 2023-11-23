#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "trace.h"
#include "os.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE extern "C" int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE extern "C" int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  ITM_SendChar(ch);
  return ch;
}

void trace(LogSeverity severity, const char *fileName, int lineNum, const char *format, ...) {

    va_list argptr;
    va_start(argptr, format);

#ifdef _WIN32
        char slash = '\\';
#else
        char slash = '/';
#endif

        const char *file = strrchr(fileName, slash) ? strrchr(fileName, slash) + 1 : fileName;
        printf("[%09lu](%s:%d): ", OS_GetTime(), file, lineNum);
        vprintf(format, argptr);
        printf("\r\n");

        va_end(argptr);
}

