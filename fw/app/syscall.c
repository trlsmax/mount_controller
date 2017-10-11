#include "hw_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#undef errno
extern int errno;
extern int  _end;

caddr_t _sbrk(int incr)
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if(heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

    heap += incr;

    return (caddr_t) prev_heap;
}

int link(char *old, char *new)
{
    return -1;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _read(int file, char *ptr, int len)
{
    return 0;
}

void abort(void)
{
    /* Abort called */
    while(1);
}

int _write(int file, char *ptr, int len)
{
    uint16_t todo;
    for(todo = 0; todo < len; todo++) {
        /* Write a character to the USART */
        USART_SendData(USART1, *ptr++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }

    return len;
}

#pragma GCC diagnostic pop
