#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f1xx_hal.h"

//TODO: io using UART //extern UART_HandleTypeDef * huart;

int _write(int file, char *ptr, int len);

extern uint32_t __get_MSP(void);

void _exit(int status)
{
    while (1);
}
void usart_send_str(char* str);

int _close(int file)
{
    return -1;
}
int _execve(char *name, char **argv, char **env)
{
    errno = ENOMEM;
    return -1;
}

int _fork()
{
    errno = EAGAIN;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}


int _getpid()
{
    return 1;
}

int _isatty(int file)
{
    switch (file)
    {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return (-1);
}


int _link(char *_old, char *_new)
{
    errno = EMLINK;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}


caddr_t _sbrk(int incr)
{
    extern char _ebss;
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

    char * stack = (char*) __get_MSP();
    if (heap_end + incr > stack)
    {
        _write(STDERR_FILENO, "Heap and stack collision\n", 25);
        errno = ENOMEM;
        return (caddr_t) -1;
        //abort ();
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}


int _read(int file, char *ptr, int len)
{
    int num = 0;
    switch (file)
    {
    case STDIN_FILENO:
        //TODO: check conversion below
    	//TODO: io using UART //HAL_UART_Receive(huart, (uint8_t*)ptr, len, 1000);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}


int _stat(const char *filepath, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}


clock_t _times(struct tms *buf)
{
    return -1;
}


int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}


int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}


int _write(int file, char *ptr, int len)
{
    switch (file)
    {
    case STDOUT_FILENO: /*stdout*/
        //TODO: check conversion below
    	//TODO: io using UART //HAL_UART_Transmit(huart, (uint8_t*)ptr, len, 1000);
        break;
    case STDERR_FILENO: /* stderr */
        //TODO: check conversion below
    	//TODO: io using UART //HAL_UART_Transmit(huart, (uint8_t*)ptr, len, 1000);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}
