#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart1;
int _write(int file, char *ptr, int len);

extern uint32_t __get_MSP(void);

// exit - экстренный выход. В качестве выхода - зацикливаемся.
void _exit(int status)
{
    while (1);
}
void usart_send_str(char* str);

// close - закрытие файла - возвращаем ошибку
int _close(int file)
{
    return -1;
}
/*
 execve - передача управления новому процессу - процессов нет -> возвращаем ошибку.
 */
int _execve(char *name, char **argv, char **env)
{
    errno = ENOMEM;
    return -1;
}

/*
 fork = создание нового процесса
 */
int _fork()
{
    errno = EAGAIN;
    return -1;
}

/*
 fstat - состояние открытого файла
 */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid - получить ID текущего процесса
 */

int _getpid()
{
    return 1;
}

/*
 isatty - является ли файл терминалом.
 */
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

/*
 kill - послать сигнал процессу
 */
int _kill(int pid, int sig)
{
    errno = EINVAL;
    return (-1);
}

/*
 link - устанвить новое имя для существующего файла.
 */

int _link(char *_old, char *_new)
{
    errno = EMLINK;
    return -1;
}

/*
 lseek - установить позицию в файле
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/*
 sbrk - увеличить размер области данных, использутся для malloc
 */
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

/*
 read - чтение из файла, у нас пока для чтения есть только stdin
 */

int _read(int file, char *ptr, int len)
{
    int num = 0;
    switch (file)
    {
    case STDIN_FILENO:
        //TODO: check conversion below
        HAL_UART_Receive(&huart1, (uint8_t*)ptr, len, 1000);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}

/*
 stat - состояние открытого файла.
 */

int _stat(const char *filepath, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times - временная информация о процессе (сколько тиков: системных, процессорных и т.д.)
 */

clock_t _times(struct tms *buf)
{
    return -1;
}

/*
 unlink - удалить имя файла.
 */
int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

/*
 wait - ожидания дочерних процессов
 */
int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

/*
 write - запись в файл - у нас есть только stderr/stdout
 */
int _write(int file, char *ptr, int len)
{
    int n;
    switch (file)
    {
    case STDOUT_FILENO: /*stdout*/
        //TODO: check conversion below
        HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
        break;
    case STDERR_FILENO: /* stderr */
        //TODO: check conversion below
        HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}
