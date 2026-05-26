#ifndef __SE_IO_H__
#define __SE_IO_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static inline void
se_write_all(int fd, const char *s)
{
    size_t len = strlen(s);
    while (len > 0) {
        ssize_t ret = write(fd, s, len);
        if (ret <= 0) {
            return;
        }
        s += ret;
        len -= (size_t)ret;
    }
}

static inline void
se_puts(const char *s)
{
    se_write_all(STDOUT_FILENO, s);
}

static inline void
se_eputs(const char *s)
{
    se_write_all(STDERR_FILENO, s);
}

static inline void
se_printf_uint(const char *label, uint64_t value)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s=%llu\n", label,
             (unsigned long long)value);
    se_puts(buf);
}

static inline void
se_printf_hex(const char *label, uint64_t value)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s=0x%llx\n", label,
             (unsigned long long)value);
    se_puts(buf);
}

static inline void
se_dump_hex(const char *label, const void *data, unsigned len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    char buf[128];

    snprintf(buf, sizeof(buf), "%s", label);
    se_puts(buf);

    for (unsigned i = 0; i < len; ++i) {
        snprintf(buf, sizeof(buf), "%s%02x", i == 0 ? "=" : " ",
                 bytes[i]);
        se_puts(buf);
    }

    se_puts("\n");
}

#endif
