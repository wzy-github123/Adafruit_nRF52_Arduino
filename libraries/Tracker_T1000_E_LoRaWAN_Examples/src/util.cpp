#include "util.hpp"

void memcpyr(void *dst, const void *src, uint16_t size)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;

    d = d + (size - 1);

    while (size--)
    {
        *d-- = *s++;
    }
}