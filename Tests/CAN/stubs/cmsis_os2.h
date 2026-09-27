#pragma once
#include <stdint.h>

typedef void *osMessageQueueId_t;
typedef int32_t osStatus_t;
#define osOK 0
typedef struct {
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
    void *mq_mem;
    uint32_t mq_size;
} osMessageQueueAttr_t;

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const osMessageQueueAttr_t *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message,
                             uint8_t priority, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message,
                             uint8_t *priority, uint32_t timeout);
