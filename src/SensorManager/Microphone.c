

#include <data_fifo.h>
#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(microphone);

extern struct data_fifo fifo_rx;

void init_fifo() {
    int ret;
    if (!fifo_rx.initialized) {
        ret = data_fifo_init(&fifo_rx);
        ERR_CHK_MSG(ret, "Failed to set up rx FIFO");
    }
}

void empty_fifo() {
    int ret;
    if (fifo_rx.initialized) {
        ret = data_fifo_empty(&fifo_rx);
        ERR_CHK_MSG(ret, "Failed to empty rx FIFO");
    }
}