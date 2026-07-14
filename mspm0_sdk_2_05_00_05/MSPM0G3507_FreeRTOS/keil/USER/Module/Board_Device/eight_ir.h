#ifndef EIGHT_IR_H
#define EIGHT_IR_H

#include <stdint.h>

#define EIGHT_IR_CHANNEL_COUNT    (8U)

typedef struct {
    uint8_t values[EIGHT_IR_CHANNEL_COUNT];
    uint8_t mask;
} eight_ir_data_t;

/*
 * Initialize the selector to channel 0. GPIO direction and pin mux are
 * configured by SYSCFG_DL_init().
 */
void eight_ir_init(void);

/* Read all raw channel levels and their bitmap in one scan. */
void eight_ir_read(eight_ir_data_t *data);

#endif
