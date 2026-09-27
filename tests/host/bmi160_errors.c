/* Include the driver to exercise its private FIFO-length error helper. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bmi160.c"

static uint8_t fail_addr;
static int foc_writes;

static int8_t read_bus(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len)
{
    (void)id;
    memset(data, 0xAA, len);
    return reg == fail_addr ? BMI160_E_COM_FAIL : BMI160_OK;
}

static int8_t write_bus(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len)
{
    (void)id;
    (void)data;
    (void)len;
    if (reg == BMI160_FOC_CONF_ADDR || reg == BMI160_COMMAND_REG_ADDR) {
        foc_writes++;
    }
    return BMI160_OK;
}

static void delay(uint32_t ms)
{
    (void)ms;
}

int main(void)
{
    struct bmi160_dev dev = {
        .read = read_bus,
        .write = write_bus,
        .delay_ms = delay,
        .intf = BMI160_I2C_INTF,
    };
    uint16_t count = 0x7777;
    struct bmi160_foc_conf foc = {0};
    struct bmi160_offsets offset = {0};

    fail_addr = BMI160_FIFO_LENGTH_ADDR;
    assert(get_fifo_byte_counter(&count, &dev) == BMI160_E_COM_FAIL);
    assert(count == 0x7777);

    fail_addr = 0xFF;
    assert(get_fifo_byte_counter(&count, &dev) == BMI160_OK);
    assert(count == (((0xAA & BMI160_FIFO_BYTE_COUNTER_MASK) << 8) | 0xAA));

    fail_addr = BMI160_FOC_CONF_ADDR;
    assert(bmi160_start_foc(&foc, &offset, &dev) == BMI160_E_COM_FAIL);
    assert(foc_writes == 0);
    puts("PASS: failed reads preserve outputs and do not trigger calibration");
}
