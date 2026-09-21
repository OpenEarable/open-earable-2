#include "MAXM86161.h"
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAXM86161, 3);

char databuffer[32*BYTES_PER_CH*LED_NUM];

/*****************************************************************************/
// Constructor
/*****************************************************************************/
/*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*/
MAXM86161::MAXM86161(TWIM * i2c):_i2c(i2c)
{
    // Empty block
}

MAXM86161::~MAXM86161(void)
{
    // Empty block
}


// Initialize the sensor
// Sets the PPG sensor to starting condition, then puts it in SHDN mode ready to
// take data
int MAXM86161::init(enum sample_rate sample_rate)
{
    int read_value;

    _i2c->begin();

    // Use function to do software reset
    _write_to_reg(REG_SYSTEM_CONTROL, 0x09);

    k_msleep(1);

    // Shut Down
    _write_to_reg(REG_SYSTEM_CONTROL, 0x0A);

    k_msleep(2);

    // Clear Interrupt 1 by reading
    _read_from_reg(REG_IRQ_STATUS1, read_value);

    // Clear Interrupt 2 by reading
    _read_from_reg(REG_IRQ_STATUS2, read_value);

    // Set integration time and ADC range with ALC and ADD
    _write_to_reg(REG_PPG_CONFIG1, 0x0F);

    // Set sample rate and averaging
    // cmd[0] = 0x12; cmd[1] = 0x50;  // 8Hz with no averaging
    // cmd[1] = 0x08; 50 Hz with no averaging
    //_write_to_reg(REG_PPG_CONFIG2, 0x08);
    _write_to_reg(REG_PPG_CONFIG2, sample_rate << POS_PPG_SR);

    // Set LED settling, digital filter, burst rate, burst enable
    //  No Burst mode with default settling
    _write_to_reg(REG_PPG_CONFIG3, 0x40);

    // Set Photodiode bias to 0pF to 65pF
    _write_to_reg(REG_PD_BIAS, 0x40);

    // Set LED driver range to 124 mA
    _write_to_reg(REG_LED_RANGE1, 0x3F);

    // Set LED current
    set_all_led_current(0x14);

    // Enable Low Power Mode
    _write_to_reg(REG_SYSTEM_CONTROL, 0xC);

    //********************************
    // FIFO enable

    // Set FIFO full to 15 empty spaces left
    _write_to_reg(REG_FIFO_CONFIG1, 0xF);

    // Enable FIFO rollover when full (A_FULL_TYPE = 0)
    _write_to_reg(REG_FIFO_CONFIG2, 0b00001010);

    // Enable interrupt when new sample detected
    _write_to_reg(REG_IRQ_ENABLE1, 0b11000000); // Data ready and FIFO full

    // Set LED exposure to timeslots
    _write_to_reg(REG_LED_SEQ1, 0x12);
    _write_to_reg(REG_LED_SEQ2, 0x93);
    _write_to_reg(REG_LED_SEQ3, 0x00);

    // Shutdown at the end and wait for signal to start
    stop();

    // Read device ID, if it matches the value for MAXM86161, return 0, otherwise return 1.
    _read_from_reg(REG_PART_ID, read_value);

    _clear_interrupt();

    if (read_value == PPG_PART_ID) {
        return 0;
    } else {
        LOG_WRN("Part ID: %i", read_value);
        return 1;
    }
}

int MAXM86161::start(void)
{
    int existing_reg_values;
    int status;
    // Get value of register
    _read_from_reg(REG_SYSTEM_CONTROL, existing_reg_values);

    // Clear the bit to start the device
    existing_reg_values = _clear_one_bit(existing_reg_values, POS_START_STOP);

    // Write to the register to start the device
    status = _write_to_reg(REG_SYSTEM_CONTROL, existing_reg_values);

    return status;
}

int MAXM86161::stop(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_SYSTEM_CONTROL, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _set_one_bit(existing_reg_values, POS_START_STOP); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_SYSTEM_CONTROL, existing_reg_values);
    return status;
}

int MAXM86161::read(ppg_sample * buffer) {
    int status;
    int number_of_bytes;
    int num_samples = 0;
    int output_idx = -1;

    status = _read_from_reg(REG_FIFO_DATA_COUNTER, num_samples);
    if (status == 0){
        number_of_bytes = num_samples / LED_NUM * LED_NUM * BYTES_PER_CH;
        
        status = _read_block(REG_FIFO_DATA, number_of_bytes, (uint8_t *) databuffer);

        for (int i=0; i < num_samples / LED_NUM * LED_NUM; i++) {
            int idx = BYTES_PER_CH * i;

            uint32_t val = databuffer[idx] << 16 | databuffer[idx + 1] << 8 | databuffer[idx+2];

            uint8_t tag = val >> 19;
            val = val & ((1 << 19) - 1);

            //LOG_INF("tag: %i, val: %i", tag, val);

            if (tag == 1) output_idx++;
            if (tag > 6 || output_idx < 0) continue;

            buffer[output_idx][tag-1] = val;
        }
    }
    
    return output_idx+1;
}


/*******************************************************************************/
int MAXM86161::set_interrogation_rate(int rate)
{
    int existing_reg_values;
    int status;

    // Get value of register to avoid overwriting sample average value
    _read_from_reg(REG_PPG_CONFIG2, existing_reg_values);

    // Set the appropriate bits, while leaving the others.
    existing_reg_values = _set_multiple_bits(existing_reg_values, MASK_SMP_AVE, rate, POS_PPG_SR);

    status = _write_to_reg(REG_PPG_CONFIG2, existing_reg_values);
    return status;
}

int MAXM86161::set_sample_averaging(int average)
{
    int existing_reg_values;
    int status;

    // Get value of register to avoid overwriting sample average value
    _read_from_reg(REG_PPG_CONFIG2, existing_reg_values);

    // Set the appropriate bits, while leaving the others.
    existing_reg_values = _set_multiple_bits(existing_reg_values, MASK_PPG_SR, average, POS_SMP_AVG);

    status = _write_to_reg(REG_PPG_CONFIG2, existing_reg_values);
    return status;
}

int MAXM86161::set_all_led_current(int current)
{
    int status_1;
    int status_2;
    int status_3;
    int status_total;

    // Set each LED current
    status_1 = set_led1_current(current);
    status_2 = set_led2_current(current);
    status_3 = set_led3_current(current);
    // Return the sum of the status values.
    // Will be zero for sucessful writing of LED currents.
    status_total = status_1 + status_2 + status_3;
    return status_total;
}


int MAXM86161::set_led1_current(int current)
{
    int status;
    status = _write_to_reg(REG_LED1_PA, current);
    return status;
}

int MAXM86161::set_led2_current(int current)
{
    int status;
    status = _write_to_reg(REG_LED2_PA, current);
    return status;
}

int MAXM86161::set_led3_current(int current)
{
    int status;
    status = _write_to_reg(REG_LED3_PA, current);
    return status;
}

int MAXM86161::set_ppg_tint(int time)
{
    int existing_reg_values;
    int status;

    // Get value of register to avoid overwriting sample average value
    _read_from_reg(REG_PPG_CONFIG1, existing_reg_values);

    // Set the appropriate bits, while leaving the others.
    existing_reg_values = _set_multiple_bits(existing_reg_values, MASK_PPG_TINT_WRITE, time, POS_PPG_TINT);

    status = _write_to_reg(REG_PPG_CONFIG1, existing_reg_values);
    return status;

}

/*******************************************************************************/
int MAXM86161::alc_on(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_PPG_CONFIG1, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _clear_one_bit(REG_PPG_CONFIG1, POS_ALC_DIS); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_PPG_CONFIG1, existing_reg_values);
 
    return status;
}

int MAXM86161::alc_off(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_PPG_CONFIG1, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _set_one_bit(REG_PPG_CONFIG1, POS_ALC_DIS); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_PPG_CONFIG1, existing_reg_values);
 
    return status;
}


int MAXM86161::picket_off(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_PICKET_FENCE, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _clear_one_bit(REG_PICKET_FENCE, POS_PICKET_DIS); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_PICKET_FENCE, existing_reg_values);
 
    return status;
}

int MAXM86161::picket_on(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_PICKET_FENCE, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _set_one_bit(REG_PICKET_FENCE, POS_PICKET_DIS); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_PICKET_FENCE, existing_reg_values);
 
    return status;
}

int MAXM86161::new_value_read_on(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_IRQ_ENABLE1, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _set_one_bit(REG_IRQ_ENABLE1, POS_DATA_RDY_EN); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_IRQ_ENABLE1, existing_reg_values);
 
    return status;
}

int MAXM86161::new_value_read_off(void)
{
    int existing_reg_values;
    int status;

    // Get value of register
    _read_from_reg(REG_IRQ_ENABLE1, existing_reg_values);

    // Set the bit to stop the device
    existing_reg_values = _clear_one_bit(REG_IRQ_ENABLE1, POS_DATA_RDY_EN); 
    
    // Send the Shutdown command to the device
    status = _write_to_reg(REG_IRQ_ENABLE1, existing_reg_values);
 
    return status;
}


/*******************************************************************************/
// Function to read from a registry
int MAXM86161::_read_from_reg(int address, int &data) {
    int ret;

    _i2c->aquire();

    uint8_t buffer;
    ret = i2c_burst_read(_i2c->master, _addr, address, &buffer, sizeof(buffer));
    if (ret) LOG_WRN("I2C read failed: %d\n", ret);

    data = buffer;

    _i2c->release();
    return ret; //Fail
}

// Function to write to a registry
// TODO Check about mfio -> might not need it.
int MAXM86161::_write_to_reg(int address, int value) {

    _i2c->aquire();

    uint8_t buffer = value;
    int ret = i2c_burst_write(_i2c->master, _addr, address, &buffer, sizeof(buffer));
    if (ret) LOG_WRN("I2C read failed: %d\n", ret);

    _i2c->release();

    return 0;
}


int MAXM86161::_read_block(int address, int length, uint8_t *data)
{
    int ret;

    _i2c->aquire();

    ret = i2c_burst_read(_i2c->master, _addr, address, data, length);
    if (ret) LOG_WRN("I2C read failed: %d\n", ret);

    _i2c->release();

    return ret;
}

int MAXM86161::_set_one_bit(int current_bits, int position)
{
    current_bits = current_bits | 1 << position;
    return  current_bits;
}

int MAXM86161::_clear_one_bit(int current_bits, int position)
{
    current_bits = current_bits & ~(1 << position);
    return current_bits;
}

int MAXM86161::_set_multiple_bits(int current_bits, int mask, int new_value, int position)
{
    current_bits = (current_bits & mask) | (new_value << position);
    return current_bits;
}

int MAXM86161::_clear_interrupt(void)
{
    int status;
    int value;
    status = _read_from_reg(REG_IRQ_STATUS1, value);
    return status;
}

int MAXM86161::read_interrupt_state(int &value)
{
    int status;
    status = _read_from_reg(REG_IRQ_STATUS2, value);
    status = _read_from_reg(REG_IRQ_STATUS1, value);
    return status;
}

int MAXM86161::set_watermark(int level)
{
    int status;
    status = _write_to_reg(REG_FIFO_CONFIG1, level);
    return status;
}
