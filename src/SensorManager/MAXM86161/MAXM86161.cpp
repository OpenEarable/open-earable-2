#include "MAXM86161.h"
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAXM86161, 3);

char databuffer[32*BYTES_PER_CH*LED_NUM];

/*****************************************************************************/
// Constructor
/*****************************************************************************/
/*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*/
MAXM86161::MAXM86161(TwoWire &i2c):_i2cPort(i2c)
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
int MAXM86161::init(void)
{
    int read_value;
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
    _write_to_reg(REG_PPG_CONFIG2, 0x05 << 3);

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

    // Enable FIFO rollover when full
    _write_to_reg(REG_FIFO_CONFIG2, 0b00001110);

    // Enable interrupt when new sample detected
    _write_to_reg(REG_IRQ_ENABLE1, 0b01000000);

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
        number_of_bytes = num_samples * BYTES_PER_CH;
        
        status = _read_block(REG_FIFO_DATA, number_of_bytes, (uint8_t *) databuffer);

        for (int i=0; i < num_samples / LED_NUM * LED_NUM; i++) {
            int idx = BYTES_PER_CH * i;

            uint32_t val = databuffer[idx] << 16 | databuffer[idx + 1] << 8 | databuffer[idx+2];

            uint8_t tag = val >> 19;
            val = val & ((1 << 19) - 1);

            if (tag == 1) output_idx++;
            if (tag > 6 || output_idx < 0) continue;

            buffer[output_idx][tag-1] = val;
        }
    }
    
    //_clear_interrupt();
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
    _i2cPort.aquire();

    _i2cPort.beginTransmission(PPG_ADDR);
    _i2cPort.write(address);
    ret = _i2cPort.endTransmission(false); //false

    if (ret != 0)  LOG_WRN("I2C Error read: End transmission");

    _i2cPort.requestFrom(PPG_ADDR, 1); // Request 1 byte
    if (_i2cPort.available()) {
        data = _i2cPort.read();
        _i2cPort.release();
        return 0;
    }

    data = 0;

    _i2cPort.release();
    return -1; //Fail
}

// Function to write to a registry
// TODO Check about mfio -> might not need it.
int MAXM86161::_write_to_reg(int address, int value) {
    _i2cPort.aquire();
    int ret;

    _i2cPort.beginTransmission(PPG_ADDR);
    _i2cPort.write(address);
    _i2cPort.write(value);
    ret = _i2cPort.endTransmission();

    if (ret != 0) LOG_WRN("I2C Error write: End transmission");

    _i2cPort.release();
    //k_usleep(1000);

    return 0;
}


int MAXM86161::_read_block(int address, int length, uint8_t *data)
{
    int ret;

    _i2cPort.aquire();

    _i2cPort.beginTransmission(PPG_ADDR);
    _i2cPort.write(address);
    ret = _i2cPort.endTransmission(false); // false

    if (ret != 0) LOG_WRN("I2C Error block read: End transmission");

    _i2cPort.requestFrom(PPG_ADDR, length);

    for (int i = 0; i < length; i++) {
        if (_i2cPort.available()) data[i] = _i2cPort.read();
        else {
            // _i2cPort.endTransmission();
            _i2cPort.release();
            LOG_WRN("I2C Error block read: Not enough data (%i/%i)", i , length);
            return -1;
        }
    }

    // _i2cPort.endTransmission();
    _i2cPort.release();

    return 0;
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

int MAXM86161::get_fifo_count(int &fifo_count) {
    return _read_from_reg(REG_FIFO_DATA_COUNTER, fifo_count);
}

int MAXM86161::read_interrupt_state(int &value)
{
    int status;
    status = _read_from_reg(REG_IRQ_STATUS1, value);
    return status;
}

int MAXM86161::get_sample_rate() {
    int existing_reg_values;
    _read_from_reg(REG_PPG_CONFIG2, existing_reg_values);

    // Extract sample rate (bits POS_PPG_SR)
    int sample_rate_code = (existing_reg_values >> POS_PPG_SR) & 0b11111;

    // Extract sample averaging factor (bits POS_SMP_AVG)
    int averaging_code = (existing_reg_values >> POS_SMP_AVG) & 0b111;

    // Convert the sample rate register value to actual frequency (see datasheet)
    int base_sample_rate;
    switch(sample_rate_code) {
        case 0x00: base_sample_rate = 25; break;    // 24.995 Hz
        case 0x01: base_sample_rate = 50; break;    // 50.027 Hz
        case 0x02: base_sample_rate = 84; break;    // 84.021 Hz
        case 0x03: base_sample_rate = 100; break;   // 99.902 Hz
        case 0x04: base_sample_rate = 200; break;   // 199.805 Hz
        case 0x05: base_sample_rate = 400; break;   // 399.610 Hz
        case 0x0A: base_sample_rate = 8; break;     // 8 Hz
        case 0x0B: base_sample_rate = 16; break;    // 16 Hz
        case 0x0C: base_sample_rate = 32; break;    // 32 Hz
        case 0x0D: base_sample_rate = 64; break;    // 64 Hz
        case 0x0E: base_sample_rate = 128; break;   // 128 Hz
        case 0x0F: base_sample_rate = 256; break;   // 256 Hz
        case 0x10: base_sample_rate = 512; break;   // 512 Hz
        case 0x11: base_sample_rate = 1024; break;  // 1024 Hz
        case 0x12: base_sample_rate = 2048; break;  // 2048 Hz
        case 0x13: base_sample_rate = 4096; break;  // 4096 Hz
        default: return -1;  // Error case
    }

    // Convert the averaging code to the number of averaged samples
    int averaging_factor;
    switch(averaging_code) {
        case 0b000: averaging_factor = 1; break;
        case 0b001: averaging_factor = 2; break;
        case 0b010: averaging_factor = 4; break;
        case 0b011: averaging_factor = 8; break;
        case 0b100: averaging_factor = 16; break;
        case 0b101: averaging_factor = 32; break;
        case 0b110: averaging_factor = 64; break;
        case 0b111: averaging_factor = 128; break;
        default: return -1; // Should never happen
    }

    // Compute the actual effective sample rate
    int effective_sample_rate = base_sample_rate / averaging_factor;
    return effective_sample_rate;
}

