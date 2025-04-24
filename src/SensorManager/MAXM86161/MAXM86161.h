/**
 *  Code to get the raw data from a MAX86161
 */
#ifndef __MAXM86161_H_
#define __MAXM86161_H_

//#include <Wire.h>
#include <TWIM.h>
#include <stdint.h>


/*******************************************************************************
 ************************** Maxm86161 Value Enum *******************************
 ******************************************************************************/

 enum sample_rate {
    SR_24_995_SPS = 0x00, 
    SR_50_027_SPS = 0x01, 
    SR_84_021_SPS = 0x02, 
    SR_99_902_SPS = 0x03, 
    SR_199_805_SPS = 0x04, 
    SR_399_610_SPS = 0x05, 
    SR_8_0_SPS = 0x0A, 
    SR_16_0_SPS = 0x0B, 
    SR_32_0_SPS = 0x0C, 
    SR_64_0_SPS = 0x0D, 
    SR_128_0_SPS = 0x0E, 
    SR_256_0_SPS = 0x0F, 
    SR_512_0_SPS = 0x10, 
    SR_1024_0_SPS = 0x11, 
    SR_2048_0_SPS = 0x12, 
    SR_4096_0_SPS = 0x13
};

typedef uint32_t ppg_sample[6];

class MAXM86161 {
    public:
    /** @brief Constructor
     * @param i2c Pointer for bus for communication via I2C
    */
    MAXM86161(TWIM * i2c);

    /** @brief Destructor */
    ~MAXM86161(void);
    /** @brief Initialization of the object */
    int init(enum sample_rate sample_rate = SR_199_805_SPS);
    /** @brief Start collecting data samples */
    int start(void);
    /** @brief Stop collecting data samples */
    int stop(void);
    /** @brief Read data from the sensor */
    int read(ppg_sample * buffer);

    // Configuration adjustments
    /** @brief Set the rate of the PPG sensor */
    int set_interrogation_rate(int rate);
    /** @brief Set the number of samples to average */
    int set_sample_averaging(int average);
    /** @brief Set the LED current for all LEDs */
    int set_all_led_current(int current);
    /** @brief Set the LED current for LED1 */   
    int set_led1_current(int current);
    /** @brief Set the LED current for LED2 */   
    int set_led2_current(int current);
    /** @brief Set the LED current for LED3 */   
    int set_led3_current(int current);
    /** @brief Set the integration time for the photodiode */   
    int set_ppg_tint(int time);

    // Setting adjustments
    /** @brief Set the ALC on */  
    int alc_on(void);
    /** @brief Set the ALC off */ 
    int alc_off(void);
    /** @brief Set the picket fence detection on */ 
    int picket_on(void);
    /** @brief Set the picket fence detection off */ 
    int picket_off(void);
    /** @brief Set the interrupt to trigger with new values in FIFO */ 
    int new_value_read_on(void);
    /** @brief Stop the interrupt from triggering with new values in FIFO */ 
    int new_value_read_off(void);

    // void read_fifo(int* red, int* green, int* ir);
    // char read_package_temp();

    // Functions not yet working
    // char read_status();

    int read_interrupt_state(int &value);

    int set_watermark(int level);

private:
    TWIM * _i2c = &I2C2;

    uint8_t _addr = DT_REG_ADDR(DT_NODELABEL(maxm86161));

    // void _set_led_sequence(char sequence);

    int _read_from_reg(int address, int &data);
    int _read_block(int address, int length, uint8_t *data);
    int _write_to_reg(int address, int value);
    int _set_one_bit(int current_bits, int position);
    int _clear_one_bit(int current_bits, int position);
    int _set_multiple_bits(int current_bits, int mask, int new_value, int position);
    int _clear_interrupt(void);
    // int _get_data_label(int &data, int &label, char[int] &raw);

};

/*******************************************************************************/
// I2C address of the PPG sensor
//#define PPG_ADDR 0xC4
#define PPG_ADDR 0x62

// Part ID of the MAXM86161
#define PPG_PART_ID 0x36

// Constants for reading data
#define BYTES_PER_CH 3
#define LED_NUM 4 // 3 LEDs plus the ambient sample
#define FIFO_SIZE 128


// Bit positions.
#define POS_DATA_RDY_EN 6

#define POS_START_STOP  1  

#define POS_ALC_DIS  7
#define POS_PPG_TINT 0

#define POS_PICKET_DIS  7

#define POS_PPG_SR 3
#define POS_SMP_AVG 0

#define MAXM86161_INT_FULL                            0x80
#define MAXM86161_INT_DATA_RDY                        0x40

/*******************************************************************************/
// Bit masks.
#define MASK_SMP_AVE 0b00000111  //Register 0x12
#define MASK_PPG_SR 0b11111000  // Register 0x12
#define MASK_PPG_TINT_WRITE 0b11111100  // Register 0x11

#define MASK_PPG_LABEL 0x7FFFF;
#define MASK_PPG_ID 0xF8;

/*******************************************************************************
 ************************** Maxm86161 I2C Registers *******************************
 ******************************************************************************/

//Status group
#define REG_IRQ_STATUS1		0x00
#define REG_IRQ_STATUS2		0x01
#define REG_IRQ_ENABLE1		0x02
#define REG_IRQ_ENABLE2		0x03

//FIFO group
#define REG_FIFO_WRITE_PTR	  0x04
#define REG_FIFO_READ_PTR		  0x05
#define REG_OVF_COUNTER			  0x06
#define REG_FIFO_DATA_COUNTER	0x07
#define REG_FIFO_DATA			    0x08
#define REG_FIFO_CONFIG1		  0x09
#define REG_FIFO_CONFIG2		  0x0A

//System
#define REG_SYSTEM_CONTROL		0x0D

//PPG configuration
#define REG_PPG_SYNC_CONTROL	  0x10
#define REG_PPG_CONFIG1			    0x11
#define REG_PPG_CONFIG2			    0x12
#define REG_PPG_CONFIG3			    0x13
#define REG_PROX_INT_THRESHOLD	0x14
#define REG_PD_BIAS				      0x15

//PPG picket fence detect and replace
#define REG_PICKET_FENCE	0x16

//LEd sequence control
#define REG_LED_SEQ1		0x20
#define REG_LED_SEQ2		0x21
#define REG_LED_SEQ3		0x22

//LED pulse amplitude
#define REG_LED1_PA			  0x23
#define REG_LED2_PA			  0x24
#define REG_LED3_PA			  0x25
#define REG_LED_PILOT_PA	0x29
#define REG_LED_RANGE1		0x2A

//PPG1_HI_RES_DAC
#define REG_S1_HI_RES_DAC1	0x2C
#define REG_S2_HI_RES_DAC1	0x2D
#define REG_S3_HI_RES_DAC1	0x2E
#define REG_S4_HI_RES_DAC1	0x2D
#define REG_S5_HI_RES_DAC1	0x30
#define REG_S6_HI_RES_DAC1	0x31

//Die temperature
#define REG_DIE_TEMP_CONFIG		0x40
#define REG_DIE_TEMP_INT		  0x41
#define REG_DIE_TEMP_FRACTION	0x42

//DAC calibration
#define REG_DAC_CALIB_ENABLE	0x50

//SHA256
#define REG_SHA_CMD		  0xF0
#define REG_SHA_CONFIG	0xF1

//Memory
#define REG_MEMORY_CONTROL	0xF2
#define REG_MEMORY_INDEX	  0xF3
#define REG_MEMORY_DATA		  0xF4

// Part ID
#define REG_REV_ID		0xFE
#define REG_PART_ID		0xFF

#define REG_FIFO_DATA_MASK  0x07FFFF
#define REG_FIFO_RES        19
#define REG_FIFO_TAG_MASK   0x1F


#endif /* __MAXM86161_H_*/