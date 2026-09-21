#ifndef _ADAU1860_H
#define _ADAU1860_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//#include <Wire.h>
#include <TWIM.h>

#define OUT_VOLUME_DEFAULT 0x80
#define MAX_VOLUME_REG_VAL 0xC0
#define MIN_VOLUME_REG_VAL 0x20
#define MAX_VOLUME_DB 24

//#define OUT_VOLUME_DEFAULT 0x40
//#define MAX_VOLUME_REG_VAL 0x80

#define EQ_PROG_MEM 0x4000A000
#define EQ_BANK_0   0x4000A200
#define EQ_BANK_1   0x4000A400

#define FDSP_NUM_BANKS 3

#define FDSP_PARAM_SIZE 0x100
#define FDSP_NUM_PARAMS 5

#define FDSP_PROG_MEM 0x40008000
#define FDSP_BANK(K,N)  (0x40008100 + K * FDSP_PARAM_SIZE * FDSP_NUM_PARAMS + N * FDSP_PARAM_SIZE)
#define FDSP_BANK_A(N)   FDSP_BANK(0,N)
#define FDSP_BANK_B(N)   FDSP_BANK(1,N)
#define FDSP_BANK_C(N)   FDSP_BANK(2,N)
#define FDSP_STATE    0x40009000

#define DAC_ROUTE_EQ 75
#define DAC_ROUTE_I2S 0
#define DAC_ROUTE_DSP_CH(N) (32 + N)

#define FDSP_USED_BANK_SIZE 5
#define NOISE_GATE_ACTIVE

typedef uint32_t safe_load_params[FDSP_NUM_PARAMS];

enum sl_address {
    BIQ_0 = 0,
# ifdef NOISE_GATE_ACTIVE
    EXPANDER = FDSP_USED_BANK_SIZE,
    VOLUME,
# else
    VOLUME = FDSP_USED_BANK_SIZE,
#endif
    MUTE,
    MIXER,
    LIMITER_MASTER,
};

class ADAU1860 {
public:
    enum registers : uint32_t {
        VENDOR_ID = 0x4000C000,          // ADI Vendor ID
        DEVICE_ID1 = 0x4000C001,         // Device ID 1
        DEVICE_ID2 = 0x4000C002,         // Device ID 2
        REVISION = 0x4000C003,           // Revision Code
        ADC_DAC_HP_PWR = 0x4000C004,     // ADC, DAC, Headphone Power Controls
        PLL_PGA_PWR = 0x4000C005,        // PLL, Mic Bias, and PGA Power Controls
        DMIC_PWR = 0x4000C006,           // Digital Mic Power Controls
        SAI_CLK_PWR = 0x4000C007,        // Serial Port, PDM Output, and DMIC CLK Power Controls
        DSP_PWR = 0x4000C008,            // DSP Power Controls
        ASRC_PWR = 0x4000C009,           // ASRC Power Controls
        FINT_PWR = 0x4000C00A,           // Interpolator Power Controls
        FDEC_PWR = 0x4000C00B,           // Decimator Power Controls
        KEEP_CM = 0x4000C00C,            // State Retention Controls
        MEM_RETAIN = 0x4000C00D,         // Retention Control for ADP and SOC Memories
        CHIP_PWR = 0x4000C00E,           // Chip Power Control
        CLK_CTRL1 = 0x4000C010,          // Clock Control
        CLK_CTRL2 = 0x4000C011,          // PLL Input Divider
        CLK_CTRL3 = 0x4000C012,          // PLL Feedback Integer Divider (LSBs)
        CLK_CTRL4 = 0x4000C013,          // PLL Feedback Integer Divider (MSBs)
        CLK_CTRL5 = 0x4000C014,          // PLL Fractional numerator value (LSBs)
        CLK_CTRL6 = 0x4000C015,          // PLL Fractional numerator value (MSBs)
        CLK_CTRL7 = 0x4000C016,          // PLL Fractional denominator (LSBs)
        CLK_CTRL8 = 0x4000C017,          // PLL Fractional denominator (MSBs)
        CLK_CTRL9 = 0x4000C018,          // PLL Update
        CLK_CTRL10 = 0x4000C019,         // TDSP Clock Rate, BUS(AHB/APB) Clock Rate
        CLK_CTRL11 = 0x4000C01A,         // AON and UART_CTRL Clock Rate
        CLK_CTRL12 = 0x4000C01B,         // Frequency Multiplier Enable, Ratio
        CLK_CTRL13 = 0x4000C01C,         // PLL Frequency Index and ADC Frequency Index
        CLK_CTRL14 = 0x4000C01D,         // ADP_Clock_Enable, Low byte
        CLK_CTRL15 = 0x4000C01E,         // ADP_Clock_Enable, High byte
        ADC_CTRL1 = 0x4000C020,          // ADC Sample Rate Control
        ADC_CTRL2 = 0x4000C021,          // ADC IBias Controls
        ADC_CTRL3 = 0x4000C022,          // ADC IBias Controls
        ADC_CTRL4 = 0x4000C023,          // ADC HPF Control
        ADC_CTRL5 = 0x4000C024,          // ADC Mute and Compensation Control
        ADC_CTRL6 = 0x4000C025,          // Analog Input Pre-Charge Time
        ADC_CTRL7 = 0x4000C026,          // Analog Input Pre-Charge Time
        ADC_MUTES = 0x4000C027,          // ADC Channel Mutes
        ADC0_VOL = 0x4000C028,           // ADC Channel 0 Volume Control
        ADC1_VOL = 0x4000C029,           // ADC Channel 1 Volume Control
        ADC2_VOL = 0x4000C02A,           // ADC Channel 2 Volume Control
        ADC_DITHER_LEV = 0x4000C02B,     // ADC Dithering level
        PGA0_CTRL1 = 0x4000C030,         // PGA Channel 0 Gain Control LSB's
        PGA0_CTRL2 = 0x4000C031,         // PGA Channel 0 Gain Control MSB's
        PGA1_CTRL1 = 0x4000C032,         // PGA Channel 1 Gain Control LSB's
        PGA1_CTRL2 = 0x4000C033,         // PGA Channel 1 Gain Control MSB's
        PGA2_CTRL1 = 0x4000C034,         // PGA Channel 2 Gain Control LSB's
        PGA2_CTRL2 = 0x4000C035,         // PGA Channel 2 Gain Control MSB's
        PGA_CTRL1 = 0x4000C036,          // PGA Slew Rate and Gain Link
        PGA_CTRL2 = 0x4000C037,          // PGA RIN and Power Mode
        DMIC_CTRL1 = 0x4000C040,         // DMIC clock rate control
        DMIC_CTRL2 = 0x4000C041,         // DMIC Channels 0 and 1 rate, order, mapping, and edge control
        DMIC_CTRL3 = 0x4000C042,         // DMIC Channels 2 and 3 rate, order, mapping, and edge control
        DMIC_CTRL4 = 0x4000C043,         // DMIC Volume Options
        DMIC_MUTES = 0x4000C044,         // DMIC Channel Mute Controls
        DMIC_VOL0 = 0x4000C045,          // DMIC Channel 0 Volume Control
        DMIC_VOL1 = 0x4000C046,          // DMIC Channel 1 Volume Control
        DMIC_VOL2 = 0x4000C047,          // DMIC Channel 2 Volume Control
        DMIC_VOL3 = 0x4000C048,          // DMIC Channel 3 Volume Control
        DMIC_CTRL5 = 0x4000C049,         // DMIC Channels 4 and 5 rate, order, mapping, and edge control
        DMIC_CTRL6 = 0x4000C04A,         // DMIC Channels 6 and 7 rate, order, mapping, and edge control
        DMIC_CTRL7 = 0x4000C04B,         // DMIC Clock Map, DMIC Clock 1 Source Pin Select
        DMIC_VOL4 = 0x4000C04C,          // DMIC Channel 4 Volume Control
        DMIC_VOL5 = 0x4000C04D,          // DMIC Channel 5 Volume Control
        DMIC_VOL6 = 0x4000C04E,          // DMIC Channel 6 Volume Control
        DMIC_VOL7 = 0x4000C04F,          // DMIC Channel 7 Volume Control
        DAC_CTRL1 = 0x4000C050,          // DAC Sample Rate, Filtering, and Power Controls
        DAC_CTRL2 = 0x4000C051,          // DAC Volume Link, HPF, and Mute Controls
        DAC_VOL0 = 0x4000C052,           // DAC Channel 0 Volume
        DAC_ROUTE0 = 0x4000C053,         // DAC Channel 0 Routing
        HP_CTRL = 0x4000C060,            // Headphone control
        HP_LVMODE_CTRL1 = 0x4000C061,    // HP Low voltage Mode Enable, CM Enable
        HP_LVMODE_CTRL2 = 0x4000C062,    // HP Low Voltage Auto Switch Mode, Delay, CM Delay
        HP_LVMODE_CTRL3 = 0x4000C063,    // HP Low Voltage Auto Switch(Go)
        HPLDO_CTRL = 0x4000C066,         // HPLDO CTRL
        PB_CTRL = 0x4000C06C,            // HP Low voltage mode control
        PMU_CTRL1 = 0x4000C070,          // Memory Power Control
        PMU_CTRL2 = 0x4000C071,          // CM Control
        PMU_CTRL3 = 0x4000C072,          // IRQ Wakeup Control
        PMU_CTRL4 = 0x4000C073,          // IRQ Wakeup Control
        PMU_CTRL5 = 0x4000C074,          // DLDO Control
        FDEC_CTRL1 = 0x4000C080,         // Fast to Slow Decimator Sample Rates Channels 0 and 1
        FDEC_CTRL2 = 0x4000C081,         // Fast to Slow Decimator Sample Rates Channels 2 and 3
        FDEC_CTRL3 = 0x4000C082,         // Fast to Slow Decimator Sample Rates Channels 4 and 5
        FDEC_CTRL4 = 0x4000C083,         // Fast to Slow Decimator Sample Rates Channels 6 and 7
        FDEC_ROUTE0 = 0x4000C084,        // Fast to Slow Decimator Channel 0 Input Routing
        FDEC_ROUTE1 = 0x4000C085,        // Fast to Slow Decimator Channel 1 Input Routing
        FDEC_ROUTE2 = 0x4000C086,        // Fast to Slow Decimator Channel 2 Input Routing
        FDEC_ROUTE3 = 0x4000C087,        // Fast to Slow Decimator Channel 3 Input Routing
        FDEC_ROUTE4 = 0x4000C088,        // Fast to Slow Decimator Channel 4 Input Routing
        FDEC_ROUTE5 = 0x4000C089,        // Fast to Slow Decimator Channel 5 Input Routing
        FDEC_ROUTE6 = 0x4000C08A,        // Fast to Slow Decimator Channel 6 Input Routing
        FDEC_ROUTE7 = 0x4000C08B,        // Fast to Slow Decimator Channel 7 Input Routing
        FINT_CTRL1 = 0x4000C090,         // Slow to Fast Interpolator Sample Rates Channels 0/1
        FINT_CTRL2 = 0x4000C091,         // Slow to Fast Interpolator Sample Rates Channels 2/3
        FINT_CTRL3 = 0x4000C092,         // Slow to Fast Interpolator Sample Rates Channels 4/5
        FINT_CTRL4 = 0x4000C093,         // Slow to Fast Interpolator Sample Rates Channels 6/7
        FINT_ROUTE0 = 0x4000C094,        // Slow to Fast Interpolator Channel 0 Input Routing
        FINT_ROUTE1 = 0x4000C095,        // Slow to Fast Interpolator Channel 1 Input Routing
        FINT_ROUTE2 = 0x4000C096,        // Slow to Fast Interpolator Channel 2 Input Routing
        FINT_ROUTE3 = 0x4000C097,        // Slow to Fast Interpolator Channel 3 Input Routing
        FINT_ROUTE4 = 0x4000C098,        // Slow to Fast Interpolator Channel 4 Input Routing
        FINT_ROUTE5 = 0x4000C099,        // Slow to Fast Interpolator Channel 5 Input Routing
        FINT_ROUTE6 = 0x4000C09A,        // Slow to Fast Interpolator Channel 6 Input Routing
        FINT_ROUTE7 = 0x4000C09B,        // Slow to Fast Interpolator Channel 7 Input Routing
        ASRCI_CTRL = 0x4000C0A0,         // Input ASRC Control, Source, and Rate Selection
        ASRCI_ROUTE01 = 0x4000C0A1,      // Input ASRC Channel 0 and 1 Input Routing
        ASRCI_ROUTE23 = 0x4000C0A2,      // Input ASRC Channel 2 and 3 Input Routing
        ASRCO_CTRL = 0x4000C0A3,         // Output ASRC Control
        ASRCO_ROUTE0 = 0x4000C0A4,       // Output ASRC Channel 0 Input Routing
        ASRCO_ROUTE1 = 0x4000C0A5,       // Output ASRC Channel 1 Input Routing
        ASRCO_ROUTE2 = 0x4000C0A6,       // Output ASRC Channel 2 Input Routing
        ASRCO_ROUTE3 = 0x4000C0A7,       // Output ASRC Channel 3 Input Routing
        FDSP_RUN = 0x4000C0B0,           // FastDSP Run
        FDSP_CTRL1 = 0x4000C0B1,         // FastDSP Current Bank and Bank Ramping Controls
        FDSP_CTRL2 = 0x4000C0B2,         // FastDSP Bank Ramping Stop Point
        FDSP_CTRL3 = 0x4000C0B3,         // FastDSP Bank Copying
        FDSP_CTRL4 = 0x4000C0B4,         // FastDSP Frame Rate Source
        FDSP_CTRL5 = 0x4000C0B5,         // FastDSP Fixed Rate Division MSBs
        FDSP_CTRL6 = 0x4000C0B6,         // FastDSP Fixed Rate Division LSBs
        FDSP_CTRL7 = 0x4000C0B7,         // FastDSP Modulo N Counter for Lower Rate Conditional Execution
        FDSP_CTRL8 = 0x4000C0B8,         // FastDSP Generic Conditional Execution Registers
        FDSP_SL_ADDR = 0x4000C0B9,       // Fast DSP Safeload Address
        FDSP_SL_P0_0 = 0x4000C0BA,       // FastDSP Safeload Parameter 0 Value
        FDSP_SL_P0_1 = 0x4000C0BB,       // FastDSP Safeload Parameter 0 Value
        FDSP_SL_P0_2 = 0x4000C0BC,       // FastDSP Safeload Parameter 0 Value
        FDSP_SL_P0_3 = 0x4000C0BD,       // FastDSP Safeload Parameter 0 Value
        FDSP_SL_P1_0 = 0x4000C0BE,       // FastDSP Safeload Parameter 1 Value
        FDSP_SL_P1_1 = 0x4000C0BF,       // FastDSP Safeload Parameter 1 Value
        FDSP_SL_P1_2 = 0x4000C0C0,       // FastDSP Safeload Parameter 1 Value
        FDSP_SL_P1_3 = 0x4000C0C1,       // FastDSP Safeload Parameter 1 Value
        FDSP_SL_P2_0 = 0x4000C0C2,       // FastDSP Safeload Parameter 2 Value
        FDSP_SL_P2_1 = 0x4000C0C3,       // FastDSP Safeload Parameter 2 Value
        FDSP_SL_P2_2 = 0x4000C0C4,       // FastDSP Safeload Parameter 2 Value
        FDSP_SL_P2_3 = 0x4000C0C5,       // FastDSP Safeload Parameter 2 Value
        FDSP_SL_P3_0 = 0x4000C0C6,       // FastDSP Safeload Parameter 3 Value
        FDSP_SL_P3_1 = 0x4000C0C7,       // FastDSP Safeload Parameter 3 Value
        FDSP_SL_P3_2 = 0x4000C0C8,       // FastDSP Safeload Parameter 3 Value
        FDSP_SL_P3_3 = 0x4000C0C9,       // FastDSP Safeload Parameter 3 Value
        FDSP_SL_P4_0 = 0x4000C0CA,       // FastDSP Safeload Parameter 4 Value
        FDSP_SL_P4_1 = 0x4000C0CB,       // FastDSP Safeload Parameter 4 Value
        FDSP_SL_P4_2 = 0x4000C0CC,       // FastDSP Safeload Parameter 4 Value
        FDSP_SL_P4_3 = 0x4000C0CD,       // FastDSP Safeload Parameter 4 Value
        FDSP_SL_UPDATE = 0x4000C0CE,     // FastDSP Safeload Update
        FDSP_ONZ_MASK0 = 0x4000C0CF,     // FastDSP ONZ Mask 0
        FDSP_ONZ_MASK1 = 0x4000C0D0,     // FastDSP ONZ Mask 1
        FDSP_ONZ_MASK2 = 0x4000C0D1,     // FastDSP ONZ Mask 2
        EQ_CFG = 0x4000C0D2,             // EQ Configure
        EQ_ROUTE = 0x4000C0D3,           // EQ Routing
        TDSP_SOFT_RESET = 0x4000C0D4,    // Tensilica DSP Software Reset
        TDSP_ALTVEC_EN = 0x4000C0D5,     // Tensilica DSP Alternative Reset Vector Enable
        TDSP_ALTVEC_ADDR0 = 0x4000C0D8,  // Tensilica DSP Alternative Reset Vector Address Byte 0
        TDSP_ALTVEC_ADDR1 = 0x4000C0D9,  // Tensilica DSP Alternative Reset Vector Address Byte 1
        TDSP_ALTVEC_ADDR2 = 0x4000C0DA,  // Tensilica DSP Alternative Reset Vector Address Byte 2
        TDSP_ALTVEC_ADDR3 = 0x4000C0DB,  // Tensilica DSP Alternative Reset Vector Address Byte 3
        TDSP_RUN = 0x4000C0DC,           // Tensilica DSP Run
        SPT0_CTRL1 = 0x4000C0E0,         // Serial Port 0 Control 1
        SPT0_CTRL2 = 0x4000C0E1,         // Serial Port 0 Control 2
        SPT0_CTRL3 = 0x4000C0E2,         // Serial Port 0 Control 3
        SPT0_ROUTE0 = 0x4000C0E3,        // Serial Port 0 Output Routing Slot 0 (Left)
        SPT0_ROUTE1 = 0x4000C0E4,        // Serial Port 0 Output Routing Slot 1 (Right)
        SPT0_ROUTE2 = 0x4000C0E5,        // Serial Port 0 Output Routing Slot 2
        SPT0_ROUTE3 = 0x4000C0E6,        // Serial Port 0 Output Routing Slot 3
        SPT0_ROUTE4 = 0x4000C0E7,        // Serial Port 0 Output Routing Slot 4
        SPT0_ROUTE5 = 0x4000C0E8,        // Serial Port 0 Output Routing Slot 5
        SPT0_ROUTE6 = 0x4000C0E9,        // Serial Port 0 Output Routing Slot 6
        SPT0_ROUTE7 = 0x4000C0EA,        // Serial Port 0 Output Routing Slot 7
        SPT0_ROUTE8 = 0x4000C0EB,        // Serial Port 0 Output Routing Slot 8
        SPT0_ROUTE9 = 0x4000C0EC,        // Serial Port 0 Output Routing Slot 9
        SPT0_ROUTE10 = 0x4000C0ED,       // Serial Port 0 Output Routing Slot 10
        SPT0_ROUTE11 = 0x4000C0EE,       // Serial Port 0 Output Routing Slot 11
        SPT0_ROUTE12 = 0x4000C0EF,       // Serial Port 0 Output Routing Slot 12
        SPT0_ROUTE13 = 0x4000C0F0,       // Serial Port 0 Output Routing Slot 13
        SPT0_ROUTE14 = 0x4000C0F1,       // Serial Port 0 Output Routing Slot 14
        SPT0_ROUTE15 = 0x4000C0F2,       // Serial Port 0 Output Routing Slot 15
        SPT1_CTRL1 = 0x4000C0F3,         // Serial Port 1 Control 1
        SPT1_CTRL2 = 0x4000C0F4,         // Serial Port 1 Control 2
        SPT1_CTRL3 = 0x4000C0F5,         // Serial Port 1 Control 3
        SPT1_ROUTE0 = 0x4000C0F6,        // Serial Port 1 Output Routing Slot 0 (Left)
        SPT1_ROUTE1 = 0x4000C0F7,        // Serial Port 1 Output Routing Slot 1 (Right)
        SPT1_ROUTE2 = 0x4000C0F8,        // Serial Port 1 Output Routing Slot 2
        SPT1_ROUTE3 = 0x4000C0F9,        // Serial Port 1 Output Routing Slot 3
        SPT1_ROUTE4 = 0x4000C0FA,        // Serial Port 1 Output Routing Slot 4
        SPT1_ROUTE5 = 0x4000C0FB,        // Serial Port 1 Output Routing Slot 5
        SPT1_ROUTE6 = 0x4000C0FC,        // Serial Port 1 Output Routing Slot 6
        SPT1_ROUTE7 = 0x4000C0FD,        // Serial Port 1 Output Routing Slot 7
        SPT1_ROUTE8 = 0x4000C0FE,        // Serial Port 1 Output Routing Slot 8
        SPT1_ROUTE9 = 0x4000C0FF,        // Serial Port 1 Output Routing Slot 9
        SPT1_ROUTE10 = 0x4000C100,       // Serial Port 1 Output Routing Slot 10
        SPT1_ROUTE11 = 0x4000C101,       // Serial Port 1 Output Routing Slot 11
        SPT1_ROUTE12 = 0x4000C102,       // Serial Port 1 Output Routing Slot 12
        SPT1_ROUTE13 = 0x4000C103,       // Serial Port 1 Output Routing Slot 13
        SPT1_ROUTE14 = 0x4000C104,       // Serial Port 1 Output Routing Slot 14
        SPT1_ROUTE15 = 0x4000C105,       // Serial Port 1 Output Routing Slot 15
        PDM_CTRL1 = 0x4000C118,          // PDM Sample Rate and Filtering Control
        PDM_CTRL2 = 0x4000C119,          // PDM Muting, High-Pass, and Volume Options
        PDM_VOL0 = 0x4000C11A,           // PDM Output Channel 0 Volume
        PDM_VOL1 = 0x4000C11B,           // PDM Output Channel 1 Volume
        PDM_ROUTE0 = 0x4000C11C,         // PDM Output Channel 0 Routing
        PDM_ROUTE1 = 0x4000C11D,         // PDM Output Channel 1 Routing
        MP_CTRL1 = 0x4000C121,           // MultiPurpose Pins 0/1 Mode Select
        MP_CTRL2 = 0x4000C122,           // MultiPurpose Pins 2/3 Mode Select
        MP_CTRL3 = 0x4000C123,           // MultiPurpose Pins 4/5 Mode Select
        MP_CTRL4 = 0x4000C124,           // MultiPurpose Pins 6/7 Mode Select
        MP_CTRL5 = 0x4000C125,           // MultiPurpose Pins 8/9 Mode Select
        MP_CTRL6 = 0x4000C126,           // MultiPurpose Pin 10/11 Mode Select
        MP_CTRL7 = 0x4000C127,           // MultiPurpose Pin 12/13 Mode Select
        MP_CTRL8 = 0x4000C128,           // MultiPurpose Pin 14/15 Mode Select
        MP_CTRL9 = 0x4000C129,           // MultiPurpose Pin 16/17 Mode Select
        MP_CTRL10 = 0x4000C12A,          // MultiPurpose Pin 18/19 Mode Select
        MP_CTRL11 = 0x4000C12B,          // MultiPurpose Pin 20/21 Mode Select
        MP_CTRL12 = 0x4000C12C,          // MultiPurpose Pin 22/23 Mode Select
        MP_CTRL13 = 0x4000C12D,          // MultiPurpose Pin 24/25 Mode Select
        MP_DB_CTRL = 0x4000C12E,         // General Purpose Input Debounce Control and IRQ Input Debounce Control
        MP_MCLKO_RATE = 0x4000C12F,      // MCLKO Rate Selection
        MP_GPIO_CTRL1 = 0x4000C130,      // General Purpose Outputs Control Pins 0-7
        MP_GPIO_CTRL2 = 0x4000C131,      // General Purpose Outputs Control Pins 8-15
        MP_GPIO_CTRL3 = 0x4000C132,      // General Purpose Outputs Control Pins 16-23
        MP_GPIO_CTRL4 = 0x4000C133,      // General Purpose Outputs Control Pins 24-25
        DMIC_CLK_CTRL = 0x4000C134,      // DMIC_CLK Pin Controls
        DMIC01_CTRL = 0x4000C135,        // DMIC01 Pin Controls
        DMIC23_CTRL = 0x4000C136,        // DMIC23 Pin Controls
        BCLK0_CTRL = 0x4000C137,         // BCLK_0 Pin Controls
        FSYNC0_CTRL = 0x4000C138,        // FSYNC_0 Pin Controls
        SDATAO0_CTRL = 0x4000C139,       // SDATAO_0 Pin Controls
        SDATAI0_CTRL = 0x4000C13A,       // SDATAI_0 Pin Controls
        BCLK1_CTRL = 0x4000C13B,         // BCLK_1 Pin Controls
        FSYNC1_CTRL = 0x4000C13C,        // FSYNC_1 Pin Controls
        SDATAO1_CTRL = 0x4000C13D,       // SDATAO_1 Pin Controls
        SDATAI1_CTRL = 0x4000C13E,       // SDATAI_1 Pin Controls
        QSPIM_CLK_CTRL = 0x4000C13F,     // QSPIM_CLK Pin Controls
        QSPIM_CS_CTRL = 0x4000C140,      // QSPIM_CS Pin Controls
        QSPIM_SDIO0_CTRL = 0x4000C141,   // QSPIM_SDIO0 Pin Controls
        QSPIM_SDIO1_CTRL = 0x4000C142,   // QSPIM_SDIO1 Pin Controls
        QSPIM_SDIO2_CTRL = 0x4000C143,   // QSPIM_SDIO2 Pin Controls
        QSPIM_SDIO3_CTRL = 0x4000C144,   // QSPIM_SDIO3 Pin Controls
        UART_COMM_TX_CTRL = 0x4000C145,  // UART_COMM_TX Pin Controls
        UART_COMM_RX_CTRL = 0x4000C146,  // UART_COMM_RX Pin Controls
        SELFBOOT_CTRL = 0x4000C147,      // SELFBOOT Pin Controls
        IRQ_CTRL = 0x4000C148,           // IRQ Pin Controls
        ROM_BOOT_MODE_CTRL = 0x4000C149, // ROM Boot Mode Pin Controls
        TCK_CTRL = 0x4000C14A,           // TCK Pin Controls
        TMS_CTRL = 0x4000C14B,           // TMS Pin Controls
        TDO_CTRL = 0x4000C14C,           // TDO Pin Controls
        TDI_CTRL = 0x4000C14D,           // TDI Pin Controls
        I2C_SPI_CTRL = 0x4000C14E,       // SDA/MISO Pin Controls
        IRQ_CTRL1 = 0x4000C150,          // IRQ signaling and clearing
        IRQ1_MASK1 = 0x4000C151,         // IRQ1 Masking
        IRQ1_MASK2 = 0x4000C152,         // IRQ1 Masking
        IRQ1_MASK3 = 0x4000C153,         // IRQ1 Masking
        IRQ1_MASK4 = 0x4000C154,         // IRQ1 Masking
        IRQ1_MASK5 = 0x4000C155,         // IRQ1 Masking
        IRQ2_MASK1 = 0x4000C156,         // IRQ2 Masking
        IRQ2_MASK2 = 0x4000C157,         // IRQ2 Masking
        IRQ2_MASK3 = 0x4000C158,         // IRQ2 Masking
        IRQ2_MASK4 = 0x4000C159,         // IRQ2 Masking
        IRQ2_MASK5 = 0x4000C15A,         // IRQ2 Masking
        IRQ3_MASK1 = 0x4000C15B,         // IRQ3 Masking
        IRQ3_MASK2 = 0x4000C15C,         // IRQ3 Masking
        IRQ3_MASK3 = 0x4000C15D,         // IRQ3 Masking
        IRQ3_MASK4 = 0x4000C15E,         // IRQ3 Masking
        IRQ3_MASK5 = 0x4000C15F,         // IRQ3 Masking
        IRQ4_MASK1 = 0x4000C160,         // IRQ4 Masking
        IRQ4_MASK2 = 0x4000C161,         // IRQ4 Masking
        IRQ4_MASK3 = 0x4000C162,         // IRQ4 Masking
        IRQ4_MASK4 = 0x4000C163,         // IRQ4 Masking
        IRQ4_MASK5 = 0x4000C164,         // IRQ4 Masking
        SW_INT = 0x4000C165,             // Software Interrupts which can be set by external host or TDSP
        MP_INPUT_IRQ_CTRL1 = 0x4000C168, // MP IRQ clearing
        MP_INPUT_IRQ_CTRL2 = 0x4000C169, // MP IRQ clearing
        MP_INPUT_IRQ1_MASK1 = 0x4000C16A, // MP IRQ1 Masking
        MP_INPUT_IRQ1_MASK2 = 0x4000C16B, // MP IRQ1 Masking
        MP_INPUT_IRQ1_MASK3 = 0x4000C16C, // MP IRQ1 Masking
        MP_INPUT_IRQ1_MASK4 = 0x4000C16D, // MP IRQ1 Masking
        MP_INPUT_IRQ2_MASK1 = 0x4000C16E, // MP IRQ2 Masking
        MP_INPUT_IRQ2_MASK2 = 0x4000C16F, // MP IRQ2 Masking
        MP_INPUT_IRQ2_MASK3 = 0x4000C170, // MP IRQ2 Masking
        MP_INPUT_IRQ2_MASK4 = 0x4000C171, // MP IRQ2 Masking
        MP_INPUT_IRQ3_MASK1 = 0x4000C172, // MP IRQ3 Masking
        MP_INPUT_IRQ3_MASK2 = 0x4000C173, // MP IRQ3 Masking
        MP_INPUT_IRQ3_MASK3 = 0x4000C174, // MP IRQ3 Masking
        MP_INPUT_IRQ3_MASK4 = 0x4000C175, // MP IRQ3 Masking
        RESETS = 0x4000C200,             // Chip Resets
        READ_LAMBDA = 0x4000C400,        // FastDSP Current Lambda
        STATUS1 = 0x4000C401,            // Chip Status 1
        STATUS2 = 0x4000C402,            // Chip Status 2
        SELFBOOT_STATUS = 0x4000C403,    // Tensilica DSP Self-boot Indicator
        EQ_STATUS = 0x4000C404,          // EQ Status
        FDSP_STATUS = 0x4000C405,        // FastDSP ONZ Status
        PMU_STATUS1 = 0x4000C406,        // Power Mode Status, DLDO Scale Busy, CM Delay Counter Done
        PMU_STATUS2 = 0x4000C407,        // Memory Retention Status
        PMU_STATUS3 = 0x4000C408,        // ADP Memory Shutdown Status
        PMU_STATUS4 = 0x4000C409,        // SOC Memory Shutdown Status
        TDSP_MODE_STATUS = 0x4000C40A,   // TDSP Mode Status
        TDSP_ERROR_STATUS = 0x4000C40B,  // TDSP Exception/Error Status
        TDSP_FAULT_INFO1 = 0x4000C40C,   // TDSP Fault Information
        TDSP_FAULT_INFO2 = 0x4000C40D,   // TDSP Fault Information
        TDSP_FAULT_INFO3 = 0x4000C40E,   // TDSP Fault Information
        TDSP_FAULT_INFO4 = 0x4000C40F,   // TDSP Fault Information
        TDSP_GPO1 = 0x4000C410,          // TDSP General Purpose Output Bit 0~7
        TDSP_GPO2 = 0x4000C411,          // TDSP General Purpose Output Bit 8~15
        TDSP_GPO3 = 0x4000C412,          // TDSP General Purpose Output Bit 16~23
        TDSP_GPO4 = 0x4000C413,          // TDSP General Purpose Output Bit 24~31
        IRQ1_STATUS1 = 0x4000C414,       // IRQ1 Status 1
        IRQ1_STATUS2 = 0x4000C415,       // IRQ1 Status 2
        IRQ1_STATUS3 = 0x4000C416,       // IRQ1 Status 3
        IRQ1_STATUS4 = 0x4000C417,       // IRQ1 Status 4
        IRQ1_STATUS5 = 0x4000C418,       // IRQ1 Status 5
        IRQ2_STATUS1 = 0x4000C419,       // IRQ2 Status 1
        IRQ2_STATUS2 = 0x4000C41A,       // IRQ2 Status 2
        IRQ2_STATUS3 = 0x4000C41B,       // IRQ2 Status 3
        IRQ2_STATUS4 = 0x4000C41C,       // IRQ2 Status 4
        IRQ2_STATUS5 = 0x4000C41D,       // IRQ2 Status 5
        IRQ3_STATUS1 = 0x4000C41E,       // IRQ3 Status 1
        IRQ3_STATUS2 = 0x4000C41F,       // IRQ3 Status 2
        IRQ3_STATUS3 = 0x4000C420,       // IRQ3 Status 3
        IRQ3_STATUS4 = 0x4000C421,       // IRQ3 Status 4
        IRQ3_STATUS5 = 0x4000C422,       // IRQ3 Status 5
        IRQ4_STATUS1 = 0x4000C423,       // IRQ4 Status 1
        IRQ4_STATUS2 = 0x4000C424,       // IRQ4 Status 2
        IRQ4_STATUS3 = 0x4000C425,       // IRQ4 Status 3
        IRQ4_STATUS4 = 0x4000C426,       // IRQ4 Status 4
        IRQ4_STATUS5 = 0x4000C427,       // IRQ4 Status 5
        MP_INPUT_IRQ1_STATUS1 = 0x4000C428, // Multi Purpose IRQ1 Status 1
        MP_INPUT_IRQ1_STATUS2 = 0x4000C429, // Multi Purpose IRQ1 Status 2
        MP_INPUT_IRQ1_STATUS3 = 0x4000C42A, // Multi Purpose IRQ1 Status 3
        MP_INPUT_IRQ1_STATUS4 = 0x4000C42B, // Multi Purpose IRQ1 Status 4
        MP_INPUT_IRQ2_STATUS1 = 0x4000C42C, // Multi Purpose IRQ2 Status 1
        MP_INPUT_IRQ2_STATUS2 = 0x4000C42D, // Multi Purpose IRQ2 Status 2
        MP_INPUT_IRQ2_STATUS3 = 0x4000C42E, // Multi Purpose IRQ2 Status 3
        MP_INPUT_IRQ2_STATUS4 = 0x4000C42F, // Multi Purpose IRQ2 Status 4
        MP_INPUT_IRQ3_STATUS1 = 0x4000C430, // Multi Purpose IRQ3 Status 1
        MP_INPUT_IRQ3_STATUS2 = 0x4000C431, // Multi Purpose IRQ3 Status 2
        MP_INPUT_IRQ3_STATUS3 = 0x4000C432, // Multi Purpose IRQ3 Status 3
        MP_INPUT_IRQ3_STATUS4 = 0x4000C433, // Multi Purpose IRQ3 Status 4
        GPI1 = 0x4000C434,               // General Purpose Input Read 0-7
        GPI2 = 0x4000C435,               // General Purpose Input Read 8-15
        GPI3 = 0x4000C436,               // General Purpose Input Read 16-20
        GPI4 = 0x4000C437,               // General Purpose Input Read 16-20
        CTRL_PORT_MODE = 0x4000C438,     // Control Port Mode
        DAC_NOISE_CTRL2 = 0x4000CC04,    // DAC/PLL Test Modes
        DAC_NOISE_CTRL1 = 0x4000CC12     // DAC Noise Control1
    };

    ADAU1860(TWIM * i2c);

    int begin();
    int end();
    int setup();
    int mute(bool active);
    int set_volume(uint8_t volume);

    int soft_reset(bool full_reset = false);

    uint8_t get_volume();

#if CONFIG_FDSP
    int fdsp_bank_select(uint8_t bank);
#endif
private:
    bool readReg(uint32_t reg, uint8_t * buffer, uint16_t len);
    void writeReg(uint32_t reg, uint8_t * buffer, uint16_t len);

    int setup_EQ();
    int setup_FDSP();
    int setup_DAC();

    int fdsp_mute(bool active);
    int fdsp_set_volume(uint8_t volume);
    uint8_t fdsp_get_volume();

    int fdsp_safe_load(sl_address address, safe_load_params params, bool update_inactive = false);
    int fdsp_safe_load(sl_address address, int n, uint32_t param, bool update_inactive = false);

    const uint16_t address = DT_REG_ADDR(DT_NODELABEL(adau1860));

    const struct gpio_dt_spec dac_enable_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(adau1860), enable_gpios);

    static void check_ascr_lock(struct k_work *work);

    uint64_t last_i2c;
    TWIM *_i2c;

    bool _active = false;

    uint8_t _active_bank = 0;

    //const struct gpio_dt_spec pg_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), pg_gpios);

    friend int cmd_dsp_noise_gate(const struct shell *shell, size_t argc, char **argv);

    friend int32_t adau_read(void* user_data, uint8_t *rd_buf, uint32_t rd_len, uint8_t *wr_buf, uint32_t wr_len);
    friend int32_t adau_write(void* user_data, uint8_t *wr_buf, uint32_t len);
};

extern ADAU1860 dac;

#endif