#include "ADAU1860.h"
#include "zbus_common.h"
#include "openearable_common.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ADAU1860, 3);

uint32_t eq_program[78] =
{
    0x00010000, 0x00025680, 0x0000C000, 0x00001024, 0x00015724, 0x0000C000, 0x00001024, 0x000157A4, 0x0000C000, 0x00001024, 0x000140A0, 0x0000C021, 0x0000C222, 0x0000C1A3, 0x0000C124, 0x0000A024, 0x00001022, 0x0001431C, 0x0000C29D, 0x0000C49E, 0x0000C41F, 0x0000C3A2, 0x0000A020, 0x0000101E, 0x00014598, 0x0000C519, 0x0000C71A, 0x0000C69B, 0x0000C61E, 0x0000A01C, 0x0000101A, 0x00014814, 0x0000C795, 0x0000C996, 0x0000C917, 0x0000C89A, 0x0000A018, 0x00001016, 0x00014A90, 0x0000CA11, 0x0000CC12, 0x0000CB93, 0x0000CB16, 0x0000A014, 0x00001012, 0x00014D0C, 0x0000CC8D, 0x0000CE8E, 0x0000CE0F, 0x0000CD92, 0x0000A010, 0x0000100E, 0x00014F88, 0x0000CF09, 0x0000D10A, 0x0000D08B, 0x0000D00E, 0x0000A00C, 0x0000100A, 0x00015204, 0x0000D185, 0x0000D386, 0x0000D307, 0x0000D28A, 0x0000A008, 0x00001006, 0x00015480, 0x0000D401, 0x0000D602, 0x0000D583, 0x0000D506, 0x0000A004, 0x00001002, 0x00028000, 0x00018000, 0x0003C000, 0x00000000, 0x00000000
};

uint32_t eq_param_bank0[50] =
{
    0x01F51DE7, 0x0F0AC4B4, 0x011E44E7, 0x0DCDB809, 0x01141A69, 0x01F9FF66, 0x0F05E7A1, 0x00FF2342, 0x0E06009A, 0x00FAF51C, 0x01E6665A, 0x0F18189D, 0x00FB8DC2, 0x0E1999A6, 0x00EC59A0, 0x018D7F52, 0x0F41E0D8, 0x00EDC634, 0x0E7280AE, 0x00D058F4, 0x012C8DDB, 0x0F8528EA, 0x00EF5915, 0x0ED37225, 0x008B7E01, 0x014FBC55, 0x0F4172A7, 0x00E99FD1, 0x0EB043AB, 0x00D4ED87, 0x00A87458, 0x0F47CEA8, 0x00EE173F, 0x0F578BA8, 0x00CA1A18, 0x0FC7206C, 0x0F4C46BE, 0x00ECF9E8, 0x0038DF94, 0x00C6BF5A, 0x00B4F678, 0x0FBD1978, 0x00914A08, 0x0FE2AC4F, 0x0019F9B9, 0x01000000, 0x01000000, 0x01000000, 0x01000000, 0x01000000
};

uint32_t eq_param_bank1[50] =
{
    0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01DE28C5, 0x0F1DB6F9, 0x0101D018, 0x0E21D73B, 0x00E078EF, 0x01000000, 0x01000000, 0x01000000, 0x01000000, 0x01000000
};

ADAU1860 dac(&Wire1);

ADAU1860::ADAU1860(TwoWire * wire) : _pWire(wire) {

}

int ADAU1860::begin() {
        int ret;

        ret = pm_device_runtime_get(ls_1_8);

        // pull-up PD pin
        ret = gpio_pin_configure_dt(&dac_enable_pin, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
                LOG_ERR("Failed to set DAC enable as output.\n");
                return ret;
        }

        _pWire->begin();

        //k_msleep(1);

        //last_i2c = micros();

        /*uint8_t status = 0x1;
        writeReg(registers::RESETS, &status, sizeof(status));
        writeReg(registers::RESETS, &status, sizeof(status));*/

        k_msleep(35); // CM rise time (see datasheet)

        uint8_t power_mode = 0x01 | 0x04; // Hibernate 1, Master enable
        writeReg(registers::CHIP_PWR, &power_mode, sizeof(power_mode));

        //uint8_t test;

        // Non self-boot
        uint8_t startup_dlycnt_byp = 1;
        writeReg(registers::PMU_CTRL2, &startup_dlycnt_byp, sizeof(startup_dlycnt_byp));

        // Power saving
        uint8_t cm_startup_over = 1 << 4 | power_mode;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        //uint8_t sai_clk_pwr = 0x01; // I2S_IN enable
        //writeReg(registers::SAI_CLK_PWR, &sai_clk_pwr, sizeof(sai_clk_pwr));

        // bypass PLL
        uint8_t clk_ctrl13 = (1 << 7) | (1 << 4) | 0x01; // (0x01 = 49.152 MHz) // | 0x3;
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));

        //uint8_t clk_ctrl13 = (0 << 7) | (1 << 4) | 0x01; // (0x01 = 49.152 MHz)
        //writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));

        //uint8_t clk_ctrl1 = (0x3 << 6) | (1 << 3); //MCLK/XTAL
        //writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));

        uint8_t clk_ctrl12 = 0x01; // frequency multiplier enabled
        writeReg(registers::CLK_CTRL12, &clk_ctrl12, sizeof(clk_ctrl12));

        clk_ctrl13 &= ~(1 << 7);
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));

        // uint8_t pll_ctrl = 0x2; // XTAL_EN = 1, PLL_EN = 0
        // writeReg(registers::PLL_PGA_PWR, &pll_ctrl, sizeof(pll_ctrl));

        uint8_t asrc_pwr = 0x1; // ASRCI0_EN 
        writeReg(registers::ASRC_PWR, &asrc_pwr, sizeof(asrc_pwr));

        k_msleep(1);

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));
        LOG_INF("STATUS2: 0x%x", status2);

        // ASRCI_CTRL
        //uint8_t asrci_route01 = 0x0; // ASRCI0_EN 
        //writeReg(registers::ASRCI_ROUTE01, &asrci_route01, sizeof(asrci_route01));

        // ASRCI_CTRL

        if (!(status2 & (1 << 7))) LOG_WRN("No power up");

        while (!(status2 & (1 << 7))) {
                readReg(registers::STATUS2, &status2, sizeof(status2));
                LOG_INF("STATUS2: 0x%x", status2);

                // power up complete and PLL lock
                if (!(status2 & (1 << 7))) LOG_WRN("No power up");

                k_usleep(10);
        }

        //LOG_INF("DAC_CTRL: 0x%02X", status);
        // Power saving
        // uint8_t cm_startup_over = 1 << 2 | 0x01; // Master block en | Hibernate 1 (SOC off, ADP on);
        // writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

#if CONFIG_EQAULIZER_DSP
        // DAC_ROUTE0 - EQ 0 / serial port 0 channel 0
        uint8_t dac_route = DAC_ROUTE_EQ;
#else
        uint8_t dac_route = DAC_ROUTE_I2S;
#endif
        writeReg(registers::DAC_ROUTE0, &dac_route, sizeof(dac_route));

        // SPT0_CTRL1 - reset val (32 BCLKs?)
        uint8_t spt0_ctrl1 = 0x10; // 1 << 4 (16 BCLKs)
        writeReg(registers::SPT0_CTRL1, &spt0_ctrl1, sizeof(spt0_ctrl1));
        // SPT0_CTRL2 - reset val

        uint8_t sai_clk_pwr = 0x01; // I2S_IN enable
        if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
                sai_clk_pwr |= (1 << 4) | (1 << 1); // DMIC enable | I2S_IN enable
        }

        writeReg(registers::SAI_CLK_PWR, &sai_clk_pwr, sizeof(sai_clk_pwr));

        if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
                // DMIC_CTRL - reset val (32 BCLKs?)
                uint8_t dmic_pwr = 0x03; // DMIC Channel 0 & 1
                writeReg(registers::DMIC_PWR, &dmic_pwr, sizeof(dmic_pwr));

                // SPT0_ROUTE0 - i2s output route
                uint8_t spt0_route0 = 39; // DMIC Channel 0
                writeReg(registers::SPT0_ROUTE0, &spt0_route0, sizeof(spt0_route0));

                uint8_t spt0_route1 = 40; // DMIC Channel 1
                writeReg(registers::SPT0_ROUTE1, &spt0_route1, sizeof(spt0_route1));

                // DMIC_VOL0
                uint8_t dmic_vol = 0x10;
                writeReg(registers::DMIC_VOL0, &dmic_vol, sizeof(dmic_vol));
                writeReg(registers::DMIC_VOL1, &dmic_vol, sizeof(dmic_vol));

                uint8_t dmic_ctrl = 0x02; // DMIC Channel 0
                writeReg(registers::DMIC_CTRL2, &dmic_ctrl, sizeof(dmic_ctrl));
        }

#if CONFIG_EQAULIZER_DSP

        // EQ_CFG - engine running
        // stop eq and clear
        uint8_t eq_cfg = 0x00; // eq stop
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));
        eq_cfg = 0x10; // eq clear
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));

        // clear done check
        uint8_t eq_status = 0;
        while (eq_status != 0x1) {
                readReg(registers::EQ_STATUS, &eq_status, sizeof(eq_status));
                k_usleep(100);
        }

        // EQ_ROUTE - serial port 0 channel 0
        uint8_t eq_route = 64; // Serial port 0 channel 0
        writeReg(registers::EQ_ROUTE, &eq_route, sizeof(eq_route));

        // uint8_t eq_cfg = 0x10; // Serial port 0 channel 0
        //writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));

        // set Eq params
        writeReg(EQ_PROG_MEM, (uint8_t*) eq_program, sizeof(eq_program));
        writeReg(EQ_BANK_0, (uint8_t*) eq_param_bank0, sizeof(eq_param_bank0));
        writeReg(EQ_BANK_1, (uint8_t*) eq_param_bank1, sizeof(eq_param_bank1));

        // activate eq
        eq_cfg = 0x01;
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));
#endif

        // PB_POWER_CTRL enhanced mode?

        //Headphone power on
        uint8_t headphone_power = 1 << 4; // DAC/HP channel 0 enabled
        writeReg(registers::ADC_DAC_HP_PWR, &headphone_power, sizeof(headphone_power));

        // low voltage 
        uint8_t lvmode = 0x03; //HP_LVMODE_EN | HP_LVMODE_CM_EN
        writeReg(registers::HP_LVMODE_CTRL1, &lvmode, sizeof(lvmode));

        uint8_t lvmode_ctrl2 = (0x3 << 4) | 0x01;
        writeReg(registers::HP_LVMODE_CTRL2, &lvmode_ctrl2, sizeof(lvmode_ctrl2));

        uint8_t lvmode_ctrl3 = 0x01;
        writeReg(registers::HP_LVMODE_CTRL3, &lvmode_ctrl3, sizeof(lvmode_ctrl3));

        uint8_t hpldo_ctrl = 0x01;
        writeReg(registers::HPLDO_CTRL, &hpldo_ctrl, sizeof(hpldo_ctrl));

        // high performance? PB_CTRL
        // DAC_NOISE_CTRL?

        return 0;
}

int ADAU1860::end() {
        int ret;

        // pull-down PD pin
        gpio_pin_set_dt(&dac_enable_pin, 0);

        ret = pm_device_runtime_put(ls_1_8);

        return ret;
}

int ADAU1860::setup() {
        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));
        LOG_INF("STATUS2: 0x%x", status2);

        if (!(status2 & (1 << 7))) LOG_WRN("No power up");
        if (!(status2 & 4)) LOG_WRN("ASCR not locked");

        while (!(status2 & (1 << 7)) || !(status2 & 4)) {
                readReg(registers::STATUS2, &status2, sizeof(status2));
                LOG_INF("STATUS2: 0x%x", status2);

                // power up complete and PLL lock
                if (!(status2 & (1 << 7))) LOG_WRN("No power up");
                if (!(status2 & 4)) LOG_WRN("waiting for ASCR to lock");

                k_usleep(10);
        }

        // unmute dac
        uint8_t dac_ctrl2 = 0x0;
        writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2));

        return 0;
}

int ADAU1860::write_dsp_filter_bank(uint32_t * bank[5], int size) {
}

int ADAU1860::mute(bool active) {
        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));

        // clear last bit
        status &= ~(1 << 6); // 1 << 7 force mute
        if (active) status |= (1 << 6);

        writeReg(registers::DAC_CTRL2, &status, sizeof(status));

        return 0;
}

int ADAU1860::set_volume(uint8_t volume) {
        uint8_t dac_vol = 0xFF-volume;
        writeReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol)); // unmute

        return 0;
}

uint8_t ADAU1860::get_volume() {
        uint8_t dac_vol = 0;
        readReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol));
        return 0xFF-dac_vol;
}

int ADAU1860::soft_reset(bool full_reset) {
        int ret = 0;

        uint8_t status = 0x1;
        if (!full_reset) status <<= 4;
        writeReg(registers::RESETS, &status, sizeof(status));
        return ret;
}

bool ADAU1860::readReg(uint32_t reg, uint8_t * buffer, uint16_t len) {
        int ret;
        //uint64_t now = micros();
        //int delay = MIN(ADAU1860_I2C_TIMEOUT_US - (int)(now - last_i2c), ADAU1860_I2C_TIMEOUT_US);

        //if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        // 32-Bit-Adresse in ein Byte-Array umwandeln (Big-Endian)
        uint8_t addr_buf[4] = {
                (uint8_t)((reg >> 24) & 0xFF),
                (uint8_t)((reg >> 16) & 0xFF),
                (uint8_t)((reg >> 8) & 0xFF),
                (uint8_t)(reg & 0xFF),
        };

        // Adresse senden und Daten lesen
        ret = i2c_write_read(_pWire->master, address, addr_buf, sizeof(addr_buf), buffer, len);
        if (ret) {
                LOG_WRN("I2C read failed: %d\n", ret);
                _pWire->release();
                return ret;
        }
        _pWire->release();
        //last_i2c = micros();

        return true;  // Erfolg

}

// Erstelle einen Puffer für Adresse + Daten
//uint8_t buf[4 + 78 * 4];
uint8_t buf[512];

void ADAU1860::writeReg(uint32_t reg, uint8_t *buffer, uint16_t len) {
        //uint64_t now = micros();
        //int delay = MIN(ADAU1860_I2C_TIMEOUT_US - (int)(now - last_i2c), ADAU1860_I2C_TIMEOUT_US);

        //if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        // 32-Bit-Adresse in ein Byte-Array umwandeln (Big-Endian)
        buf[0] = (uint8_t)((reg >> 24) & 0xFF);
        buf[1] = (uint8_t)((reg >> 16) & 0xFF);
        buf[2] = (uint8_t)((reg >> 8) & 0xFF);
        buf[3] = (uint8_t)(reg & 0xFF);

        // Daten anhängen
        memcpy(&buf[4], buffer, len);

        // Schreibe Adresse und Daten an das I2C-Gerät
        int ret = i2c_write(_pWire->master, buf, len + sizeof(uint32_t), address);
        if (ret) {
                LOG_WRN("I2C write failed: %d\n", ret);
        }

        _pWire->release();

        //last_i2c = micros();

        //return 0;  // Erfolg
}