#include "ADAU1860.h"
#include "LoadSwitch.h"
#include "nrf5340_audio_common.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ADAU1860, 3);

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

        /*uint8_t power_mode = 0x01 | 0x04; // Hibernate 1, Master enable
        writeReg(registers::CHIP_PWR, &power_mode, sizeof(power_mode));

        uint8_t test;

        // Non self-boot
        uint8_t startup_dlycnt_byp = 1;
        writeReg(registers::PMU_CTRL2, &startup_dlycnt_byp, sizeof(startup_dlycnt_byp));

        readReg(registers::PMU_CTRL2, &test, sizeof(test));
        LOG_INF("PMU_CTRL2: 0x%x", test);

        // LOG_INF("available: %i", _pWire->available());

        // Power saving
        uint8_t cm_startup_over = 1 << 4 | power_mode;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        readReg(registers::CHIP_PWR, &test, sizeof(test));
        LOG_INF("CHIP_PWR: 0x%x", test);

        readReg(registers::PLL_PGA_PWR, &test, sizeof(test));
        LOG_INF("PLL_PGA_PWR: 0x%x", test);

        // bypass PLL
        uint8_t clk_ctrl13 = (1 << 7) | (1 << 4) | 0x1; // (0x01 = 49.152 MHz) // | 0x3;
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));
        //k_msleep(10);

        readReg(registers::CLK_CTRL13, (uint8_t*) &test, sizeof(test));
        LOG_INF("CLK_CTRL13: 0x%x", test);
        //_pWire->endTransmission();
        //k_msleep(10);

        // DSP_PWR - reset val

        //setup clock (PLL integer, MCLK source)
        uint8_t clk_ctrl1 = (0x3 << 6); // | 0x4;
        writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));
        //writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));

        readReg(registers::CLK_CTRL1, &test, sizeof(test));
        LOG_INF("CLK_CTRL1: 0x%x", test);

        //uint8_t clk_ctrl2 = 0x0; // = reset val
        readReg(registers::CLK_CTRL2, &test, sizeof(test));
        LOG_INF("CLK_CTRL2: 0x%x", test);

        //uint8_t clk_ctrl3 = 0x4; // = reset val
        readReg(registers::CLK_CTRL3, &test, sizeof(test));
        LOG_INF("CLK_CTRL3: 0x%x", test);

        //DAC PLL multiple of 24.576MHz (=2x12.288)
        //CLK_CTRL2 Prescaler = 1
        //CLK_CTRL3 CLK_CTRL4 R = 4

        // PLL update CLK_CTRL9 ?
        uint8_t clk_ctrl9 = 1;
        writeReg(registers::CLK_CTRL9, &clk_ctrl9, sizeof(clk_ctrl9));
        //writeReg(registers::CLK_CTRL9, &clk_ctrl9, sizeof(clk_ctrl9));

        readReg(registers::CLK_CTRL9, &test, sizeof(test));
        LOG_INF("CLK_CTRL9: 0x%x", test);
        clk_ctrl13 &= ~(1 << 7);
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));
        //writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));

        readReg(registers::CLK_CTRL13, &test, sizeof(test));
        LOG_INF("CLK_CTRL13: 0x%x", test);
        //_pWire->endTransmission();

        // set up CLK_CTRL and PLL
        //uint8_t pll_ctrl = 0x1; // XTAL_EN = 0, PLL_EN = 1 ?
        uint8_t pll_ctrl = 0x3; // XTAL_EN = 1, PLL_EN = 1 ?
        writeReg(registers::PLL_PGA_PWR, &pll_ctrl, sizeof(pll_ctrl));

        readReg(registers::PLL_PGA_PWR, &test, sizeof(test));
        LOG_INF("PLL_PGA_PWR: 0x%x", test);

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));
        LOG_INF("STATUS2: 0x%x", status2);

        /*for (int i = 0; i < 10; i++) {
                readReg(registers::STATUS2, &status2, sizeof(status2));
                LOG_INF("STATUS2: 0x%x", status2);
                k_msleep(10);
        }*/
        //readReg(registers::STATUS2, &status2, sizeof(status2));

        // power up complete and PLL lock (0 or 1)?
        //if ((status2 & (1 << 7)) && (status2 & 1)) return 0;
        //else return -1;

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
        // reset all registers
        // soft_reset(true);

        //k_msleep(35);

        uint8_t power_mode = 0x01 | 0x04; // Hibernate 1, Master enable
        writeReg(registers::CHIP_PWR, &power_mode, sizeof(power_mode));

        //uint8_t test;

        // Non self-boot
        uint8_t startup_dlycnt_byp = 1;
        writeReg(registers::PMU_CTRL2, &startup_dlycnt_byp, sizeof(startup_dlycnt_byp));

        //readReg(registers::PMU_CTRL2, &test, sizeof(test));
        //LOG_INF("PMU_CTRL2: 0x%x", test);

        // LOG_INF("available: %i", _pWire->available());

        // Power saving
        uint8_t cm_startup_over = 1 << 4 | power_mode;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        //readReg(registers::CHIP_PWR, &test, sizeof(test));
        //LOG_INF("CHIP_PWR: 0x%x", test);

        //readReg(registers::PLL_PGA_PWR, &test, sizeof(test));
        //LOG_INF("PLL_PGA_PWR: 0x%x", test);

        // bypass PLL
        uint8_t clk_ctrl13 = (1 << 7) | (1 << 4) | 0x01; // (0x01 = 49.152 MHz) // | 0x3;
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));
        //k_msleep(10);

        //readReg(registers::CLK_CTRL13, (uint8_t*) &test, sizeof(test));
        //LOG_INF("CLK_CTRL13: 0x%x", test);

        // DSP_PWR - reset val

        //setup clock (PLL integer, MCLK source)
        uint8_t clk_ctrl1 = (0x3 << 6); // | 0x4;
        writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));

        // readReg(registers::CLK_CTRL1, &test, sizeof(test));
        // LOG_INF("CLK_CTRL1: 0x%x", test);

        // //uint8_t clk_ctrl2 = 0x0; // = reset val
        // readReg(registers::CLK_CTRL2, &test, sizeof(test));
        // LOG_INF("CLK_CTRL2: 0x%x", test);

        // //uint8_t clk_ctrl3 = 0x4; // = reset val
        // readReg(registers::CLK_CTRL3, &test, sizeof(test));
        // LOG_INF("CLK_CTRL3: 0x%x", test);

        uint8_t clk_ctrl3 = 0x08; // R = 8 with mck setup (10?)
        writeReg(registers::CLK_CTRL3, &clk_ctrl3, sizeof(clk_ctrl3));

        //DAC PLL multiple of 24.576MHz (=2x12.288)
        //CLK_CTRL2 Prescaler = 1
        //CLK_CTRL3 CLK_CTRL4 R = 4

        // PLL update CLK_CTRL9 ?
        uint8_t clk_ctrl9 = 0x01;
        writeReg(registers::CLK_CTRL9, &clk_ctrl9, sizeof(clk_ctrl9));

        //
        // uint8_t clk_ctrl14 = 0x01;
        // writeReg(registers::CLK_CTRL14, &clk_ctrl14, sizeof(clk_ctrl14));

        // uint8_t clk_ctrl15 = (1 << 5); // Bit 13 for DAC
        // writeReg(registers::CLK_CTRL15, &clk_ctrl15, sizeof(clk_ctrl15));

        // 
        clk_ctrl13 &= ~(1 << 7);
        writeReg(registers::CLK_CTRL13, &clk_ctrl13, sizeof(clk_ctrl13));

        // set up CLK_CTRL and PLL
        //uint8_t pll_ctrl = 0x1; // XTAL_EN = 0, PLL_EN = 1 ?
        uint8_t pll_ctrl = 0x3; // XTAL_EN = 1, PLL_EN = 1 ?
        writeReg(registers::PLL_PGA_PWR, &pll_ctrl, sizeof(pll_ctrl));

        k_msleep(1);

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));
        LOG_INF("STATUS2: 0x%x", status2);

        // power up complete and PLL lock
        if (!(status2 & (1 << 7))) LOG_WRN("No power up");
        if (!(status2 & 1)) LOG_WRN("PLL not locking");

        while (!(status2 & (1 << 7)) || !(status2 & 1)) {
                readReg(registers::STATUS2, &status2, sizeof(status2));
                LOG_INF("STATUS2: 0x%x", status2);

                // power up complete and PLL lock
                if (!(status2 & (1 << 7))) LOG_WRN("No power up");
                if (!(status2 & 1)) LOG_WRN("PLL not locking");

                k_usleep(10);
        }

        //LOG_INF("DAC_CTRL: 0x%02X", status);
        // Power saving
        // uint8_t cm_startup_over = 1 << 2 | 0x01; // Master block en | Hibernate 1 (SOC off, ADP on);
        // writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        // DAC_ROUTE0 - EQ 0 / serial port 0 channel 0
        // uint8_t dac_route_eq = 75;
        uint8_t dac_route_i2s = 0;
        writeReg(registers::DAC_ROUTE0, &dac_route_i2s, sizeof(dac_route_i2s));

        // SPT0_CTRL1 - reset val (32 BCLKs?)
        uint8_t spt0_ctrl1 = 0x10; // 1 << 4 (16 BCLKs)
        writeReg(registers::SPT0_CTRL1, &spt0_ctrl1, sizeof(spt0_ctrl1));
        // SPT0_CTRL2 - reset val

        uint8_t sai_clk_pwr = 0x01; // I2S_IN enable
        writeReg(registers::SAI_CLK_PWR, &sai_clk_pwr, sizeof(sai_clk_pwr));

        // SPT0_ROUTE0 - i2s output route
        /*uint8_t spt0_route0 = 39; // DMIC Channel 0
        writeReg(registers::SPT0_ROUTE0, &spt0_route0, sizeof(spt0_route0));*/

        // EQ_CFG - engine running
        // EQ_ROUTE - serial port 0 channel 0

        // set Eq params

        // stop eq and clear
        /*uint8_t eq_cfg = 0x10; // eq clear
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));

        // clear done check
        uint8_t eq_status = 0;
        while (eq_status != 0x1) {
                readReg(registers::EQ_STATUS, &eq_status, sizeof(eq_status));
                k_usleep(100);
        }

        // write parameters

        // activate eq
        eq_cfg = 0x01;
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));*/

        // PDM_VOL0

        // ASRCI_CTRL

        // PB_POWER_CTRL enhanced mode?

        // HPVDD_L ?

        // HP_CTRL - reset val
        // HP_LVMODE_CTRL1

        //Headphone power on
        uint8_t headphone_power = 1 << 4; // DAC/HP channel 0 enabled
        writeReg(registers::ADC_DAC_HP_PWR, &headphone_power, sizeof(headphone_power));

        // unmute dac
        uint8_t dac_ctrl2 = 0x0;
        writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2));

        return 0;
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

        /*uint8_t raw_volume;
        readReg(registers::DAC_VOL0, &raw_volume, sizeof(raw_volume));

        uint8_t mute_state;
        readReg(registers::DAC_CTRL2, &mute_state, sizeof(mute_state));

        LOG_INF("Volume: 0x%X Mute: 0x%x", raw_volume, mute_state);*/

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
uint8_t buf[4 + 8];

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
                return; // ret;
        }

        _pWire->release();

        //last_i2c = micros();

        //return 0;  // Erfolg
}