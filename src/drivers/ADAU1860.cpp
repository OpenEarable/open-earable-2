#include "ADAU1860.h"
#include "zbus_common.h"
#include "openearable_common.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ADAU1860, 3);

/*
extern uint32_t eq_program[];
extern uint32_t eq_param_bank0[];
extern uint32_t eq_param_bank1[];*/

#include "Lark-eq.c"
#include "Lark-fdsp.c"

ADAU1860 dac(&I2C2);

static struct k_work_delayable ascr_lock_work;

void ADAU1860::check_ascr_lock(struct k_work *work)
{
    uint8_t status2;
    dac.readReg(registers::STATUS2, &status2, sizeof(status2));

    if ((status2 & (1 << 7)) && (status2 & 4)) {
        LOG_INF("ASCR locked, proceeding...");
        
        // Unmute DAC
        uint8_t dac_ctrl2 = 0x0;
        dac.writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2));
    } else {
        LOG_WRN("Waiting for ASCR to lock... STATUS2: 0x%x", status2);
        k_work_reschedule(&ascr_lock_work, K_USEC(10));
    }
}

ADAU1860::ADAU1860(TWIM * i2c) : _i2c(i2c) {

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

        _i2c->begin();

        //k_msleep(1);

        //last_i2c = micros();

        /*uint8_t status = 0x1;
        writeReg(registers::RESETS, &status, sizeof(status));*/

        k_msleep(35); // CM rise time (see datasheet)

        uint8_t power_mode = 0x01 | 0x04; // Hibernate 1, Master enable
        writeReg(registers::CHIP_PWR, &power_mode, sizeof(power_mode));

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

        //uint8_t asrc_pwr = 0x1; // ASRCI0_EN 
        //writeReg(registers::ASRC_PWR, &asrc_pwr, sizeof(asrc_pwr));

        //k_msleep(1);

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));
        LOG_INF("STATUS2: 0x%x", status2);

        if (!(status2 & (1 << 7))) LOG_WRN("No power up");
        if (!(status2 & (1 << 1))) LOG_WRN("FM not ready");

        while (!(status2 & (1 << 7)) || !(status2 & (1 << 1))) {
                readReg(registers::STATUS2, &status2, sizeof(status2));
                LOG_INF("STATUS2: 0x%x", status2);

                // power up complete and PLL lock
                if (!(status2 & (1 << 7))) LOG_WRN("No power up");
                if (!(status2 & (1 << 1))) LOG_WRN("FM not ready");

                k_usleep(10);
        }

        //LOG_INF("DAC_CTRL: 0x%02X", status);
        // Power saving
        // uint8_t cm_startup_over = 1 << 2 | 0x01; // Master block en | Hibernate 1 (SOC off, ADP on);
        // writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        // SPT0_CTRL1 - reset val (32 BCLKs?)
        uint8_t spt0_ctrl1 = 0x10; // 1 << 4 (16 BCLKs)
        writeReg(registers::SPT0_CTRL1, &spt0_ctrl1, sizeof(spt0_ctrl1));
        // SPT0_CTRL2 - reset val

        // ASRCI_CTRL
        //uint8_t asrci_route01 = 0x0; // ASRCI0_EN 
        //writeReg(registers::ASRCI_ROUTE01, &asrci_route01, sizeof(asrci_route01));

        if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL) || (CONFIG_AUDIO_DEV == HEADSET)) {
                // I2S_IN enable | I2S_OUT enable | MIC enable
                uint8_t sai_clk_pwr = 0x01 | (1 << 1) | (1 << 4);
                writeReg(registers::SAI_CLK_PWR, &sai_clk_pwr, sizeof(sai_clk_pwr));

                uint8_t asrc_pwr = 0x41; // ASRCO0_EN | ASRCI0_EN 
                writeReg(registers::ASRC_PWR, &asrc_pwr, sizeof(asrc_pwr));

                // DMIC_CTRL - reset val (32 BCLKs?)
                uint8_t dmic_pwr = 0x03; // DMIC Channel 0 & 1
                writeReg(registers::DMIC_PWR, &dmic_pwr, sizeof(dmic_pwr));

                // SPT0_ROUTE0 - i2s output route
                /*uint8_t spt0_route0 = 39; // DMIC Channel 0
                writeReg(registers::SPT0_ROUTE0, &spt0_route0, sizeof(spt0_route0));

                uint8_t spt0_route1 = 40; // DMIC Channel 1
                writeReg(registers::SPT0_ROUTE1, &spt0_route1, sizeof(spt0_route1));*/

                uint8_t ascro0_route = 35; // DMIC Channel 0
                writeReg(registers::ASRCO_ROUTE0, &ascro0_route, sizeof(ascro0_route));

                uint8_t ascro1_route = 36; // DMIC Channel 1
                writeReg(registers::ASRCO_ROUTE1, &ascro1_route, sizeof(ascro1_route));

                uint8_t spt0_route0 = 32; // ASCRO 0
                writeReg(registers::SPT0_ROUTE0, &spt0_route0, sizeof(spt0_route0));

                uint8_t spt0_route1 = 33; // ASCRO 1
                writeReg(registers::SPT0_ROUTE1, &spt0_route1, sizeof(spt0_route1));

                // DMIC_VOL0
                uint8_t dmic_vol = 0x10;
                writeReg(registers::DMIC_VOL0, &dmic_vol, sizeof(dmic_vol));
                writeReg(registers::DMIC_VOL1, &dmic_vol, sizeof(dmic_vol));

                uint8_t dmic_ctrl = 0x02; // DMIC Channel 0
                writeReg(registers::DMIC_CTRL2, &dmic_ctrl, sizeof(dmic_ctrl));
        } else {
                uint8_t asrc_pwr = 0x1; // ASRCI0_EN 
                writeReg(registers::ASRC_PWR, &asrc_pwr, sizeof(asrc_pwr));

                // I2S_IN enable
                uint8_t sai_clk_pwr = 0x01;
                writeReg(registers::SAI_CLK_PWR, &sai_clk_pwr, sizeof(sai_clk_pwr));
        }

        uint8_t dac_route = DAC_ROUTE_I2S;

#if CONFIG_EQAULIZER_DSP
        setup_EQ();
        dac_route = DAC_ROUTE_EQ;

#if CONFIG_FDSP
        setup_FDSP();
        dac_route = DAC_ROUTE_DSP_CH(0);
#endif
#endif
        writeReg(registers::DAC_ROUTE0, &dac_route, sizeof(dac_route));

        setup_DAC();

        return 0;
}

int ADAU1860::setup_DAC() {
        //Headphone power on
        uint8_t headphone_power = 0x10; // DAC/HP channel 0 enabled
        writeReg(registers::ADC_DAC_HP_PWR, &headphone_power, sizeof(headphone_power));

        // low voltage 
        uint8_t lvmode = 0x03; //HP_LVMODE_EN | HP_LVMODE_CM_EN
        writeReg(registers::HP_LVMODE_CTRL1, &lvmode, sizeof(lvmode));

        uint8_t lvmode_ctrl2 = 0x31;
        writeReg(registers::HP_LVMODE_CTRL2, &lvmode_ctrl2, sizeof(lvmode_ctrl2));

        uint8_t lvmode_ctrl3 = 0x01;
        writeReg(registers::HP_LVMODE_CTRL3, &lvmode_ctrl3, sizeof(lvmode_ctrl3));

        uint8_t hpldo_ctrl = 0x01;
        writeReg(registers::HPLDO_CTRL, &hpldo_ctrl, sizeof(hpldo_ctrl));

        // DAC_NOISE_CTRL1&2
        uint8_t dac_noise_1 = 0x10;
        uint8_t dac_noise_2 = 0x02;
        writeReg(registers::DAC_NOISE_CTRL1, &dac_noise_1, sizeof(dac_noise_1));
        writeReg(registers::DAC_NOISE_CTRL2, &dac_noise_2, sizeof(dac_noise_2));

        // high performance
        uint8_t pb_ctrl = 0x02;
        writeReg(registers::PB_CTRL, &pb_ctrl, sizeof(pb_ctrl));

#if CONFIG_FDSP
        // Unmute DAC
        uint8_t dac_ctrl2 = 0x0;
        dac.writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2));
#endif

        return 0;
}

int ADAU1860::setup_EQ() {
        // EQ_CFG - engine running
        // stop EQ
        uint8_t eq_cfg = 0x00;
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));

        // clear EQ
        eq_cfg = 0x10;
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

        return 0;
}

int ADAU1860::setup_FDSP() {
        uint8_t dsp_pwr = 0x1;
        writeReg(registers::DSP_PWR, &dsp_pwr, sizeof(dsp_pwr));

        writeReg(FDSP_PROG_MEM, (uint8_t*) fdsp_program, sizeof(fdsp_program));

        for (int i = 0; i < FDSP_NUM_PARAMS; i++) {
                writeReg(FDSP_BANK_A(i), (uint8_t*) fdsp_param_bank_a[i], sizeof(fdsp_param_bank_a[i]));
                writeReg(FDSP_BANK_B(i), (uint8_t*) fdsp_param_bank_b[i], sizeof(fdsp_param_bank_b[i]));
                writeReg(FDSP_BANK_C(i), (uint8_t*) fdsp_param_bank_c[i], sizeof(fdsp_param_bank_c[i]));
        }

        uint8_t fdsp_ctrl4 = 2; // framrate source DMIC01
        writeReg(registers::FDSP_CTRL4, &fdsp_ctrl4, sizeof(fdsp_ctrl4));

        // run dsp
        uint8_t fdsp_run = 0x1;
        writeReg(registers::FDSP_RUN, &fdsp_run, sizeof(fdsp_run));

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

#if !CONFIG_FDSP
        // Unmute DAC (no need to wait for the ASCRs to lock)
        uint8_t dac_ctrl2 = 0x0;
        dac.writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2));
#endif

        return 0;
}

int ADAU1860::mute(bool active) {
#if CONFIG_FDSP
        // Unmute DAC
        int ret;
        ret = fdsp_mute(active);
        return ret;
#else
        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));

        // clear last bit
        status &= ~(1 << 6); // 1 << 7 force mute
        if (active) status |= (1 << 6);

        writeReg(registers::DAC_CTRL2, &status, sizeof(status));

        return 0;
#endif
}

int ADAU1860::fdsp_safe_load(uint8_t address, safe_load_params params) {
        writeReg(registers::FDSP_SL_ADDR, &address, sizeof(address));
        writeReg(registers::FDSP_SL_P0_0, (uint8_t *) params, sizeof(safe_load_params));

        uint8_t val = 1;
        writeReg(registers::FDSP_SL_UPDATE, &val, sizeof(val));
        
        return 0;
}

int ADAU1860::fdsp_safe_load(uint8_t address, int n, uint32_t param) {
        uint32_t params[FDSP_NUM_PARAMS];

        for (int i = 0; i < FDSP_NUM_PARAMS; i++) {
                params[i] = fdsp_param_bank_a[i][address];
        }

        params[n] = param;

        writeReg(registers::FDSP_SL_ADDR, &address, sizeof(address));
        //writeReg(registers::FDSP_SL_P0_0 + n * sizeof(param), (uint8_t *) &param, sizeof(param));
        writeReg(registers::FDSP_SL_P0_0, (uint8_t *) params, sizeof(safe_load_params));

        uint8_t val = 1;
        writeReg(registers::FDSP_SL_UPDATE, &val, sizeof(val));
        
        return 0;
}

int ADAU1860::fdsp_mute(bool active) {
        uint32_t val = active ? 0x00000000 : 0x08000000;
        fdsp_safe_load(1, 4, val);

        return 0;
}

int ADAU1860::set_volume(uint8_t volume) {
        uint8_t dac_vol = 0xFF-volume;
        writeReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol)); // unmute

        return 0;
}


int ADAU1860::fdsp_set_volume(uint8_t volume) {
        uint8_t dac_vol = 0xFF-volume;
        writeReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol)); // unmute

        return 0;
}

uint8_t ADAU1860::get_volume() {
        uint8_t dac_vol = 0;
        readReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol));
        return 0xFF-dac_vol;
}

uint8_t ADAU1860::fdsp_get_volume() {
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
        struct i2c_msg msg[2];

        // 32-Bit-Adresse in ein Byte-Array umwandeln (Big-Endian)
        uint8_t addr_buf[4] = {
                (uint8_t)((reg >> 24) & 0xFF),
                (uint8_t)((reg >> 16) & 0xFF),
                (uint8_t)((reg >> 8) & 0xFF),
                (uint8_t)(reg & 0xFF),
        };

	msg[0].buf = addr_buf;
	msg[0].len = sizeof(addr_buf);
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = buffer;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

        _i2c->aquire();

        // Adresse senden und Daten lesen
        ret = i2c_transfer(_i2c->master, msg, 2, address);
        if (ret) {
                LOG_WRN("I2C read failed: %d\n", ret);
        }
        _i2c->release();

        return (ret == 0);

}

void ADAU1860::writeReg(uint32_t reg, uint8_t *buffer, uint16_t len) {
        int ret;
        struct i2c_msg msg[2];

        uint8_t addr_buf[4] = {
                (uint8_t)((reg >> 24) & 0xFF),
                (uint8_t)((reg >> 16) & 0xFF),
                (uint8_t)((reg >> 8) & 0xFF),
                (uint8_t)(reg & 0xFF),
        };

	msg[0].buf = addr_buf;
	msg[0].len = sizeof(addr_buf);
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = buffer;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        _i2c->aquire();

	ret = i2c_transfer(_i2c->master, msg, 2, address);
        if (ret) {
                LOG_WRN("I2C write failed: %d\n", ret);
        }

        _i2c->release();
}

void ADAU1860::writeReg_u8(uint32_t reg, uint8_t &buffer) {
        writeReg(reg, &buffer, sizeof(buffer));
}