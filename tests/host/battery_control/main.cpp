#include <cassert>
#include <cerrno>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <new>
#include "BQ25120a.h"
#include "battery_profile.h"
#include <zephyr/logging/log.h>

const device fake_device{};
std::atomic<uint64_t> clock_us{0};
bool cd_high=true,usb_power=true;
static uint8_t regs[256];
static int fail_read=-1,fail_write=-1,drop_write=-1;
int i2c_burst_read(const device *,uint16_t addr,uint8_t reg,uint8_t *data,uint32_t len) {
    if(addr==0x68 || addr==0x69) {
        if(clock_us<10000 || addr!=0x69)return -EIO;
        data[0]=0xd1;return 0;
    }
    if(reg==fail_read)return -EIO;
    std::memcpy(data,regs+reg,len);return 0;
}
int i2c_burst_write(const device *,uint16_t,uint8_t reg,const uint8_t *data,uint32_t len) {
    if(reg==fail_write)return -EIO;
    if(reg!=drop_write)std::memcpy(regs+reg,data,len);
    return 0;
}

static std::mutex signals;
static std::condition_variable cv;
static bool pause_worker=false,worker_at_voltage=false,release_worker=false;
static bool configuration_entered=false,release_configuration=false,block_configuration=false;
static int configuration_result=-EIO,reboots=0;
struct bat_status {bool SYSDWN=false;};
struct PowerManager {
    bool _fuel_gauge_ready=true;
    battery_settings _battery_settings=battery_profile_for_imu(0x69)->settings;
    bool check_battery();
    void reboot() {
        // Joining a different bus user detects a lock held across reboot/drain.
        std::thread reader([]{TWIMLock lock(I2C1);});reader.join();reboots++;
    }
} power_manager;
struct Gauge {
    float voltage() {
        std::unique_lock<std::mutex> lock(signals);
        if(pause_worker){worker_at_voltage=true;cv.notify_all();cv.wait(lock,[]{return release_worker;});}
        return 3.8f;
    }
    float temperature(){return 25.0f;}
    bat_status battery_status(){return {};}
    int setup(const battery_settings &) {
        assert(cd_high && !power_manager._fuel_gauge_ready);
        std::unique_lock<std::mutex> lock(signals);
        configuration_entered=true;cv.notify_all();
        if(block_configuration)cv.wait(lock,[]{return release_configuration;});
        return configuration_result;
    }
} fuel_gauge;
struct shell {};
static void shell_error(const shell *,const char *,...) {}
static constexpr uint8_t BMI160_CHIP_ID_ADDR=0,BMI160_CHIP_ID=0xd1,BMX160_CHIP_ID=0xd8;
static constexpr uint8_t BMI160_I2C_ADDR_SDO_LOW=0x68,BMI160_I2C_ADDR_SDO_HIGH=0x69;
struct BMX160 {
    TWIM *_i2c=&I2C3;
    uint8_t _addr=0;
    bool _standalone_bmi160=false;
    bool probeAddress(uint8_t,uint8_t *);
    bool detect();
};
#include "production_functions.inc"

static void reset() {
    battery_controller.~BQ25120a();new(&battery_controller)BQ25120a(&I2C1);
    fail_read=fail_write=drop_write=-1;std::memset(regs,0,sizeof(regs));
    regs[5]=0x8c; // Simulate a retained 4.30 V charger configuration.
    usb_power=true;cd_high=false;
    assert(battery_controller.begin()==0 && cd_high);
    power_manager._fuel_gauge_ready=true;
    power_manager._battery_settings=battery_profile_for_imu(0x69)->settings;
}
static void configured() {
    assert(battery_controller.setup(power_manager._battery_settings)==0);
    assert(battery_controller.configured() && cd_high);
}
int main() {
    reset();battery_controller.enable_charge();assert(cd_high);
    for(int mode=0;mode<3;mode++) {
        reset();
        if(mode==0)fail_write=5;
        if(mode==1)drop_write=5;
        if(mode==2)fail_read=5;
        assert(battery_controller.setup(power_manager._battery_settings)==-EIO);
        assert(!power_manager.check_battery() && cd_high);
        fail_read=fail_write=drop_write=-1;
        assert(battery_controller.setup(power_manager._battery_settings)==-EIO);
        battery_controller.enable_charge();assert(cd_high);
    }
    std::puts("PASS: failed, ignored and unreadable voltage writes latch charging off");
    for(uint8_t address : {0x68,0x69,0x00}) {
        reset();power_manager._battery_settings=battery_profile_for_imu(address)->settings;
        configured();assert(power_manager.check_battery() && !cd_high);
        assert(std::fabs(battery_controller.read_battery_voltage_control()-power_manager._battery_settings.u_term)<0.005f);
        assert(battery_controller.read_charging_control().mAh==power_manager._battery_settings.i_charge);
    }
    std::puts("PASS: VARTA, Grepow and fallback profiles program/read back correctly");
    reset();regs[7]=0x7c;drop_write=7; // LSCTRL holds the LDO output enabled.
    configured();assert(power_manager.check_battery() && !cd_high);
    assert(regs[7]==0x7c);
    std::puts("PASS: an enabled LDO with a locked voltage field does not block verified charging");
    reset();configured();assert(power_manager.check_battery() && !cd_high);
    drop_write=3;assert(battery_controller.set_charge_current_verified(40)==-EIO && cd_high);
    reset();configured();assert(power_manager.check_battery());fail_read=0;
    battery_controller.read_charging_state();battery_controller.enable_charge();assert(cd_high);
    std::puts("PASS: current readback mismatch and runtime I/O failure inhibit charging");
    reset();configured();pause_worker=true;block_configuration=true;
    std::thread old_worker([]{assert(power_manager.check_battery());});
    {std::unique_lock<std::mutex> lock(signals);assert(cv.wait_for(lock,std::chrono::seconds(2),[]{return worker_at_voltage;}));}
    std::atomic<bool> command_started{false};
    std::thread command([&]{command_started=true;assert(cmd_setup_fuel_gauge(nullptr,0,nullptr)==-EIO);});
    while(!command_started)std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    {std::lock_guard<std::mutex> lock(signals);assert(!configuration_entered);release_worker=true;cv.notify_all();}
    {std::unique_lock<std::mutex> lock(signals);assert(cv.wait_for(lock,std::chrono::seconds(2),[]{return configuration_entered;}));}
    std::atomic<bool> new_finished{false};
    std::thread new_worker([&]{assert(!power_manager.check_battery());new_finished=true;});
    std::this_thread::sleep_for(std::chrono::milliseconds(30));assert(!new_finished);
    {std::lock_guard<std::mutex> lock(signals);release_configuration=true;cv.notify_all();}
    old_worker.join();command.join();new_worker.join();
    assert(!power_manager._fuel_gauge_ready && cd_high);
    std::puts("PASS: manual configuration waits for in-flight work and blocks new work");
    pause_worker=false;block_configuration=false;configuration_result=0;
    assert(cmd_setup_fuel_gauge(nullptr,0,nullptr)==0 && reboots==1);
    std::puts("PASS: reboot releases the bus before draining other workers");
    clock_us=300;BMX160 imu;assert(imu.detect());
    assert(clock_us>=10300 && imu._addr==0x69 && imu._standalone_bmi160);
    std::puts("PASS: a sensor requiring 10 ms POR is identified after rail startup");
}
