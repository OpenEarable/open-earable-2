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
bool cd_high=true,usb_power=true,lsctrl_high=true;
static uint8_t regs[256];
static int fail_read=-1,fail_write=-1,drop_write=-1;
int i2c_burst_read(const device *,uint16_t addr,uint8_t reg,uint8_t *data,uint32_t len) {
    if(addr==0x68 || addr==0x69) {
        if(clock_us<10000 || addr!=0x69)return -EIO;
        data[0]=0xd1;return 0;
    }
    if(reg==fail_read)return -EIO;
    std::memcpy(data,regs+reg,len);
    // Hardware observation: a read of 0x07 consumes the adjacent wake flags,
    // even if the master receives only one byte. A two-byte read captures them.
    if (reg <= 8 && reg + len >= 8) { regs[8] &= ~3; regs[0] &= ~0x10; }
    if (reg == 0) regs[0] &= ~0x10;
    return 0;
}
int i2c_burst_write(const device *,uint16_t,uint8_t reg,const uint8_t *data,uint32_t len) {
    if(reg==fail_write)return -EIO;
    if(reg!=drop_write) {
        if(reg==7 && len==1 && (lsctrl_high || (regs[7]&0x80))) {
            // Voltage bits are locked while either enable path is active.
            regs[7]=(data[0]&~0x7c)|(regs[7]&0x7c);
        } else std::memcpy(regs+reg,data,len);
    }
    return 0;
}

static std::mutex signals;
static std::condition_variable cv;
static bool pause_worker=false,worker_at_voltage=false,release_worker=false;
static bool configuration_entered=false,release_configuration=false,block_configuration=false;
static int configuration_result=-EIO,reboots=0;
struct k_work {};
struct k_work_delayable {};
#define K_SECONDS(x) (x)
#define K_NO_WAIT 0
#define CONFIG_BATTERY_CHARGE_CONTROLLER_FAST_INTERVAL_SECONDS 1
#define CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS 10
static bool battery_update_pending=false;
static int shutdown_requests=0,charge_cancellations=0;
static int k_work_submit(k_work *) {battery_update_pending=true;return 0;}
static int k_work_schedule(k_work_delayable *,int) {return 0;}
static int k_work_reschedule(k_work_delayable *,int) {shutdown_requests++;return 0;}
static int k_work_cancel_delayable(k_work_delayable *) {charge_cancellations++;return 0;}
enum charging_state {DISCHARGING,BATTERY_LOW,BATTERY_CRITICAL,POWER_CONNECTED,
                     CHARGING,PRECHARGING,TRICKLE_CHARGING,FULLY_CHARGED,FAULT};
struct battery_data {uint8_t battery_level;enum charging_state charging_state;};
static battery_data msg{},published_msg{};
static int battery_chan,published=0;
static int zbus_chan_pub(int *,battery_data *value,int) {
    // Battery observers may acquire I2C; publication must release the bus first.
    std::thread reader([]{TWIMLock lock(I2C1);});reader.join();
    published_msg=*value;published++;return 0;
}
struct battery_level_status {uint32_t flags=0,power_state=0;};
struct bat_status {bool SYSDWN=false,CHGINH=false,BATTPRES=true,FC=false,FD=false;};
struct gauge_status {bool edv1=false,edv2=false;};
struct PowerManager {
    bool _fuel_gauge_ready=true,power_on=true,charging_disabled=false;
    battery_settings _battery_settings=battery_profile_for_imu(0x69)->settings;
    uint16_t last_charging_state=1;
    charging_state last_charging_msg_state=DISCHARGING;
    int chrg_interval=10;
    static k_work fuel_gauge_work;
    static k_work_delayable charge_ctrl_delayable,power_down_work;
    static void power_good_callback(const device *,gpio_callback *,uint32_t);
    static void fuel_gauge_work_handler(k_work *);
    void charge_task();
    void get_battery_status(battery_level_status &);
    bool check_battery();
    void reboot() {
        // Joining a different bus user detects a lock held across reboot/drain.
        std::thread reader([]{TWIMLock lock(I2C1);});reader.join();reboots++;
    }
} power_manager;
k_work PowerManager::fuel_gauge_work;
k_work_delayable PowerManager::charge_ctrl_delayable,PowerManager::power_down_work;
struct Gauge {
    uint8_t level=70;
    float voltage_value=3.8f, temperature_value=25.0f;
    bat_status status;
    gauge_status gauge;
    float state_of_charge(){return level;}
    float current(){return 100.0f;}
    float charge_current(){return 100.0f;}
    gauge_status gauging_state(){return gauge;}
    float voltage() {
        std::unique_lock<std::mutex> lock(signals);
        if(pause_worker){worker_at_voltage=true;cv.notify_all();cv.wait(lock,[]{return release_worker;});}
        return voltage_value;
    }
    float temperature(){return temperature_value;}
    bat_status battery_status(){return status;}
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
    usb_power=true;cd_high=false;lsctrl_high=true;
    assert(battery_controller.begin()==0 && cd_high);
    power_manager._fuel_gauge_ready=true;
    power_manager.power_on=true;power_manager.charging_disabled=false;
    power_manager.last_charging_state=1;
    power_manager._battery_settings=battery_profile_for_imu(0x69)->settings;
    fuel_gauge.level=70;fuel_gauge.voltage_value=3.8f;fuel_gauge.temperature_value=25.0f;
    fuel_gauge.status={};fuel_gauge.gauge={};
    msg={};published_msg={};published=shutdown_requests=charge_cancellations=0;
    battery_update_pending=false;
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
        battery_controller.enable_charge();assert(cd_high);
        assert(battery_controller.setup(power_manager._battery_settings)==-EIO);
        fail_read=fail_write=drop_write=-1;
        configured();assert(power_manager.check_battery() && !cd_high);
    }
    std::puts("PASS: failed, ignored and unreadable voltage writes inhibit charging until a successful retry");
    for(uint8_t address : {0x68,0x69,0x00}) {
        reset();power_manager._battery_settings=battery_profile_for_imu(address)->settings;
        configured();assert(power_manager.check_battery() && !cd_high);
        assert(std::fabs(battery_controller.read_battery_voltage_control()-power_manager._battery_settings.u_term)<0.005f);
        assert(battery_controller.read_charging_control().mAh==power_manager._battery_settings.i_charge);
    }
    std::puts("PASS: VARTA, Grepow and fallback profiles program/read back correctly");
    for (uint8_t address : {uint8_t(0x68), uint8_t(0x69), uint8_t(0)}) {
        reset();power_manager._battery_settings=battery_profile_for_imu(address)->settings;
        fuel_gauge.voltage_value=2.4f;
        configured();assert(!power_manager.check_battery() && cd_high);
        power_manager.last_charging_state=0;power_manager.charge_task();
        assert(!cd_high && battery_controller.configured());
        assert(std::fabs(battery_controller.read_battery_voltage_control()-power_manager._battery_settings.u_term)<0.005f);
    }
    reset();configured();fuel_gauge.temperature_value=46.0f;
    assert(!power_manager.check_battery() && cd_high);
    power_manager.last_charging_state=0;power_manager.charge_task();
    assert(!cd_high); // Pre-2.3.3 setup did not recheck temperature.
    reset();configured();fuel_gauge.voltage_value=2.4f;
    power_manager.last_charging_state=0;drop_write=5;regs[5]=0x8c;
    power_manager.charge_task();assert(cd_high && !battery_controller.configured());
    std::puts("PASS: legacy recovery re-enables charging after verified setup; profile voltages and failed-setup inhibition remain intact");
    for (uint8_t initial : {uint8_t(0x7c),uint8_t(0xfd),uint8_t(0x64)}) {
        reset();regs[7]=initial;
        assert(battery_controller.configure_3v3_at_boot()==0);
        assert(!lsctrl_high && regs[7]==(0x64 | (initial & 1)));
        lsctrl_high=true;configured();
        assert(lsctrl_high && regs[7]==(0x64 | (initial & 1)));
    }
    std::puts("PASS: startup disables both enable paths, programs 3.3 V and retains it during charging setup");
    for (uint8_t flags : {1, 2, 3}) {
        reset(); regs[7] = 0x7c; regs[8] = 0x68 | flags;
        assert(battery_controller.configure_3v3_at_boot() == 0);
        assert((regs[8] & 3) == 0);
        const button_state wake = battery_controller.read_button_state();
        assert(wake.wake_2 == bool(flags & 1) && wake.wake_1 == bool(flags & 2));
        const button_state consumed = battery_controller.read_button_state();
        assert(!consumed.wake_1 && !consumed.wake_2);
    }
    reset(); regs[7] = 0x64; regs[8] = 0x69;
    (void)battery_controller.read_ldo_voltage();
    regs[8] = 0x6a;
    const button_state combined = battery_controller.read_button_state();
    assert(combined.wake_1 && combined.wake_2);
    std::puts("PASS: LDO reads preserve button events once, including later live flags");
    for (int mode=0;mode<3;mode++) {
        reset();regs[7]=0x7c;
        if(mode==0)fail_write=7;
        if(mode==1)drop_write=7;
        if(mode==2)fail_read=7;
        assert(battery_controller.configure_3v3_at_boot()==-EIO && !lsctrl_high);
    }
    reset();regs[7]=0xfc;drop_write=7;
    assert(battery_controller.configure_3v3_at_boot()==-EIO && !lsctrl_high);
    std::puts("PASS: failed, ignored and unreadable 3.3 V writes stop startup with LSCTRL low");
    reset();configured();assert(power_manager.check_battery() && !cd_high);
    drop_write=3;assert(battery_controller.set_charge_current_verified(40)==-EIO && cd_high);
    drop_write=-1;power_manager.charge_task();
    assert(battery_controller.configured() && !cd_high && battery_update_pending);
    reset();configured();assert(power_manager.check_battery());fail_read=0;
    battery_controller.read_charging_state();
    assert(battery_controller.configured() && !cd_high);
    fail_read=-1;assert(power_manager.check_battery() && !cd_high);
    std::puts("PASS: current verification retries after mismatch; ordinary read errors do not latch charging off");

    reset();configured();regs[0]=0x40;
    PowerManager::fuel_gauge_work_handler(nullptr);
    assert(published_msg.charging_state==CHARGING && published_msg.battery_level==70);
    fail_read=0;power_manager.charge_task();
    assert(battery_update_pending);
    fail_read=-1;usb_power=false; // Leave the old hardware status at charging.
    PowerManager::power_good_callback(nullptr,nullptr,0);
    PowerManager::fuel_gauge_work_handler(nullptr);
    assert(charge_cancellations==1 && published_msg.charging_state==DISCHARGING);
    usb_power=true;regs[0]=0x40;
    PowerManager::power_good_callback(nullptr,nullptr,0);
    power_manager.charge_task();PowerManager::fuel_gauge_work_handler(nullptr);
    assert(battery_controller.configured() && !cd_high && published_msg.charging_state==CHARGING);
    std::puts("PASS: unplug publishes discharging despite a stale charger register; reconnect resumes after a transient error");

    // A configuration failure must not suppress independent gauge monitoring.
    reset();configured();regs[0]=0x40;
    PowerManager::fuel_gauge_work_handler(nullptr);
    drop_write=3;assert(battery_controller.set_charge_current_verified(40)==-EIO);
    // Make the periodic retry fail too, and verify that it still queues a report.
    regs[5]=0x8c;drop_write=5;battery_update_pending=false;
    power_manager.charge_task();assert(battery_update_pending && cd_high);
    PowerManager::fuel_gauge_work_handler(nullptr);
    assert(published_msg.charging_state==FAULT && published==2);
    usb_power=false;fuel_gauge.level=5;fuel_gauge.status.SYSDWN=true;fuel_gauge.gauge.edv1=true;
    PowerManager::power_good_callback(nullptr,nullptr,0);
    PowerManager::fuel_gauge_work_handler(nullptr);
    assert(published_msg.battery_level==5 && published_msg.charging_state==BATTERY_CRITICAL);
    assert(shutdown_requests==1 && published==3);
    drop_write=-1;usb_power=true;fuel_gauge.status.SYSDWN=false;fuel_gauge.gauge.edv1=false;
    PowerManager::power_good_callback(nullptr,nullptr,0);
    power_manager.charge_task();PowerManager::fuel_gauge_work_handler(nullptr);
    assert(battery_controller.configured() && !cd_high && published_msg.charging_state==CHARGING);
    std::puts("PASS: failed charger retries still publish battery state and request low-battery shutdown; recovery needs no reboot");
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
