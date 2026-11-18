#!/usr/bin/env python3
"""Host fault/interleaving tests of production driver and control functions.

Only board/RTOS/I2C boundaries are simulated. Compile the real charger and bus
driver; extract control/probe bodies to avoid linking the unrelated audio stack.
"""
from pathlib import Path
import os, subprocess, tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

def function(path, signature):
    source=(ROOT/path).read_text()
    start=source.index(signature);brace=source.index('{',start);depth=1;i=brace+1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[start:i]

with tempfile.TemporaryDirectory(prefix='oe-battery-control-') as temp:
    temp=Path(temp)
    (temp/'production_functions.inc').write_text('\n'.join([
        function('src/Battery/PowerManager.cpp','void PowerManager::configure_charger_for_startup()'),
        function('src/Battery/PowerManager.cpp','bool PowerManager::check_battery()'),
        function('src/Battery/PowerManager.cpp','void PowerManager::power_good_callback('),
        function('src/Battery/PowerManager.cpp','void PowerManager::fuel_gauge_work_handler('),
        function('src/Battery/PowerManager.cpp','void PowerManager::charge_task()'),
        function('src/Battery/PowerManager.cpp','void PowerManager::get_battery_status('),
        function('src/Battery/PowerManager.cpp','int cmd_setup_fuel_gauge('),
        function('src/SensorManager/BMX160/BMX160.cpp','bool BMX160::probeAddress('),
        function('src/SensorManager/BMX160/BMX160.cpp','bool BMX160::detect()'),
    ]))
    binary=temp/'battery_control'
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-pthread','-Wall','-Wextra',
        '-I'+str(HERE/'stubs'),'-I'+str(temp),'-I'+str(ROOT/'src/Wire'),
        '-I'+str(ROOT/'src/Battery'),'-I'+str(ROOT/'include'),
        str(ROOT/'src/Battery/BQ25120a.cpp'),str(ROOT/'src/Wire/TWIM.cpp'),
        '-x','c++',str(ROOT/'src/Battery/battery_profile.c'),str(HERE/'main.cpp'),
        '-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True,timeout=15)
    (temp/'bootloader_functions.inc').write_text('\n'.join([
        function('boards/teco/openearable_v2/mcuboot_hook.c', 'static int read_charger_ldo('),
        function('boards/teco/openearable_v2/mcuboot_hook.c', 'static void charger_watchdog_handler('),
        function('boards/teco/openearable_v2/mcuboot_hook.c', 'static int init_3v3_regulator('),
    ]))
    bootloader_binary=temp/'bootloader_3v3'
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra',
        '-I'+str(temp),'-I'+str(ROOT/'boards/teco/openearable_v2'),str(HERE/'bootloader.cpp'),'-o',str(bootloader_binary)],check=True)
    subprocess.run([str(bootloader_binary)],check=True,timeout=15)
