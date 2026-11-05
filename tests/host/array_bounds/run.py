#!/usr/bin/env python3
"""ASan/UBSan regressions of production PPG and Bluetooth identity code."""
from pathlib import Path
import os,subprocess,tempfile
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def function(path,signature):
    source=(ROOT/path).read_text();start=source.index(signature);brace=source.index('{',start);i=brace+1;depth=1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[start:i]
with tempfile.TemporaryDirectory(prefix='oe-array-bounds-') as temp:
    temp=Path(temp)
    flags=[os.environ.get('CXX','c++'),'-std=c++17','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-pthread']
    stubs=ROOT/'tests/host/battery_control/stubs'
    subprocess.run(flags+['-I'+str(stubs),'-I'+str(ROOT/'src/Wire'),'-I'+str(ROOT/'src/SensorManager/MAXM86161'),str(ROOT/'src/SensorManager/MAXM86161/MAXM86161.cpp'),str(ROOT/'src/Wire/TWIM.cpp'),str(HERE/'ppg.cpp'),'-o',str(temp/'ppg')],check=True)
    subprocess.run([str(temp/'ppg')],check=True,timeout=15)
    (temp/'identity.inc').write_text(function('src/bluetooth/bt_management/bt_mgmt.c','static int local_identity_addr_print('))
    for capacity in [1,2]:
        subprocess.run(flags+['-DCONFIG_BT_ID_MAX='+str(capacity),'-I'+str(temp),str(HERE/'bluetooth.cpp'),'-o',str(temp/'bluetooth')],check=True)
        subprocess.run([str(temp/'bluetooth')],check=True,timeout=15)
