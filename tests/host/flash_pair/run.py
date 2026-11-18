#!/usr/bin/env python3
"""Run pair provisioning against mock probes; never access physical hardware."""
from pathlib import Path
import json,os,subprocess,tempfile

ROOT=Path(__file__).resolve().parents[3]
MOCK=r'''#!/usr/bin/env python3
import json,os,sys
from pathlib import Path
args=sys.argv[1:];root=Path(os.environ['MOCK_PROBES'])
if '--ids' in args:
    print('987650001\n987650002');raise SystemExit(0)
snr=args[args.index('--snr')+1];p=root/(snr+'.json');d=json.loads(p.read_text())
d['calls'].append(args)
if '--memrd' in args:
    addr=args[args.index('--memrd')+1]
    if addr=='0x00FF0204':words=[int(snr[-1]),0x12345678]
    elif addr=='0x00FF8100':words=[d['hardware']]
    elif addr=='0x00FF80F4':words=[d['memory'][str(a)] for a in [0xff80f4,0xff80f8,0xff80fc,0xff8100]]
    else:raise RuntimeError(addr)
    print(addr+': '+' '.join(f'{v:08X}' for v in words))
elif '--readuicr' in args:Path(args[args.index('--readuicr')+1]).write_text('mock UICR backup\n')
elif '--program' in args and 'CP_APPLICATION' in args:d['memory']={str(0xff80fc):0xffffffff}
elif '--memwr' in args:d['memory'][str(int(args[args.index('--memwr')+1],0))]=int(args[args.index('--val')+1],0)
p.write_text(json.dumps(d))
'''

def run_case(name,hardware,option,expected=None):
    with tempfile.TemporaryDirectory(prefix='oe-pair-') as tmp:
        temp=Path(tmp);build=temp/'images';build.mkdir();bin_dir=temp/'bin';bin_dir.mkdir()
        for core in ['cpuapp','cpunet']:(build/f'merged_openearable_v2_nrf5340_{core}.hex').write_text('mock image\n')
        mock=bin_dir/'nrfjprog';mock.write_text(MOCK);mock.chmod(0o755)
        sleep=bin_dir/'sleep';sleep.write_text('#!/bin/sh\nexit 0\n');sleep.chmod(0o755)
        serials=['987650001','987650002']
        for snr,hw in zip(serials,hardware):(temp/(snr+'.json')).write_text(json.dumps({'hardware':hw,'calls':[],'memory':{}}))
        env=os.environ.copy();env.update(MOCK_PROBES=tmp,NRFJPROG=str(mock),TMPDIR=tmp,PATH=str(bin_dir)+os.pathsep+env['PATH'])
        result=subprocess.run(['bash',str(ROOT/'tools/flash/flash_pair.sh'),'--left-snr',serials[0],'--right-snr',serials[1],'--build-dir',str(build),'--hw',option],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=30)
        probes=[json.loads((temp/(snr+'.json')).read_text()) for snr in serials]
        if expected is None:
            assert result.returncode!=0,result.stdout
            assert all(not any('--program' in args for args in p['calls']) for p in probes),result.stdout
        else:
            assert result.returncode==0,result.stdout
            assert [p['memory'][str(0xff8100)] for p in probes]==expected,result.stdout
            assert [p['memory'][str(0xff80f4)] for p in probes]==[0,1]
            assert probes[0]['memory'][str(0xff80f8)]==probes[1]['memory'][str(0xff80f8)]
        print('PASS:',name)

run_case('auto preserves a mixed 2.1.0 / 2.0.1 pair',[0x02010000,0x02000100],'auto',[0x02010000,0x02000100])
run_case('blank boards retain the established 2.0.1 default',[0xffffffff,0xffffffff],'auto',[0x02000100,0x02000100])
run_case('invalid metadata stops both probes before erasing',[0xFA50FA50,0x02000100],'auto')
run_case('explicit hardware option retains its previous behavior',[0xffffffff,0xffffffff],'2.1.0',[0x02010000,0x02010000])
