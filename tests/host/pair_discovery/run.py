#!/usr/bin/env python3
"""Exercise production advertising, peer selection and pair storage with host fakes."""
from pathlib import Path
import os
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
UNICAST = ROOT / 'src/bluetooth/bt_stream/unicast'


def function(path, signature):
    source = path.read_text()
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


with tempfile.TemporaryDirectory(prefix='oe-pair-discovery-') as temporary:
    temporary = Path(temporary)
    storage = (ROOT / 'src/utils/earable_pairing.c').read_text()
    discovery = (UNICAST / 'earable_discovery.c').read_text()
    (temporary / 'production.inc').write_text('\n'.join([
        storage[storage.index('#define PAIR_SETTING'):],
        function(UNICAST / 'unicast_server.c', 'int unicast_server_adv_populate('),
        discovery[discovery.index('struct peer {'):discovery.index('K_MSGQ_DEFINE')],
        function(UNICAST / 'earable_discovery.c', 'static bool parse_peer('),
        function(UNICAST / 'earable_discovery.c', 'static void device_found('),
    ]))
    binary = temporary / 'pair_discovery'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', '-I' + str(temporary), '-I' + str(UNICAST),
                    '-I' + str(ROOT / 'src/utils'), str(HERE / 'main.c'), '-o', str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
