#!/usr/bin/env python3
"""Exercise production advertisement construction and discovery with host fakes."""
from pathlib import Path
import os
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def function(path, signature):
    source = (ROOT / path).read_text()
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


with tempfile.TemporaryDirectory(prefix='oe-pair-discovery-') as temporary:
    temporary = Path(temporary)
    (temporary / 'production.inc').write_text('\n'.join([
        function('src/bluetooth/bt_stream/unicast/unicast_server.c',
                 'int unicast_server_adv_populate('),
        function('src/audio/streamctrl.c', 'static void device_found('),
    ]))
    binary = temporary / 'pair_discovery'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                    '-I' + str(temporary), str(HERE / 'main.c'), '-o', str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
