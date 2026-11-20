"""Compile and exercise the production GATT callbacks with host BLE test doubles."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class PowerSavingServiceTests(unittest.TestCase):
    """Verify manager-to-wire mapping and preserved GATT validation behavior."""

    def test_production_callbacks(self):
        """Read generated values and apply only valid, complete mode writes."""
        vectors = json.loads((Path(__file__).parent / 'wire_vectors.json').read_text())
        expected = bytes.fromhex(next(v['hex'] for v in vectors if v['name'] == 'supported_modes'))
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            headers = {
                'zephyr/bluetooth/gatt.h': '''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
struct bt_conn { int unused; };
struct bt_gatt_attr { int unused; };
#define BT_GATT_SERVICE_DEFINE(...)
#define BT_GATT_ERR(value) (-(value))
#define BT_ATT_ERR_INVALID_OFFSET 7
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN 13
#define BT_ATT_ERR_VALUE_NOT_ALLOWED 19
#define BT_ATT_ERR_UNLIKELY 14
ssize_t bt_gatt_attr_read(struct bt_conn *, const struct bt_gatt_attr *, void *, uint16_t, uint16_t, const void *, uint16_t);
''',
                'zephyr/bluetooth/uuid.h': '#pragma once\n',
                'zephyr/sys/util.h': '#define ARG_UNUSED(value) (void)(value)\n#define ARRAY_SIZE(value) (sizeof(value)/sizeof((value)[0]))\n',
                'zephyr/logging/log.h': '#define LOG_MODULE_REGISTER(...)\n#define LOG_ERR(...)\n',
            }
            for path, content in headers.items():
                header = temp / path
                header.parent.mkdir(parents=True, exist_ok=True)
                header.write_text(content)
            source = ROOT / 'src/bluetooth/gatt_services/power_saving_service.c'
            c = f'''#include <string.h>
#include "{source}"
static power_saving_level_t selected = POWER_SAVING_LEVEL_OFF;
static const char *names[] = {{"Off", "Minimal", "Balanced", "Aggressive"}};
uint8_t auto_off_get_supported_mode_count(void) {{ return 4; }}
const char *auto_off_get_mode_name(power_saving_level_t mode) {{ return names[mode]; }}
int auto_off_mode_is_supported(power_saving_level_t mode) {{ return mode >= 0 && mode < 4; }}
power_saving_level_t auto_off_get_mode(void) {{ return selected; }}
void auto_off_set_mode(power_saving_level_t mode) {{ selected = mode; }}
ssize_t bt_gatt_attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buffer, uint16_t len, uint16_t offset, const void *data, uint16_t size) {{
    (void)conn; (void)attr;
    if (offset > size) return -7;
    size_t count = size-offset < len ? size-offset : len;
    memcpy(buffer, (const uint8_t *)data+offset, count);
    return (ssize_t)count;
}}
#define CHECK(value) do {{ if (!(value)) return __LINE__; }} while (0)
int main(void) {{
    const uint8_t expected[] = {{{','.join(map(str, expected))}}};
    uint8_t buffer[128], mode=2;
    CHECK(read_supported_power_saving_modes(NULL, NULL, buffer, sizeof(buffer), 0) == sizeof(expected));
    CHECK(!memcmp(buffer, expected, sizeof(expected)));
    CHECK(read_supported_power_saving_modes(NULL, NULL, buffer, 3, 2) == 3 && !memcmp(buffer, expected+2, 3));
    CHECK(encode_supported_modes(buffer, sizeof(expected)-1) == -ENOMEM);
    CHECK(write_power_saving_mode(NULL, NULL, &mode, 1, 0, 0) == 1 && selected == POWER_SAVING_LEVEL_BALANCED);
    CHECK(read_power_saving_mode(NULL, NULL, buffer, sizeof(buffer), 0) == 1 && buffer[0] == 2);
    mode=255;
    CHECK(write_power_saving_mode(NULL, NULL, &mode, 1, 0, 0) == -19 && selected == POWER_SAVING_LEVEL_BALANCED);
    CHECK(write_power_saving_mode(NULL, NULL, &mode, 0, 0, 0) == -13);
    CHECK(write_power_saving_mode(NULL, NULL, &mode, 2, 0, 0) == -13);
    CHECK(write_power_saving_mode(NULL, NULL, &mode, 1, 1, 0) == -7);
    return 0;
}}
'''
            (temp / 'main.c').write_text(c)
            generated = ROOT / 'protocol/generated/c'
            subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-sign-compare',
                            '-I' + str(temp), '-I' + str(generated / 'include'),
                            '-I' + str(ROOT / 'src/Battery'), str(temp / 'main.c'),
                            str(generated / 'src/protocol_runtime.c'), str(generated / 'src/power_saving_protocol.c'),
                            '-o', str(temp / 'check')], check=True)
            subprocess.run([str(temp / 'check')], check=True)
