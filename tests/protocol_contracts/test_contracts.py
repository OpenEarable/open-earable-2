"""Check frozen BLE metadata and wire vectors against existing production code."""
import json
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent


class ProtocolContractTests(unittest.TestCase):
    """Compatibility checks that require only Python and a host C/C++ compiler."""

    def test_ble_metadata(self):
        """Keep custom UUIDs, properties, and permissions equal to the baseline."""
        contracts = json.loads((FIXTURES / 'ble_contracts.json').read_text())
        for name, contract in contracts.items():
            with self.subTest(service=name):
                source = '\n'.join((ROOT / path).read_text() for path in contract['sources'])
                actual = {m[0]: '-'.join(m[1:]) for m in re.findall(
                    r'#define\s+(\w+)_VAL\s+\\\s*BT_UUID_128_ENCODE\(0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+)\)', source)}
                self.assertEqual(contract['uuids'], actual)
                actual_chars = [{'uuid_macro': m[1], 'properties': m[2].strip(),
                                 'permissions': m[3].strip()} for m in re.finditer(
                    r'BT_GATT_CHARACTERISTIC\(\s*(\w+),\s*([^,]+),\s*([^,]+),', source)]
                self.assertEqual(contract['characteristics'], actual_chars)

    def test_production_serializers(self):
        """Compare sensor framing and component serialization with literal bytes."""
        vectors = {v['name']: v for v in json.loads((FIXTURES / 'wire_vectors.json').read_text())}
        harness = r'''
#include <cstddef>
#include <cstdio>
#include <cstring>
#include "sensor_transport.h"
#include "SensorComponent.h"
/** Print one serialized payload for comparison with the frozen fixture. */
static void dump(const void *data, size_t size) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) std::printf("%02x", bytes[i]);
    std::puts("");
}
/** Exercise production encoders with deterministic compatibility inputs. */
int main() {
    oe_sensor_batch batch{};
    const unsigned char first[] = {0, 0, 0xc0, 0x3f};
    const unsigned char second[] = {0, 0, 0, 0xc0};
    if (!oe_sensor_batch_append(&batch, 6, first, 0x0102030405060708ULL, 244)) return 1;
    dump(batch.data, batch.len);
    if (!oe_sensor_batch_append(&batch, 6, second, 0x0102030405060708ULL + 1000, 244)) return 2;
    dump(batch.data, batch.len);
    auto snapshot = batch;
    if (oe_sensor_batch_append(&batch, 6, second, batch.last_time + 999, 244)) return 3;
    if (std::memcmp(&snapshot, &batch, sizeof(batch))) return 4;
    SensorComponent component{"x", "C", PARSE_TYPE_FLOAT};
    SensorComponentGroup group{"g", 1, &component};
    char buffer[32]{};
    auto size = serializeSensorComponentGroup(&group, buffer, sizeof(buffer));
    if (size != 7 || getSensorComponentGroupSize(&group) != 7) return 5;
    dump(buffer, size);
    if (serializeSensorComponentGroup(&group, buffer, 6) >= 0) return 6;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            (temp / 'main.cpp').write_text(harness)
            subprocess.run(['cc', '-std=c99', '-c', str(ROOT / 'src/bluetooth/gatt_services/sensor_transport.c'),
                            '-o', str(temp / 'transport.o')], check=True)
            subprocess.run(['c++', '-std=c++11', '-include', 'cstddef',
                            '-I' + str(ROOT / 'src/bluetooth/gatt_services'),
                            '-I' + str(ROOT / 'src/ParseInfo'), str(temp / 'main.cpp'),
                            str(ROOT / 'src/ParseInfo/SensorComponent.cpp'), str(temp / 'transport.o'),
                            '-o', str(temp / 'contracts')], check=True)
            output = subprocess.check_output([str(temp / 'contracts')], text=True).splitlines()
        self.assertEqual(output, [vectors[name]['hex'] for name in
                                 ('sensor_single', 'sensor_batch', 'parse_component')])


if __name__ == '__main__':
    unittest.main()
