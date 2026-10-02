"""Check frozen BLE metadata and wire vectors against existing production code."""
import json
import os
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
                if name in ('audio_config', 'led', 'button'):
                    self.assert_generated_metadata(name, contract, source)
                    continue
                actual = {m[0]: '-'.join(m[1:]) for m in re.findall(
                    r'#define\s+(\w+)_VAL\s+\\\s*BT_UUID_128_ENCODE\(0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+),\s*0x([0-9a-f]+)\)', source)}
                self.assertEqual(contract['uuids'], actual)
                actual_chars = [{'uuid_macro': m[1], 'properties': m[2].strip(),
                                 'permissions': m[3].strip()} for m in re.finditer(
                    r'BT_GATT_CHARACTERISTIC\(\s*(\w+),\s*([^,]+),\s*([^,]+),', source)]
                self.assertEqual(contract['characteristics'], actual_chars)

    def assert_generated_metadata(self, name, contract, source):
        """Resolve migrated metadata against the original UUID/property baseline."""
        proto, names = {
            'audio_config': ('audio_configuration', ['audio_mode', 'microphone_selection', 'audio_channel', 'microphone_gain']),
            'led': ('led', ['rgb', 'state']),
            'button': ('button', ['state']),
        }[name]
        header = (ROOT / f'protocol/generated/c/include/zephyr/{proto}_ble.h').read_text()
        definitions = dict(re.findall(r'^#define (\w+) (.+)$', header, re.MULTILINE))
        prefix = proto.upper() + '_ZEPHYR_'
        self.assertIn(f'BT_GATT_PRIMARY_SERVICE({prefix}SERVICE_UUID)', source)
        uuid_parts = re.findall(r'BT_UUID_128_ENCODE\(0x([0-9a-f]+), 0x([0-9a-f]+), 0x([0-9a-f]+), 0x([0-9a-f]+), 0x([0-9a-f]+)\)', header)
        self.assertEqual(set(contract['uuids'].values()), {'-'.join(parts) for parts in uuid_parts})
        self.assertEqual(len(names), len(contract['characteristics']))
        for char, expected in zip(names, contract['characteristics']):
            macro = prefix + char.upper() + '_CHARACTERISTIC'
            for suffix, key in [('PROPERTIES', 'properties'), ('PERMISSIONS', 'permissions')]:
                self.assertEqual(set(re.findall(r'BT_\w+', definitions[macro + '_' + suffix])),
                                 set(re.findall(r'BT_\w+', expected[key])))
                self.assertIn(macro + '_' + suffix, source)
            self.assertIn('BT_GATT_CHARACTERISTIC(' + macro + '_UUID', source)

    def test_generated_simple_codecs(self):
        """Check generated C codecs against frozen bytes, including short inputs."""
        vectors = {v['name']: bytes.fromhex(v['hex']) for v in json.loads((FIXTURES / 'wire_vectors.json').read_text())}
        cases = [
            ('audio_configuration_audio_mode', 'audio_mode_anc'),
            ('audio_configuration_microphone_selection', 'mic_select_right'),
            ('audio_configuration_audio_channel', 'audio_channel_left'),
            ('audio_configuration_microphone_gain', 'dmic_gain'),
            ('led_rgb', 'led_rgb'), ('led_state', 'led_custom'), ('button_state', 'button_pressed'),
        ]
        code = '#include <string.h>\n#include "audio_configuration_protocol.h"\n#include "led_protocol.h"\n#include "button_protocol.h"\nint main(void) {\n'
        for index, (codec, fixture) in enumerate(cases):
            values = ','.join(str(value) for value in vectors[fixture])
            size = len(vectors[fixture])
            initializers = ','.join(str(value) for value in vectors[fixture])
            code += f"""{{
                const uint8_t expected[] = {{{values}}};
                {codec}_t message;
                const {codec}_t expected_message = {{{initializers}}};
                uint8_t output[{size}];
                size_t used = 0;
                if ({codec}_decode(&message, expected, {size}, &used) != PROTOCOL_OK || used != {size}) return {index + 1};
                if (memcmp(&message, &expected_message, sizeof(message))) return {index + 40};
                if ({codec}_encode(&expected_message, output, sizeof(output), &used) != PROTOCOL_OK || used != {size} || memcmp(output, expected, {size})) return {index + 10};
                if ({codec}_decode(&message, expected, {size - 1}, NULL) == PROTOCOL_OK) return {index + 20};
                if ({codec}_encode(&message, output, {size - 1}, NULL) == PROTOCOL_OK) return {index + 30};
            }}\n"""
        code += 'return 0; }\n'
        generated = ROOT / 'protocol/generated/c'
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            (temp / 'main.c').write_text(code)
            subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-I' + str(generated / 'include'),
                            str(temp / 'main.c'), *[str(generated / 'src' / (name + '.c')) for name in
                            ('protocol_runtime', 'audio_configuration_protocol', 'led_protocol', 'button_protocol')],
                            '-o', str(temp / 'codecs')], check=True)
            subprocess.run([str(temp / 'codecs')], check=True)

    def test_generated_dart_codecs(self):
        """Use the same golden bytes to verify Dart decoding and encoding."""
        import shutil
        dart = os.environ.get('PROTOCOL_DART', 'dart')
        if shutil.which(dart) is None:
            self.skipTest('Dart SDK is unavailable')
        vectors = {v['name']: bytes.fromhex(v['hex']) for v in json.loads((FIXTURES / 'wire_vectors.json').read_text())}
        cases = [('AudioConfigurationAudioMode', 'audio_mode_anc'),
                 ('AudioConfigurationMicrophoneSelection', 'mic_select_right'),
                 ('AudioConfigurationAudioChannel', 'audio_channel_left'),
                 ('AudioConfigurationMicrophoneGain', 'dmic_gain'),
                 ('LedRgb', 'led_rgb'), ('LedState', 'led_custom'), ('ButtonState', 'button_pressed')]
        library = (ROOT / 'protocol/generated/dart/lib/open_earable_protocols.dart').as_uri()
        code = f"import 'dart:typed_data';\nimport '{library}';\nvoid main() {{\n"
        for cls, fixture in cases:
            values = ','.join(str(value) for value in vectors[fixture])
            fields = {'AudioConfigurationAudioMode': ['mode'], 'AudioConfigurationMicrophoneSelection': ['microphone'],
                      'AudioConfigurationAudioChannel': ['channel'], 'AudioConfigurationMicrophoneGain': ['outer', 'inner'],
                      'LedRgb': ['red', 'green', 'blue'], 'LedState': ['mode'], 'ButtonState': ['action']}[cls]
            arguments = ', '.join(f'{field}: {value}' for field, value in zip(fields, vectors[fixture]))
            comparisons = ' || '.join(f'decoded.{field} != {value}' for field, value in zip(fields, vectors[fixture]))
            code += f"""{{
                final input = Uint8List.fromList([{values}]);
                final decoded = {cls}.fromBytes(input);
                if ({comparisons}) throw StateError('{fixture}: fields');
                final output = {cls}({arguments}).toBytes();
                if (output.length != input.length) throw StateError('{fixture}: length');
                for (var i = 0; i < input.length; i++) {{
                    if (output[i] != input[i]) throw StateError('{fixture}: bytes');
                }}
                var rejected = false;
                try {{ {cls}.fromBytes(Uint8List.sublistView(input, 0, input.length - 1)); }}
                catch (_) {{ rejected = true; }}
                if (!rejected) throw StateError('{fixture}: truncated input');
            }}\n"""
        code += '}\n'
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / 'contracts.dart'
            script.write_text(code)
            subprocess.run([dart, str(script)], check=True)

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
