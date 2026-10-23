// Execute the production GATT callbacks with deterministic transport/workqueue doubles.
#include <cassert>
#include <cstdio>
#include "../../../src/bluetooth/gatt_services/device_control_service.cpp"

int scheduled, delay_ms, indicate_error;
bool subscribed;
bt_gatt_attr status_attr;
bt_gatt_indicate_params *sent;
PowerManager power_manager;

static void reset() {
 busy = 0; confirmed = false; scheduled = 0; delay_ms = 0;
 subscribed = true; indicate_error = 0; sent = nullptr; power_manager.calls = 0;
}
static ssize_t request(const uint8_t *data, uint16_t len = 2,
                       uint16_t offset = 0, uint8_t flags = 0) {
 static bt_conn peer;
 return write_control(&peer, nullptr, data, len, offset, flags);
}
int main() {
 const uint8_t command[] = {1, 1};
 reset();
 uint8_t version = 0;
 assert(read_control(nullptr, nullptr, &version, 1, 0) == 1 && version == 1);
 assert(request(command, 1) < 0);
 assert(request(command, 2, 1) < 0);
 assert(request(command, 2, 0, BT_GATT_WRITE_FLAG_PREPARE) < 0);
 assert(request(command, 2, 0, BT_GATT_WRITE_FLAG_CMD) < 0);
 const uint8_t unknown[] = {2, 1};
 assert(request(unknown) < 0);
 const uint8_t opcode[] = {1, 2};
 assert(request(opcode) < 0);
 assert(!sent && !scheduled && !power_manager.calls);

 subscribed = false;
 assert(request(command) == -BT_ATT_ERR_CCC_IMPROPER_CONF);
 assert(!scheduled);
 reset(); indicate_error = -1;
 assert(request(command) < 0 && busy == 0 && !scheduled);

 reset();
 assert(request(command) == 2);
 assert(sent && sent->len == 3);
 assert(std::memcmp(sent->data, "\1\1\0", 3) == 0);
 assert(!scheduled && !power_manager.calls);
 assert(request(command) == -BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
 sent->func(nullptr, sent, 1); // timeout/disconnection: no destructive side effect
 sent->destroy(sent);
 assert(!scheduled && busy == 0);

 reset();
 assert(request(command) == 2);
 sent->func(nullptr, sent, 0);
 assert(!scheduled); // parameters still owned by Bluetooth
 sent->destroy(sent);
 assert(scheduled == 1 && delay_ms == 250);
 assert(!power_manager.calls);
 assert(request(command) == -BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
 execute_command.handler(nullptr);
 assert(power_manager.calls == 1);
 assert(busy == 1); // failed shutdown cannot start an overlapping cleanup
 std::puts("Device-control callbacks: validation, acknowledgement, failure and deferred shutdown passed");
}
