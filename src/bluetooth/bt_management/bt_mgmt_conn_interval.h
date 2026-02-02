#pragma once

#include <zephyr/kernel.h>          // for k_timeout_t
#include "bt_mgmt.h"                // should pull in struct bt_conn / bt_conn typedef

#ifdef __cplusplus

class BtMgmtConnIntervalStrategy;
#endif

#ifdef __cplusplus
extern "C" {
#endif

void bt_mgmt_report_audio_underrun(uint32_t count);

void bt_mgmt_ci_on_connected(struct bt_conn *conn);
void bt_mgmt_ci_on_disconnected(struct bt_conn *conn, uint8_t reason);
void bt_mgmt_ci_on_conn_param_updated(struct bt_conn *conn,
                                      uint16_t interval,
                                      uint16_t latency,
                                      uint16_t timeout);

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus

/* C++ side registers the strategy instance here */
extern "C" void bt_mgmt_conn_interval_init(BtMgmtConnIntervalStrategy *strategy);

/**
 * Base strategy interface for connection interval handling.
 *
 * You implement subclasses with different adaptation behaviours,
 * and register one instance with bt_mgmt_conn_interval_init().
 */
class BtMgmtConnIntervalStrategy {
public:
    virtual ~BtMgmtConnIntervalStrategy() = default;

    /// Called once after registration, before first connection
    virtual void init() {}

    /// Notify the strategy of a new connection.
    virtual void on_connected(struct bt_conn *conn) {
        bt_conn_ = conn;
    }

    /// Notify the strategy of a disconnection.
    virtual void on_disconnected(uint8_t /*reason*/) {
        bt_conn_ = nullptr;
    }

    /// Notify the strategy of a connection parameter update.
    virtual void on_conn_param_updated(uint16_t interval_units,
                                       uint16_t latency,
                                       uint16_t timeout) {
        current_interval_units_ = interval_units;
        (void)latency;
        (void)timeout;
    }

    /// Notify the strategy of audio underrun events.
    virtual void on_audio_underrun(uint32_t count) {
        (void)count;
    }

    /// Notify the strategy of a timer tick.
    /// `elapsed` is the time since the last tick call.
    virtual void on_timer_tick(k_timeout_t elapsed) {
        (void)elapsed;
    }

    uint16_t current_interval_units() const {
        return current_interval_units_;
    }

    /**
     * Helper for derived strategies to request a new interval.
     *
     * min_units / max_units are in 1.25 ms units (same as Zephyr).
     * This wraps bt_conn_le_param_update() and logs errors.
     */
    void request_interval(uint16_t min_units, uint16_t max_units);

protected:
    struct bt_conn *bt_conn_ = nullptr;
    uint16_t current_interval_units_ = CONFIG_BLE_ACL_CONN_INTERVAL;
};

#endif /* __cplusplus */