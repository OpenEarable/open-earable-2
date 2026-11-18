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

/* An LE Audio stream has been established. */
void bt_mgmt_ci_on_stream_started(void);

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
    virtual void on_connected(struct bt_conn * /*conn*/) {}

    /// Notify the strategy of a disconnection.
    virtual void on_disconnected(uint8_t /*reason*/) {}

    /// Notify the strategy of a connection parameter update.
    virtual void on_conn_param_updated(uint16_t /*interval_units*/,
                                       uint16_t /*latency*/,
                                       uint16_t /*timeout*/) {}

    /// An LE Audio stream has been established: ask for the interval it
    /// should run at. Called again after a short while until it returns
    /// true, as a central may turn a request down without notice.
    /// @return true if no connection needs a different interval.
    virtual bool on_stream_started() { return true; }

    /// Notify the strategy of audio underrun events.
    virtual void on_audio_underrun(uint32_t count) {
        (void)count;
    }

    /// Notify the strategy of a timer tick, in thread context.
    /// `elapsed` is the time since the last tick call.
    virtual void on_timer_tick(k_timeout_t elapsed) {
        (void)elapsed;
    }

    /**
     * Longest interval of the connections on which this device is the
     * peripheral, in 1.25 ms units. Cached, so it may be read in an ISR.
     */
    uint16_t current_interval_units() const {
        return current_interval_units_;
    }

    /**
     * Read the intervals from the connections themselves. A stored copy goes
     * stale: a new connection starts at an interval nobody reported, and a
     * second central's updates are not those of the streaming link.
     * Thread context only.
     */
    void refresh_current_interval();

    /**
     * Helper for derived strategies to request a new interval.
     *
     * min_units / max_units are in 1.25 ms units (same as Zephyr).
     * The request goes to every connection on which this device is the
     * peripheral and that is not already at the interval: each of them
     * shares the radio with the stream. From an ISR it is deferred.
     */
    void request_interval(uint16_t min_units, uint16_t max_units);

protected:
    uint16_t current_interval_units_ = CONFIG_BLE_ACL_CONN_INTERVAL;
};

#endif /* __cplusplus */