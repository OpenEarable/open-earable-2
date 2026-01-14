// BtMgmtConnIntervalStrategy.cpp
#include "bt_mgmt_conn_interval.h"
#include <zephyr/kernel.h>              // k_timer, k_work, k_is_in_isr
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bt_mgmt, CONFIG_BT_MGMT_LOG_LEVEL);

static BtMgmtConnIntervalStrategy* g_strategy = nullptr;

/* ----------------------------------------------------------------------
 * Periodic timer for on_timer_tick()
 * -------------------------------------------------------------------- */
static struct k_timer g_ci_timer;

static void ci_timer_handler(struct k_timer* timer)
{
    ARG_UNUSED(timer);

    if (g_strategy) {
        /* You can pass a different timeout if you want */
        g_strategy->on_timer_tick(K_MSEC(1000));
    }
}

/* ----------------------------------------------------------------------
 * Deferred CI request work (to avoid calling bt_conn_le_param_update in ISR)
 * -------------------------------------------------------------------- */
struct ci_req_ctx_t {
    struct k_work work;
    BtMgmtConnIntervalStrategy* strategy;
    uint16_t min_units;
    uint16_t max_units;
};

static struct ci_req_ctx_t g_ci_req_ctx;

static void ci_req_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    BtMgmtConnIntervalStrategy* s = g_ci_req_ctx.strategy;
    if (!s) {
        return;
    }

    uint16_t min_units = g_ci_req_ctx.min_units;
    uint16_t max_units = g_ci_req_ctx.max_units;

    /* Now we are in thread context (system workqueue), so it is safe
     * to call request_interval() again. It will hit the non-ISR path.
     */
    s->request_interval(min_units, max_units);
}

extern "C" void bt_mgmt_conn_interval_init(BtMgmtConnIntervalStrategy* strategy)
{
    g_strategy = strategy;
    if (g_strategy) {
        g_strategy->init();
    }

    /* Init deferred CI work */
    g_ci_req_ctx.strategy  = nullptr;
    g_ci_req_ctx.min_units = 0;
    g_ci_req_ctx.max_units = 0;
    k_work_init(&g_ci_req_ctx.work, ci_req_work_handler);

    /* Init and start periodic timer (1s period) */
    k_timer_init(&g_ci_timer, ci_timer_handler, nullptr);
    k_timer_start(&g_ci_timer, K_MSEC(1000), K_MSEC(1000));
}

extern "C" void bt_mgmt_report_audio_underrun(uint32_t count)
{
    if (g_strategy) {
        g_strategy->on_audio_underrun(count);
    }
}

extern "C" void bt_mgmt_ci_on_connected(struct bt_conn* conn)
{
    if (g_strategy) {
        g_strategy->on_connected(conn);
    }
}

extern "C" void bt_mgmt_ci_on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (g_strategy) {
        g_strategy->on_disconnected(reason);
    }
}

extern "C" void bt_mgmt_ci_on_conn_param_updated(struct bt_conn* conn,
                                                 uint16_t interval,
                                                 uint16_t latency,
                                                 uint16_t timeout)
{
    ARG_UNUSED(conn);

    if (g_strategy) {
        g_strategy->on_conn_param_updated(interval, latency, timeout);
    }
}

void BtMgmtConnIntervalStrategy::request_interval(uint16_t min_units, uint16_t max_units)
{
    if (!bt_conn_) {
        LOG_WRN("No connection for CI request");
        return;
    }

    /* Avoid redundant requests */
    if (min_units == max_units && min_units == current_interval_units_) {
        return;
    }

    /* If we are in ISR context, defer the real call to system workqueue */
    if (k_is_in_isr()) {
        g_ci_req_ctx.strategy  = this;
        g_ci_req_ctx.min_units = min_units;
        g_ci_req_ctx.max_units = max_units;

        int err = k_work_submit(&g_ci_req_ctx.work);
        if (err < 0) {
            LOG_WRN("Failed to submit CI work from ISR: %d", err);
        }
        return;
    }

    /* Normal path: thread context, safe to call bt_conn_le_param_update */
    bt_le_conn_param param = {
        .interval_min = min_units,
        .interval_max = max_units,
        .latency      = CONFIG_BLE_ACL_SLAVE_LATENCY,
        .timeout      = CONFIG_BLE_ACL_SUP_TIMEOUT,
    };

    int err = bt_conn_le_param_update(bt_conn_, &param);
    if (err) {
        LOG_WRN("bt_conn_le_param_update failed: %d (min=%u max=%u)",
                err, min_units, max_units);
    } else {
        LOG_INF("Requested CI [%u..%u] units (%.2f..%.2f ms)",
                min_units, max_units,
                min_units * 1.25f, max_units * 1.25f);
    }
}