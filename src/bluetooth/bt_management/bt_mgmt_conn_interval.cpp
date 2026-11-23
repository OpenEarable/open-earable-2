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

static struct k_work g_ci_tick_work;
static struct k_work_delayable g_ci_stream_work;

/* The tick reads connection state and may request an update: neither is
 * allowed in the timer's interrupt context.
 */
static void ci_tick_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    /* A stream that has just started is handled by the stream requests. A
     * tick request in that window was dropped by the central, which then
     * also dropped the stream request: the interval only changed after
     * about 850 ms and every second frame was lost for 3.5 s.
     */
    if (k_work_delayable_is_pending(&g_ci_stream_work)) {
        return;
    }

    if (g_strategy) {
        g_strategy->refresh_current_interval();
        g_strategy->on_timer_tick(K_MSEC(1000));
    }
}

static void ci_timer_handler(struct k_timer* timer)
{
    ARG_UNUSED(timer);

    (void)k_work_submit(&g_ci_tick_work);
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

/* A request made while the central still sets up the CIS is dropped (seen
 * 125 ms after the central's own update, 10 ms before the stream started),
 * and nothing reports that. Ask again until the interval is in place.
 */
#define CI_STREAM_FIRST_REQUEST_MS 150
#define CI_STREAM_RETRY_MS         300
#define CI_STREAM_REQUESTS_MAX     10

static uint8_t g_ci_stream_requests_left;

static void ci_stream_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    if (!g_strategy) {
        return;
    }

    g_strategy->refresh_current_interval();
    if (g_strategy->on_stream_started() || --g_ci_stream_requests_left == 0) {
        return;
    }

    (void)k_work_reschedule(&g_ci_stream_work, K_MSEC(CI_STREAM_RETRY_MS));
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
    k_work_init(&g_ci_tick_work, ci_tick_work_handler);
    k_work_init_delayable(&g_ci_stream_work, ci_stream_work_handler);

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
        g_strategy->refresh_current_interval();
        g_strategy->on_connected(conn);
    }
}

extern "C" void bt_mgmt_ci_on_stream_started(void)
{
    g_ci_stream_requests_left = CI_STREAM_REQUESTS_MAX;
    (void)k_work_reschedule(&g_ci_stream_work, K_MSEC(CI_STREAM_FIRST_REQUEST_MS));
}

extern "C" void bt_mgmt_ci_on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (g_strategy) {
        g_strategy->refresh_current_interval();
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
        g_strategy->refresh_current_interval();
        g_strategy->on_conn_param_updated(interval, latency, timeout);
    }
}

/* Interval of a connection on which this device is the peripheral, in
 * 1.25 ms units, or 0 for any other connection.
 */
static uint16_t peripheral_interval_units(struct bt_conn* conn)
{
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) != 0 || info.state != BT_CONN_STATE_CONNECTED ||
        info.role != BT_CONN_ROLE_PERIPHERAL) {
        return 0;
    }

    return (uint16_t)(info.le.interval_us / 1250U);
}

static void longest_interval_cb(struct bt_conn* conn, void* data)
{
    uint16_t* longest = static_cast<uint16_t*>(data);
    uint16_t units = peripheral_interval_units(conn);

    if (units > *longest) {
        *longest = units;
    }
}

void BtMgmtConnIntervalStrategy::refresh_current_interval()
{
    uint16_t longest = 0;

    bt_conn_foreach(BT_CONN_TYPE_LE, longest_interval_cb, &longest);
    if (longest != 0) {
        current_interval_units_ = longest;
    }
}

struct ci_request {
    uint16_t min_units;
    uint16_t max_units;
};

static void request_interval_cb(struct bt_conn* conn, void* data)
{
    const struct ci_request* req = static_cast<const struct ci_request*>(data);
    uint16_t units = peripheral_interval_units(conn);

    if (units == 0 || (units >= req->min_units && units <= req->max_units)) {
        return;
    }

    bt_le_conn_param param = {
        .interval_min = req->min_units,
        .interval_max = req->max_units,
        .latency      = CONFIG_BLE_ACL_SLAVE_LATENCY,
        .timeout      = CONFIG_BLE_ACL_SUP_TIMEOUT,
    };

    int err = bt_conn_le_param_update(conn, &param);
    if (err) {
        LOG_WRN("bt_conn_le_param_update failed: %d (min=%u max=%u)",
                err, req->min_units, req->max_units);
    } else {
        LOG_INF("Requested CI [%u..%u] units (%.2f..%.2f ms)",
                req->min_units, req->max_units,
                (double)(req->min_units * 1.25f), (double)(req->max_units * 1.25f));
    }
}

void BtMgmtConnIntervalStrategy::request_interval(uint16_t min_units, uint16_t max_units)
{
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

    struct ci_request req = {min_units, max_units};

    bt_conn_foreach(BT_CONN_TYPE_LE, request_interval_cb, &req);
}
