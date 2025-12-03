#include "time_sync.h"
/*
 * CTS time sync helper (peripheral CTS client)
 * - No LEDs
 * - No buttons
 * - No advertising
 * - Just logs time from phone via CTS.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(time_sync, LOG_LEVEL_DBG);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

// #include <bluetooth/gatt_dm.h>
#include <bluetooth/services/cts_client.h>

#include <zephyr/settings/settings.h>

static struct bt_cts_client cts_c;
static bool has_cts;

/* Optional: nice human-readable output */
static const char *day_of_week[] = {
	"Unknown",
	"Monday",
	"Tuesday",
	"Wednesday",
	"Thursday",
	"Friday",
	"Saturday",
	"Sunday"
};

static const char *month_of_year[] = {
	"Unknown",
	"January",
	"February",
	"March",
	"April",
	"May",
	"June",
	"July",
	"August",
	"September",
	"October",
	"November",
	"December"
};

static void current_time_print(struct bt_cts_current_time *current_time)
{
	const struct bt_cts_exact_time_256 *t = &current_time->exact_time_256;
	const struct bt_cts_adjust_reason *ar = &current_time->adjust_reason;

	LOG_INF("===== Current Time (CTS) =====");

	if (t->day_of_week < ARRAY_SIZE(day_of_week)) {
		LOG_INF("Day of week   : %s", day_of_week[t->day_of_week]);
	} else {
		LOG_INF("Day of week   : (invalid %u)", t->day_of_week);
	}

	if (t->day == 0) {
		LOG_INF("Day of month  : Unknown");
	} else {
		LOG_INF("Day of month  : %u", t->day);
	}

	if (t->month < ARRAY_SIZE(month_of_year)) {
		LOG_INF("Month of year : %s", month_of_year[t->month]);
	} else {
		LOG_INF("Month of year : (invalid %u)", t->month);
	}

	if (t->year == 0) {
		LOG_INF("Year          : Unknown");
	} else {
		LOG_INF("Year          : %u", t->year);
	}

	LOG_INF("Time          : %02u:%02u:%02u.%03u (frac=%u/256)",
		t->hours,
		t->minutes,
		t->seconds,
		(t->fractions256 * 1000) / 256,
		t->fractions256);

	LOG_INF("Adjust reason : DST=%u TZ=%u EXT=%u MAN=%u",
		ar->change_of_daylight_savings_time,
		ar->change_of_time_zone,
		ar->external_reference_time_update,
		ar->manual_time_update);
}

/* Called for notifications (if we subscribe) */
static void notify_current_time_cb(struct bt_cts_client *cts,
				   struct bt_cts_current_time *current_time)
{
	ARG_UNUSED(cts);
	current_time_print(current_time);
}

/* Called for explicit reads */
static void read_current_time_cb(struct bt_cts_client *cts,
				 struct bt_cts_current_time *current_time,
				 int err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(cts->conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("Cannot read Current Time from %s (err %d)", addr, err);
		return;
	}

	LOG_INF("CTS read from %s succeeded", addr);
	current_time_print(current_time);
}

static void enable_notifications(void)
{
	int err;

	if (!has_cts) {
		return;
	}

	if (bt_conn_get_security(cts_c.conn) < BT_SECURITY_L2) {
		LOG_INF("Security level < L2, will subscribe after upgrade");
		return;
	}

	err = bt_cts_subscribe_current_time(&cts_c, notify_current_time_cb);
	if (err) {
		LOG_WRN("Cannot subscribe to CTS notifications (err %d)", err);
	} else {
		LOG_INF("Subscribed to CTS notifications");
	}
}

/* GATT Discovery Manager callbacks for CTS */
static void discover_completed_cb(struct bt_gatt_dm *dm, void *ctx)
{
	int err;

	ARG_UNUSED(ctx);

	LOG_INF("CTS service discovery completed");

	err = bt_cts_handles_assign(dm, &cts_c);
	if (err) {
		LOG_ERR("Could not assign CTS client handles (err %d)", err);
	} else {
		has_cts = true;

		if (bt_conn_get_security(cts_c.conn) < BT_SECURITY_L2) {
			LOG_INF("Upgrading security for CTS to L2");
			err = bt_conn_set_security(cts_c.conn, BT_SECURITY_L2);
			if (err) {
				LOG_ERR("Failed to set security (err %d)", err);
			}
		} else {
			/* Security already OK: enable notifications and do an initial read */
			enable_notifications();

			err = bt_cts_read_current_time(&cts_c, read_current_time_cb);
			if (err) {
				LOG_WRN("Initial CTS read failed (err %d)", err);
			}
		}
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release discovery data (err %d)", err);
	}
}

static void discover_service_not_found_cb(struct bt_conn *conn, void *ctx)
{
	char addr[BT_ADDR_LE_STR_LEN];

	ARG_UNUSED(ctx);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("CTS service not found on peer %s", addr);
}

static void discover_error_found_cb(struct bt_conn *conn, int err, void *ctx)
{
	char addr[BT_ADDR_LE_STR_LEN];

	ARG_UNUSED(ctx);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("CTS discovery failed on %s (err %d)", addr, err);
}

static const struct bt_gatt_dm_cb discover_cb = {
	.completed         = discover_completed_cb,
	.service_not_found = discover_service_not_found_cb,
	.error_found       = discover_error_found_cb,
};

/* Connection callbacks – we only care about CTS-related stuff here */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err 0x%02x %s)",
			err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);

	has_cts = false;
	cts_c.conn = conn;

	/* Start CTS discovery immediately; your other code can still have its own callbacks */
	err = bt_gatt_dm_start(conn, BT_UUID_CTS, &discover_cb, NULL);
	if (err) {
		LOG_WRN("Failed to start CTS discovery (err %d)", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected from %s (reason 0x%02x %s)",
		addr, reason, bt_hci_err_to_str(reason));

	if (cts_c.conn == conn) {
		cts_c.conn = NULL;
	}
	has_cts = false;
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
		enable_notifications();

		/* Also trigger a read after security is up, if we have CTS */
		if (has_cts) {
			int r = bt_cts_read_current_time(&cts_c, read_current_time_cb);
			if (r) {
				LOG_WRN("CTS read after security upgrade failed (err %d)", r);
			}
		}
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s",
			addr, level, err, bt_security_err_to_str(err));
	}
}

BT_CONN_CB_DEFINE(cts_conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.security_changed = security_changed,
};

/* Auth callbacks: optional but useful for logging / handling failures */
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Pairing cancelled: %s", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing completed: %s, bonded=%d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Pairing failed: %s, reason=%d %s",
		addr, reason, bt_security_err_to_str(reason));

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed   = pairing_failed,
};

/*
 * Public init function:
 * Call this AFTER bt_enable(), in your main init path.
 * It does not start advertising or manage connections – it only
 * sets up the CTS client + auth callbacks so that this module
 * can react to connections and discover CTS.
 */
int init_time_sync(void)
{
	int err;

	LOG_INF("Initializing CTS time sync");

	err = bt_cts_client_init(&cts_c);
	if (err) {
		LOG_ERR("CTS client init failed (err %d)", err);
		return err;
	}

	/* Optional: if your app already registers its own auth callbacks,
	 * you may want to merge / adjust these, or skip registering here.
	 */
	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_WRN("Failed to register auth callbacks (err %d)", err);
		/* Not fatal for CTS, so don't return error */
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_WRN("Failed to register auth info callbacks (err %d)", err);
		/* Not fatal for CTS */
	}

	LOG_INF("CTS time sync init done");
	return 0;
}
