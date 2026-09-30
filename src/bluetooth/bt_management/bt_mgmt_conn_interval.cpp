#include "bt_mgmt_conn_interval.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "streamctrl.h"
#include "wireless_audio_configuration_control.h"

LOG_MODULE_DECLARE(bt_mgmt, CONFIG_BT_MGMT_LOG_LEVEL);

namespace {

constexpr uint32_t interval_unit_us = 1250U;
constexpr uint32_t tick_period_ms = 100U;

struct IntervalRequest {
	struct k_work work;
	struct k_spinlock lock;
	struct bt_conn *conn;
	uint16_t minimum;
	uint16_t maximum;
	uint16_t latency;
	uint16_t timeout;
};

/** Common interface for replaceable ACL connection-interval strategies. */
class ConnectionIntervalStrategy {
public:
	virtual ~ConnectionIntervalStrategy() = default;
	virtual void connected(void) {}
	virtual void disconnected(void) {}
	virtual void parameters_updated(uint16_t interval, uint16_t latency, uint16_t timeout)
	{
		current_interval_ = interval;
		(void)latency;
		(void)timeout;
	}
	virtual void audio_underrun(uint32_t count) { (void)count; }
	virtual void timer_tick(int64_t now_ms) { (void)now_ms; }

protected:
	void request(uint16_t minimum, uint16_t maximum, uint16_t latency, uint16_t timeout);
	uint16_t current_interval_ = CONFIG_BLE_ACL_CONN_INTERVAL;
};

/** Leaves connection-parameter selection entirely to the controller and peer. */
class ControllerDefaultStrategy final : public ConnectionIntervalStrategy {};

/** Requests one exact connection-parameter tuple whenever an ACL is associated. */
class FixedStrategy final : public ConnectionIntervalStrategy {
public:
	void configure(const wireless_audio_configuration_fixed_acl_policy_t &policy)
	{
		interval_ = static_cast<uint16_t>(policy.interval_us / interval_unit_us);
		latency_ = policy.peripheral_latency;
		timeout_ = static_cast<uint16_t>(policy.supervision_timeout_ms / 10U);
	}

	void connected() override { request(interval_, interval_, latency_, timeout_); }

private:
	uint16_t interval_ = CONFIG_BLE_ACL_CONN_INTERVAL;
	uint16_t latency_ = CONFIG_BLE_ACL_SLAVE_LATENCY;
	uint16_t timeout_ = CONFIG_BLE_ACL_SUP_TIMEOUT;
};

/** Requests a bounded interval range and lets the peer select within that range. */
class PreferredRangeStrategy final : public ConnectionIntervalStrategy {
public:
	void configure(const wireless_audio_configuration_preferred_range_acl_policy_t &policy)
	{
		minimum_ = static_cast<uint16_t>(policy.minimum_interval_us / interval_unit_us);
		maximum_ = static_cast<uint16_t>(policy.maximum_interval_us / interval_unit_us);
		latency_ = policy.peripheral_latency;
		timeout_ = static_cast<uint16_t>(policy.supervision_timeout_ms / 10U);
	}

	void connected() override { request(minimum_, maximum_, latency_, timeout_); }

private:
	uint16_t minimum_ = CONFIG_BLE_ACL_CONN_INTERVAL;
	uint16_t maximum_ = CONFIG_BLE_ACL_CONN_INTERVAL;
	uint16_t latency_ = CONFIG_BLE_ACL_SLAVE_LATENCY;
	uint16_t timeout_ = CONFIG_BLE_ACL_SUP_TIMEOUT;
};

/** Adjusts an exact interval using underrun feedback and timed recovery steps. */
class AdaptiveLinearStrategy final : public ConnectionIntervalStrategy {
public:
	void configure(const wireless_audio_configuration_adaptive_linear_acl_policy_t &policy)
	{
		minimum_ = static_cast<uint16_t>(policy.minimum_interval_us / interval_unit_us);
		maximum_ = static_cast<uint16_t>(policy.maximum_interval_us / interval_unit_us);
		increase_ = static_cast<uint16_t>(policy.underrun_increase_step_us /
					 interval_unit_us);
		decrease_ = static_cast<uint16_t>(policy.recovery_decrease_step_us /
					 interval_unit_us);
		recovery_period_ms_ = policy.recovery_period_ms;
		minimum_update_period_ms_ = policy.minimum_update_period_ms;
		latency_ = policy.peripheral_latency;
		timeout_ = static_cast<uint16_t>(policy.supervision_timeout_ms / 10U);
		pending_target_ = 0U;
	}

	void connected() override
	{
		const int64_t now = k_uptime_get();

		last_request_ms_ = now;
		last_recovery_ms_ = now;
		request(minimum_, minimum_, latency_, timeout_);
	}

	void disconnected() override { pending_target_ = 0U; }

	void audio_underrun(uint32_t count) override
	{
		if (count == last_underrun_count_) {
			return;
		}

		last_underrun_count_ = count;
		const uint16_t target = MIN(static_cast<uint32_t>(maximum_),
					    static_cast<uint32_t>(current_interval_) + increase_);
		request_or_defer(target, k_uptime_get());
	}

	void timer_tick(int64_t now_ms) override
	{
		if (pending_target_ != 0U && update_period_elapsed(now_ms)) {
			request_target(pending_target_, now_ms);
			pending_target_ = 0U;
			return;
		}

		if (stream_state_get() != STATE_STREAMING ||
		    now_ms - last_recovery_ms_ < recovery_period_ms_) {
			return;
		}

		last_recovery_ms_ = now_ms;
		const uint16_t target = current_interval_ > minimum_ + decrease_
					? current_interval_ - decrease_
					: minimum_;
		request_or_defer(target, now_ms);
	}

private:
	bool update_period_elapsed(int64_t now_ms) const
	{
		return now_ms - last_request_ms_ >= minimum_update_period_ms_;
	}

	void request_or_defer(uint16_t target, int64_t now_ms)
	{
		if (target == current_interval_) {
			return;
		}
		if (!update_period_elapsed(now_ms)) {
			pending_target_ = target;
			return;
		}
		request_target(target, now_ms);
	}

	void request_target(uint16_t target, int64_t now_ms)
	{
		last_request_ms_ = now_ms;
		request(target, target, latency_, timeout_);
	}

	uint16_t minimum_ = CONFIG_BLE_ACL_CONN_INTERVAL;
	uint16_t maximum_ = CONFIG_BLE_ACL_CONN_INTERVAL_SLOW;
	uint16_t increase_ = 4U;
	uint16_t decrease_ = 4U;
	uint16_t latency_ = CONFIG_BLE_ACL_SLAVE_LATENCY;
	uint16_t timeout_ = CONFIG_BLE_ACL_SUP_TIMEOUT;
	uint16_t pending_target_ = 0U;
	uint32_t recovery_period_ms_ = 1000U;
	uint32_t minimum_update_period_ms_ = 1000U;
	uint32_t last_underrun_count_ = 0U;
	int64_t last_request_ms_ = 0;
	int64_t last_recovery_ms_ = 0;
};

K_MUTEX_DEFINE(engine_mutex);
struct k_work tick_work;
struct k_work underrun_work;
struct k_timer tick_timer;
IntervalRequest interval_request;
atomic_t latest_underrun_count;
struct bt_conn *managed_conn;
bt_mgmt_ci_adjustment_cb_t adjustment_callback;
ControllerDefaultStrategy controller_default_strategy;
FixedStrategy fixed_strategy;
PreferredRangeStrategy preferred_range_strategy;
AdaptiveLinearStrategy adaptive_linear_strategy;
ConnectionIntervalStrategy *active_strategy = &controller_default_strategy;

/** Execute the latest coalesced HCI connection-parameter request. */
void interval_request_handler(struct k_work *work)
{
	(void)work;
	struct bt_conn *conn;
	uint16_t minimum;
	uint16_t maximum;
	uint16_t latency;
	uint16_t timeout;

	k_spinlock_key_t key = k_spin_lock(&interval_request.lock);
	conn = interval_request.conn;
	interval_request.conn = nullptr;
	minimum = interval_request.minimum;
	maximum = interval_request.maximum;
	latency = interval_request.latency;
	timeout = interval_request.timeout;
	k_spin_unlock(&interval_request.lock, key);

	if (conn == nullptr) {
		return;
	}

	const struct bt_le_conn_param parameters = {
		.interval_min = minimum,
		.interval_max = maximum,
		.latency = latency,
		.timeout = timeout,
	};
	const int err = bt_conn_le_param_update(conn, &parameters);
	if (err != 0) {
		LOG_WRN("ACL parameter request failed: err=%d min=%u max=%u latency=%u timeout=%u",
			err, minimum, maximum, latency, timeout);
	} else {
		LOG_INF("Requested ACL parameters: min=%u max=%u latency=%u timeout=%u",
			minimum, maximum, latency, timeout);
		if (adjustment_callback != nullptr) {
			adjustment_callback();
		}
	}

	bt_conn_unref(conn);
}

void ConnectionIntervalStrategy::request(uint16_t minimum, uint16_t maximum, uint16_t latency,
					 uint16_t timeout)
{
	if (managed_conn == nullptr) {
		return;
	}

	struct bt_conn *old_conn;
	k_spinlock_key_t key = k_spin_lock(&interval_request.lock);
	old_conn = interval_request.conn;
	interval_request.conn = bt_conn_ref(managed_conn);
	interval_request.minimum = minimum;
	interval_request.maximum = maximum;
	interval_request.latency = latency;
	interval_request.timeout = timeout;
	k_spin_unlock(&interval_request.lock, key);

	if (old_conn != nullptr) {
		bt_conn_unref(old_conn);
	}
	(void)k_work_submit(&interval_request.work);
}

/** Advance the active strategy in thread context. */
void tick_work_handler(struct k_work *work)
{
	(void)work;
	k_mutex_lock(&engine_mutex, K_FOREVER);
	if (managed_conn != nullptr) {
		active_strategy->timer_tick(k_uptime_get());
	}
	k_mutex_unlock(&engine_mutex);
}

/** Process an ISR-originated audio-underrun report in thread context. */
void underrun_work_handler(struct k_work *work)
{
	(void)work;
	const uint32_t count = static_cast<uint32_t>(atomic_get(&latest_underrun_count));

	wireless_audio_configuration_underrun_observed(count);
	k_mutex_lock(&engine_mutex, K_FOREVER);
	if (managed_conn != nullptr) {
		active_strategy->audio_underrun(count);
	}
	k_mutex_unlock(&engine_mutex);
}

/** Move periodic timer processing from ISR context onto the system workqueue. */
void tick_timer_handler(struct k_timer *timer)
{
	(void)timer;
	(void)k_work_submit(&tick_work);
}

} // namespace

extern "C" int bt_mgmt_conn_interval_init(void)
{
	k_work_init(&interval_request.work, interval_request_handler);
	k_work_init(&tick_work, tick_work_handler);
	k_work_init(&underrun_work, underrun_work_handler);
	atomic_set(&latest_underrun_count, 0);
	k_timer_init(&tick_timer, tick_timer_handler, nullptr);
	k_timer_start(&tick_timer, K_MSEC(tick_period_ms), K_MSEC(tick_period_ms));
	return 0;
}

extern "C" int bt_mgmt_ci_policy_set(
	const wireless_audio_configuration_acl_connection_policy_t *policy)
{
	if (policy == nullptr) {
		return -EINVAL;
	}

	k_mutex_lock(&engine_mutex, K_FOREVER);
	switch (policy->type) {
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_CONTROLLER_DEFAULT_ACL_POLICY:
		active_strategy = &controller_default_strategy;
		break;
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_FIXED_ACL_POLICY:
		fixed_strategy.configure(policy->policy.fixed_acl_policy);
		active_strategy = &fixed_strategy;
		break;
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_PREFERRED_RANGE_ACL_POLICY:
		preferred_range_strategy.configure(policy->policy.preferred_range_acl_policy);
		active_strategy = &preferred_range_strategy;
		break;
	case WIRELESS_AUDIO_CONFIGURATION_ACL_CONNECTION_POLICY_ADAPTIVE_LINEAR_ACL_POLICY:
		adaptive_linear_strategy.configure(policy->policy.adaptive_linear_acl_policy);
		active_strategy = &adaptive_linear_strategy;
		break;
	default:
		k_mutex_unlock(&engine_mutex);
		return -EINVAL;
	}

	if (managed_conn != nullptr) {
		active_strategy->connected();
	}
	k_mutex_unlock(&engine_mutex);
	return 0;
}

extern "C" void bt_mgmt_ci_adjustment_callback_set(bt_mgmt_ci_adjustment_cb_t callback)
{
	adjustment_callback = callback;
}

extern "C" void bt_mgmt_ci_on_connected(struct bt_conn *conn)
{
	if (conn == nullptr) {
		return;
	}

	k_mutex_lock(&engine_mutex, K_FOREVER);
	if (managed_conn != conn) {
		if (managed_conn != nullptr) {
			bt_conn_unref(managed_conn);
		}
		managed_conn = bt_conn_ref(conn);
	}
	active_strategy->connected();
	k_mutex_unlock(&engine_mutex);
}

extern "C" void bt_mgmt_ci_on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	(void)reason;
	k_mutex_lock(&engine_mutex, K_FOREVER);
	if (managed_conn == conn) {
		active_strategy->disconnected();
		bt_conn_unref(managed_conn);
		managed_conn = nullptr;
	}
	k_mutex_unlock(&engine_mutex);
}

extern "C" void bt_mgmt_ci_on_conn_param_updated(struct bt_conn *conn, uint16_t interval,
						  uint16_t latency, uint16_t timeout)
{
	k_mutex_lock(&engine_mutex, K_FOREVER);
	if (managed_conn == conn) {
		active_strategy->parameters_updated(interval, latency, timeout);
	}
	k_mutex_unlock(&engine_mutex);
}

extern "C" void bt_mgmt_report_audio_underrun(uint32_t count)
{
	atomic_set(&latest_underrun_count, static_cast<atomic_val_t>(count));
	(void)k_work_submit(&underrun_work);
}
