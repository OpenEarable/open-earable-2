#include "AutoOffManager.h"

#include <array>
#include <cerrno>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include "PowerManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(auto_off, LOG_LEVEL_DBG);

void auto_off_work_handler(struct k_work *work);

// anonymous namespace for helpers
namespace {

struct PowerSavingModeConfig {
	const char *name;
	int timeout_minutes;
};

/* Indexed by power_saving_level_t. Keep the order in sync with the enum values. */
constexpr std::array<PowerSavingModeConfig, POWER_SAVING_LEVEL_COUNT> power_saving_modes = { {
	{ "Off", 0 },
	{ "Minimal", CONFIG_AUTO_OFF_TIMEOUT_MINIMAL },
	{ "Balanced", CONFIG_AUTO_OFF_TIMEOUT_BALANCED },
	{ "Aggressive", CONFIG_AUTO_OFF_TIMEOUT_AGGRESSIVE },
} };
static_assert(power_saving_modes.size() == POWER_SAVING_LEVEL_COUNT,
	      "power_saving_modes must contain every concrete power saving level");

constexpr const char *auto_off_settings_mode_key = "auto_off/mode";

power_saving_level_t loaded_mode = (power_saving_level_t)CONFIG_POWER_SAVING_DEFAULT_MODE;
bool loaded_mode_is_valid = false;

K_MUTEX_DEFINE(auto_off_mutex);
K_WORK_DELAYABLE_DEFINE(auto_off_work, auto_off_work_handler);

// RAII implementation for k_mutex instead of std::lock_guard<std::mutex>
class AutoOffLock {
public:
	AutoOffLock()
	{
		k_mutex_lock(&auto_off_mutex, K_FOREVER);
	}

	~AutoOffLock()
	{
		k_mutex_unlock(&auto_off_mutex);
	}

	AutoOffLock(const AutoOffLock &) = delete;
	AutoOffLock &operator=(const AutoOffLock &) = delete;
};

int auto_off_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (std::strcmp(name, "mode") != 0) {
		return -ENOENT;
	}

	if (len != sizeof(int)) {
		LOG_WRN("Ignoring auto-off mode setting with invalid size %zu", len);
		return 0;
	}

	int stored_mode;
	const int ret = read_cb(cb_arg, &stored_mode, sizeof(stored_mode));
	if (ret < 0) {
		return ret;
	}

	if (ret != sizeof(stored_mode)) {
		LOG_WRN("Ignoring incomplete auto-off mode setting read: %d", ret);
		return 0;
	}

	const auto mode = static_cast<power_saving_level_t>(stored_mode);
	if (!auto_off_mode_is_supported(mode)) {
		LOG_WRN("Ignoring invalid stored auto-off mode %d", stored_mode);
		return 0;
	}

	loaded_mode = mode;
	loaded_mode_is_valid = true;

	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(auto_off, "auto_off", nullptr, auto_off_settings_set, nullptr,
			       nullptr);

void save_auto_off_mode(power_saving_level_t mode)
{
	const int stored_mode = static_cast<int>(mode);
	const int ret = settings_save_one(auto_off_settings_mode_key, &stored_mode,
					  sizeof(stored_mode));
	if (ret) {
		LOG_WRN("Failed to persist auto-off mode %d: %d", stored_mode, ret);
	}
}

}

bool AutoOffManager::participant_is_considered(const ParticipantEntry &participant) const
{
	if (!participant.registered || current_mode == POWER_SAVING_LEVEL_OFF ||
	    participant.level == POWER_SAVING_LEVEL_OFF) {
		return false;
	}

	/*
	 * The participant level is the most aggressive power saving mode where it may veto
	 * auto-off. Example: a Balanced participant is considered in Minimal and
	 * Balanced modes, but ignored in Aggressive mode. Aggressive power saving
	 * considers fewer participant vetoes, so auto-offs are more likely.
	 */
	return participant.level >= current_mode;
}

AutoOffManager::ParticipantEntry *AutoOffManager::find_participant(const char *participant_token)
{
	for (auto &entry : participants) {
		if (entry.registered && std::strcmp(entry.token, participant_token) == 0) {
			return &entry;
		}
	}

	return nullptr;
}

bool AutoOffManager::all_considered_participants_allow() const
{
	for (const auto &participant : participants) {
		if (participant_is_considered(participant) && !participant.allowed) {
			return false;
		}
	}

	return true;
}

void AutoOffManager::schedule() const
{
	if (current_mode < POWER_SAVING_LEVEL_OFF ||
	    static_cast<size_t>(current_mode) >= power_saving_modes.size()) {
		LOG_WRN("Cannot schedule auto-off for invalid mode %d", current_mode);
		return;
	}

	const auto &mode = power_saving_modes[static_cast<size_t>(current_mode)];

	(void)k_work_reschedule(&auto_off_work, K_MINUTES(mode.timeout_minutes));
	LOG_INF("Auto-off armed for %d min in %s mode", mode.timeout_minutes, mode.name);
}

void AutoOffManager::cancel() const
{
	(void)k_work_cancel_delayable(&auto_off_work);
}

void AutoOffManager::evaluate()
{
	if (!initialized) {
		return;
	}

	if (current_mode == POWER_SAVING_LEVEL_OFF) {
		cancel();
		return;
	}

	if (all_considered_participants_allow()) {
		schedule();
	} else {
		cancel();
	}
}

int AutoOffManager::init()
{
	AutoOffLock lock;

	if (initialized) {
		return -EALREADY;
	}

	current_mode = loaded_mode_is_valid ?
		       loaded_mode :
		       (power_saving_level_t)CONFIG_POWER_SAVING_DEFAULT_MODE;

	if (!auto_off_mode_is_supported(current_mode)) {
		LOG_WRN("Invalid default auto-off mode %d, disabling auto-off", current_mode);
		current_mode = POWER_SAVING_LEVEL_OFF;
	}

	const auto &mode = power_saving_modes[static_cast<size_t>(current_mode)];

	initialized = true;

	LOG_INF("Auto-off initialized in %s mode", mode.name);
	evaluate();

	return 0;
}

int AutoOffManager::register_participant(const char *participant_token, power_saving_level_t level)
{
	if (participant_token == nullptr || level <= POWER_SAVING_LEVEL_OFF ||
	    static_cast<size_t>(level) >= power_saving_modes.size()) {
		return -EINVAL;
	}

	const auto &level_config = power_saving_modes[static_cast<size_t>(level)];

	AutoOffLock lock;

	ParticipantEntry *entry = find_participant(participant_token);
	if (entry != nullptr) {
		entry->level = level;
		evaluate();
		return -EALREADY;
	}

	ParticipantEntry *free_participant = nullptr;
	for (auto &participant : participants) {
		if (!participant.registered) {
			free_participant = &participant;
			break;
		}
	}

	if (free_participant == nullptr) {
		LOG_WRN("No room to register auto-off participant token %s", participant_token);
		return -ENOMEM;
	}

	free_participant->token = participant_token;
	free_participant->level = level;
	free_participant->allowed = false;
	free_participant->registered = true;

	LOG_INF("Registered auto-off participant token %s at %s level",
		participant_token, level_config.name);
	evaluate();

	return 0;
}

void AutoOffManager::allow(const char *participant_token)
{
	if (participant_token == nullptr) {
		return;
	}

	AutoOffLock lock;

	ParticipantEntry *entry = find_participant(participant_token);
	if (entry == nullptr) {
		LOG_WRN("Auto-off allow from unregistered participant token %s",
			participant_token);
		return;
	}

	if (!entry->allowed) {
		entry->allowed = true;
		LOG_DBG("Auto-off allowed by token %s", participant_token);
		evaluate();
	}
}

void AutoOffManager::prohibit(const char *participant_token)
{
	if (participant_token == nullptr) {
		return;
	}

	AutoOffLock lock;

	ParticipantEntry *entry = find_participant(participant_token);
	if (entry == nullptr) {
		LOG_WRN("Auto-off prohibit from unregistered participant token %s",
			participant_token);
		return;
	}

	if (entry->allowed) {
		entry->allowed = false;
		LOG_DBG("Auto-off prohibited by token %s", participant_token);
		evaluate();
	}
}

void AutoOffManager::set_mode(power_saving_level_t mode)
{
	if (!auto_off_mode_is_supported(mode)) {
		LOG_WRN("Ignoring invalid auto-off mode %d", mode);
		return;
	}

	const auto &mode_config = power_saving_modes[static_cast<size_t>(mode)];
	bool mode_changed = false;

	{
		AutoOffLock lock;

		if (current_mode != mode) {
			current_mode = mode;
			mode_changed = true;
			LOG_INF("Auto-off mode set to %s", mode_config.name);
			evaluate();
		}
	}

	if (mode_changed) {
		save_auto_off_mode(mode);
	}
}

power_saving_level_t AutoOffManager::get_mode()
{
	AutoOffLock lock;

	return current_mode;
}

void AutoOffManager::handle_timeout()
{
	bool should_power_down;

	{
		AutoOffLock lock;
		should_power_down = initialized && current_mode != POWER_SAVING_LEVEL_OFF &&
				    all_considered_participants_allow();
	}

	if (!should_power_down) {
		LOG_DBG("Auto-off skipped because a participant now prohibits it");
		return;
	}

	LOG_INF("Auto-off timeout reached");
	power_manager.power_down();
}

void auto_off_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	auto_off_manager.handle_timeout();
}

int auto_off_init(void)
{
	return auto_off_manager.init();
}

int auto_off_register_participant(const char *participant_token, power_saving_level_t level)
{
	return auto_off_manager.register_participant(participant_token, level);
}

void auto_off_allow(const char *participant_token)
{
	auto_off_manager.allow(participant_token);
}

void auto_off_prohibit(const char *participant_token)
{
	auto_off_manager.prohibit(participant_token);
}

void auto_off_set_mode(power_saving_level_t mode)
{
	auto_off_manager.set_mode(mode);
}

power_saving_level_t auto_off_get_mode(void)
{
	return auto_off_manager.get_mode();
}

uint8_t auto_off_get_supported_mode_count(void)
{
	return static_cast<uint8_t>(power_saving_modes.size());
}

const char *auto_off_get_mode_name(power_saving_level_t mode)
{
	if (!auto_off_mode_is_supported(mode)) {
		return nullptr;
	}

	return power_saving_modes[static_cast<size_t>(mode)].name;
}

int auto_off_mode_is_supported(power_saving_level_t mode)
{
	return mode >= POWER_SAVING_LEVEL_OFF &&
	       static_cast<size_t>(mode) < power_saving_modes.size();
}

AutoOffManager auto_off_manager;
