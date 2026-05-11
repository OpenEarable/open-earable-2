#include "AutoOffManager.h"

#include <array>
#include <cerrno>
#include <cstring>

#include <zephyr/kernel.h>
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
	{ "off", 0 },
	{ "minimal", CONFIG_AUTO_OFF_TIMEOUT_MINIMAL },
	{ "balanced", CONFIG_AUTO_OFF_TIMEOUT_BALANCED },
	{ "aggressive", CONFIG_AUTO_OFF_TIMEOUT_AGGRESSIVE },
} };
static_assert(power_saving_modes.size() == POWER_SAVING_LEVEL_COUNT,
	      "power_saving_modes must contain every concrete power saving level");

K_MUTEX_DEFINE(auto_off_mutex);
K_WORK_DELAYABLE_DEFINE(auto_off_work, auto_off_work_handler);

// RAII implementation for k_mutext instead of std::lock_guard<std::mutex>
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

}

bool AutoOffManager::participant_is_considered(const ParticipantEntry &participant) const
{
	if (!participant.registered || current_mode == POWER_SAVING_LEVEL_OFF ||
	    participant.level == POWER_SAVING_LEVEL_OFF) {
		return false;
	}

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

	current_mode = (power_saving_level_t)CONFIG_AUTO_OFF_DEFAULT_POWER_SAVING_MODE;

	if (current_mode < POWER_SAVING_LEVEL_OFF ||
	    static_cast<size_t>(current_mode) >= power_saving_modes.size()) {
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
	if (participant_token == nullptr || level < POWER_SAVING_LEVEL_OFF ||
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
	if (mode < POWER_SAVING_LEVEL_OFF ||
	    static_cast<size_t>(mode) >= power_saving_modes.size()) {
		LOG_WRN("Ignoring invalid auto-off mode %d", mode);
		return;
	}

	const auto &mode_config = power_saving_modes[static_cast<size_t>(mode)];

	AutoOffLock lock;

	if (current_mode != mode) {
		current_mode = mode;
		LOG_INF("Auto-off mode set to %s", mode_config.name);
		evaluate();
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

AutoOffManager auto_off_manager;
