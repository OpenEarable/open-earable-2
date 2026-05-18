#ifndef _AUTO_OFF_MANAGER_H
#define _AUTO_OFF_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Power saving levels used by AutoOffManager.
 *
 * Numeric values intentionally match the Kconfig settings:
 * 0=off, 1=minimal, 2=balanced, 3=aggressive.
 */
typedef enum power_saving_level {
	POWER_SAVING_LEVEL_OFF = 0,
	POWER_SAVING_LEVEL_MINIMAL = 1,
	POWER_SAVING_LEVEL_BALANCED = 2,
	POWER_SAVING_LEVEL_AGGRESSIVE = 3,
	/** Number of concrete levels. Not a selectable power saving mode. */
	POWER_SAVING_LEVEL_COUNT,
} power_saving_level_t;

/**
 * @brief C wrapper for AutoOffManager::init().
 *
 * See AutoOffManager::init() for documentation.
 */
int auto_off_init(void);

/**
 * @brief C wrapper for AutoOffManager::register_participant().
 *
 * See AutoOffManager::register_participant() for documentation.
 */
int auto_off_register_participant(const char *participant_token, power_saving_level_t level);

/**
 * @brief C wrapper for AutoOffManager::allow().
 *
 * See AutoOffManager::allow() for documentation.
 */
void auto_off_allow(const char *participant_token);

/**
 * @brief C wrapper for AutoOffManager::prohibit().
 *
 * See AutoOffManager::prohibit() for documentation.
 */
void auto_off_prohibit(const char *participant_token);

/**
 * @brief C wrapper for AutoOffManager::set_mode().
 *
 * See AutoOffManager::set_mode() for documentation.
 */
void auto_off_set_mode(power_saving_level_t mode);

/**
 * @brief C wrapper for AutoOffManager::get_mode().
 *
 * See AutoOffManager::get_mode() for documentation.
 */
power_saving_level_t auto_off_get_mode(void);

/**
 * @brief Get the number of selectable power saving modes.
 *
 * @return Number of selectable modes in the firmware-defined mode table.
 */
uint8_t auto_off_get_supported_mode_count(void);

/**
 * @brief Get the display name for a selectable power saving mode.
 *
 * @param mode Power saving mode identifier.
 *
 * @return Null-terminated mode name, or NULL if @p mode is not supported.
 */
const char *auto_off_get_mode_name(power_saving_level_t mode);

/**
 * @brief Check whether a power saving mode identifier is supported.
 *
 * @param mode Power saving mode identifier.
 *
 * @return 1 if @p mode is supported, otherwise 0.
 */
int auto_off_mode_is_supported(power_saving_level_t mode);

#ifdef __cplusplus
}

#include <array>

struct k_work;

/**
 * @brief Coordinates automatic power-off across dynamically registered participants.
 *
 * The manager uses a participant-based permission model. Firmware components
 * that may need to keep the device awake register themselves as auto-off
 * participants by providing a unique token and a power saving level. The level
 * defines the first power saving mode in which the participant is considered by
 * the auto-off decision. A participant registered for a higher level can still
 * prevent auto-off in more aggressive modes, but is ignored in less aggressive
 * modes.
 *
 * Once registered, a participant calls prohibit() whenever it enters a critical
 * region where power-off would be unsafe or disruptive, for example while
 * writing data, streaming audio, or handling a time-sensitive operation. When
 * the participant leaves that region, it calls allow(). Auto-off is armed only
 * when every participant that is relevant for the current power saving mode is
 * currently allowing power-off. If any relevant participant prohibits auto-off,
 * the pending auto-off work is cancelled until the system is allowed again.
 *
 * The current power saving mode controls both which participants are considered
 * and which timeout is used before power_down() is called. POWER_SAVING_LEVEL_OFF
 * disables auto-off. 
 *
 * Implementation notes:
 * - AutoOffManager is used as a process-wide singleton via auto_off_manager.
 *   The firmware should not create additional instances.
 * - Public C++ methods have thin C wrappers such as auto_off_allow() and
 *   auto_off_register_participant() so C firmware modules can use the same
 *   manager without depending on C++ linkage.
 */
class AutoOffManager {
public:
	/**
	 * @brief Initialize the manager and load the default mode from Kconfig.
	 *
	 * @return 0 on success or -EALREADY if already initialized.
	 */
	int init();

	/**
	 * @brief Register an auto-off participant.
	 *
	 * @param participant_token Unique, stable token identifying the participant.
	 * @param level First power saving mode where this participant participates.
	 * Higher level means the participant can prevent an auto-off even at a more
	 * aggressive power saving mode. POWER_SAVING_LEVEL_OFF is not a valid
	 * participant level.
	 *
	 * @return 0 on success, -EINVAL for invalid arguments, -EALREADY if the
	 * token was already registered, or -ENOMEM if the registry is full.
	 */
	int register_participant(const char *participant_token, power_saving_level_t level);

	/**
	 * @brief Mark a registered participant as allowing auto-off.
	 *
	 * @param participant_token The same unique token used for registration.
	 */
	void allow(const char *participant_token);

	/**
	 * @brief Mark a registered participant as prohibiting auto-off.
	 *
	 * @param participant_token The same unique token used for registration.
	 */
	void prohibit(const char *participant_token);

	/**
	 * @brief Set the current power saving mode.
	 *
	 * @param mode New power saving mode.
	 */
	void set_mode(power_saving_level_t mode);

	/**
	 * @brief Get the current power saving mode.
	 *
	 * @return Current power saving mode.
	 */
	power_saving_level_t get_mode();

private:
	static constexpr int max_participants = 8;

	struct ParticipantEntry {
		const char *token = nullptr;
		power_saving_level_t level = POWER_SAVING_LEVEL_OFF;
		bool allowed = false;
		bool registered = false;
	};

	friend void auto_off_work_handler(struct k_work *work);

	ParticipantEntry *find_participant(const char *participant_token);
	bool participant_is_considered(const ParticipantEntry &entry) const;
	bool all_considered_participants_allow() const;
	void schedule() const;
	void cancel() const;
	void evaluate();
	void handle_timeout();

	std::array<ParticipantEntry, max_participants> participants{};
	power_saving_level_t current_mode = POWER_SAVING_LEVEL_OFF;
	bool initialized = false;
};

/** @brief Global AutoOffManager singleton. */
extern AutoOffManager auto_off_manager;
#endif

#endif
