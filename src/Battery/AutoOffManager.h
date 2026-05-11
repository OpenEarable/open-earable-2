#ifndef _AUTO_OFF_MANAGER_H
#define _AUTO_OFF_MANAGER_H

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

#ifdef __cplusplus
}

#include <array>

struct k_work;

/**
 * @brief Coordinates automatic power-off across dynamically registered participants.
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
     * aggressive power saving mode
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
