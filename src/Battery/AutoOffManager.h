#ifndef _AUTO_OFF_MANAGER_H
#define _AUTO_OFF_MANAGER_H

#include <zephyr/kernel.h>

struct zbus_channel;

/**
 * @brief Manages automatic power-off after an idle timeout.
 *
 * The manager listens for LE audio streaming events and schedules or cancels
 * a delayed power-down accordingly.
 * TODO: Extend this description as soon as the class is fully implemented
 */
class AutoOffManager {
public:
    /**
     * @brief Register listeners and start auto-off scheduling.
     *
     * @return 0 on success, -EALREADY if already registered, or another
     * negative errno code on failure.
     */
    int init();

    /**
     * @brief Arm or re-arm the idle auto-off timer.
     */
    void schedule();

    /**
     * @brief Cancel any pending idle auto-off timer.
     */
    void cancel();
private:
    static k_work_delayable auto_off_work;
    static void auto_off_work_handler(struct k_work *work);
};

/**
 * @brief Global auto-off manager instance.
 *
 * AutoOffManager is intended to be used as a single instance in the system.
 * Do not create additional instances.
 */
extern AutoOffManager auto_off_manager;

#endif
