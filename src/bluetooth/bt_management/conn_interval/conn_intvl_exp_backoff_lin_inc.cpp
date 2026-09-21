#include "conn_intvl_exp_backoff_lin_inc.h"
#include "streamctrl.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(conn_intvl_exp, LOG_LEVEL_DBG);


ConnIntvlExpBackoffLinIncr::ConnIntvlExpBackoffLinIncr(uint8_t backoff_factor,
                                                       uint16_t inc_step_units,
                                                       uint16_t min_interval_units,
                                                       uint16_t max_interval_units): backoff_factor_(backoff_factor),
                                                                                 inc_step_units_(inc_step_units),
                                                                                 min_interval_units_(min_interval_units),
                                                                                 max_interval_units_(max_interval_units) {
}

void ConnIntvlExpBackoffLinIncr::init() {
    // nothing to do
}

void ConnIntvlExpBackoffLinIncr::on_audio_underrun(uint32_t count) {
    uint16_t current_interval_units_ = this->current_interval_units();

    LOG_DBG("Audio underrun reported: %u, current interval: %u units", count, current_interval_units_);
    uint16_t new_interval_units = current_interval_units_ * backoff_factor_;
    if (new_interval_units > max_interval_units_) {
        new_interval_units = max_interval_units_;
    }
    LOG_DBG("Requesting new interval: %u units", new_interval_units);
    this->request_interval(new_interval_units, new_interval_units);
}

void ConnIntvlExpBackoffLinIncr::on_timer_tick(k_timeout_t elapsed) {
    bool is_streaming = stream_state_get() == STATE_STREAMING;
    if (!is_streaming) {
        return;
    }

    uint16_t new_interval_units = this->current_interval_units() - inc_step_units_;
    if (new_interval_units < min_interval_units_) {
        new_interval_units = min_interval_units_;
    }
    this->request_interval(new_interval_units, new_interval_units);
}