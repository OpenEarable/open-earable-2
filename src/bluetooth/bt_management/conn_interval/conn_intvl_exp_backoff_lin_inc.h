#ifndef _CONN_INTVL_EXP_BACKOFF_H
#define _CONN_INTVL_EXP_BACKOFF_H

#include "bt_mgmt_conn_interval.h"

class ConnIntvlExpBackoffLinIncr : public BtMgmtConnIntervalStrategy {
public:
  ConnIntvlExpBackoffLinIncr(uint8_t backoff_factor,
                              k_timeout_t inc_timeout,
                              uint16_t inc_step_units,
                              uint16_t min_interval_units = CONFIG_BLE_ACL_CONN_INTERVAL,
                              uint16_t max_interval_units = 3200);

  void init() override;

  void on_audio_underrun(uint32_t count) override;

  void on_timer_tick(k_timeout_t elapsed) override;

private:
  uint8_t backoff_factor_;
  k_timeout_t inc_timeout_;
  uint16_t inc_step_units_;
  uint16_t min_interval_units_;
  uint16_t max_interval_units_;
};

#endif // _CONN_INTVL_EXP_BACKOFF_H