#ifndef _CONN_INTVL_LINEAR_H
#define _CONN_INTVL_LINEAR_H

#include "bt_mgmt_conn_interval.h"

class ConnIntvlLinear : public BtMgmtConnIntervalStrategy {
public:
  ConnIntvlLinear(uint16_t inc_step_units,
                  uint16_t min_interval_units = CONFIG_BLE_ACL_CONN_INTERVAL,
                  uint16_t max_interval_units = CONFIG_BLE_ACL_CONN_INTERVAL_SLOW);

  void init() override;

  void on_audio_underrun(uint32_t count) override;

  void on_timer_tick(k_timeout_t elapsed) override;

private:
  uint16_t inc_step_units_;
  uint16_t min_interval_units_;
  uint16_t max_interval_units_;
};

#endif // _CONN_INTVL_LINEAR_H