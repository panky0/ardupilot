#pragma once

#include "AP_BattMonitor_Backend.h"

#define AP_BATTMON_MAVLINK_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH)

#if AP_BATTMON_MAVLINK_ENABLED
#include <AP_HAL/Semaphores.h>

#define AP_BATTMONITOR_MAVLINK_TIMEOUT_MICROS        (AP_BATT_MONITOR_TIMEOUT * 1000UL)

class AP_BattMonitor_MAVLink : public AP_BattMonitor_Backend
{
public:
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    // void init(void) override {};

    /* read latest values received from mavlink */
    void read(void) override;

    /* returns true if battery monitor instance provides consumed energy info */
    bool has_consumed_energy() const override { return true; }

    /* returns true if battery monitor instance provides current info */
    bool has_current() const override { return true; }

    /* returns true if battery monitor provides individual cell voltages */
    bool has_cell_voltages() const override { return true; }

    /* returns true if battery monitor instance provides time remaining info */
    bool has_time_remaining() const override { return _state.time_remaining != 0; }

    /* capacity_remaining_pct - returns the % battery capacity if provided */
    bool capacity_remaining_pct(uint8_t &cap_remain_pct) const override;

    bool has_temperature() const override { return _has_temperature; }

    bool get_cycle_count(uint16_t &cycles) const override;

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override { return _fault_bitmask; }

    /* don't allow reset of remaining capacity */
    bool reset_remaining(float percentage) override { return false; }

    // handle main mavlink msg
    bool handle_message(const mavlink_message_t &msg) override;

private:

    bool allow_packet(const uint8_t sysid, const uint8_t compid, const uint8_t battery_id) const;

    /* process specific mavlink msg */
    void handle_message_smart_battery_info(const mavlink_smart_battery_info_t &msg);
    void handle_message_battery_status(const mavlink_battery_status_t &msg);

    HAL_Semaphore   _sem_battmon;
    bool            _has_temperature;
    int8_t          _capacity_rem_pct;
    uint16_t        _cycle_count;
    uint32_t        _fault_bitmask;

    AP_BattMonitor::BattMonitor_State _interim_state;
    mavlink_smart_battery_info_t _msg_info;
};

#endif // AP_BATTMON_MAVLINK_ENABLED