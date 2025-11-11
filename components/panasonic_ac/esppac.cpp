#include "esppac.h"

#include "esphome/core/log.h"

namespace esphome {
namespace panasonic_ac {

static const char *const TAG = "panasonic_ac";

climate::ClimateTraits PanasonicAC::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_supports_action(false);

  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(false);
  traits.set_visual_min_temperature(MIN_TEMPERATURE);
  traits.set_visual_max_temperature(MAX_TEMPERATURE);
  traits.set_visual_temperature_step(TEMPERATURE_STEP);

  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL,
                              climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY});

  traits.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH});

  traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH,
                                    climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL});

  traits.set_supported_presets({climate::CLIMATE_PRESET_COMFORT, climate::CLIMATE_PRESET_BOOST, climate::CLIMATE_PRESET_ECO});

  return traits;
}

void PanasonicAC::setup() {
  // Initialize times
  this->init_time_ = millis();
  this->last_packet_sent_ = millis();

  ESP_LOGI(TAG, "Panasonic AC component v%s starting...", VERSION);
}

void PanasonicAC::loop() {
  read_data();  // Read data from UART (if there is any)

  // Check and apply temperature compensation periodically
  update_temperature_compensation();
}

void PanasonicAC::read_data() {
  while (available())  // Read while data is available
  {
    // if (this->receive_buffer_index >= BUFFER_SIZE) {
    //   ESP_LOGE(TAG, "Receive buffer overflow");
    //   receiveBufferIndex = 0;
    // }

    uint8_t c;
    this->read_byte(&c);  // Store in receive buffer
    this->rx_buffer_.push_back(c);

    this->last_read_ = millis();  // Update lastRead timestamp
  }
}

void PanasonicAC::update_outside_temperature(int8_t temperature) {
  ESP_LOGV(TAG, "Received outside temperature %d", temperature);
  temperature += this->outside_temperature_offset_;

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range outside temperature: %d", temperature);
    return;
  }

  if (this->outside_temperature_sensor_ != nullptr && this->outside_temperature_sensor_->state != temperature) {
    this->outside_temperature_sensor_->publish_state(
        temperature);  // Set current (outside) temperature; no temperature steps
    ESP_LOGV(TAG, "Outside temperature incl. offset: %d", temperature);
  }
}

void PanasonicAC::update_current_temperature(int8_t temperature) {
  ESP_LOGV(TAG, "Received current temperature %d", temperature);
  temperature += this->current_temperature_offset_;

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range inside temperature: %d", temperature);
    return;
  }

  this->current_temperature = temperature;
  ESP_LOGV(TAG, "Current temperature incl. offset: %d", temperature);
}

void PanasonicAC::update_target_temperature(uint8_t raw_value) {
  float temperature = raw_value * TEMPERATURE_STEP;
  ESP_LOGV(TAG, "Received target temperature %.2f", temperature);

  temperature += this->current_temperature_offset_;

  if (temperature > TEMPERATURE_THRESHOLD) {
    ESP_LOGW(TAG, "Received out of range target temperature %.2f", temperature);
    return;
  }

  // When external compensation is enabled, don't update the displayed target
  // The displayed target should always be what the USER wants, not what the AC has
  if (this->external_compensation_enabled_) {
    ESP_LOGV(TAG, "Target temperature from AC: %.2f (ignored - using user target: %.2f)", 
             temperature, this->user_target_temperature_);
    // Update target_temperature to user's desired value for display
    this->target_temperature = this->user_target_temperature_;
  } else {
    // Normal mode: show what the AC has
    this->target_temperature = temperature;
    ESP_LOGV(TAG, "Target temperature incl. offset: %.2f", temperature);
  }
}

void PanasonicAC::update_swing_horizontal(const std::string &swing) {
  this->horizontal_swing_state_ = swing;

  if (this->horizontal_swing_select_ != nullptr &&
      this->horizontal_swing_select_->state != this->horizontal_swing_state_) {
    this->horizontal_swing_select_->publish_state(
        this->horizontal_swing_state_);  // Set current horizontal swing position
  }
}

void PanasonicAC::update_swing_vertical(const std::string &swing) {
  this->vertical_swing_state_ = swing;

  if (this->vertical_swing_select_ != nullptr && this->vertical_swing_select_->state != this->vertical_swing_state_)
    this->vertical_swing_select_->publish_state(this->vertical_swing_state_);  // Set current vertical swing position
}

void PanasonicAC::update_nanoex(bool nanoex) {
  if (this->nanoex_switch_ != nullptr) {
    this->nanoex_state_ = nanoex;
    this->nanoex_switch_->publish_state(this->nanoex_state_);
  }
}

void PanasonicAC::update_eco(bool eco) {
  if (this->eco_switch_ != nullptr) {
    this->eco_state_ = eco;
    this->eco_switch_->publish_state(this->eco_state_);
  }
}

void PanasonicAC::update_econavi(bool econavi) {
  if (this->econavi_switch_ != nullptr) {
    this->econavi_state_ = econavi;
    this->econavi_switch_->publish_state(this->econavi_state_);
  }
}

void PanasonicAC::update_mild_dry(bool mild_dry) {
  if (this->mild_dry_switch_ != nullptr) {
    this->mild_dry_state_ = mild_dry;
    this->mild_dry_switch_->publish_state(this->mild_dry_state_);
  }
}

climate::ClimateAction PanasonicAC::determine_action() {
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    return climate::CLIMATE_ACTION_OFF;
  } else if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) {
    return climate::CLIMATE_ACTION_FAN;
  } else if (this->mode == climate::CLIMATE_MODE_DRY) {
    return climate::CLIMATE_ACTION_DRYING;
  } else if ((this->mode == climate::CLIMATE_MODE_COOL || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
             this->current_temperature + TEMPERATURE_TOLERANCE >= this->target_temperature) {
    return climate::CLIMATE_ACTION_COOLING;
  } else if ((this->mode == climate::CLIMATE_MODE_HEAT || this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
             this->current_temperature - TEMPERATURE_TOLERANCE <= this->target_temperature) {
    return climate::CLIMATE_ACTION_HEATING;
  } else {
    return climate::CLIMATE_ACTION_IDLE;
  }
}

void PanasonicAC::update_current_power_consumption(int16_t power) {
  if (this->current_power_consumption_sensor_ != nullptr && this->current_power_consumption_sensor_->state != power) {
    this->current_power_consumption_sensor_->publish_state(
        power);  // Set current power consumption
  }
}

/*
 * Sensor handling
 */

void PanasonicAC::set_outside_temperature_sensor(sensor::Sensor *outside_temperature_sensor) {
  this->outside_temperature_sensor_ = outside_temperature_sensor;
}

void PanasonicAC::set_outside_temperature_offset(int8_t outside_temperature_offset) {
  ESP_LOGV(TAG, "Outside temperature offset %d", outside_temperature_offset);
  this->outside_temperature_offset_ = outside_temperature_offset;

  if (this->outside_temperature_sensor_) {
    ESP_LOGV(TAG, "Corrected outside temperature: %d", this->outside_temperature_sensor_->state + outside_temperature_offset);
  }
}

void PanasonicAC::set_current_temperature_offset(int8_t current_temperature_offset)
{
  ESP_LOGV(TAG, "Current temperature offset %d", current_temperature_offset);
  this->current_temperature_offset_ = current_temperature_offset;

  if (this->current_temperature_sensor_) {
    ESP_LOGV(TAG, "Corrected current temperature: %d", this->current_temperature_sensor_->state + current_temperature_offset);
  }
}

void PanasonicAC::set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor)
{
  this->current_temperature_sensor_ = current_temperature_sensor;
  this->current_temperature_sensor_->add_on_state_callback([this](float state)
                                                           {
                                                             this->current_temperature = state + this->current_temperature_offset_;
                                                             this->publish_state();
                                                           });
}

void PanasonicAC::set_vertical_swing_select(select::Select *vertical_swing_select) {
  this->vertical_swing_select_ = vertical_swing_select;
  this->vertical_swing_select_->add_on_state_callback([this](const std::string &value, size_t index) {
    if (value == this->vertical_swing_state_)
      return;
    this->on_vertical_swing_change(value);
  });
}

void PanasonicAC::set_horizontal_swing_select(select::Select *horizontal_swing_select) {
  this->horizontal_swing_select_ = horizontal_swing_select;
  this->horizontal_swing_select_->add_on_state_callback([this](const std::string &value, size_t index) {
    if (value == this->horizontal_swing_state_)
      return;
    this->on_horizontal_swing_change(value);
  });
}

void PanasonicAC::set_nanoex_switch(switch_::Switch *nanoex_switch) {
  this->nanoex_switch_ = nanoex_switch;
  this->nanoex_switch_->add_on_state_callback([this](bool state) {
    if (state == this->nanoex_state_)
      return;
    this->on_nanoex_change(state);
  });
}

void PanasonicAC::set_eco_switch(switch_::Switch *eco_switch) {
  this->eco_switch_ = eco_switch;
  this->eco_switch_->add_on_state_callback([this](bool state) {
    if (state == this->eco_state_)
      return;
    this->on_eco_change(state);
  });
}

void PanasonicAC::set_econavi_switch(switch_::Switch *econavi_switch) {
  this->econavi_switch_ = econavi_switch;
  this->econavi_switch_->add_on_state_callback([this](bool state) {
    if (state == this->econavi_state_)
      return;
    this->on_econavi_change(state);
  });
}

void PanasonicAC::set_mild_dry_switch(switch_::Switch *mild_dry_switch) {
  this->mild_dry_switch_ = mild_dry_switch;
  this->mild_dry_switch_->add_on_state_callback([this](bool state) {
    if (state == this->mild_dry_state_)
      return;
    this->on_mild_dry_change(state);
  });
}

void PanasonicAC::set_current_power_consumption_sensor(sensor::Sensor *current_power_consumption_sensor) {
  this->current_power_consumption_sensor_ = current_power_consumption_sensor;
}

/*
 * External temperature compensation
 */

void PanasonicAC::set_external_temperature_sensor(sensor::Sensor *external_temperature_sensor) {
  this->external_temperature_sensor_ = external_temperature_sensor;
  ESP_LOGI(TAG, "External temperature sensor configured");

  // Set up callback to trigger compensation when external temp changes
  if (external_temperature_sensor != nullptr) {
    external_temperature_sensor->add_on_state_callback([this](float state) {
      ESP_LOGD(TAG, "External temperature sensor callback: %.1f°C (compensation %s)", 
               state, this->external_compensation_enabled_ ? "ENABLED" : "DISABLED");
      if (this->external_compensation_enabled_) {
        // Force immediate compensation check on sensor update
        this->last_compensation_update_ = 0;
      }
    });
  }
}

void PanasonicAC::set_external_temperature_compensation_enabled(bool enabled) {
  this->external_compensation_enabled_ = enabled;
  if (enabled) {
    ESP_LOGI(TAG, "✓ External temperature compensation: ENABLED");
  } else {
    ESP_LOGI(TAG, "External temperature compensation: DISABLED");
  }
}

void PanasonicAC::set_compensation_dampening_factor(float factor) {
  // Clamp between 0.0 and 1.0
  this->compensation_dampening_factor_ = std::max(0.0f, std::min(1.0f, factor));
  ESP_LOGI(TAG, "Compensation dampening factor: %.2f (%.0f%% correction)", 
           this->compensation_dampening_factor_, this->compensation_dampening_factor_ * 100);
}

void PanasonicAC::set_compensation_update_interval(uint32_t interval_ms) {
  this->compensation_update_interval_ = interval_ms;
  ESP_LOGI(TAG, "Compensation update interval: %u ms (%.1f min)", interval_ms, interval_ms / 60000.0f);
}

void PanasonicAC::update_temperature_compensation() {
  // Only run if enabled and we have an external sensor
  if (!this->external_compensation_enabled_) {
    ESP_LOGVV(TAG, "Compensation skipped: not enabled");
    return;
  }
  
  if (this->external_temperature_sensor_ == nullptr) {
    ESP_LOGVV(TAG, "Compensation skipped: no external sensor");
    return;
  }

  // Rate limiting: only update every X minutes
  uint32_t now = millis();
  uint32_t time_since_last = now - this->last_compensation_update_;
  if (time_since_last < this->compensation_update_interval_) {
    ESP_LOGVV(TAG, "Compensation skipped: timer (%.1f/%.1f min)", 
             time_since_last / 60000.0f, this->compensation_update_interval_ / 60000.0f);
    return;
  }
  this->last_compensation_update_ = now;

  // Get external temperature
  float external_temp = this->external_temperature_sensor_->state;

  ESP_LOGD(TAG, "Temperature compensation check:");
  ESP_LOGD(TAG, "  External sensor reading: %.1f°C", external_temp);
  ESP_LOGD(TAG, "  User target temperature: %.1f°C", this->user_target_temperature_);
  ESP_LOGD(TAG, "  Current AC target: %.1f°C", this->target_temperature);

  // Validate sensor reading
  if (std::isnan(external_temp) || external_temp < -10 || external_temp > 50) {
    ESP_LOGW(TAG, "Invalid external temperature reading: %.1f°C - skipping compensation", external_temp);
    return;
  }

  // Calculate temperature error
  float temp_error = external_temp - this->user_target_temperature_;

  // Calculate compensated AC target (proportional control)
  // If room is 2°C too warm, set AC target 2°C lower (with dampening)
  float compensation = temp_error * this->compensation_dampening_factor_;
  this->calculated_ac_target_ = this->user_target_temperature_ - compensation;

  ESP_LOGD(TAG, "  Temperature error: %.1f°C", temp_error);
  ESP_LOGD(TAG, "  Compensation (×%.2f): %.1f°C", this->compensation_dampening_factor_, compensation);
  ESP_LOGD(TAG, "  Calculated AC target: %.1f°C", this->calculated_ac_target_);

  // Clamp to AC limits
  this->calculated_ac_target_ = std::max(
      static_cast<float>(MIN_TEMPERATURE),
      std::min(static_cast<float>(MAX_TEMPERATURE), this->calculated_ac_target_));

  // Round to AC's temperature step (0.5°C)
  this->calculated_ac_target_ = std::round(this->calculated_ac_target_ / TEMPERATURE_STEP) * TEMPERATURE_STEP;

  ESP_LOGD(TAG, "  After clamping & rounding: %.1f°C", this->calculated_ac_target_);

  // Only send update if significantly different (> 0.5°C hysteresis)
  float delta = std::abs(this->target_temperature - this->calculated_ac_target_);
  if (delta > 0.5f) {
    ESP_LOGI(TAG, "🎯 COMPENSATION APPLIED: Ext=%.1f°C, Target=%.1f°C, Error=%.1f°C → AC: %.1f°C → %.1f°C",
             external_temp, this->user_target_temperature_, temp_error, this->target_temperature,
             this->calculated_ac_target_);

    // Update the AC target (this will be sent in next control cycle)
    this->target_temperature = this->calculated_ac_target_;

    // Trigger control update
    climate::ClimateCall call = this->make_call();
    call.set_target_temperature(this->calculated_ac_target_);
    call.perform();
  } else {
    ESP_LOGD(TAG, "  ✓ No adjustment needed (delta %.1f°C < 0.5°C threshold)", delta);
  }
}

/*
 * Debugging
 */

void PanasonicAC::log_packet(std::vector<uint8_t> data, bool outgoing) {
  if (outgoing) {
    ESP_LOGV(TAG, "TX: %s", format_hex_pretty(data).c_str());
  } else {
    ESP_LOGV(TAG, "RX: %s", format_hex_pretty(data).c_str());
  }
}

}  // namespace panasonic_ac
}  // namespace esphome
