#include "dallas_temp_searcher.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include <cstring>

namespace esphome {
namespace dallas_temp_searcher {

static const char *const TAG = "dallas.temp.searcher";

void DallasTemperatureSearcher::setup() {
  if (this->bus_ == nullptr)
    return;

  const std::vector<uint64_t> &addresses = this->bus_->get_devices();

  if (search_mode_ == SearchMode::ALL) {
    this->sensors_params_.reserve(addresses.size());
    this->sensors_.reserve(addresses.size());

    for (const uint64_t &address : addresses) {
      add_sensor_(address);
    }
    return;
  }

  // search_mode_ == SearchMode::ADDRESS_MAP

  uint8_t current_work_sensors_num = this->max_sensors_num_;

  this->sensors_params_.reserve(current_work_sensors_num);
  this->sensors_.reserve(current_work_sensors_num);
  this->saved_addresses_.reserve(current_work_sensors_num);
  this->addresses_pref_.reserve(current_work_sensors_num);

  uint32_t hash = fnv1_hash(std::string("_dallas_temp_searcher_test"));
  this->sensors_count_pref_ = global_preferences->make_preference<uint8_t>(hash, true);
  this->restore_sensors_count_();

  // need restore addresses

  // Создаем ESPPreferenceObject по максимальному количеству
  for (uint8_t i = 0; i != current_work_sensors_num; i++) {
    ESPPreferenceObject address_data_perf = global_preferences->make_preference<uint64_t>(hash + i + 1, true);
    this->addresses_pref_.push_back(address_data_perf);
  }

  // Восстанавливаем сколько сохранено адресов
  for (uint8_t i = 0; i != std::min(this->saved_sensors_num_, this->max_sensors_num_); i++) {
    if (!restore_address_data_(addresses_pref_[i])) {
      this->saved_sensors_num_ = i;
      break;
    }
  }

  // Здесь получили вектор сохраненных значений

  uint8_t i = 0;
  // Первый проход - добавление всех адресов из сохраненных и добавление nullptr отсутствующих
  for (uint64_t &address : saved_addresses_) {
    auto it = std::find(addresses.begin(), addresses.end(), address);

    if (it != addresses.end()) {
      add_sensor_(address);
    } else {
      ESP_LOGD(TAG, "Cannot find sensor with address 0x%s", format_hex(address).c_str());
      sensors_.push_back(nullptr);
    }
  }

  // Второй проход - попытка привязать новые адреса по возможности
  for (const uint64_t &address : addresses) {
    // Те, что уже есть - мимо
    auto it = std::find(saved_addresses_.begin(), saved_addresses_.end(), address);
    if (it != saved_addresses_.end())
      continue;

    // Если есть еще места - добавляем в конец
    if (this->saved_addresses_.size() < this->max_sensors_num_) {
      ESP_LOGD(TAG, "New sensor was added. Address 0x%s", format_hex(address).c_str());
      saved_addresses_.push_back(address);
      add_sensor_(address);
      continue;
    }

    // Пытаемся найти lost и их заменить
    auto it2 = std::find(sensors_.begin(), sensors_.end(), nullptr);

    // Нашелся lost - заменяем
    if (it2 != sensors_.end()) {
      size_t index = it2 - sensors_.begin();
      ESP_LOGD(TAG, "Replacing the lost sensor 0x%s with a new one with an address 0x%s",
               format_hex(saved_addresses_[index]).c_str(), format_hex(address).c_str());
      *it2 = make_sensor_(address);
      saved_addresses_[index] = address;
      App.register_sensor(*it2);
      App.register_component(*it2);

    } else {
      // Достигли максимального значения ничего сделать не можем
      break;
    }
  }

  // Синхронизировать найденное
  this->saved_sensors_num_ = saved_addresses_.size();
  if (!this->sensors_count_pref_.save(&this->saved_sensors_num_)) {
    ESP_LOGE(TAG, "Error saving registered sensor count");
  }

  for (uint8_t i = 0; i < this->saved_sensors_num_; i++) {
    this->addresses_pref_[i].save(&this->saved_addresses_[i]);
  }
  global_preferences->sync();
}

void DallasTemperatureSearcher::dump_config() {
  ESP_LOGCONFIG(TAG, "Dallas sensor searcher:");
  uint8_t index = 0;
  for (dallas_temp::DallasTemperatureSensor *sensor : this->sensors_) {
    if (sensor)
      ESP_LOGCONFIG(TAG, "  Added %s", sensor->get_name().c_str());
    else if (search_mode_ == SearchMode::ADDRESS_MAP)
      ESP_LOGCONFIG(TAG, "  Lost sensor 0x%s", format_hex(this->saved_addresses_[index]).c_str());

    index++;
  }
}

void DallasTemperatureSearcher::set_default_parameters_(dallas_temp::DallasTemperatureSensor *sensor) {
  sensor->set_device_class("temperature");
  sensor->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  sensor->set_unit_of_measurement("\302\260C");
  sensor->set_accuracy_decimals(1);
  sensor->set_force_update(false);
  sensor->set_update_interval(this->update_interval_ms_);
  sensor->set_component_source("dallas_temp.sensor");
  sensor->set_resolution(12);
}

void DallasTemperatureSearcher::add_sensor_(const uint64_t &address) {
  dallas_temp::DallasTemperatureSensor *sensor = make_sensor_(address);
  this->sensors_.push_back(sensor);

  App.register_sensor(sensor);
  App.register_component(sensor);
}

dallas_temp::DallasTemperatureSensor *DallasTemperatureSearcher::make_sensor_(const uint64_t &address) {
  const size_t string_buffer_size = 32;
  char string_buff[string_buffer_size];

  auto *sensor = new dallas_temp::DallasTemperatureSensor();
  EntityBaseInfo entity_base_info;
  sensor->set_one_wire_bus(bus_);

  std::string address_str = format_hex(address);

  strcpy(string_buff, "Temp Sensor 0x");
  strlcat(string_buff, address_str.c_str(), string_buffer_size);
  entity_base_info.name = string_buff;

  strcpy(string_buff, "ts_0x");
  strlcat(string_buff, address_str.c_str(), string_buffer_size);
  entity_base_info.object_id = string_buff;

  sensor->set_name(entity_base_info.name.c_str());
  sensor->set_object_id(entity_base_info.object_id.c_str());
  sensor->set_address(address);
  set_default_parameters_(sensor);

  this->sensors_params_.push_back(std::move(entity_base_info));
  return sensor;
}

bool DallasTemperatureSearcher::restore_address_data_(ESPPreferenceObject &obj) {
  uint64_t temp;
  if (obj.load(&temp)) {
    ESP_LOGD(TAG, "Loaded save address from memory 0x%s", format_hex(temp).c_str());
    saved_addresses_.push_back(temp);
    return true;
  }
  return false;
}

void DallasTemperatureSearcher::restore_sensors_count_() {
  if (this->sensors_count_pref_.load(&this->saved_sensors_num_)) {
    ESP_LOGI(TAG, "Successfully restored sensor count from memory - %d", this->saved_sensors_num_);
  } else {
    ESP_LOGW(TAG, "No stored sensor count found");
  }
}

}  // namespace dallas_temp_searcher
}  // namespace esphome
