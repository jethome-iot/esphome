#include "display_menu_render_dallas.h"

#include "esphome/components/display_menu_render_sensor/display_menu_render_sensor.h"

namespace esphome {
namespace display_menu_render_dallas {

size_t DallasTempMenuRender::render_entity(MenuItemMenu *menu, EntityBase *entity) {
  dallas_temp::DallasTemperatureSensor *sensor_obj = static_cast<dallas_temp::DallasTemperatureSensor *>(entity);

  MenuItemMenu *item = new MenuItemMenu();
  auto lambda = display_menu_render_sensor::SensorMenuRender::get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  proccess_submenu(item, sensor_obj);

  menu->add_generated_items(item);
  return 1;
}

void DallasTempMenuRender::proccess_submenu(MenuItemMenu *menu, dallas_temp::DallasTemperatureSensor *sensor_obj) {
  MenuItem *name_info = new MenuItem(MENU_ITEM_LABEL);
  name_info->set_text(sensor_obj->get_name());
  auto internal_lambda = display_menu_render_sensor::SensorMenuRender::get_render_lambda(sensor_obj, false);

  MenuItem *temp_info = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
    return "Temp: " + internal_lambda(nullptr);
  };
  temp_info->set_text(lambda);

  MenuItem *address_info = new MenuItem(MENU_ITEM_LABEL);
  auto address_lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
    return sensor_obj->get_address_name();
  };
  address_info->set_text(address_lambda);

  menu->add_item(name_info);
  menu->add_item(temp_info);
  menu->add_item(address_info);
}

}  // namespace display_menu_render_dallas
}  // namespace esphome
