#include "display_menu_render_sensor.h"

namespace esphome {
namespace display_menu_render_sensor {

size_t SensorMenuRender::render_entity(MenuItemMenu *menu, EntityBase *entity) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}

}  // namespace display_menu_render_sensor
}  // namespace esphome
