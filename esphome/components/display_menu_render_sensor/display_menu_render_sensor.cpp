#include "display_menu_render_sensor.h"

namespace esphome {
namespace display_menu_renderers {

size_t SensorMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(info.entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}

}  // namespace display_menu_renderers
}  // namespace esphome