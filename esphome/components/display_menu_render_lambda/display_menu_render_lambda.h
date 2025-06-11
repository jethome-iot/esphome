#pragma once

#include "esphome/components/display_menu_render_base/display_menu_render_base.h"

namespace esphome {
namespace display_menu_render_lambda {

using namespace display_menu_base;
using namespace display_menu_render_base;

using render_lambda_t = std::function<size_t(MenuItemMenu *menu, EntityBase *entity)>;

class LambdaMenuRender : public MenuRenderInterface {
 public:
  LambdaMenuRender() : MenuRenderInterface(EntityType::NONE) {}
  size_t render_entity(MenuItemMenu *menu, EntityBase *entity) override {
    if (lambda_)
      return lambda_(menu, entity);
    return 0;
  }
  void set_lambda(render_lambda_t &&lambda) { this->lambda_ = lambda; }
  render_lambda_t lambda_;
};

}  // namespace display_menu_render_lambda
}  // namespace esphome
