#include "graphical_display_menu.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cstdlib>
#include "esphome/components/display/display.h"

namespace esphome {
namespace graphical_display_menu {

static const char *const TAG = "graphical_display_menu";

void GraphicalDisplayMenu::setup() {
  if (this->display_ != nullptr) {
    display::display_writer_t writer = [this](display::Display &it) { this->draw_menu(); };
    this->display_page_ = make_unique<display::DisplayPage>(writer);
  }

  if (!this->menu_item_value_.has_value()) {
    this->menu_item_value_ = [](const MenuItemValueArguments *it) {
      std::string label;
      if (it->is_item_selected && it->is_menu_editing) {
        label.append(" >");
        label.append(it->item->get_value_text());
        label.append("< ");
      } else if (it->item->get_type() == display_menu_base::MENU_ITEM_SWITCH ||
                 it->item->get_type() == display_menu_base::MENU_ITEM_BINARY_SENSOR) {
        label.append(" (");
        label.append(it->item->get_value_text());
        label.append(") ");
      } else if (it->item->has_value()) {
        label.append(": ");
        label.append(it->item->get_value_text());
      }
      return label;
    };
  }

  display_menu_base::DisplayMenuComponent::setup();
}

void GraphicalDisplayMenu::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Graphical Display Menu\n"
                "Has Display: %s\n"
                "Popup Mode: %s\n"
                "Advanced Drawing Mode: %s\n"
                "Has Font: %s\n"
                "Mode: %s\n"
                "Active: %s\n"
                "Menu items:",
                YESNO(this->display_ != nullptr), YESNO(this->display_ != nullptr), YESNO(this->display_ == nullptr),
                YESNO(this->font_ != nullptr),
                this->mode_ == display_menu_base::MENU_MODE_ROTARY ? "Rotary" : "Joystick", YESNO(this->active_));
  for (size_t i = 0; i < this->displayed_item_->items_size(); i++) {
    auto *item = this->displayed_item_->get_item(i);
    ESP_LOGCONFIG(TAG, "  %i: %s (Type: %s, Immediate Edit: %s, Visible: %s)", i, item->get_text().c_str(),
                  LOG_STR_ARG(display_menu_base::menu_item_type_to_string(item->get_type())),
                  YESNO(item->get_immediate_edit()), YESNO(item->is_visible()));
  }
}

void GraphicalDisplayMenu::set_display(display::Display *display) { this->display_ = display; }

void GraphicalDisplayMenu::set_font(display::BaseFont *font) { this->font_ = font; }

void GraphicalDisplayMenu::set_foreground_color(Color foreground_color) { this->foreground_color_ = foreground_color; }
void GraphicalDisplayMenu::set_background_color(Color background_color) { this->background_color_ = background_color; }
void GraphicalDisplayMenu::set_fill_row(bool val) { this->fill_row_ = val; }
void GraphicalDisplayMenu::set_shrink_label(bool val) { this->shrink_label_ = val; }
void GraphicalDisplayMenu::set_restore_page(bool val) { this->restore_page_ = val; }

void GraphicalDisplayMenu::on_before_show() {
  if (this->display_ != nullptr) {
    this->previous_display_page_ = this->display_->get_active_page();
    this->display_->show_page(this->display_page_.get());
    this->display_->clear();
  } else {
    this->update();
  }
}

void GraphicalDisplayMenu::on_before_hide() {
  if (this->restore_page_ && this->previous_display_page_ != nullptr) {
    this->display_->show_page((display::DisplayPage *) this->previous_display_page_);
    this->display_->clear();
    this->update();
    this->previous_display_page_ = nullptr;
  } else {
    this->update();
  }
}

void GraphicalDisplayMenu::draw_and_update() {
  this->ensure_cursor_visible_();
  this->update();

  // If we're in advanced drawing mode we won't have a display and will instead require the update callback to do
  // our drawing
  if (this->display_ != nullptr) {
    draw_menu();
  }
}

void GraphicalDisplayMenu::draw_menu() {
  if (this->display_ == nullptr) {
    ESP_LOGE(TAG, "draw_menu() called without a display_. This is only available when using the menu in pop up mode");
    return;
  }
  display::Rect bounds(0, 0, this->display_->get_width(), this->display_->get_height());
  this->draw_menu_internal_(this->display_, &bounds);
}

void GraphicalDisplayMenu::draw(display::Display *display, const display::Rect *bounds) {
  this->draw_menu_internal_(display, bounds);
}

void GraphicalDisplayMenu::draw_menu_internal_(display::Display *display, const display::Rect *bounds) {
  int16_t total_height = 0;
  int16_t max_width = 0;
  int y_padding = 2;
  bool scroll_menu_items = false;
  std::vector<display::Rect> menu_dimensions;
  std::vector<size_t> visible_indices;
  int number_items_fit_to_screen = 0;
  const int max_item_index = this->displayed_item_->items_size() - 1;

  // First pass: measure only visible items and collect their indices
  bool first_visible = true;
  size_t cursor_visible_pos = 0;
  bool cursor_found = false;
  for (size_t i = 0; max_item_index >= 0 && i <= static_cast<size_t>(max_item_index); i++) {
    const auto *item = this->displayed_item_->get_item(i);
    if (!item->is_visible()) {
      continue;
    }

    size_t visible_pos = visible_indices.size();
    visible_indices.push_back(i);
    const bool selected = i == this->cursor_index_;
    const display::Rect item_dimensions = this->measure_item(display, item, bounds, selected);

    if (selected) {
      cursor_visible_pos = visible_pos;
      cursor_found = true;
    }

    menu_dimensions.push_back(item_dimensions);
    total_height += item_dimensions.h + (first_visible ? 0 : y_padding);
    first_visible = false;
    max_width = std::max(max_width, item_dimensions.w);

    if (total_height <= bounds->h) {
      number_items_fit_to_screen++;
    } else {
      if ((selected) || (cursor_found && visible_pos == cursor_visible_pos + 1)) {
        scroll_menu_items = true;
      }
    }
  }

  if (visible_indices.empty()) {
    return;
  }

  // Determine what items to draw (using visible indices)
  size_t first_visible_idx = 0;
  size_t last_visible_idx = visible_indices.size() - 1;

  if (number_items_fit_to_screen <= 1) {
    // If only one item can fit to the bounds draw the current cursor item
    last_visible_idx = std::min(last_visible_idx, cursor_visible_pos + 1);
    first_visible_idx = cursor_visible_pos;
  } else {
    if (scroll_menu_items) {
      // Attempt to draw the item after the current item (+1 for equality check in the draw loop)
      last_visible_idx = std::min(last_visible_idx, cursor_visible_pos + 1);

      // Go back through the measurements to determine how many prior items we can fit
      int height_left_to_use = bounds->h;
      for (int vi = static_cast<int>(last_visible_idx); vi >= 0; vi--) {
        const display::Rect item_dimensions = menu_dimensions[vi];
        height_left_to_use -= (item_dimensions.h + y_padding);

        if (height_left_to_use <= 0) {
          // Ran out of space - this is our first item to draw
          first_visible_idx = vi;
          break;
        }
      }
      const size_t items_to_draw = last_visible_idx - first_visible_idx;
      // Don't draw last item partially if it is the selected item
      if ((cursor_visible_pos == last_visible_idx) &&
          (static_cast<size_t>(number_items_fit_to_screen) <= items_to_draw) &&
          (first_visible_idx < visible_indices.size() - 1)) {
        first_visible_idx++;
      }
    }
  }

  // Render the items into the view port
  display->start_clipping(*bounds);

  display->filled_rectangle(bounds->x, bounds->y, max_width, total_height, this->background_color_);
  auto y_offset = bounds->y;
  for (size_t vi = first_visible_idx; vi <= last_visible_idx; vi++) {
    size_t item_index = visible_indices[vi];
    const auto *item = this->displayed_item_->get_item(item_index);
    const bool selected = item_index == this->cursor_index_;
    display::Rect dimensions = menu_dimensions[vi];

    dimensions.y = y_offset;
    dimensions.x = bounds->x;
    this->draw_item(display, item, &dimensions, selected);

    y_offset += dimensions.h + y_padding;
  }

  display->end_clipping();
}

display::Rect GraphicalDisplayMenu::measure_item(display::Display *display, const display_menu_base::MenuItem *item,
                                                 const display::Rect *bounds, const bool selected) {
  display::Rect dimensions(0, 0, 0, 0);

  if (selected) {
    // TODO: Support selection glyph
    dimensions.w += 0;
    dimensions.h += 0;
  }

  std::string label = item->get_text();
  if (item->has_value()) {
    // Append to label
    MenuItemValueArguments args(item, selected, this->editing_);
    label.append(this->menu_item_value_.value(&args));
  }

  int x1;
  int y1;
  int width;
  int height;
  display->get_text_bounds(0, 0, label.c_str(), this->font_, display::TextAlign::TOP_LEFT, &x1, &y1, &width, &height);

  dimensions.w = this->fill_row_ ? bounds->w : std::min((int16_t) width, bounds->w);
  dimensions.h = std::min((int16_t) height, bounds->h);

  return dimensions;
}

inline void GraphicalDisplayMenu::draw_item(display::Display *display, const display_menu_base::MenuItem *item,
                                            const display::Rect *bounds, const bool selected) {
  const auto background_color = selected ? this->foreground_color_ : this->background_color_;
  const auto foreground_color = selected ? this->background_color_ : this->foreground_color_;

  // int background_width = std::max(bounds->width, available_width);
  int background_width = bounds->w;

  display->filled_rectangle(bounds->x, bounds->y, background_width, bounds->h, background_color);

  std::string label = item->get_text();
  std::string value;
  if (item->has_value()) {
    MenuItemValueArguments args(item, selected, this->editing_);
    value = this->menu_item_value_.value(&args);
  }

  int width_label = background_width;

  // Measure value size
  int value_x1, value_y1, value_width = 0, value_height;
  if (!value.empty()) {
    display->get_text_bounds(0, 0, value.c_str(), this->font_, display::TextAlign::TOP_LEFT, &value_x1, &value_y1,
                             &value_width, &value_height);
    width_label -= value_width;
  }
  if (this->shrink_label_) {
    label = this->display_->shrink_text_to_width(label, this->font_, width_label);
  }
  std::string res = label + value;

  display->print(bounds->x, bounds->y, this->font_, foreground_color, display::TextAlign::TOP_LEFT, res.c_str());
}

void GraphicalDisplayMenu::draw_item(const display_menu_base::MenuItem *item, const uint8_t row, const bool selected) {
  ESP_LOGE(TAG, "draw_item(MenuItem *item, uint8_t row, bool selected) called. The graphical_display_menu specific "
                "draw_item should be called.");
}

void GraphicalDisplayMenu::update() { this->on_redraw_callbacks_.call(); }

}  // namespace graphical_display_menu
}  // namespace esphome
