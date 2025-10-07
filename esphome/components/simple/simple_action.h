#pragma once
#include "esphome/core/automation.h"

namespace esphome {
namespace simple {

template<typename... Ts> class EmptyAction : public Action<Ts...> {
 public:
  void play(Ts... x) override { /* ignore - see play_complex */
  }
};

}  // namespace simple
}  // namespace esphome
