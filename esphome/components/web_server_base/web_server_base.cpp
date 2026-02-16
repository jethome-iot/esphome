#include "web_server_base.h"
#ifdef USE_NETWORK
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace web_server_base {

static const char *const TAG = "web_server_base";

WebServerBase *global_web_server_base = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

#ifdef USE_WEBSERVER_AUTH
void WebServerBase::set_credentials(std::string username, std::string password) {
  credentials_.username = std::move(username);
  credentials_.password = std::move(password);
  std::string user_info = credentials_.username + ':' + credentials_.password;
  credentials_.auth_base64 = base64_encode(reinterpret_cast<const uint8_t *>(user_info.c_str()), user_info.size());
}
#endif

void WebServerBase::add_handler(AsyncWebHandler *handler, bool add_auth) {
#ifdef USE_WEBSERVER_AUTH
  if (add_auth && !credentials_.username.empty()) {
    handler = new internal::AuthMiddlewareHandler(handler, &credentials_);
  }
#endif
  this->handlers_.push_back(handler);
  if (this->server_ != nullptr) {
    this->server_->addHandler(handler);
  }
}

float WebServerBase::get_setup_priority() const {
  // Before WiFi (captive portal)
  return setup_priority::WIFI + 2.0f;
}

}  // namespace web_server_base
}  // namespace esphome
#endif
