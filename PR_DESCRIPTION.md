# JetHome Project Implementation - ESP32 Platform Focus

## Summary
This PR implements the JetHome project, a customized ESPHome firmware focused on ESP32 platform testing with 5 specific modifiable components.

## Changes Made

### 1. Project Rebranding
- Renamed project from ESPHome to JetHome
- Updated all documentation and test scripts with JetHome branding
- Renamed test runner: `test_custom_components` → `test_jethome_components`

### 2. Component Testing Infrastructure
- Established ESP32-only testing framework (removed ESP8266, RP2040 support)
- Created test configurations for 5 modifiable components:
  - **dallas_temp_searcher** - Dallas temperature sensor searcher
  - **dynamic_entity_settings** - Dynamic entity settings
  - **groups** - Groups component
  - **ota_rollback** - OTA rollback (ESP-IDF only)
  - **user_names** - User names component

### 3. Test Configuration Files
Each component now has:
- `common.yaml` - Base configuration
- `test.esp32-ard.yaml` - ESP32 Arduino framework test
- `test.esp32-idf.yaml` - ESP32 IDF framework test (where applicable)

### 4. Bug Fixes
- Fixed dallas_temp_searcher configuration to use single dictionary format
- Added missing sensor dependency for dallas_temp component
- Resolved ESP32-IDF compilation issues with pip module availability
- Fixed Python environment configuration for NixOS compatibility

### 5. Testing Framework
- Created `script/test_jethome_components` for testing only the 5 allowed components
- All components pass validation on both ESP32-Arduino and ESP32-IDF frameworks
- Unit test suite passes with 531 tests

## Test Results
✅ All 5 JetHome components validated successfully:
- ESP32-Arduino: 4/4 components pass (ota_rollback is IDF-only)
- ESP32-IDF: 5/5 components pass

## Testing Instructions
```bash
# Test all JetHome components
script/test_jethome_components

# Test individual component
esphome compile tests/components/<component_name>/test.esp32-ard.yaml
esphome compile tests/components/<component_name>/test.esp32-idf.yaml

# Run unit tests
script/unit_test
```

## Compatibility
- Platform: ESP32 only
- Frameworks: Arduino and ESP-IDF
- Python: 3.11+
- ESPHome: 2025.7.5-dev base

## Documentation
Updated `replit.md` with:
- Complete list of modifiable components
- Testing instructions and rules
- Component dependencies
- JetHome branding throughout