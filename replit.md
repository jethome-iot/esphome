# JetHome Project

## Overview
This is a JetHome project built on ESPHome framework with customized components and testing infrastructure.

## Modifiable Components

You can modify only the following components and their associated tests:
- **dallas_temp_searcher** - Dallas temperature sensor searcher component
- **dynamic_entity_settings** - Dynamic entity settings component  
- **groups** - Groups component
- **ota_rollback** - OTA rollback component (ESP-IDF only)
- **user_names** - User names component

## Component Testing Rules

### Platform Requirements
**All tests for our modifiable components are ESP32 platform only:**
- ESP32 with Arduino framework (`test.esp32-ard.yaml`)
- ESP32 with ESP-IDF framework (`test.esp32-idf.yaml`)
- No ESP8266, RP2040, or other platform tests are created or maintained

### Test Locations
Each modifiable component has tests in:
```
tests/components/<component_name>/
├── common.yaml              # Base configuration shared by all tests
├── test.esp32-ard.yaml      # ESP32 Arduino framework test
└── test.esp32-idf.yaml      # ESP32 IDF framework test (if applicable)
```

**Note:** `ota_rollback` component only supports ESP-IDF framework, so it only has `test.esp32-idf.yaml`.

### Running Tests for Our Components

#### JetHome Component Test Runner
Run tests specifically for our 5 modifiable components:
```bash
script/test_jethome_components
# Tests only: dallas_temp_searcher, dynamic_entity_settings, groups, ota_rollback, user_names
```

#### Individual Component Testing
Test a specific component by compiling its YAML files:
```bash
# Example: Test dallas_temp_searcher on ESP32 Arduino
esphome compile tests/components/dallas_temp_searcher/test.esp32-ard.yaml

# Example: Test ota_rollback on ESP32 IDF
esphome compile tests/components/ota_rollback/test.esp32-idf.yaml
```

## General Testing Framework

### 1. Unit Tests - Python code testing
```bash
script/unit_test
# This runs pytest on tests/unit_tests/
```

### 2. Component Tests
```bash
script/component_test
# This runs pytest on tests/component_tests/
```

### 3. YAML Compilation Tests
```bash
script/test
# This compiles all test*.yaml files to verify they work
```

### 4. Full Test Suite
```bash
script/fulltest
# Runs all tests including linting, unit tests, component tests, and compilation tests
```

## Prerequisites

Install test dependencies:
```bash
# Install test requirements
pip install -r requirements_test.txt

# Or install all dev dependencies
pip install -e ".[dev]"
```

## Test Structure

- **Unit Tests**: Located in `tests/unit_tests/` - Test core Python functionality
- **Component Tests**: Located in `tests/component_tests/` - Test individual component integrations
- **Component YAML Tests**: Located in `tests/components/` - Test YAML configurations for each component
- **JetHome Component Tests**: Located in `tests/components/` for our 5 modifiable components (ESP32 only)

## Component Dependencies

- **dallas_temp_searcher**: Requires `one_wire` and `dallas_temp` components
- **dynamic_entity_settings**: Standalone component
- **groups**: Can be used with various entity types (sensors, switches, binary_sensors)
- **ota_rollback**: Requires ESP-IDF framework and OTA component
- **user_names**: Standalone component

## Recent Changes
- 2025-10-02: Initial documentation of JetHome testing framework
- 2025-10-02: Added list of modifiable components
- 2025-10-02: Created ESP32-only test configurations for all modifiable components
- 2025-10-02: Added JetHome test runner for modifiable components
- 2025-10-02: Rebranded project from ESPHome to JetHome