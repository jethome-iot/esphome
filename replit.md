# ESPHome Project

## Overview
This is an ESPHome repository containing the complete ESPHome source code and testing framework.

## Running Unit Tests

ESPHome has several types of tests:

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

## Docker Alternative

Run tests using Docker:
```bash
# Run full lint + test suite
docker run --rm -v "${PWD}/":/esphome -it ghcr.io/esphome/esphome-lint script/fulltest

# Quick lint only
docker run --rm -v "${PWD}/":/esphome -it ghcr.io/esphome/esphome-lint script/quicklint
```

## Test Structure

- **Unit Tests**: Located in `tests/unit_tests/` - Test core Python functionality
- **Component Tests**: Located in `tests/component_tests/` - Test individual component integrations
- **Component YAML Tests**: Located in `tests/components/` - Test YAML configurations for each component across different platforms (ESP32, ESP8266, RP2040)

## Recent Changes
- 2025-10-02: Initial documentation of ESPHome testing framework

