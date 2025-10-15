# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository contains ESPHome external components for reading Wireless M-Bus (wM-Bus) utility meters using ESP32 devices with CC1101 or SX1276 radio transceivers. Version 5 represents a major architectural rewrite based on Kuba's fork (IoTLabs-pl/esphome-components) that migrated from Arduino to ESP-IDF framework and restructured the codebase into modular components.

**Key Technologies:**
- ESPHome framework with ESP-IDF (not Arduino - migrated in v5)
- C++ for core components
- Python for ESPHome configuration/codegen
- Wireless M-Bus protocol (modes: S1, T1, C1, C2, etc.)
- CC1101 Sub-1 GHz transceiver (polling-based GPIO reception)
- SX1276 LoRa transceiver (interrupt-driven reception)

## Historical Context - Version 5 Changes

Version 5.0.0 (May 2025) introduced fundamental changes:

1. **Framework Migration**: Dropped Arduino framework, migrated to pure ESP-IDF
   - Required custom esp32 component to increase loop task stack size (components/esp32/core.cpp:88)
   - Stack size progressively increased: 32768 → 40960 → 65536 bytes to handle wmbusmeters code complexity

2. **Architecture Restructure**: Split monolithic `wmbus` component into three separate components:
   - `wmbus_radio` - Radio hardware abstraction and frame reception
   - `wmbus_meter` - Logical meter representation and ESPHome integration
   - `wmbus_common` - Upstream wmbusmeters code (managed via git subtree)

3. **Upstream Integration**: Version 4 (earlier) started using wmbusmeters library directly for telegram parsing
   - Replaced custom parsing code with battle-tested wmbusmeters implementation
   - Driver files renamed from `.cpp` to `.cc` extension
   - Pulled as git subtree for easier upstream synchronization

4. **Radio Support Changes**:
   - ~~Dropped CC1101 support~~ → **CC1101 support restored!** (2025-10-15)
   - CC1101 implemented with modern architecture (polling-based state machine)
   - SX1276 support with separate FreeRTOS task for reception
   - Both radios use unified RadioTransceiver interface
   - SX1262 planned but not yet implemented

5. **Component Modernization**:
   - Removed ethernet component
   - Added socket_transmitter for TCP/UDP forwarding
   - Restructured automation triggers (on_frame, on_telegram)

## Architecture

### Component Structure

The codebase is organized into modular ESPHome components:

#### 1. wmbus_radio - Radio Communication Layer
   - **Supports two transceiver types:**
     - CC1101 (components/wmbus_radio/transceiver_cc1101.cpp) - Polling-based GPIO
     - SX1276 (components/wmbus_radio/transceiver_sx1276.cpp) - Interrupt-driven
   - **Runs receiver in separate FreeRTOS task** (components/wmbus_radio/component.cpp)
     - Non-blocking operation via task queue
     - CC1101: Polls GDO0/GDO2 pins in state machine
     - SX1276: DIO1 interrupt wakes receiver task when FIFO has data
   - Decodes 3-of-6 Manchester encoding (components/wmbus_radio/decode3of6.cpp)
   - Provides Frame objects with RSSI measurement
   - Exposes `on_frame` automation trigger
   - Unified RadioTransceiver interface (polymorphism)

#### 2. wmbus_common - Shared wM-Bus Protocol (Git Subtree)
   - **Pulled from upstream wmbusmeters via git subtree** (see Updating wmbusmeters Code below)
   - Contains telegram parsing, CRC validation, encryption/decryption
   - Driver implementations for ~100 meter types (driver_*.cc files)
   - **DO NOT manually edit files in this component** - they're overwritten on subtree pull
   - Handles complex wM-Bus protocol layers (DLL, ELL, TPL, etc.)

#### 3. wmbus_meter - Meter Abstraction and ESPHome Integration
   - Represents logical meters with ID, type (driver), encryption key
   - Subscribes to radio frames, filters by meter ID and link mode
   - Extracts sensor values from telegrams using field names
   - Integrates with ESPHome sensor/text_sensor platforms
   - Provides `on_telegram` automation trigger

#### 4. socket_transmitter - Network Forwarding Utility
   - Sends telegram data via TCP/UDP to external systems
   - Supports HEX and RTLwMBUS formats for wmbusmeters compatibility
   - Used for forwarding to Home Assistant wmbusmeters addon

#### 5. esp32 - Custom ESP32 Component (Override)
   - **Overrides standard ESPHome esp32 component**
   - Primary purpose: Increase loop task stack size (components/esp32/core.cpp:88)
   - Currently set to 65536 bytes (progressively increased from 32768)
   - Required because wmbusmeters code uses significant stack space
   - This is a "dirty hack" acknowledged in commit history

### Data Flow

**SX1276 (Interrupt-driven):**
```
SX1276 Hardware → SPI Interrupt (DIO1)
                      ↓
           Receiver FreeRTOS Task wakes
                      ↓
           Read FIFO, decode 3-of-6 → Frame object
                      ↓
           Queue to main loop → Radio component receives Frame
                      ↓
           Trigger on_frame automation handlers
                      ↓
           wmbus_meter filters by ID/mode
                      ↓
           Parse telegram (wmbusmeters code) → Extract fields
                      ↓
           Update ESPHome sensors → Publish to Home Assistant
```

**CC1101 (Polling-based):**
```
CC1101 Hardware → Receiver FreeRTOS Task polls (10ms loop)
                      ↓
           Check GDO2 (sync detected) and GDO0 (FIFO ready)
                      ↓
           State machine: WAIT_SYNC → WAIT_DATA → READ_DATA
                      ↓
           Decode Mode T (3-of-6) or Mode C
                      ↓
           Frame complete → Queue to main loop
                      ↓
           [Same as SX1276 from here...]
```

### Key Design Patterns

- **FreeRTOS Tasks**: Radio receiver runs in separate task to avoid blocking main loop (critical for v5)
- **Callback Pattern**: Frame handlers and telegram callbacks for event-driven processing
- **Python Codegen**: ESPHome config validation and C++ code generation in `__init__.py` files
- **Template Actions**: Lambda expressions in YAML for custom data formatting
- **Git Subtree**: wmbus_common maintained as subtree from upstream wmbusmeters

## Development Commands

### Updating wmbusmeters Code

The `wmbus_common` component is maintained as a git subtree from upstream wmbusmeters:

```bash
git subtree pull --prefix components/wmbus_common https://github.com/wmbusmeters/wmbusmeters.git <REF> --squash
```

Replace `<REF>` with a commit hash or tag from wmbusmeters repository. After pulling, you may need to adjust stack size in components/esp32/core.cpp if new code increases memory usage.

### Testing with ESPHome

This is an external component, referenced in ESPHome YAML:

```yaml
external_components:
  - source: github://SzczepanLeon/esphome-components@main
    refresh: 0d
```

For local development testing:

```yaml
external_components:
  - source: /path/to/esphome-components
```

### Building and Uploading

ESPHome handles compilation automatically:

```bash
esphome compile config.yaml
esphome upload config.yaml
```

To view logs:
```bash
esphome logs config.yaml
```

## Important Implementation Details

### ESP-IDF Framework Requirements

Version 5 requires ESP-IDF framework (not Arduino):

```yaml
esp32:
  board: heltec_wifi_lora_32_V2
  framework:
    type: esp-idf  # Required!
```

### Stack Size Management

The custom esp32 component increases loop task stack to 65536 bytes (components/esp32/core.cpp:88). If you encounter stack overflow crashes after updating wmbusmeters code, you may need to increase this further.

### Link Modes

wM-Bus supports multiple link modes (S1, T1, C1, C2, etc.) defined in `wmbus_common/wmbus.h`. Meters can be configured to listen on specific modes:

```yaml
wmbus_meter:
  - id: my_meter
    meter_id: 0x12345678
    type: amiplus
    mode:
      - T1
      - C1
```

### Radio Configuration

#### CC1101 Configuration

Polling-based reception with GDO pins:

```yaml
wmbus_radio:
  radio_type: CC1101
  cs_pin: GPIO18
  reset_pin: GPIO14
  gdo0_pin: GPIO4   # FIFO threshold indicator
  gdo2_pin: GPIO16  # Sync word detected
  frequency: 868.95 # MHz (optional, EU default)
```

#### SX1276 Configuration

Interrupt-driven reception:

```yaml
wmbus_radio:
  radio_type: SX1276
  cs_pin: GPIO18
  reset_pin: GPIO14
  irq_pin: GPIO35  # DIO1 - triggers on FIFO threshold
```

### CC1101 vs SX1276 Comparison

| Feature | CC1101 | SX1276 |
|---------|---------|---------|
| **Reception Mode** | Polling (GPIO state checks) | Interrupt-driven (hardware IRQ) |
| **CPU Usage** | Moderate (~100 Hz polling) | Low (idle until interrupt) |
| **Pins Required** | CS, RST, GDO0, GDO2 | CS, RST, IRQ |
| **Frequency** | Configurable (868.95 default) | Fixed in code |
| **Mode T Support** | ✓ (3-of-6 decode) | ✓ (3-of-6 decode) |
| **Mode C Support** | ✓ | ✓ |
| **RSSI Reading** | ✓ | ✓ |
| **Cost** | Lower (~$2-3) | Higher (~$5-8) |
| **Availability** | Common, easy to source | Common |
| **Power Usage** | Lower | Higher |
| **Implementation** | State machine | Interrupt + FIFO |

**When to use CC1101:**
- Lower cost projects
- Power-sensitive applications
- When hardware interrupts are limited
- Modules already on hand from version 4

**When to use SX1276:**
- Lowest CPU usage desired
- Fast interrupt response critical
- Using Heltec/TTGO boards (built-in SX1276)
- LoRa capability needed for future expansion

### Encryption

Many meters encrypt telegrams using AES-128. Keys are configured per meter (32 hex characters = 16 bytes). The `wmbus_common` code handles decryption automatically when parsing.

### Field Extraction

Meter drivers define available fields (e.g., `total_energy_consumption_kwh`, `current_power_consumption_kw`). Sensors subscribe to specific fields:

```yaml
sensor:
  - platform: wmbus_meter
    parent_id: electricity_meter
    field: current_power_consumption_kw
    name: "Current Power"
```

To discover available fields, decode a telegram at https://wmbusmeters.org/analyze/ and look at the JSON output.

### Automation and Formatting

Frame data can be formatted as HEX or RTLwMBUS format for external processing:

```yaml
wmbus_radio:
  on_frame:
    - mqtt.publish:
        topic: wmbus/telegram
        payload: !lambda return frame->as_rtlwmbus();
    - socket_transmitter.send:
        data: !lambda return frame->as_hex();
```

### Driver Selection

The `wmbus_common` component supports selective driver compilation to reduce binary size:

```yaml
wmbus_common:
  drivers: all  # Or list specific drivers
  # drivers:
  #   - apator162
  #   - amiplus
```

## Code Style Conventions

- C++ uses `snake_case` for variables/functions, `PascalCase` for classes
- ESPHome namespace structure: `esphome::component_name`
- Python uses ESPHome's config validation patterns (cv.Schema, etc.)
- Header guards use `#pragma once`
- Driver files in wmbus_common use `.cc` extension (upstream convention)

## Known Limitations and TODOs

From README.md TODO list:

- ~~CC1101 support removed~~ → **✓ DONE!** CC1101 support restored (2025-10-15)
- SX1262 support planned but not implemented (limited frame length)
- Aggressive cleanup of wmbusmeters classes/structs needed
- Refactor traces/logs system
- Prepare packages for ready-made boards (UltimateReader) with displays

### CC1101-Specific Notes

- **No Arduino library dependency**: Pure ESP-IDF implementation
- **Polling overhead**: ~100 Hz GPIO checks consume moderate CPU
- **No external library**: Custom low-level SPI driver
- **Version 4 compatibility**: Same RF settings, different architecture
- **See examples/cc1101_wmbus.yaml** for complete working configuration

## Troubleshooting

### Stack Overflow Crashes
If you see stack overflow errors, increase the stack size in components/esp32/core.cpp:88. The value has been progressively increased as wmbusmeters code grows.

### No Telegrams Received
- Check link mode matches your meter's transmission mode
- Verify meter ID is correct (use `log_all: True` to see all received telegrams)
- Ensure encryption key is correct (00000000... for no encryption)
- Check RSSI values - weak signal may cause decoding failures

### Component Loading Errors
Ensure `external_components` has correct refresh setting and the esp32 framework is set to `esp-idf` (not Arduino).

## Related Documentation

- ESPHome Custom Components: https://esphome.io/custom/custom_component.html
- wM-Bus Protocol: EN 13757 standard
- wmbusmeters upstream: https://github.com/wmbusmeters/wmbusmeters
- UltimateReader board: Commercial hardware using these components
