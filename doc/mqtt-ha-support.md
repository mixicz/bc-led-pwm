# Analysis: HomeAssistant MQTT Compatibility for LED PWM Firmware

This document analyzes the requirements and necessary changes to make the LED PWM firmware compatible with HomeAssistant's MQTT integration, specifically for automatic discovery and control of light entities (on/off/brightness).

## HomeAssistant MQTT Light Integration Overview
HomeAssistant supports MQTT-based lights using a specific topic structure and payload format. Key features include:
- **Discovery**: HomeAssistant can automatically discover devices if they publish a configuration message to a discovery topic.
- **Control**: HomeAssistant sends commands (on/off, brightness) to specific topics, and expects state updates on other topics.

### Typical MQTT Topics for Light Entity
- `homeassistant/light/<object_id>/config` (discovery)
- `<state_topic>` (publishes current state: ON/OFF, brightness)
- `<command_topic>` (receives commands: ON/OFF, brightness)

#### Example Discovery Payload
```json
{
  "name": "LED PWM Light",
  "command_topic": "/led-pwm/0/command/set",
  "state_topic": "/led-pwm/0/-/state",
  "brightness_command_topic": "/led-pwm/0/brightness/set",
  "brightness_state_topic": "/led-pwm/0/brightness/state",
  "payload_on": "ON",
  "payload_off": "OFF",
  "brightness_scale": 100
}
```

## Current Firmware State
- **Subscribed topics**: `led-pwm/{N}/trigger/set`, `led-pwm/{N}/brightness/set`
- **Published topics**: `led-pwm/{N}/trigger/state`, `event/{N}/off`, `event/-/brightness/state`
- **No HomeAssistant discovery support**
- **No explicit ON/OFF command topic** (trigger/set is used for on, but not for off)
- **No state topic with ON/OFF payload**

## Required Changes for HomeAssistant Compatibility

### 1. Implement HomeAssistant Discovery
- Publish a retained discovery message to `homeassistant/light/led-pwm-{N}/config` for each LED channel.
- Include all required fields: `command_topic`, `state_topic`, `brightness_command_topic`, `brightness_state_topic`, etc.

### 2. Add ON/OFF Command Topic
- Subscribe to a new topic: `led-pwm/{N}/set`.
- Accept payloads: `ON` (turn on), `OFF` (turn off).
- Map `ON` to current trigger logic, `OFF` to force LED off.

### 3. Add State Topic
- Publish to `led-pwm/{N}/state` with payload `ON` or `OFF` whenever LED state changes.

### 4. Add Brightness State Topic
- Publish to `led-pwm/{N}/brightness/state` with current brightness value (0-100).

### 5. Brightness Command Topic
- Already supported via `led-pwm/{N}/brightness/set` (may need to ensure payload is 0-100).

### 6. Retained Messages
- Ensure discovery and state topics are published as retained messages for proper HomeAssistant operation.

## Implementation Steps
1. **Discovery**: On boot, publish HomeAssistant discovery message for each LED.
2. **ON/OFF Control**: Add handler for `led-pwm/{N}/set` topic, process `ON`/`OFF` payloads.
3. **State Publishing**: Publish to `led-pwm/{N}/state` and `led-pwm/{N}/brightness/state` on state/brightness change.
4. **Retained Messages**: Use retained flag for discovery and state topics.
5. **Testing**: Validate with HomeAssistant that lights are discovered and controllable.

## External Forwarding Service and Other Dependencies

The device firmware in this project publishes using device-specific topics (typically under `node/{id}/...` or `led-pwm/...`) and cannot itself publish HomeAssistant discovery/config topics (`homeassistant/...`). To integrate with HomeAssistant reliably you must deploy an external forwarding/translation service (forwarder) that bridges device topics to HomeAssistant topics and performs payload/type conversions.

### Required capabilities of the forwarder
- Subscribe to the device-side topics (e.g. `node/{id}/+`, `led-pwm/+/+`) and republish to HomeAssistant topics under `homeassistant/...` and separate HA state/command topics.
- Publish HomeAssistant discovery messages to `homeassistant/light/<object_id>/config` with retained=true so HA can auto-discover entities.
- Convert payloads and ranges (type coercion):
  - Convert device floats/percentages to the numeric range expected by HA discovery (e.g. 0–100 or 0–255) and to integer types where HA expects integers.
  - Convert device numeric on/off indicators (e.g. trigger_count > 0) to the string payloads `ON`/`OFF` expected by HA when using `payload_on`/`payload_off`.
  - Ensure JSON payloads are kept separate from plain payloads used for state (avoid reusing the same topic for JSON and plain values).
- Publish retained state messages for `state_topic` and `brightness_state_topic` so HA shows correct state after restart.
- Support authentication (username/password, TLS) when publishing discovery/state if the broker requires it.
- Optionally support Last Will/Will Topic (LWT) to mark device offline in HA.

### Topic separation policy (prevent clashing)
- Keep HomeAssistant-facing topics strictly under the `homeassistant/` prefix and separate from device topics. Do not republish device-native topics as HA topics in-place.
- Maintain a clear distinction between types:
  - Use `<device-tree>/.../brightness/state` for device-native floats if needed, but the forwarder MUST convert and publish to `<ha-tree>/.../brightness/state` with the type and range expected by HA.
  - Never publish JSON and primitive values to the same topic expected by HA; use distinct topics for JSON events (e.g. `event/{N}/off`) and for HA state values.

### Example mapping rules
- Device: `node/{id}/led-pwm/0/brightness/state` (float 0.0..1.0)  --> Forwarder: map *100 -> `led-pwm/0/brightness/state` (int 0..100) -> HA brightness_state_topic
- Device: `node/{id}/led-pwm/-/trigger/state` (int trigger_count) --> Forwarder: publish `ON` to `led-pwm/0/state` if trigger_count>0, else `OFF`.
- Device: `event/0/off` JSON -> Forwarder: forward as `json_attributes_topic` (if desired) or ignore for HA state.

### Example forwarder options
- Mosquitto bridge configuration (simple topic remap):
  - Works for 1:1 topic renames but cannot perform payload transforms; use only if device already emits HA-compatible payloads.
- Node-RED flow:
  - Subscribe to device topics, use JavaScript nodes to convert payloads and publish discovery/state with retained=true.
  - Good for complex conversions, JSON parsing, and persistence.
- Minimal Python forwarder (paho-mqtt):
  - Subscribe to device topics, implement mapping rules, convert types, publish discovery and retained state. Keep it small and deployable on the same host as the MQTT broker or on a small gateway device (Raspberry Pi).

### Operational recommendations
- Publish discovery messages retained so HA can discover entities after reboot.
- Publish immediate retained state after (re)publishing discovery.
- Use unique object_id names incorporating the device id (e.g. `led_pwm_<nodeid>_<led>`) to avoid collisions when multiple devices are present.
- Monitor and log conversions to detect type/range issues early.

### Security and reliability
- Run the forwarder on a trusted machine with persistent network access to the MQTT broker and HomeAssistant.
- Use broker authentication and TLS if available.
- Consider running the forwarder as a service with auto-restart.

## Next steps
- Choose forwarder implementation (Node-RED or Python recommended for payload transforms).
- Implement discovery publishing and the conversion rules listed above.
- Test discovery, on/off and brightness control in HomeAssistant with retained discovery and retained state enabled.

## References
- [HomeAssistant MQTT Light Documentation](https://www.home-assistant.io/integrations/light.mqtt/)
- [MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/)

---
*Generated by GitHub Copilot on 30. srpna 2025*
