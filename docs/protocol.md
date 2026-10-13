# Pi–ESP32 protocol v1

Transport: UART, 115200 baud, 8N1, UTF-8, one JSON object per line. TX/RX are crossed and grounds are common. Use 3.3 V-compatible UART logic.

Example:
```json
{"v":1,"seq":42,"timestamp_ms":123456,"zone":"CAUTION","distance_m":1.6,"confidence":0.91,"pedestrian":true,"heartbeat":true}
```

Fields: `v` protocol version; `seq` increasing sequence number; `timestamp_ms` sender monotonic time; `zone` FAR/CAUTION/DANGER; `distance_m` finite non-negative number or null; `confidence` in [0,1]; `pedestrian` and `heartbeat` booleans.

Safety: invalid messages or heartbeat timeout stop motors. DANGER latches stop. Re-arm only after valid fresh messages show clear for 2 seconds. Startup state is STOPPED. Starter firmware does not automatically drive forward.

This is a student prototype protocol, not a safety-certified protocol. Future hardening may add CRC, replay checks, acknowledgements, and explicit arming.
