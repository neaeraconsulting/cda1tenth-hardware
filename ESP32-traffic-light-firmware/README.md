# CDA1Tenth Traffic Light Firmware

PlatformIO ESP-IDF project for the ESP32-C6-DevKitC-1.

The firmware drives four 3-pixel RGB strips wired as one 12-pixel WS2812/NeoPixel chain. Each section is ordered red, yellow, green:

- Pixels 0-2: signal 1
- Pixels 3-5: signal 2
- Pixels 6-8: signal 3, opposite signal 1
- Pixels 9-11: signal 4, opposite signal 2

Signals 1 and 3 receive green together for 10 seconds, yellow for 2 seconds, then all signals are red for 1 second. Signals 2 and 4 then take the same turn. This repeats continuously.

The firmware publishes SAE J2735 2024 JSON `MessageFrame` objects over MQTT:

- SPaT (`messageId` 19) every 100 ms (10 Hz)
- MAP (`messageId` 18) whenever MQTT connects
- SSM (`messageId` 30) at 10 Hz while an emergency request is pending or active

It also accepts lane-based Signal Request Messages (SRM, `messageId` 29) from
model emergency vehicles and safely preempts the normal signal cycle.

Signal group 1 represents signals 1 and 3 (north/south), while signal group
2 represents signals 2 and 4 (east/west). The MAP defines four ingress and
four egress lanes. Each straight-through ingress connection points to the
same signal group used by the corresponding SPaT movement state.

`minEndTime` is an absolute decisecond within the current UTC hour. Wi-Fi
startup also starts SNTP synchronization. Until UTC is available, `moy`,
`timeStamp`, and `minEndTime` use their J2735 unavailable or unknown values
instead of device uptime.

The default MAP geometry models 35 cm lanes and 2 m approaches in the
physical 1/10-scale coordinate system. Its latitude and longitude are set to
the J2735 unavailable values. Before using GNSS/map matching, update the
surveyed reference point and measured geometry in
`src/j2735_config.h`, then increment both the intersection revision and MAP
issue revision.

The output is a schema-shaped, readable representation. It is not an SAE
J2735 UPER wire message and is not sent over a V2X radio transport.

## MQTT publishing

The firmware follows the `MQTT-SPAT-demo` branch defaults:

| Topic | Payload | Delivery |
| ----- | ------- | -------- |
| `esp32/1/spat` | Complete SPaT `MessageFrame` | QoS 0, retained, 10 Hz |
| `esp32/1/map` | Complete MAP `MessageFrame` | QoS 1, retained |
| `esp32/1/srm` | Incoming emergency-vehicle SRM | QoS 0, not retained |
| `esp32/1/ssm` | SSM request acknowledgement/status | QoS 0, not retained, 10 Hz while active |
| `esp32/1/color` | Plain-text control command | QoS 0, incoming, not retained |
| `esp32/1/status/color` | Plain-text physical phase | QoS 0, retained |

The default broker is `172.250.250.111:1883` using anonymous, unencrypted
MQTT 3.1.1. The ESP32 must be on the same LAN, VPN, or ZeroTier route. Copy
`src/secrets.example.h` to `src/secrets.h` and enter the Wi-Fi credentials.
The secrets file is ignored by Git. Broker, topic, and time-server defaults
can also be overridden there.

If `src/secrets.h` is absent or the Wi-Fi SSID is empty, MQTT is disabled
while the traffic lights continue normally. Wi-Fi and MQTT reconnect
automatically after an interruption, and the retained MAP is republished after
every broker connection. SPaT updates replace the retained SPaT value, so new
consumers immediately receive the latest state.

Consumers must unwrap `value.SPAT` and `value.MapData` from the complete
MessageFrames. They should use each MAP lane connection's `signalGroup`
instead of assuming that physical head numbers are J2735 signal-group IDs.
This intersection uses group 1 for the synchronized north/south movements and
group 2 for the synchronized east/west movements.

Commands `1` and `3` request north/south, while `2` and `4` request east/west.
The controller never switches directly between conflicting greens: it
finishes the active phase, yellow, and all-red clearance before granting the
requested direction, then holds that direction. Command `traffic` resumes the
automatic cycle. Do not retain command messages.

Status values are `north-south-green`, `north-south-yellow`,
`east-west-green`, `east-west-yellow`, and `all-red`. The shared broker is
anonymous and unencrypted, so it must not be exposed directly to the public
internet.

## Emergency-vehicle preemption

An emergency vehicle sends a schema-shaped J2735 Signal Request Message to
`esp32/1/srm`. The request identifies an ingress lane from the published MAP:

- Lanes 1 and 3 request north/south signal group 1.
- Lanes 5 and 7 request east/west signal group 2.

The requestor role must be `emergency`, `police`, `fire`, or `ambulance`.
The four-byte `entityID` is encoded as eight hexadecimal characters and must
match `V2X_AUTHORIZED_ENTITY_ID`, which defaults to `01020304`. Override that
value in `src/secrets.h` for the model emergency vehicle.

Publish the included fire-vehicle example without MQTT retain:

```bash
mosquitto_pub \
  -h 172.250.250.111 \
  -t esp32/1/srm \
  -f examples/emergency-srm-request.json
```

The example requests lane 5, so the controller grants east/west after the
safe transition. `duration` is expressed in J2735 deciseconds. This model uses
it as a renewable request lease, limited to 2-30 seconds; an omitted or zero
duration defaults to 10 seconds. A vehicle can renew the lease with
`priorityRequestUpdate` using the same `entityID` and `requestID`.

After the vehicle clears the intersection, publish the matching cancellation:

```bash
mosquitto_pub \
  -h 172.250.250.111 \
  -t esp32/1/srm \
  -f examples/emergency-srm-cancel.json
```

The intersection broadcasts an SSM on `esp32/1/ssm` with `processing`,
`granted`, `rejected`, or `maxPresence` status. Only one emergency request is
serviced at a time. A competing request is rejected rather than replacing the
active vehicle.

An accepted preemption obeys these model safety timings:

- At least 3 seconds of the current green before it may be terminated.
- The normal 2-second yellow and 1-second all-red clearance.
- At least 5 seconds of green for the emergency movement.
- Cancellation or automatic expiry before returning to normal/manual control.

During preemption, SPaT sets the `preemptIsActive` intersection-status bit and
reports unknown `minEndTime` values because a vehicle can extend or cancel its
request.

This is a model-scale J2735-shaped JSON/MQTT implementation. A road deployment
uses a V2X onboard unit and roadside unit, ASN.1 UPER encoding, signed
credentials, and an approved traffic-signal controller. Comparing `entityID`
on an anonymous MQTT broker is not secure authentication and must not be used
for a public-road installation.

- Data pin: GPIO 3
- Pixel count: 12
- Pixel protocol: GRB, 800 kHz
- Brightness: 64/255
- Board status RGB LED: GPIO 8

GPIO 8 drives the ESP32-C6-DevKitC-1 onboard addressable RGB LED and is also a strapping pin. Connect the external strip data input to GPIO 3 and connect the strip ground to ESP32 ground.

Build and upload from this directory:

```bash
pio run
pio run --target upload
pio device monitor
```
