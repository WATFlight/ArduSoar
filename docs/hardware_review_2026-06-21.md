# Hardware Plan Review — ArduSoar (reply to hardware team)

Date: 2026-06-21
Re: `glider_autopilot_hardware_report.docx` (Matek F405-Wing-V2 based avionics plan)

## Verdict

The plan is solid and can be used as-is for the avionics/power core. The power
architecture in particular is well done: three independent high-current rails
(motor ESC / servo UBEC / Pi buck) with a common ground, and high-current paths
kept off the flight-controller signal wiring — that is exactly the failure mode
that kills flight controllers, and it's handled correctly. Matek F405-Wing-V2 is
a first-class ArduPilot target; airspeed (I2C), GPS, ELRS, and Pi MAVLink fit the
6 UARTs with room to spare.

Below are additions/decisions specific to **this project** (weather-driven soaring
+ Raspberry Pi strategic layer) and to **soaring** as a flight mode.

## Must add / decide

1. **Ground telemetry radio — biggest gap.**
   The BOM has an RC link (ELRS) and the Pi, but no independent air↔ground datalink.
   Autonomous soaring tests need live attitude / mode / SOAR-state / battery in
   Mission Planner or QGroundControl, and we must not put the safety-monitoring
   link on the Pi (if the Pi hangs, we're blind). The Pi-as-MAVLink-bridge is a
   nice-to-have, not a substitute.
   → **Add a cheap SiK radio (~$35, e.g. Holybro 915 MHz)** on a spare FC UART as
   an independent safety link.

2. **Temperature/humidity sensor — missing, and it's our differentiator.**
   The agreed sensor suite includes a temp/humidity sensor. In-flight lapse-rate
   measurement validates/corrects our W* thermal prior — it upgrades "weather
   prior" from forecast-only to onboard-verified.
   → **Add a BME280 or SHT31 (I2C, ~$5, a few grams)** on the existing I2C bus.

3. **Camera + video link — missing; pick a path.**
   The sensor suite lists a camera + video downlink; the BOM omits both.
   - **Pi CSI camera** (digital, over Pi WiFi/LTE) — better for our cloud-street /
     vision work. Recommended.
   - Analog FPV camera + VTX — traditional, but a second parallel system.
   → Not needed for first flights, but decide early; it drives Pi load and power.

## Soaring-specific concerns (raise with mechanical / motor team)

4. **Three motors is counter-intuitive for a glider.**
   Soaring depends on cutting power and gliding clean. Three props that don't fold
   create large drag when stopped, hurting glide ratio and the endurance ArduSoar
   exists to deliver.
   → Ask the motor team: why three? Can we use **folding props**? The ideal soaring
   platform is a **single motor with a folding prop**. The ESC must also let the
   motor fully stop and free-wheel.

5. **Airspeed is longitudinal-only vs the "lateral + longitudinal" requirement.**
   The Matek ASPD-4525 is a single differential-pressure sensor = forward airspeed
   only; it cannot measure sideslip. For ArduSoar / TECS, **longitudinal is
   sufficient** — recommend dropping the "lateral airspeed" requirement rather than
   adding a multi-hole probe (over-spec for our mission).

## Minor notes (non-blocking)

- **Pi 5 power**: feeding 5 V directly may trip the USB-C PD 5 A negotiation; power
  from the GPIO 5 V pins or set `usb_max_current_enable=1`. The Pololu 9 A has
  margin — keep wiring short/thick.
- **F405 flash**: F405-Wing is 1 MB; recent ArduPlane is tight on flash. SOAR is a
  core feature and will be present, but **don't plan on Lua scripting on the FC** —
  fine for us, since the heavy logic lives on the Pi.
- **Battery (3S/4S/6S) still open** — correctly flagged; it drives ESC / UBEC input
  / Pi buck input / connectors, so decide soon.

## Summary

Use the power + interface design as-is. **Add: SiK telemetry radio + temp/humidity
sensor** (both cheap, both already in the agreed sensor suite). **Decide the camera
path.** The highest-impact open item is **"three motors + folding props?"** with the
motor team — it directly determines how well the aircraft soars.
