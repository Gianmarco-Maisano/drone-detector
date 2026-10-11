# DroneDetect

DroneDetect is a passive, spectrum-based detector for signals that may be associated with drone links such as DJI OcuSync and Lightbridge. It scans configurable sections of the 2.4 GHz and 5.8 GHz bands using the wideband spectrum baseband image. It does not decode a radio protocol and cannot reliably identify a drone: Wi-Fi, video links, interference, and other transmitters can produce similar spectrum patterns.

The default profile starts on 5.8 GHz, uses the 5725-5850 MHz scan range, and requires a 14 dB margin over the estimated noise floor. The band button cycles through 2.4 GHz, 5.8 GHz, and Both. Both mode alternates between bands after the configured dwell interval.

## Screen Controls

| Control | Default | Range or choices | Description |
| --- | --- | --- | --- |
| Band | 5.8 GHz | 2.4 GHz, 5.8 GHz, Both | Selects the scan band. Both alternates between the bands. |
| Beep | On | On/Off | Enables or mutes confidence alarm beeps. |
| WF | On | On/Off | Enables or hides the scrolling waterfall display. |
| Margin | 14 dB | 3-40 dB | Minimum level above the per-slice estimated noise floor for a bin to count as signal. Higher values reject weaker signals and noise. |
| Width | 3 MHz | 1-20 MHz | Minimum contiguous above-threshold signal width required for a detection. |
| Alarm | 70% | 20-95%, steps of 5 | Confidence required to activate the alarm and increment Hit. |
| Hold | 15 sweeps | 1-60 | Number of sweeps to keep an alarm active after confidence falls below the alarm threshold. |
| Both dwell | 3 seconds | 1-60 seconds | Time spent on each band while Band is set to Both. |
| 2.4 L / 2.4 V | 32 / 32 | Hardware gain ranges | LNA and VGA receive gain used while scanning 2.4 GHz. |
| 5.8 L / 5.8 V | 32 / 32 | Hardware gain ranges | LNA and VGA receive gain used while scanning 5.8 GHz. |
| 2.4 start / end | 2400 / 2483 MHz | 1000-6000 MHz | Lower and upper scan limits for 2.4 GHz. Start is kept below end. |
| 5.8 start / end | 5725 / 5850 MHz | 1000-6000 MHz | Lower and upper scan limits for 5.8 GHz. Start is kept below end. |

## Readouts

- **Status** shows the current classification and confidence: `DRONE!` at 75% or above, `LIKELY` at or above the alarm threshold, `WiFi?` or `SIGNAL` for lower-confidence activity, and `---` when no recent signal is present.
- **Hit** counts transitions into the alarm state. It resets when the band mode is changed.
- **Level** shows the strongest observed signal's SNR and center frequency.
- **SNR** is a bar for the strongest signal's SNR, scaled to 0-40 dB.
- **Conf** is the heuristic confidence bar, scaled to 0-100%.
- The frequency line shows the slice currently being measured.
- The waterfall shows received power across the selected band over time.

## Detection Heuristic

The receiver measures overlapping 20 MHz spectrum slices. For each slice it estimates noise from the 25th percentile of spectrum bins, then detects contiguous bins above the noise estimate plus Margin. Confidence is assembled from signal presence, frequency changes between sweeps, wide and narrow activity in the same sweep, activity in multiple slices, persistence, and SNR. Wide fixed signals near standard 802.11 channel centers receive a penalty, and Wi-Fi-like confidence is capped below the default alarm threshold.

The heuristic favors patterns associated with drone links but is not a classifier trained to distinguish transmitters. Results depend on antenna, location, gain, interference, and the signal mode. Treat the display as an RF activity indicator, not proof that a drone is present.
