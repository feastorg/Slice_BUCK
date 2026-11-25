# Buck Converter / Analog Signal Generator Slice

Buck Converter / Analog Signal Generator Slice for BREAD.

## Introduction

A programmable DC supply on one slice: a P-FET high-side buck stage takes the
backplane's 12 V bus and gives 0-12 V on J2, set over serial and regulated by the
Nano on A0.

## Ratings

| | |
|---|---|
| Input | +12 V bus through J1 (pins 2 and 9). Nothing else. |
| Output | 0-12 V on J2 (pin 1 +, pin 2 GNDPWR). In regulation up to about 11 V at 3 A; above that the stage goes to 100 % duty, 11.7 V into 4 ohm (2.9 A) in simulation. |
| Current | Up to 3 A out. At 100 % duty the bus carries the full load current, 3 A, which is the per-slot budget. |
| Switching | 31.25 kHz from the Nano's D9 (Timer1). |
| Feedback | Vout through 20.0k / 10.0k to A0, so 12 V out reads 4.00 V. |

**12 V bus only.** The gate drive is the input voltage: with Q1 on, its gate sits
about 0.7 V above ground, so Vgs is about -(Vin - 0.7 V). The DMP6023LE is rated
for +/-20 V gate-source, so this stage must not be fed from a 24 V bus. A bus
clamped at 15 V still gives -14.3 V, inside the rating.

**The output is regulated by firmware, not by the hardware.** At light load the
stage runs discontinuous and the output rises toward the input at any fixed duty:
in simulation 50 % duty gives 5.8 V into 4 ohm and 10.9 V into 1 kohm. Use SETV,
which regulates on A0. R9 (1k) keeps a minimum load so low voltages stay
regulable with nothing connected. The shortest on-time the stage makes is about
0.45 us (simulation), so with nothing else connected the lowest steady output is
about 1.4 V; SETV below that holds the average by toggling between that pulse and off.

At reset, or with no sketch running, D9 floats and Q1 stays off (R3 holds Q3 off,
R4 pulls the driver to +12 V).

## Usage

Serial at 115200 baud, one command per line:

| Command | Does |
|---|---|
| `ON` | Enable the output (resumes SETV regulation or the last SET duty). |
| `OFF` | Disable the output (D9 low, Q1 off). |
| `SETV <0-12>` | Regulate Vout to this many volts. The setpoint ramps at 5 V/s. |
| `SET <0-255>` | Open-loop duty (0-255 scaled onto the 9-bit timer); leaves SETV. |
| `SETTOP <ohm>` | Calibrate the divider's top resistor (default 20000). |
| `STATUS` | Print the state; also printed once a second. |

Over-voltage cut-off: if Vout stays above 13 V, or more than 1 V above the SETV
setpoint while the stage is still switching, for 50 ms, the output turns off and
the sketch prints `ERR: OV`. Send `ON` to restart.

## Documentation

Please refer to the [docs](/docs) for more info.

The switch stage has a SPICE check in `hardware/sim/`: run `python3 hardware/sim/check.py`
from the repo root with `kicad-cli` and `ngspice` on PATH.

---

<div align="center">
  <p>
    <img src="https://img.shields.io/badge/Developed_by-UWO--FAST--et--al-orange" alt="Developed by UWO-FAST et al.">
    <img src="https://img.shields.io/badge/Powered_by-Open_Source-blue" alt="Powered by Open Source">
  </p>
</div>
