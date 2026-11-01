# Hardware setup

## What you need

| Item | Notes |
|---|---|
| Terasic DE2-115 | Cyclone IV E `EP4CE115F29C7` |
| Terasic TRDB-D5M | 5 MP camera on the 40-pin GPIO header (JP5), header voltage set to **3.3 V** |
| HC-SR04 ultrasonic sensor | The classic 5 V version needs a level divider on ECHO; the 3.3 V HC-SR04P does not |
| VGA monitor + cable | 640x480 @ 60 Hz |
| Resistors | 1 kOhm and 2 kOhm for the ECHO divider |

## HC-SR04 wiring

| HC-SR04 pin | Connects to |
|---|---|
| VCC | 5 V |
| GND | Board GND (common ground is essential) |
| TRIG | `EX_IO[0]` |
| ECHO | `EX_IO[1]` through a divider: ECHO -> 1 kOhm -> `EX_IO[1]`, and 2 kOhm from `EX_IO[1]` to GND |

Mount the sensor right next to the camera lens, pointing the same way, so both
look at the same object.

## Camera pin mapping

The mapping inside the top level follows Terasic's DE2-115 camera reference
design:

| Signal | GPIO pin |
|---|---|
| PIXCLK | GPIO[0] |
| D[11:0] | GPIO[1], GPIO[3..13] |
| LVAL / FVAL | GPIO[21] / GPIO[22] |
| XCLKIN | GPIO[16] |
| RESET_N | GPIO[17] |
| SDATA / SCLK | GPIO[23] / GPIO[24] |

Double-check against the TRDB-D5M user guide for your board revision.

## Pin assignments

The Terasic pin-assignment file is not redistributed here. Use
`Assignments -> Import Assignments` with `DE2_115_pin_assignments.csv` from the
DE2-115 System CD. The top-level port names match that file.

## Troubleshooting

* **Red and blue swapped:** change the `BAYER` parameter of the top level.
* **Image upside down or mirrored:** set `READ_MODE2` in `d5m_i2c_config.v`
  (`0xC000` mirrors rows and columns).
* **Dark image:** raise the exposure with `SW[16:13]`.
* **No detection:** flip `SW[11]` to view the binary mask. The shape should be
  a clean solid blob. Pick the right `SW[1:0]` mode for your background.
* **Testing without the sensor:** `SW[12]` bypasses the distance gate.
