# Simulation

| File | Purpose |
|---|---|
| `gen_frames.py` | Builds synthetic 640x480 Bayer scenes and `tests.txt` |
| `tests.txt` | Test schedule: index, name, mode, distance (cm), expected class, frames |
| `tb_top.v` | Full-system testbench (camera, I2C, HC-SR04, LCD and VGA models) |
| `check_results.py` | Recomputes each frame with the Python model and compares |
| `sim.log` | Reference log from the included run |

Commands are in the main [README](../README.md#run-the-simulation).
`gen_frames.py` writes `img_*.hex` and `img_*.npy`, which are git-ignored
because they are regenerated on demand.
