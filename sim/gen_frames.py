"""Builds synthetic camera scenes for the testbench.
For every test image it writes
   img_<n>.hex  : 640x480 raw 12-bit Bayer (GRBG) frame, one word per line
   img_<n>.npy  : the 320x240 RGB image (reference for the Python model)
and writes tests.txt describing the schedule used by tb_top.v."""
import sys, os, math, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'model'))
from shapes import shape
from scipy import ndimage
rng = np.random.default_rng(7)
H, W = 240, 320

def scene(mask, fg, bg_lo=165, bg_hi=215, noise=6.0, tint=(1.0, 1.0, 0.97)):
    yy, xx = np.mgrid[0:H, 0:W]
    grad = bg_lo + (bg_hi - bg_lo) * (0.6 * xx / W + 0.4 * yy / H)     # uneven lighting
    img = np.stack([grad * t for t in tint], -1)
    m = ndimage.gaussian_filter(mask.astype(float), 0.8)[..., None]      # soft edges
    img = img * (1 - m) + np.array(fg, float)[None, None, :] * m
    img += rng.normal(0, noise, img.shape)
    return np.clip(np.round(img), 0, 255).astype(np.uint8)

def to_bayer(rgb):
    raw = np.zeros((480, 640), np.int64)
    R, G, B = [rgb[..., i].astype(np.int64) for i in range(3)]
    n = lambda: rng.integers(0, 15, (H, W))          # sub-LSB sensor noise
    raw[0::2, 0::2] = (G << 4) + n()    # G1 (TL)
    raw[0::2, 1::2] = (R << 4) + n()    # R  (TR)
    raw[1::2, 0::2] = (B << 4) + n()    # B  (BL)
    raw[1::2, 1::2] = (G << 4) + n()    # G2 (BR)
    return raw

d = math.radians
dark = (38, 40, 52)
tests = [  # (name, mask, fg colour, mode, distance_cm, expected, frames)
    # gate opens only after two agreeing ultrasonic readings -> NONE at power-up
    ('warmup',   shape('circle', 150, 115, 55),                         dark, 0, 45, 'NONE', 2),
    ('circle',   shape('circle', 170, 125, 62),                         dark, 0, 45, 'CIRCLE', 3),
    ('square',   shape('square', 150, 120, 48, d(30)),                  dark, 0, 50, 'SQUARE', 3),
    ('rect',     shape('rect',   165, 118, 38, d(20), 1.8),             dark, 0, 60, 'RECTANGLE', 3),
    ('triangle', shape('tri_up', 160, 125, 60, d(15)),                  dark, 0, 40, 'TRIANGLE', 3),
    ('pentagon', shape('pent_up',155, 120, 58, d(10)),                  dark, 0, 70, 'PENTAGON', 3),
    ('hexagon',  shape('hex_flat',170,118, 58, d(20)),                  dark, 0, 30, 'HEXAGON', 3),
    ('far',      shape('square', 160, 120, 50, d(5)),                   dark, 0, 120, 'NONE', 4),
    ('edge76',   shape('hex_flat',160,120, 55, d(0)),                   dark, 0, 76, 'NONE', 4),
    ('edge75',   shape('square', 160, 120, 50, d(-15)),                 dark, 0, 75, 'SQUARE', 4),
    ('chroma',   shape('tri_up', 150, 120, 55, d(-25)),        (200, 45, 40), 2, 55, 'TRIANGLE', 4),
]
with open('tests.txt', 'w') as f:
    for i, (name, m, fg, mode, dist, exp, nfr) in enumerate(tests):
        rgb = scene(m, fg)
        np.save('img_%d.npy' % i, rgb)
        raw = to_bayer(rgb)
        with open('img_%d.hex' % i, 'w') as h:
            h.write('\n'.join('%03x' % v for v in raw.flatten()) + '\n')
        f.write('%d %s %d %d %s %d\n' % (i, name, mode, dist, exp, nfr))
        print(i, name, 'mode', mode, 'dist', dist, 'expect', exp)
