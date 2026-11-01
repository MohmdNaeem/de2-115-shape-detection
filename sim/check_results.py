"""Compares every per-frame result printed by tb_top.v ("RES ..." lines) with
the bit-exact Python reference model, and checks the final class of every
test against the expected shape.  Also converts the captured VGA frames to
PNG and builds a montage.

usage: python3 check_results.py sim.log
"""
import sys, os, re, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'model'))
import hw_model as M

log = open(sys.argv[1] if len(sys.argv) > 1 else 'sim.log').read().splitlines()
tests = {}
for line in open('tests.txt'):
    i, name, mode, dist, exp, nfr = line.split()
    tests[int(i)] = dict(name=name, mode=int(mode), dist=int(dist), exp=exp)

ok = True
n_frames = 0
print('%-9s %-5s %-4s %-4s %-10s %-10s %-22s %s' %
      ('test', 'frame', 'gate', 'T', 'RTL', 'model', 'bbox (RTL)', 'match'))
for line in log:
    if not line.startswith('RES '):
        continue
    kv = dict(re.findall(r'(\w+)=([\d,\-]+)', line))
    t = int(kv['test'])
    if t < 0:
        continue
    gate = int(kv['gate']); T = int(kv['T']); raw = int(kv['raw'])
    box = tuple(int(v) for v in kv['box'].split(',')); area = int(kv['area'])
    rgb = np.load('img_%d.npy' % t)
    f = M.feature_val(rgb, tests[t]['mode'])
    mask = M.mask_from(f, T) if gate else np.zeros((236, 316), bool)
    cid, bb, dbg = M.classify_mask(mask)
    m_area = int(mask.sum())
    same = (cid == raw)
    if cid != 0:
        same = same and bb is not None and tuple(int(v) for v in bb) == box and m_area == area
    ok &= same
    n_frames += 1
    print('%-9s %-5s %-4d %-4d %-10s %-10s %-22s %s' %
          (tests[t]['name'], kv['frame'], gate, T, M.CLASSES[raw], M.CLASSES[cid],
           str(box) if raw else '-', 'OK' if same else 'MISMATCH (model box %s area %d)' % (bb, m_area)))

print()
for line in log:
    if line.startswith('FINAL') or line.startswith('LCD'):
        print(line)
        if line.startswith('FINAL'):
            kv = dict(re.findall(r'(\w+)=(\S+)', line))
            exp = kv['expect']; got = M.CLASSES[int(kv['class'])]
            if exp != got:
                ok = False
                print('   ^^^ expected', exp, 'got', got)

# VGA frames -> PNG + montage
try:
    from PIL import Image, ImageDraw
    lcd_lines = [l for l in log if l.startswith('LCD')]
    tiles = []
    for t in sorted(tests):
        p = 'vga_%d.ppm' % t
        if os.path.exists(p):
            im = Image.open(p).convert('RGB')
            im.save('vga_%d.png' % t)
            tiles.append((t, im))
    if tiles:
        tw, th = 320, 240
        cols = 3
        rows = (len(tiles) + cols - 1) // cols
        th2 = th + 56
        mont = Image.new('RGB', (cols * tw, rows * th2), (20, 20, 20))
        d = ImageDraw.Draw(mont)
        for k, (t, im) in enumerate(tiles):
            x, y = (k % cols) * tw, (k // cols) * th2
            mont.paste(im.resize((tw, th)), (x, y + 20))
            d.text((x + 6, y + 4), 'scene: %s, distance %d cm' % (tests[t]['name'], tests[t]['dist']), fill=(230, 230, 230))
            if k < len(lcd_lines):                    # LCD text captured for this test
                parts = lcd_lines[k].split('|')
                d.rectangle((x + 4, y + th + 22, x + tw - 4, y + th2 - 4), fill=(60, 90, 40))
                d.text((x + 10, y + th + 25), 'LCD ' + parts[1], fill=(220, 255, 200))
                d.text((x + 10, y + th + 39), '    ' + parts[3], fill=(220, 255, 200))
        mont.save('../docs/images/vga_montage.png')
        print('\nwrote ../docs/images/vga_montage.png')
except ImportError:
    pass

print('\n%d frames compared, ' % n_frames + ('ALL CHECKS PASSED' if ok else 'FAILURES PRESENT'))
sys.exit(0 if ok else 1)
