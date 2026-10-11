# Multi-link video demo

A stand-alone module (no imports from `edge_mlc/`) that sends one camera over
several links so that **one link gives a lower-quality picture and every extra
link that delivers packets raises the quality**, with forward error correction
across the links.

## How it works

Every frame is encoded twice (`multilink/codec.py`):

* **Base stream:** a small JPEG (half the width and height). It goes whole on
  every link, so any single link shows a usable picture, upscaled by
  interpolation.
* **Full stream:** the full-resolution frame cut into 16-pixel stripes, each its
  own JPEG, packed whole into 1100-byte packets. Any packet that arrives
  sharpens its stripes on its own. The packets are the data of a systematic
  Reed-Solomon code (`zfec`), and parity packets (50% by default) let the
  receiver rebuild the whole frame once enough packets arrive from any links.

With two links, link 1 carries the base and the parity packets, and link 2
carries the base and the full-resolution stripes. The receiver shows the full
picture when it has enough packets, otherwise the upscaled base with every
stripe that arrived pasted on top, and holds the previous picture if nothing
arrived.

## Running it

    pip install -r requirements.txt
    mkdir -p samples            # put your clips here (git-ignored)
    python3 run_demo.py samples/<clip>.mp4

It runs the clip three times, always sending on two links, and changes only
what link 2 delivers:

| Case | Link 1 | Link 2 |
|---|---|---|
| `one_link` | all packets | none (link down) |
| `one_and_half_links` | all packets | half, dropped at random |
| `two_links` | all packets | all packets |

Outputs in `output/` (git-ignored): one video per case, `comparison.mp4` with
the three side by side, and `stats.json`. Options: `--seconds N` to use only
the start of a clip, `--parity 0.5` for the parity share, `--seed` for the
random drops. Videos are re-encoded to H.264 when `ffmpeg` is installed so
they play in a browser.

## Files

    multilink/codec.py   Sender (two encodes, stripes, FEC, per-link packets) and Receiver
    multilink/links.py   Link: drops a share of packets at random
    run_demo.py          the three-case demo
