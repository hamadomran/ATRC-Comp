ATRC EDGE track · handover and multi-link video · 9 to 11 Oct 2026

# Multi-link video for the EDGE vehicle: how we got to two-stream FEC

The question started as "make predictive handover stronger". This report keeps every experiment that moved the decision, in the order they were run: first on simulator v1 (240 s scripted mission), then on simulator v2 (position-based 3 km round trip, now on `main`). The mission-based bandwidth allocation is left alone throughout.

## Decision

1. **Send the protected camera as two streams from the same capture, protected across links with forward error correction.** A small base stream (the mode's minimum level, for example 320×240 at 10 fps while driving) goes whole on every alive link, so any single link shows a usable picture. The full stream is Reed-Solomon coded across the links so that *any one link can drop out* and full quality survives. On simulator v2 with overlapping coverage this raised usable view from 89.9% to 93.6%, halved the longest interruption (22 s to 12 s) and cut distance driven without a usable view from 411 m to 247 m, while keeping full quality 80% of the time, for 26% extra data.
2. **Do not code the full stream so that it needs two links.** That is the purest form of "one link gives low quality, more links add quality", and it was tested first. With only two links alive it splits the full stream across both, so any hiccup on either link drops the view to the base stream: time at full quality fell to 62% and frame delay rose by 12 ms, for no gain in usable view over the cheaper base-layer copy.
3. **Keep one cheaper setting in the same sender.** The base-layer copy (full stream on one link, base stream on the others) gets the same usable view for 9% extra data instead of 26%, at the cost of lower quality during outages. It is the right setting on metered 4G. Both settings are the same code: two encodes plus a per-link parity count.
4. **Prediction stays, with a smaller job.** Across both simulators a perfect predictor removes fewer interruptions than sending on every link. Its remaining use is choosing which link carries the full stream first.
5. **Hardware.** A **Raspberry Pi 4 has a hardware H.264 encoder; a Raspberry Pi 5 does not** and must encode video in software on its CPU. Today's car code encodes MJPEG in software (OpenCV) on either board, so the second, smaller JPEG encode is cheap on both. The difference matters if the car moves to H.264 to save bandwidth: on a Pi 4 both streams can be encoded in hardware, on a Pi 5 they compete with everything else for the CPU (section 7).

## The trail at a glance

| # | Question | Experiment | What it settled |
| --- | --- | --- | --- |
| 1 | Is there room for a better predictor? | Two ground-truth oracles on v1's five scenarios | No: a perfect predictor gains 0.1 points of useful view |
| 2 | What does today's predictor do? | Prediction on vs off, paired by seed | −1.1 interruptions, but false switches triple |
| 3 | When handover can matter, what wins? | New overlap scenarios, ten strategies | Redundancy beats prediction; base-layer copy is cheapest |
| 4 | Does SpeedFusion-style FEC help? | Per-frame erasure code across links | Fixed 13–27% parity hurts; parity sized to lose one link works |
| 5 | Does it still hold on simulator v2? | Oracles and overlap scenarios ported to v2 (PR #3) | Yes: perfect prediction 144 → 119 interruptions, every link 106 |
| 6 | Can more links mean more quality? | Two-stream graded FEC on v2, 8 schemes | Yes, if the full stream survives one link loss; not if it needs two links |
| 7 | Can a Raspberry Pi do it? | Review of the car's video code and Pi encoders | Yes on both; Pi 4 has a hardware encoder, Pi 5 does not |

---

Part 1 · Simulator v1 (9 to 10 Oct)

## 1. The current scenarios leave nothing for handover to win

Simulator v1, 240 s scripted mission. I added two oracles that read the simulator's ground truth. One picks the link that will stay within limits for the next 0.5 s. The other sends the protected view on every link that is truly up. Both still run the real mode detector, allocator and speed caps. Averaged over the five repo scenarios (nominal, harsh handovers, heavy jamming, no 4G, fast fade), 20 seeds each:

| Configuration | Useful view | Interruptions | Driven blind | False switches | Extra data |
| --- | --- | --- | --- | --- | --- |
| Baseline (threshold, 3 checks) | 87.7% | 14.2 | 22.9 m | 0.02 | 0% |
| Proposed without prediction | 94.5% | 13.3 | 1.6 m | 0.22 | 0% |
| Proposed (current predictor) | 94.5% | 12.2 | 0.9 m | 0.68 | 1.6% |
| Oracle: perfect link choice | 94.6% | 12.1 | 0.8 m | 0.69 | 0% |
| Oracle: every truly-up link | 94.6% | 11.6 | 0.7 m | 0.17 | 46% |

The reason is the mission script in `sim/scenario.py`. From about 65 s to 200 s Starlink is the only link apart from a 4G patch at 80 to 96 s, so most Starlink handovers, the crater, and the jamming ramp happen with nowhere to switch to. The 152 to 158 s outage has no link at all. Tracing one run, the view gaps that remain for Proposed come from three places: no link exists, Starlink alone is lossy, or the vehicle changes manoeuvre and the camera needs a higher quality level before the mode detector catches up. None of these is a handover decision.

The first two rows also mean the gap between Baseline and Proposed comes almost entirely from mission-aware allocation and graceful degradation, not from prediction.

## 2. What the current predictor does

It does help a little. Against Proposed with prediction off, paired by seed, it removes 1.1 interruptions and 0.8 blind metres per mission, and it leaves the radio about 4 s before the radio actually fails. Its weaknesses are structural:

- **It fits the script, not the radio.** The trend rule (`proposed.py:79-98`) fires when at least 5 of the last 7 smoothed steps worsen over a 2 s window. That shape exists in the scenario as the scripted linear radio fade (35 to 65 s) and jamming ramp. Real fades with shadowing and multipath are rarely that monotone.
- **Its inputs are coarse.** Loss comes from 10 Hz heartbeats over a 3 s window (`monitor.py`), so it moves in steps of about 3% and lags. The 4G signal is polled once a second, which leaves about two distinct values in each 2 s trend window.
- **A wrong prediction is expensive.** A predicted failure moves the stream and blocks the old link for 3 s (`proposed.py:343-350`). That is where false switches go from 0.22 to 0.68 per mission.
- **The Starlink schedule rarely has anything to act on.** `scheduled_risk` (`proposed.py:121-130`) only duplicates when a backup exists, and in this mission Starlink is usually alone at handover time. Its window also runs from 0.5 s before to 1.5 s after each boundary, matching how the simulator injects outages. A September 2026 hour-long measurement on a Starlink Mini found the disruption in the 1 s *before* the 12, 27, 42 and 57 s marks, with outages of at most 200 ms and the p99 RTT rising from about 60 ms to 97 ms. That measurement was taken while stationary, so a moving vehicle may see more, but the window should be re-centred.
- **Faster probing does not change this.** Running heartbeats at 20 Hz all the time made no measurable difference (interruptions 12.2 vs 12.2).

## 3. When coverage overlaps, redundancy beats prediction

To give handover something to decide, I added two scratch scenarios, overlap and overlap_harsh. They are the nominal and harsh-handover missions with 4G present along the whole route at cell-edge quality, fading in and out of usability on a scale of seconds. Everything else is unchanged. The chart shows view interruptions per 240 s mission, averaged over both scenarios and 20 seeds each (lower is better). Dashed bars are oracles.

**−37%**interruptions with a base-layer copy on all alive links (10.6 → 6.6), paired difference −3.9 ± 1.0

**−6%**interruptions from a *perfect* predictor in the current switch-or-duplicate design (10.6 → 9.9)

**+0.4 pp**useful view for the base-layer copy (97.7% → 98.2%); the oracle ceiling is 98.4%

**13–25%**extra data for the base-layer copy (repo scenarios / overlap scenarios)

Why the copy wins: once a frame goes out on two or three links, the first copy to arrive is shown, so a link that drops for 300 ms costs nothing and there is no switching decision to get wrong. Prediction only helps if it is right before each event, and the risk signal used to trigger a copy also has to find a backup that already looks healthy. At cell edge the 4G link often fails that test even though it would have delivered the frame. Sending on every link that is merely alive avoids both problems.

The copy can be cheap because the protected view's minimum useful level (L2, 300 kbps in DRIVE) is much smaller than the full level (L5, 2 Mbps). A full-quality copy on every link does slightly better (6.1) but costs 46 to 57% extra data, which is what Baseline + duplicate video already pays for a much smaller gain.

Base-layer copies are modelled as the same frames at the lower level's bit rate (simx.py). On the car that means a second, lower-resolution encode of the protected camera, or a scalable codec.

## 4. Cross-link forward error correction

Question: would SpeedFusion-style FEC beat plain copies, and can it be improved? I added a per-frame erasure code to a scratch copy of the simulator. Each camera frame is cut into k fragments of about 1100 bytes, parity fragments are added, and all of them are spread across the alive links. The operator sees the frame once any k fragments arrive. Recovering the missing fragments from parity is polynomial interpolation (Reed-Solomon), which is probably the "interpolation" in the question. Because MJPEG frames are independent, the code works inside one frame and adds no buffering delay.

| Overlapping coverage, 2 scenarios × 20 seeds | Interruptions | vs Proposed | Extra data | Avg quality level | p95 frame delay |
| --- | --- | --- | --- | --- | --- |
| Proposed (no redundancy) | 10.6 | – | 2% | 4.06 | 62 ms |
| SpeedFusion-style FEC, Low (13% parity) | 13.1 | +2.6 ± 1.1 | 15% | 3.85 | 77 ms |
| SpeedFusion-style FEC, High (27% parity) | 12.3 | +1.8 ± 0.9 | 26% | 3.87 | 78 ms |
| Risk- and mode-adaptive FEC (my first attempt) | 9.8 | −0.8 ± 0.8 | 33% | 3.94 | 93 ms |
| Base-layer copy on all alive links (section 3) | 6.6 | −3.9 ± 1.0 | 25% | 3.82 | 67 ms |
| FEC sized to survive losing any one link | 6.2 | −4.4 ± 0.8 | 42% | 4.20 | 65 ms |
| Full copy on all alive links | 6.1 | −4.5 ± 0.9 | 57% | 4.25 | 62 ms |
| Oracle: every truly-up link | 5.6 | −5.0 ± 0.9 | 57% | 4.27 | 62 ms |

- **SpeedFusion's fixed levels make things worse when spread across links.** 13% or 27% parity is sized for random packet loss on one path. Spread over several links, it cannot cover a whole link dropping out, and every frame now depends on more than one link.
- **The improvement is to size parity to the link, not to the packet loss.** With n links each carrying 1/(n−1) of the frame, any one link can vanish and the frame still decodes. That is 50% parity with three links and the same as a copy with two. It removes almost as many interruptions as full copies (−4.4 vs −4.5) for 15 points less extra data, and unlike the base-layer copy it keeps full quality through a single-link outage (average level 4.20 vs 3.82).
- **Making parity follow the predictor did not work.** Raising parity only when a link looked risky missed the outages that were not predicted, the same lesson as section 3. A hybrid (FEC with 3+ links, base-layer copy with 2) landed between the two at 32% extra data, with no gain over the base-layer copy.

Where this left things on 10 Oct: base-layer copy is still the cheapest per interruption removed. Survive-one-link FEC is the choice when the view must keep its quality during a handover, and costs about 17 points more data. On the car it needs a small fragment-and-reassemble layer in the Pi and hub transport (Reed-Solomon libraries exist for Python), and it touches no allocation code beyond the per-link load.

---

Part 2 · Simulator v2 (11 Oct)

## 5. Re-checking on simulator v2

The mission-aware branch replaced the simulator: link quality now follows the vehicle's position on a 3 km round trip (about 22 minutes), 4G comes from two towers with RSRP-based quality, the vehicle uploads data, and usable view is scored against a fixed operator requirements table that the controllers never see. It also dropped the oracles and overlap scenarios, so they were ported to v2 (PR #3) and rerun, 3 seeds nominal and 2 per variant.

| Interruptions per mission | Nominal | Overlap | Overlap, harsh handovers |
| --- | --- | --- | --- |
| Baseline | 74 | 133 | 177 |
| Proposed without prediction | 48 | 120 | 150 |
| Proposed | 47 | 108 | 144 |
| Oracle: perfect link choice | 41 | 107 | 119 |
| Oracle: every up link (+26–35% data) | 48 | 106 | 106 |

The picture from part 1 holds. On the nominal mission a perfect predictor would remove about 6 of 47 interruptions. Where coverage overlaps and Starlink handovers are harsh, sending on every link does better than perfect prediction. Interruption counts are much higher than in v1 because the mission is about five times longer and the operator requirements are stricter.

## 6. Two streams with graded FEC

The idea under test: send redundancy so that one link alone gives a lower-quality picture and each extra link raises the quality (known in the literature as multiple description coding). On a Raspberry Pi the practical form is two encodes of every captured frame, a small base stream and the full stream, rather than a layered codec. Eight ways of spreading the protected camera over the alive links were run on v2, all on top of the real Proposed decider (mode detection, allocation, prediction and uploads unchanged), 5 seeds per scenario.

- **Base-layer copy:** full stream on Proposed's link, base stream on every other alive link.
- **Full copy:** full stream on every alive link; the first copy wins.
- **FEC, survive one link loss:** full stream coded over n links so that any n−1 rebuild it (the part 1 winner).
- **Two-stream, full needs any 2 links:** base whole on every link; full coded so any two links rebuild it. One link gives base, two or more give full.
- **Two-stream, full survives one loss:** base whole on every link; full coded so any n−1 links rebuild it. With two links that is a full copy on each; with three, full needs any two and base still shows if only one survives.

"Full quality" below means the shown picture is 640×480 (ladder L4 or L5). A frame counts once, at its first arrival; a later full-quality copy of the same capture upgrades what is shown. Overlapping-coverage results (overlap and overlap with harsh handovers, 10 paired runs per row):

| Scheme | Usable view | Longest gap | Driven without view | Time at 640×480 | Extra data | p95 frame delay |
| --- | --- | --- | --- | --- | --- | --- |
| Proposed (no redundancy) | 89.9% | 22.4 s | 411 m | 77.8% | 0.3% | 74 ms |
| Base-layer copy | 93.4% | 10.9 s | 259 m | 73.3% | 9.1% | 76 ms |
| FEC, survive one link loss | 93.4% | 12.5 s | 258 m | 81.0% | 22.3% | 76 ms |
| Two-stream, full needs any 2 links | 93.5% | 10.8 s | 253 m | 62.3% | 18.3% | 88 ms |
| Two-stream, full survives one loss | 93.6% | 12.2 s | 247 m | 80.4% | 26.1% | 78 ms |
| Full copy on every link | 93.5% | 11.7 s | 250 m | 81.3% | 33.2% | 77 ms |
| Oracle: perfect link choice | 90.7% | 17.0 s | 355 m | 78.2% | 0% | 77 ms |
| Oracle: every up link | 93.6% | 10.1 s | 242 m | 81.9% | 34.0% | 79 ms |

Every redundancy scheme lands at 93.4–93.6% usable view, so the chart compares what separates them: how often the picture is at full quality, and what that costs. Hover a point for all three numbers.

- **Any redundancy beats any prediction here.** Every scheme that sends on more than one link gains 3.5 points of usable view over Proposed and halves the longest gap; the perfect-prediction oracle gains 0.8 points.
- **The "needs any 2 links" design loses on quality.** Most of the time only two links are alive, and then the full stream is split across both. Losing a packet burst on either link drops that frame to base quality, and the frame waits for the slower link. Time at full quality falls from 78% to 62% and p95 delay rises from 76 to 88 ms. It is dominated by the plain base-layer copy, which costs half the data.
- **Coding the full stream to survive one link loss fixes it.** Full quality goes back to 80%, usable view is the best measured (93.6%), and the base stream still covers the case where only one link survives.
- **The base stream did not add a measurable gain over plain survive-one-loss FEC in this simulator** (93.6% vs 93.4% usable, within noise), and costs 4 points of data. Two of three links failing together is rare in these scenarios. It is kept because real outages are more correlated than the simulator's (the same tree line blocks Starlink and 4G), and because the base stream is the code the cheap setting needs anyway.
- **Interruption counts did not separate the schemes.** All redundancy schemes cut interruptions by 12 to 26 per mission against Proposed, but the 95% intervals (±10 to 15) overlap, so the table above uses steadier measures.

On the three repository scenarios without overlapping coverage (nominal, harsh handovers, fast fade), every scheme gains only 0.3 to 0.4 points of usable view, because there is rarely a second link to use. The two-stream design is a coverage-overlap feature, and costs little where there is no overlap: with one alive link it sends exactly what Proposed sends.

## 7. Building it on a Raspberry Pi

**What the car does today.** `common/video.py` encodes every frame as a JPEG with OpenCV (`cv2.imencode`), splits it into 1100-byte fragments and sends a whole copy on each listed link. That encode runs in software on the CPU on any Pi. Because MJPEG frames are independent, FEC can work inside one frame with no extra buffering.

|  | Raspberry Pi 4 | Raspberry Pi 5 |
| --- | --- | --- |
| Hardware video encoder | **Yes:** H.264, up to 1080p30 | **No:** H.264 is encoded in software on the CPU |
| Today's MJPEG path (OpenCV) | Software, CPU | Software, CPU (faster cores) |
| Two-stream MJPEG (640×480 + 320×240) | Fine: the second encode is a quarter of the pixels | Fine, with more CPU headroom |
| Two-stream H.264 (to cut bandwidth) | Both streams can use the hardware encoder; CPU stays free for FEC and control | Both streams on the CPU, competing with FEC, control and uploads |

CPU headroom for each case is inferred, not measured. `tools/measure_ladder.py` already times the JPEG encodes and should be run on the car's Pi before committing.

1. **Second encode.** In `Camera`, encode each captured frame twice: the full level and the mode's minimum level (resize first). Tag fragments with a stream id (base or full).
2. **Erasure code.** For the full stream, add Reed-Solomon parity so that any n−1 of n links' fragments rebuild the frame (`zfec` is a C library with Python bindings; a few Mbps is trivial for it). Send the base stream whole on every alive link.
3. **Reassembly at the hub.** Per frame, show the full stream as soon as enough fragments arrive; if it cannot be rebuilt by the frame deadline, show the base stream. Never wait for a slow link past the deadline.
4. **One setting, two modes.** A per-link parity count switches between the cheap base-layer copy (no full-stream parity, about 9% extra data) and survive-one-loss (about 26%). The allocator must reserve the extra load on each link, which is your friend's code, so agree the interface first.

---

## 8. Every option reviewed

| Approach | Where it comes from | Tested here | Result | Verdict |
| --- | --- | --- | --- | --- |
| Threshold switching with hysteresis | 3GPP ATSSS, Peplink failover | Repo baseline | Check count (1 to 8) makes no difference | baseline |
| Trend extrapolation, time-until-unusable | Current Proposed | Yes | −1.1 interruptions, triples false switches | rework |
| Known Starlink handover schedule | Measurement studies (Mohan et al. WWW 2024; 2026 Starlink Mini test) | Yes, on its own | No gain: nothing to switch to when Starlink is alone | keep, re-centre window |
| Faster link probing (20 Hz) | SRT link-stability timeout idea | Yes | No change | skip |
| Risk-triggered make-before-break copy | SRT main/backup bonding | Yes | Same view, false switches 0.68 → 0.17, 1.7% data | small win |
| Same, with perfect risk knowledge | Upper bound for any predictor | Yes (oracle) | −0.7 interruptions only | limits prediction |
| Slow down before a Starlink handover with no backup | Predictive QoS for tele-operated driving | Yes | About 1.5 pp less progress, blind distance barely changes (0.76 to 0.71 m) | skip |
| Minimum-quality copy on every alive link | Simulcast / SVC base layer; Peplink WAN smoothing in lighter form | Yes | −37% interruptions with overlap, 13 to 25% data | cheap setting |
| Full copy on every alive link | ATSSS redundant mode, Peplink WAN smoothing | Yes | −43% interruptions, 46 to 57% data | if data is free |
| SpeedFusion-style fixed FEC (13% / 27%) | Peplink SpeedFusion | Yes (v1) | +1.8 to +2.6 interruptions | skip |
| FEC sized to survive one link loss | Erasure coding across paths | Yes (v1, v2) | Near full-copy results for less data | superseded |
| Two streams, full needs any 2 links | Multiple description coding, priority encoding transmission | Yes (v2) | Time at full quality 78% → 62%, +12 ms delay | skip |
| Two streams, full survives one link loss | The same, with parity sized per link | Yes (v2) | Usable view 89.9% → 93.6%, longest gap 22 → 12 s, 26% data | decision |
| Layered codec (H.264 SVC, VP9 spatial layers) | Textbook layered video | No | Pi hardware encoders cannot produce it; software VP9 SVC is likely too heavy | skip |
| Redundancy only when it would matter | XLINK (Alibaba, SIGCOMM 2021): 2.1% overhead vs 15%, rebuffering −24 to −68% | No | Natural next step: upgrade the copy to full quality when risk is high | next step |
| Packet-level multipath scheduling | MPTCP / multipath QUIC schedulers, SRT broadcast bonding, LiveU-style bonding | No: simulator works per stream | Removes the idea of a switch; costs a transport rewrite | phase 2 |
| Learned QoS prediction from radio maps | Schippers et al. (TU Dortmund): map plus live signals beat live-only by up to 40% | No: needs logged data | The simulator replays conditions by time, so a map predictor would be trivially perfect there and prove nothing | phase 2 |
| Communication-aware motion planning | RCAMP and similar robotics work | No | Changes the route, not the link | out of scope |

## 9. Better inputs for prediction on the real vehicle

If prediction moves to choosing which link carries the full stream first, these signals are worth logging now so a model can be fit later:

- **Starlink:** the dish's local gRPC status and per-second history (obstruction fraction, ping drop rate, latency, outage events), readable with the open-source starlink-grpc-tools. Combine with the 15 s clock schedule.
- **Mesh radio:** per-neighbour RSSI, SNR and MCS from the radio, plus GPS distance to the base station. Signal margin and its slope through a Kalman filter is a much stronger predictor than heartbeat loss.
- **4G:** RSRP, RSRQ and SINR, plus serving-cell changes, from mmcli --signal-get or modem AT commands, rather than the single signal-quality percentage used now.
- **History:** a coarse radio map from a recon pass. This works on the car but cannot be credited in the simulator, as noted above.

## 10. Next steps

1. **Agree the allocator interface with your friend.** The extra per-link load (base stream plus parity) must be reserved in allocation, and the bulk upload budget has to see it. In these experiments it was not reserved, as for the existing duplicate-video option.
2. **Put the two-stream sender in the simulator properly.** Today it is a scratch subclass (handover-study/v2/mdc.py); moving it into sim/engine.py and the decider makes it part of the results everyone sees.
3. **Build it on the car.** Second encode, Reed-Solomon parity and deadline-based reassembly (section 7). Measure encode time on the car's Pi, and confirm whether it is a Pi 4 or a Pi 5.
4. **Re-scope the predictor.** Use it to choose which link carries the full stream first, and re-centre the Starlink window to the second before each 12/27/42/57 s boundary.

## Caveats

- All numbers are from the repository's simulators, not hardware. The overlap scenarios are an assumption about cell-edge 4G, added so that handover can matter.
- Part 1 uses 20 seeds per scenario with ± as 95% intervals on paired differences. Part 2 uses 3 to 5 seeds per scenario, so differences under about 1 point of usable view or 15 interruptions are within noise.
- In part 2 the redundant load was not reserved by the allocator, and the base stream is modelled as frames at the lower ladder level's bit rate. Real JPEG sizes come from tools/measure_ladder.py.
- The simulator's link outages are mostly independent. Correlated outages (one obstacle blocking two links) would favour keeping the base stream on every link.
- Extra data matters on metered 4G, and a second encode costs CPU; both should be measured on the car.

---

## Files and sources

Part 1 code and per-run results are in the project folder at handover-study/ (exp.py, mbb.py, simx.py, fec.py, results_runs.csv, results_fec_runs.csv). Part 2 code and results are in handover-study/v2/ (mdc.py, results_mdc_runs.csv). The v2 oracles and overlap scenarios are in PR #3.

- [Raspberry Pi Ltd, H.264 encoding performance on Raspberry Pi 5 series computers (software encoding whitepaper)](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-010033-WP-1-H.264%20encoding%20performance%20on%20Raspberry%20Pi%205_series%20computers.pdf)
- [zfec: fast erasure codec with Python bindings](https://github.com/tahoe-lafs/zfec)
- [Observing Starlink Mini's 15-second handover (Classmethod, Sept 2026)](https://dev.classmethod.jp/en/articles/starlink-mini-15-second-handover/)
- [Mohan et al., A Multifaceted Look at Starlink Performance (WWW 2024)](https://arxiv.org/abs/2310.09242v2)
- [Zheng et al., XLINK: QoE-Driven Multi-Path QUIC Transport in Large-scale Video Services (SIGCOMM 2021)](https://conferences.sigcomm.org/sigcomm/2021/files/papers/3452296.3472893.pdf)
- [SRT main/backup bonding design notes](https://github.com/jeandube/srt/blob/master/docs/features/bonding-main-backup.md)
- [Peplink SpeedFusion whitepaper](https://www.venntelecom.com/documents/13/SpeedFusion_Whitepaper_ver04-02.pdf)
- [Schippers et al., Predictive QoS for Future Tele-Operated Driving (TU Dortmund)](https://arxiv.org/pdf/2201.11432)
- [Peplink: Introducing Forward Error Correction (FEC) levels and overhead](https://forum.peplink.com/t/introducing-forward-error-correction-fec/19809)
- [starlink-grpc-tools](https://github.com/sparky8512/starlink-grpc-tools)
- [RCAMP: Resilient Communication-Aware Motion Planner](https://arxiv.org/pdf/1710.09303)