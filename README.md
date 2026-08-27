# TBRe25 Suspension Analysis from Endurance Telemetry

Measured roll and pitch behaviour, suspension travel usage, and setup
conclusions for the Team Bath Racing Electric 2025 car, derived from a full
FSG25 endurance run (AiM logger, 20 Hz). MATLAB, toolbox-free, also runs in
GNU Octave.

The headline result is a suspension model validated against measured data,
and a camber recommendation that follows from it.

---

## What this analysis concludes about the car

### 1. The measured roll gradient agrees with the design model to 1.7%

Measured **0.712 deg/g** from damper telemetry (r = 0.904, n = 34,481)
against an independently predicted **0.724 deg/g** from the team's design
model. Two entirely separate routes — one from logged suspension motion, one
from geometry and rates — reach the same number.

This is the result that stands on its own: it depends on neither roll centre
height nor static camber, so it is unaffected by the input conflicts noted
in the limitations. Everything below follows from it and inherits those
dependencies.

### 2. The outside tyre is short of its working camber at the limit

At the measured peak of **2.08 g** the chassis rolls **1.80°** relative to
the road — 1.48° of suspension roll measured from the dampers, plus 0.32° of
tyre roll computed from rig-measured tyre vertical rates (186/187 N/mm).
Suspension travel compresses the outside front wheel **15.5 mm**.

From the −1.0° static camber run at FSG25, net camber at the road:

| camber gain | net camber, outside front, 2.08 g |
|---|---|
| −0.05 °/mm | +0.03° |
| −0.066 °/mm | −0.22° |
| −0.08 °/mm | −0.44° |
| −0.10 °/mm | −0.75° |

Against a Hoosier R20 working range of −1.5 to −2.5°, the outside tyre is
short in every case — at the shallow end of the gain range it reaches the
limit at **positive** camber. Achieving −1.5° at the road would need
−0.148 °/mm, well outside the design target range, so **the recommendation —
roughly 0.8 to 1.5° more static negative camber, or more camber gain from
geometry — does not depend on which camber-gain figure is correct.**

Tyre roll here is computed, not caveated: it contributes 0.32° of camber
loss that a suspension-only calculation misses entirely. The tyre-roll
figure (0.154 deg/g) independently agrees with the team's design submission
(0.171/0.174) to about 10%.

![Roll and pitch gradients](fsg_roll_pitch_gradients.png)

*Both axes 0.1–3 Hz band-passed. Left: suspension roll vs lateral
acceleration, 0.712 deg/g at r = 0.904. Right: pitch vs longitudinal
acceleration, braking and traction fitted separately — the two slopes are
visually indistinguishable, which is conclusion 6.*

### 3. The suspension model closes against measured data

| | N·m/deg |
|---|---|
| Springs (300 lb/in both ends, MR 0.90 / 0.69) | 844 |
| **Measured total, from the roll gradient** | **899** |
| Residual, attributable to the rear ARB | 55 |

No front ARB is fitted, so the rear bar is the only other elastic path.
Springs and ARB account for the measured roll stiffness — the design model
predicts the car that was actually driven.

### 4. Roll stiffness distribution: 59.5% front

Front 535, rear 364 N·m/deg including the ARB. Front-biased elastic load
transfer, i.e. an understeer-biased balance by design. The rear ARB is worth
**3.9 percentage points** of distribution at this setting — the balance
adjuster, quantified from data rather than assumed.

### 5. Travel headroom is asymmetric, and tighter than a single figure suggests

The pots read damper millimetres and the 55 mm limit is damper stroke, so
this calculation contains **no motion ratio anywhere** — it is unaffected by
the effective-versus-geometric motion-ratio question that qualifies other
results here.

Travel is not symmetric about static: static compression consumes 19.6 mm of
stroke at the front and 30.5 mm at the rear, so each axle has a different
binding constraint.

| | bump used | of available | rebound used | of available |
|---|---|---|---|---|
| FL | 13.7 mm | 39% | 14.1 mm | **72%** |
| FR | 12.8 mm | 36% | 13.8 mm | **70%** |
| RL | 11.5 mm | 47% | 12.7 mm | 42% |
| RR | 11.1 mm | 45% | 10.8 mm | 35% |

**Front rebound is the binding constraint** — only 19.6 mm exists and 72% of
it is used. At the rear it is bump, with 24.5 mm available. The car is not
running with a large uniform reserve, and a softer spring would erode the
tighter side further, so no rate reduction is recommended on travel grounds.

A single stroke-utilisation figure would have shown roughly half the travel
unused and pointed the opposite way. The asymmetry is the result.

![Travel usage](fsg_travel_usage.png)

*Damper position distribution per corner, dashed line = static. The
bimodality is roll: each corner's extended tail sits at ~1.1 g mean lateral
with left and right mirrored in sign.*

### 6. Two null results

- **Pitch is spring-dominated.** Braking 0.687 and traction 0.684 deg/g
  agree to 0.4% — no measurable anti-dive/anti-squat asymmetry. The response
  is linear and direction-independent.
- **The chassis is torsionally stiff relative to the suspension.** Front and
  rear roll gradients agree to 0.016 deg/g, 2.2% of total. No meaningful
  compliance in the roll path.

### Measured quantities, summary

| Quantity | Value | Basis |
|---|---|---|
| Roll gradient (suspension) | **0.712 deg/g** | r = 0.904, n = 34,481; design model 0.724 |
| Roll gradient (to road, incl tyre) | 0.866 deg/g | tyre roll 0.154 from rig-measured rates |
| Total roll stiffness | **899 N·m/deg** | inversion of the above |
| Roll stiffness distribution | 59.5% front | springs + rear ARB |
| Pitch gradient, braking / traction | 0.687 / 0.684 deg/g | fitted separately |
| Peak lateral acceleration | 2.08 g | — |
| Body roll at peak | 1.48° | — |
| Front rebound headroom used | 70–72% | of 19.6 mm available |
| Rear bump headroom used | 45–47% | of 24.5 mm available |

---

## How the measurement was made

**Roll and pitch from damper position.** Wheel displacement per corner from
calibrated damper travel and motion ratio; roll from the left-right
differential across the track, pitch from the front-rear differential across
the wheelbase. Gradients fitted against band-passed lateral and longitudinal
acceleration over moving samples only.

**Counts-to-mm calibration, derived from the data.** The logger carries both
raw ADC and calibrated damper streams. Fitting one against the other, with
the 88 ms inter-stream lag corrected and sample-held points excluded, gives
**−0.02260 mm/count**, agreeing across all four pots to **0.12%**, residual
0.92–1.19 mm per corner. One sensor type, one scale, derived rather than
assumed.

**Motion ratios verified against the telemetry.** On a torsionally stiff
chassis, front and rear roll angles must agree. That constraint is a direct
test of the motion-ratio pair: rear dampers move **77.8%** as much as the
fronts in roll per unit lateral g, which fixes the rear/front motion-ratio
ratio at **0.784**. The pair used here (0.90 / 0.69) gives 0.767 and
satisfies the front-rear roll agreement to 2.2%. This makes the roll gradient a verified measurement
rather than a value conditional on an assumed input.

**Accelerometer axes identified from kinematics, not from names.** The IMU on
this car is mounted rotated about the vertical axis, so the logger's channel
names do not correspond to the physical axes. Rather than rely on labels, the
tools identify the lateral channel by correlation against v·(yaw rate) —
centripetal acceleration, scale-invariant — requiring |r| ≥ 0.80 before use.
Measured: 0.974. The longitudinal channel is identified against dV/dt and
sign-corrected. Roll rate is identified the same way when needed.

**Sign convention verified at runtime.** Decreasing damper reading equals
compression, confirmed on every run against the hardest braking samples
(fronts must compress: +3.98 mm front, −9.07 mm rear). The analysis aborts
rather than produce numbers from an unverified convention.

**Raw ADC stream used throughout.** The logger's calibrated damper stream
sample-holds ~20% of moving samples with stalls up to 5.85 s, all four
channels freezing for identical durations — transport-level gaps padded at
the logger. Analysis uses the raw stream and reports calibrated-stream
integrity separately.

---

## Data integrity: automated channel health check

Every channel is screened before any of the above is computed. Three tests,
moving samples only:

| Test | Metric | Healthy | Faulty | Threshold |
|---|---|---|---|---|
| Flatline | fraction of zero consecutive diffs | ≤ 0.011 | 0.994 | > 0.20 |
| **Roll response** (primary) | \|r\| vs band-passed lateral acceleration | ≥ 0.59 | ≤ 0.19 | < 0.40 |
| Low-freq fraction | PSD power 0.1–1 Hz / power > 0.1 Hz | 0.94 | 0.25–0.82 | < 0.85 |

The primary test rests on a physical argument: a functioning damper channel
on a circuit must respond to roll. It is amplitude-invariant by design,
because a channel can fail while retaining normal amplitude.

**Validation.** 4/4 pass on the healthy session and 4/4 fail on a session
with confirmed sensor failures, with fault modes classified automatically —
a hard failure, an intermittent fault localised to t ≈ 30 s, and two
channels carrying brake-pressure-correlated interference rather than
suspension motion. **Zero false positives across 220 rolling healthy
channel-windows.** Synthetic fault injection covers six failure types; a
channel running at 50% amplitude passes all three tests and is documented as
a known miss, since catching it needs a cross-session amplitude baseline
this tool does not have.

---

![Channel health metrics](fsg_channel_health_metrics.png)

*All four corners of the healthy file against the three thresholds (dashed).
The low-frequency test has the tightest margin — 0.94 against a 0.85
threshold — which is why it is documented as file-level support rather than
a primary test.*

![Channel timelines](fsg_channel_timelines.png)

*Per-channel timeline with the verdict in each title. The two driving stints
sit either side of the driver change at t ≈ 1745–1911 s.*

## Limitations

- Roll and pitch measured from dampers are **suspension** angles
  (wheel-to-chassis); that is the correct quantity to compare against a
  springs-plus-ARB stiffness figure. Where roll relative to the road is
  needed — the camber conclusion — tyre roll is added from rig-measured
  tyre vertical rates rather than neglected.
- Gradients are fitted on 0.1–3 Hz band-passed signals — transient and
  corner-scale response, comparable to but not identical with a steady-state
  skidpad gradient.
- Static roll centres from a 2D front-view projection; migration under roll
  is not included. At ±4% per 10 mm this is the largest remaining
  sensitivity in the roll stiffness figure.
- Camber gain is taken from a kinematics solver; the camber conclusion is
  stated across the full design-target range precisely because of this.
- Motion ratio treated as constant through the stroke. It is in fact
  travel-dependent; the pair used is the telemetry-effective value at the
  operating point, verified by the front-rear roll consistency check.
- Static camber is the as-run value reported by the VD engineer present at
  the event. The camber conclusion is robust across the full design-target
  camber-gain range but scales directly with this value.
- Roll stiffness figures use roll centres derived from the current hardpoint
  set. Roll centre height enters the roll arm directly, so the measured
  stiffness and the ARB residual scale with it.
- **20 Hz logging** puts wheel hop (measured at 20–27 Hz on a four-post rig)
  far above Nyquist; adequate capture needs ≥60 Hz logging. Without
  documented anti-alias filtering ahead of the sampler, low-frequency
  content cannot be certified free of folded high-frequency energy. The
  low-speed/high-speed damper histogram split that drives damping decisions
  is not achievable at this rate, and is not attempted.
- Travel extremes are observed, not mechanical stop positions.

---

![Damper position spectra](fsg_damper_spectra.png)

*Every corner shows the same shape: energy collapsing above ~1 Hz with no
resolvable resonance peak. Included because the negative result is the
point — 20 Hz logging cannot support conventional damper frequency analysis
on this car.*

![Low-frequency damper velocity distribution](fsg_damper_velocity_lowfreq.png)

*Velocity distributions, ~2 Hz bandwidth. Symmetric and heavy-tailed, with
bump/rebound mean ratios of 0.89–1.06. These are **not** peak damper
velocities — the sign convention is labelled unverified on the axis, and the
low-speed/high-speed split that drives damping decisions is unavailable at
this logging rate.*

## Files

| File | Purpose |
|---|---|
| `tbre25_vehicle_data.m` | Single authoritative parameter set; every tool imports from it |
| `suspension_position_analysis.m` | Roll/pitch gradients, travel usage, static checks |
| `roll_stiffness_inversion.m` | Roll stiffness from the gradient; comparison against springs + ARB |
| `suspension_conclusions.m` | Produces every conclusion quoted above, from the measured gradients |
| `damper_channel_health.m` | Automated channel screening |
| `validate_channel_health.m` | False-alarm sweep and synthetic fault injection |
| `damper_frequency_response.m` | Spectra and low-frequency velocity distributions |

Run the health check first; the remaining tools are only meaningful on
channels that pass it. `suspension_conclusions.m` reproduces every figure in
the conclusions section above — no number in this README is asserted without
code that computes it.

All code self-authored. Telemetry courtesy of Team Bath Racing Electric;
raw data not published.
