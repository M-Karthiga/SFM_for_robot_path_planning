# Social Force Model for Autonomous Robot Navigation

> TurtleBot4 path planning using the Helbing & Molnar (1995) Social Force Model,
> A\* global planning, anticipatory dynamic obstacle avoidance, and anti-stuck recovery.

## Demo

(https://drive.google.com/file/d/1-EoEraLEPfZRc0cRlneEOtBvpLT7s2xx/view?usp=sharing)

*Robot navigates from Entry Hall → Corridor → Lab Room → Main Office in a
randomised floor plan with static and sinusoidally moving dynamic obstacles.*

---

## Overview

This simulation implements a two-layer navigation stack:

| Layer | Method | Role |
|-------|--------|------|
| Global planner | A\* on inflated occupancy grid | Collision-free waypoint path |
| Local controller | Social Force Model (SFM) | Real-time velocity control |
| Recovery | Anti-stuck mechanisms (5 fixes) | Escape local minima |

---

## Algorithm Summary

The robot's velocity is governed by a superposition of virtual forces:

$$\mathbf{F}(t) = \mathbf{F}_{\text{drive}} + \mathbf{F}_{\text{wall}} + \mathbf{F}_{\text{static}} + \mathbf{F}_{\text{dynamic}}$$

| Force term | Description |
|------------|-------------|
| Drive | Accelerates toward desired waypoint at relaxation time τ |
| Wall repulsion | Exponential decay from nearest wall point |
| Static obstacle | Exponential decay from obstacle surface |
| Dynamic reactive | Responds to current obstacle position |
| Dynamic anticipatory | Predicts collision via Time-to-Closest-Approach (TCA) |

For full mathematical derivation see [`docs/SFM_Robot_Navigation.pdf`](docs/SFM_Robot_Navigation.pdf).

---

## Environment

- **Domain**: 20 m × 14 m indoor floor plan (Entry Hall, Corridor, Lab Room, Main Office)
- **Static obstacles**: 6–9 randomly placed circular obstacles
- **Dynamic obstacles**: 3–5 agents following sinusoidal trajectories
- **Start**: randomised in Entry Hall | **Goal**: randomised in Main Office

  <img width="695" height="531" alt="image" src="https://github.com/user-attachments/assets/8361a167-62ff-4cf3-a1ae-169949cb03a6" />


---

## Requirements

- MATLAB R2016b or later
- No additional toolboxes required
- VideoWriter uses the built-in `MPEG-4` profile (Windows/macOS)

---

## Usage

```matlab
% Clone the repo, open MATLAB, navigate to src/, then:
run('run_single_agent_with_anti_stuck.m')
```

Set `SAVE_VIDEO = true` and update `videoPath` in the script to export MP4.
Each run is randomised via `rng('shuffle')` — no two runs are identical.

---

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `tau` | 0.4 s | Drive force relaxation time |
| `v_desired` | 0.55 m/s | Cruise speed |
| `vMax` | 0.8 m/s | Hard velocity cap |
| `A_w / B_w` | 7.0 / 0.16 | Wall repulsion strength / decay |
| `A_antic` | 5.0 | Anticipatory force magnitude |
| `tauAntic` | 1.5 s | TCA prediction horizon |
| `A_drive_max` | 5.0 | Anti-stuck max gain multiplier |

---

## Known Limitations

- Sinusoidal obstacle motion is not realistic pedestrian behaviour
- Euler integration (Δt = 0.05 s) accumulates small trajectory error
- No collision recovery if a dynamic obstacle moves into the robot
- Local minima can still occur in highly concave obstacle arrangements

---

## References

1. D. Helbing and P. Molnár, *Social force model for pedestrian dynamics*,
   Physical Review E, 51(5), 1995.
2. D. Helbing, I. Farkas, T. Vicsek, *Simulating dynamical features of escape panic*,
   Nature, 407, 2000.

---
