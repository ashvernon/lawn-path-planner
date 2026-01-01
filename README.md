Yep — GitHub is treating your fences weirdly because something in the file is breaking Markdown parsing (most often: a missing closing ``` further down, or smart quotes/odd characters). Easiest fix: use **simple fences (no language tags)** and keep them isolated.

Paste this **entire README** exactly as-is:

```markdown
# Lawn Path Planner (MVP)

An interactive Python prototype that computes and visualises an efficient mowing path for an arbitrary lawn shape, given mower blade width and basic constraints.

This project explores **coverage path planning (CPP)** applied to residential lawn mowing (boustrophedon sweeping).

---

## Features

- Draw an arbitrary lawn boundary interactively
- Set mower blade width and planning resolution
- Automatically selects an efficient sweep orientation
- Visualises mowing lanes (planned stripes), traversal path, and coverage progress
- Estimates total mowing distance and time
- Runs fully locally (no GPS, no cloud)

---

## Screenshots

**Drawing the lawn shape**

![Drawing lawn shape](screen1.png)

**Planned mowing lanes and estimated time**

![Planned mowing lanes](screen2.png)

---

## How it works

1. Draw a lawn polygon
2. Rasterise the polygon to a grid
3. Evaluate multiple sweep angles
4. Score each candidate by:
   - Path length
   - Turn count (penalised)
5. Render the best plan

---

## Requirements

- Python 3.10+
- pygame
- numpy

---

## Install

```

pip install pygame numpy

```

---

## Run

```

python lawn_carer3.1.py

```

---

## Controls

### Drawing mode
- Left click (release): add point
- Hold left mouse + drag: draw shape quickly
- P: add point at cursor
- BACKSPACE: undo last point
- ENTER: compute mowing plan
- R: reset shape
- ESC: quit

### Planning / playback
- SPACE: pause / resume animation
- + / -: change animation speed
- R: redraw lawn

### Parameters
- [ / ]: decrease / increase blade width (m)
- , / .: decrease / increase grid resolution
- A: cycle sweep angle resolution
- 1 / 2: decrease / increase mower speed (m/s)
- L: toggle lane visualisation

---

## Output metrics

- Coverage percentage
- Approximate total travel distance (meters)
- Estimated mowing time (distance ÷ mower speed)
- Turn count and planning score

Note: turn time is not yet included in the time estimate.

---

## Limitations (current MVP)

- Flat terrain only
- No obstacles yet (trees, garden beds)
- Turn time not modelled
- Grid approximation (not continuous geometry)

---

## Status

Experimental MVP for exploration and prototyping.
```
