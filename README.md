# IRIS — Hyperrealistic LEO Earth Simulator

Real-time 3D simulator of Low Earth Orbit (160–2000 km) with a hyperreal Earth,
exaggerated orbital shells for navigation, and a user-built fleet system:
independent active satellites, independent action bots,
and sensory bots attached to the active satellites.

No hardcoded satellites ship with the scene. You build the fleet yourself,
either with the on-screen Fleet Builder or from the console.

## Files

| File | What it is |
|---|---|
| `index-hyperreal-directx.html` | DirectX-path build. Same code, bigger Earth `R=1.5` (50% bigger than 1.0). |
| `DirectX-IRIS/` | Native DirectX 12 scaffold (`IRIS_DirectX12.cpp`, `shaders.hlsl`, CMake). Optional desktop path. |
| `libs/` | Local textures (`earth-day.jpg`, `earth-night.jpg`, `earth-topology.png`, `earth_clouds_1024.png`) + vendored three.js. |

Open any HTML file directly in Chrome/Edge — no build step.

## Quick start

1. Open `index.html` (or `index-hyperreal-directx.html` for the bigger Earth).
2. Drag to orbit, scroll to zoom through Kármán → VLEO → LEO Core → Upper LEO.
3. Open the left panel → **Fleet Builder**:
   - Click **+ Active Sat** to add an active satellite (defaults to 450–800 km LEO Core).
   - Click **+ Action Bot** to add an independent action bot (own orbit, defaults to 500–640 km).
   - Click an active satellite to select it, then click **+ Sensory Bot** to attach a sensory bot to it (else newest active).
   - **Clear Fleet** removes everything.
4. Hover any object for its tooltip. Click to track (neon orbit + tail) and fly to it.

## Fleet system

### Concepts

- **Active satellite** — independent Keplerian orbiter (green body + solar panels, orbit ellipse + trail). Label: `ACTIVE-1`, `ACTIVE-2`, …
- **Action bot** — independent Keplerian orbiter (orange cylinder, own ellipse + trail). Label: `ACTION-1`, `ACTION-2`, …
- **Sensory bot** — gold octahedron + sensor cone **attached to one active satellite** (child of its group: inherits the orbit plus a small hover bob). Label: `SENSORY-1`, …
- Deleting a satellite also deletes its attached sensory bots.
- Counters: top pill = sats + sensory bots total, `ACTIVE` = active sats, `SWARM` = action bots + sensory bots, `DEBRIS` = always 0 (no debris in the scene).

### UI

Left panel → Fleet Builder:

| Button | Effect |
|---|---|
| `+ Active Sat` | Spawns an independent active sat, selects it, flies the camera to it. |
| `+ Action Bot` | Spawns an independent action bot (`ACTION-n`), selects it, flies the camera to it. |
| `+ Sensory Bot` | Attaches a sensory bot (`SENSORY-n`) to the selected active sat, else the newest one. Warns if no active sat exists. |
| `Clear Fleet` | Removes all sats and bots. |

Sensory bots are spread around the parent with a golden-angle offset so multiples do not stack.

### Console API

Every build exposes `window.IRIS`:

```js
// Independent orbiters (all fields optional)
const s = IRIS.addActive({ alt: 550, inc: 53, raan: 10, argP: 0, M0: 0, name: "ACTIVE-ALPHA" });
const a = IRIS.addAction({ alt: 560, inc: 60, name: "ACTION-ALPHA" });

// Attached sensory bot (parent optional — defaults to selected, else newest active)
const b = IRIS.addSensory(s);       // or IRIS.addSensory() for auto-parent

// Inspect / remove
IRIS.sats;    // independent sats (active + action)
IRIS.bots;    // attached sensory bots (each has .parent, .kind, .id)
IRIS.remove(a);
IRIS.clear();
```

Options for `addActive` / `addAction`: `alt` (km), `e`, `inc`, `raan`, `argP`, `M0`, `name`.
Options for `addSensory(parent?, { name, bobAmp, offset })` — offset is a `THREE.Vector3`.

## Controls

| Input | Action |
|---|---|
| Left-drag | Orbit / rotate |
| Scroll / pinch | Zoom (dive through shells) |
| Right-drag | Pan |
| Arrow keys | Rotate camera (Shift = faster) |
| `WASD` + `Q`/`E`/`C`, Space | Roam fly (always on), up/down |
| `Shift+WASD` | Insane speed x25 |
| `F` | Toggle roam look mode |
| `X` | Intercept hovered object |
| `Delete`/`Backspace` (roam mode) | Dispose hovered object |
| `O` | Toggle orbit lines |
| `Esc` | Reset view |
| Double-click shell | Jump 180 km outward |

Sliders: Time (pause → fast-forward), FOV, Bloom, Roam speed.
Zone bar (right): teleport to Kármán / VLEO / LEO Core / Upper LEO / MEO / GPS / GEO.

## Scaling

Scene units are `u`. Real physics (velocity, period, ECI) always uses real km.

- DirectX builds: altitude exaggeration `30x`.
  - `1 km = 30/6371 = 0.004708u`. So `1u ≈ 212 km`.
  - `sceneRadius(alt) = R_EARTH + alt × 0.004708`.
  - `index-hyperreal-directx.html`: `R_EARTH = 1.5` → `0 km=1.5, 100 km=1.97, 160 km=2.25, 450 km=3.62, 800 km=5.27, 2000 km=10.92`.
  - `index.html`: `R_EARTH = 1.25` → `0 km=1.25, 100 km=1.72, 160 km=2.00, 450 km=3.37, 800 km=5.02, 2000 km=10.67`.
- Classic: true scale, `1 km = 1/6371 = 0.000157u`, Earth `R=1.25`.
- Satellite meshes are exaggerated to stay visible; tooltips show real size in meters and the exaggeration factor.

## Architecture notes

- `Sat` class (`type: 'active'` or `'action'`) owns a `THREE.Group` positioned each frame by solving Kepler's equation (`solveKepler` + `at(t)` mapped through `sceneRadius`). Orbit ellipse + trail line included.
- Sensory bots are plain meshes parented to `sat.group` with `baseOffset` + sinusoidal bob in `animate()`, scaled to 85% of the parent mesh scale.
- Selection: neon-green orbit + tube + tail (`selectSat`/`deselectSat`), camera fly-to (`focusSat`, bots resolve to parent).
- Hover: single raycast over sat meshes + bot meshes (`resolveHover`).
- Counts: `updateFleetCounts()` drives `pillCount` / `activeCount` / `swarmCount`.
- Layer volumes are empty hidden groups (no hardcoded particles); shells are wireframe spheres from `LEO_SHELL_DEFS`.
- Earth: high-tess sphere with day/night/bump textures, procedural water mask, custom GLSL (WebGL2 path), atmosphere shells, bloom composer.

## Native DirectX 12 folder

`DirectX-IRIS/` is a separate optional C++ scaffold, not required for the web sim. See `DirectX-IRIS/README.md` for the Visual Studio / CMake setup.
