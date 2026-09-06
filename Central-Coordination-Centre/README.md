# Central Coordination Centre — Brain

Directs disposal and coordinates the fleet:
- Sensory bots → map debris orbits
- Action bots → rendezvous + vector-thrust deorbit
- Active sats → assets to protect (Kessler guard)

This folder is separate from the web sim (`../index-hyperreal-directx.html`).
Sim talks to it via `window.IRIS` (sats / bots / addActive / addAction / addSensory).

## Files
- `brain.js` — `CentralCoordinationCentre` skeleton. Wire it up step by step.
- `config.json` — thresholds (min separation, risk radius, deorbit target).
