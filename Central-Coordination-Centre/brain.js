// Central Coordination Centre — brain skeleton
// Directs disposal, coordinates sensory → action loop.
// Loaded after the sim; talks to window.IRIS only. Fill in step by step.

class CentralCoordinationCentre {
  constructor(iris, config) {
    this.iris = iris || window.IRIS;
    this.config = config || { minSepKm: 5, riskRadiusKm: 12, deorbitPerigeeKm: 100 };
    this.tasks = [];
  }

  // 1. SENSE — sensory bots map debris orbits (TODO: fuse observations → exact orbit)
  sense() { return []; }

  // 2. RISK — rank debris→sat conjunctions (TODO: miss distance + Kessler score)
  assessRisk() { return []; }

  // 3. TASK — assign nearest free action bot to top risk (TODO)
  assign(risk) { return null; }

  // 4. DISPOSE — vector-thrust: forward motion + downward push → diagonal decay (TODO)
  commandDeorbit(bot, debris) {}
}

window.CentralCoordinationCentre = CentralCoordinationCentre;
