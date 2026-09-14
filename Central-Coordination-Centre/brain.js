// Central Coordination Centre — brain skeleton
// Directs disposal, coordinates sensory → action loop.
// Loaded after the sim; talks to window.IRIS only. Fill in step by step.

class CentralCoordinationCentre {
  constructor(iris, config) {
    this.iris = iris || window.IRIS;
    this.config = config || { minSepKm: 5, riskRadiusKm: 12, deorbitPerigeeKm: 100 };
    // IBS detumble sizing (mirrors mapping_stage.py design values)
    this.ibs = Object.assign(
      { thrustN: 0.05, plumeEta: 0.7, targetBeamTimeS: 600, stopRadS: 0.05, thrusterN: 500 },
      (config && config.ibs) || {}
    );
    this.tasks = [];
    this.detumblePlans = [];
  }

  // inertia approx shared with mapping stage: solid sphere I = 2/5 m r^2
  debrisInertia(massKg, sizeM) {
    const r = Math.max(0.05, sizeM / 2);
    return 0.4 * massKg * r * r;
  }

  // 1. SENSE — sensory bots map debris orbits (TODO: fuse observations → exact orbit)
  sense() { return []; }

  // 2. RISK — rank debris→sat conjunctions (TODO: miss distance + Kessler score)
  assessRisk() { return []; }

  // 3. TASK — assign nearest free action bot to top risk (TODO)
  assign(risk) { return null; }

  // NEW — DETUMBLE PLAN: size IBS ion-beam force FROM mapped tumble speed.
  // More spin => more force/time. Returns { forceN, beamTimeS, ... }.
  // debris: { mass, size_m|sizeM, tumbleRadS|tumble_speed, alt }
  planDetumbleAndPush(debris) {
    const mass = debris.mass || 40;
    const sizeM = debris.size_m || debris.sizeM || debris.size || 0.8;
    const tumble = debris.tumbleRadS != null ? debris.tumbleRadS
      : (debris.tumble_speed != null ? debris.tumble_speed : 0.01);
    const alt = debris.alt || 550;
    const omega = Math.max(0, tumble); // full spin to bleed to 0 (stop spinning)
    const I = this.debrisInertia(mass, sizeM);
    const L = I * omega;
    const arm = Math.max(0.05, sizeM / 2);
    const torqueAvail = this.ibs.thrustN * arm * this.ibs.plumeEta;
    const beamTimeS = torqueAvail > 0 ? L / torqueAvail : Infinity;
    // calculated force that stops it in targetBeamTimeS (scales with spin)
    const forceN = (arm * this.ibs.plumeEta * this.ibs.targetBeamTimeS) > 0
      ? L / (arm * this.ibs.plumeEta * this.ibs.targetBeamTimeS) : 0;
    // disposal push after spin ~= 0: retrograde drop to target perigee (crash/burnup)
    const MU = 398600.4418, RE = 6371.0;
    const r = RE + alt, rP = RE + (this.config.deorbitPerigeeKm || 65);
    const aNew = (r + rP) / 2;
    const vC = Math.sqrt(MU / r);
    const vNew = Math.sqrt(Math.max(1e-9, MU * (2 / r - 1 / aNew)));
    const dv = Math.max(0, vC - vNew);
    const plan = {
      tumbleRadS: tumble, omegaBleed: omega, inertia: I, angMomentum: L,
      forceN, beamTimeS, beamTimeSizedS: this.ibs.targetBeamTimeS,
      impulseNs: forceN * this.ibs.targetBeamTimeS,
      dvKmS: dv, perigeeKm: (this.config.deorbitPerigeeKm || 65),
    };
    this.detumblePlans.push(plan);
    return plan;
  }

  // NEW — DETUMBLE EXEC: IBS beam ON with calculated force until spin stops,
  // then hand to thruster push. Design-level: sim applies via case1Tick SHEPHERD.
  commandDetumble(bot, debris, plan) {
    plan = plan || this.planDetumbleAndPush(debris);
    // contract the sim understands: attach plan to the run, beam uses plan.forceN
    try {
      if (this.iris && this.iris.case1) {
        this.iris.case1.data.detumble = plan;
        this.iris.case1.data.detumbleBot = (bot && bot.label) || null;
      }
    } catch (_) {}
    return plan;
  }

  // 4. DISPOSE — vector-thrust: forward motion + downward push → diagonal decay (TODO)
  // NOTE: only call after detumble (spin ~= 0). Docked thruster pushes the stable
  // stack retrograde so perigee drops and debris crashes/burns; bot undocks + survives.
  commandDeorbit(bot, debris) {
    const plan = this.planDetumbleAndPush(debris);
    return this.commandDetumble(bot, debris, plan);
  }
}

window.CentralCoordinationCentre = CentralCoordinationCentre;
