"""CCS Stage 1 — PASS-BASED mapping (real-life style).
Debris flies through the sensory swarm. Only when it PASSES near a host sat
(detection range) does that bot record: its own exact position + debris exact
position/speed at that instant, and SEND to CCS. CCS STORES every pass.

Between passes CCS live-propagates coordinates from last speed+trajectory.
After the 2nd orbit completes, calculations trigger -> one unified result.
Trajectory = connected detection dots (no prior orbit knowledge).

Run:  python mapping_stage.py  (stdlib only)
"""

import json
import math
import os
import random
import time

MU = 398600.4418
R_EARTH_KM = 6371.0
RAD = math.pi / 180.0
DET_RANGE_KM = 1200.0   # pass = debris within this of a host sat
COOLDOWN_S = 90.0       # one packet per pass per bot

# --- Ion Beam Shepherd detumble design (Stage 2 input comes from Stage 1 mapping) ---
ION_THRUST_N = 0.05    # available IBS ion-beam thrust on debris (50 mN class)
PLUME_ETA = 0.7        # plume coupling efficiency (off-centre impingement)
TARGET_BEAM_TIME_S = 600.0  # desired detumble duration for force sizing
TUMBLE_STOP_RAD_S = 0.05    # detumbled threshold (matches sim CASE_1_CONFIG.tumble)
THRUST_N = 500.0       # docked chemical thruster for disposal push
TARGET_PERIGEE_KM = 65.0    # crash dive target (burns <135km, gone <65km)

LEO_RHO_TABLE = [
    (100, 5.0e-7), (120, 2.2e-8), (150, 2.0e-9), (160, 1.2e-9),
    (200, 2.5e-10), (250, 6.0e-11), (300, 1.8e-11), (350, 6.0e-12),
    (400, 2.8e-12), (450, 1.2e-12), (500, 5.5e-13), (550, 2.8e-13),
    (600, 1.5e-13), (700, 4.0e-14), (800, 1.5e-14), (1000, 3.0e-15),
    (1500, 5.0e-16), (2000, 1.2e-16),
]


def rho_moderate(alt):
    if alt <= LEO_RHO_TABLE[0][0]:
        return LEO_RHO_TABLE[0][1]
    for i in range(1, len(LEO_RHO_TABLE)):
        if alt <= LEO_RHO_TABLE[i][0]:
            a0, r0 = LEO_RHO_TABLE[i - 1]
            a1, r1 = LEO_RHO_TABLE[i]
            f = (alt - a0) / (a1 - a0)
            return math.exp(math.log(r0) + (math.log(r1) - math.log(r0)) * f)
    return LEO_RHO_TABLE[-1][1]


def solve_kepler(M, e):
    E = M
    for _ in range(6):
        E -= (E - e * math.sin(E) - M) / (1 - e * math.cos(E))
    return E


def eci_of(a, e, inc_d, raan_d, argp_d, M_d):
    inc, raan, argp = inc_d * RAD, raan_d * RAD, argp_d * RAD
    M = M_d * RAD
    if e < 1e-4:
        r, nu = a, M
    else:
        E = solve_kepler(M % (2 * math.pi), e)
        nu = 2 * math.atan2(math.sqrt(1 + e) * math.sin(E / 2),
                            math.sqrt(1 - e) * math.cos(E / 2))
        r = a * (1 - e * math.cos(E))
    p, q = r * math.cos(nu), r * math.sin(nu)
    co, so = math.cos(raan), math.sin(raan)
    ci, si = math.cos(inc), math.sin(inc)
    cw, sw = math.cos(argp), math.sin(argp)
    x = (co * cw - so * sw * ci) * p + (-co * sw - so * cw * ci) * q
    y = (so * cw + co * sw * ci) * p + (-so * sw + co * cw * ci) * q
    z = (sw * si) * p + (cw * si) * q
    return (x, y, z)


def eci_state(a, e, inc, raan, argp, M0, t, sf=1.0):
    n = math.sqrt(MU / a ** 3) * sf
    M = (M0 * RAD + n * t) / RAD % 360.0
    p1 = eci_of(a, e, inc, raan, argp, M)
    M2 = (M0 * RAD + n * (t + 1.0)) / RAD % 360.0
    p2 = eci_of(a, e, inc, raan, argp, M2)
    return p1, (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])


def sub(u, v):
    return (u[0] - v[0], u[1] - v[1], u[2] - v[2])


def norm(u):
    return math.sqrt(sum(c * c for c in u))


def cross(u, v):
    return (u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0])


def debris_inertia(mass_kg, size_m):
    """Solid-sphere approx: I = 2/5 * m * r^2, r = size/2."""
    r = max(0.05, size_m / 2.0)
    return 0.4 * mass_kg * r * r


def plan_detumble_and_push(mass_kg, size_m, tumble_rad_s, alt_km,
                           ion_thrust_n=ION_THRUST_N, eta=PLUME_ETA,
                           target_time_s=TARGET_BEAM_TIME_S):
    """NEW: tumbling-spin bleed plan from MAPPED tumble speed.

    Mapping (Stage 1) already measures tumble_speed per pass. This sizes the
    IBS ion-beam force from that spin: L = I*omega, torque = F*arm*eta.
      beam_time = L / (ion_thrust * arm * eta)
      force_req = L / (arm * eta * target_time)  <- force that stops it in target_time
    After omega -> ~0 the docked thruster pushes the (now stable) debris
    retrograde so perigee drops to TARGET_PERIGEE_KM and it crashes/burns.
    Returns a dict with beam + push numbers (design-level, no new physics).
    """
    omega = max(0.0, tumble_rad_s)  # full spin to bleed to 0 (stop spinning)
    inertia = debris_inertia(mass_kg, size_m)
    ang_mom = inertia * omega
    arm = max(0.05, size_m / 2.0)  # off-centre plume moment arm
    torque_avail = ion_thrust_n * arm * eta
    beam_time_s = ang_mom / torque_avail if torque_avail > 0 else float("inf")
    force_req = ang_mom / (arm * eta * target_time_s) if target_time_s > 0 else 0.0
    impulse = force_req * target_time_s
    # disposal push (vis-viva, retrograde burn at current radius -> target perigee)
    r = R_EARTH_KM + alt_km
    r_p = R_EARTH_KM + TARGET_PERIGEE_KM
    a_new = (r + r_p) / 2.0
    v_c = math.sqrt(MU / r)
    v_new = math.sqrt(max(1e-9, MU * (2.0 / r - 1.0 / a_new)))
    dv = max(0.0, v_c - v_new)
    push_impulse = dv * 1000.0 * mass_kg
    push_dur = push_impulse / max(1.0, THRUST_N)
    return {
        "tumble_rad_s": tumble_rad_s,
        "omega_bleed": omega,
        "inertia": inertia,
        "ang_momentum": ang_mom,
        "ion_force_applied_N": force_req,
        "ion_thrust_avail_N": ion_thrust_n,
        "beam_time_s": beam_time_s,
        "beam_time_sized_s": target_time_s,
        "impulse_Ns": impulse,
        "thruster_dv_km_s": dv,
        "thruster_impulse_Ns": push_impulse,
        "thruster_dur_s": push_dur,
        "target_perigee_km": TARGET_PERIGEE_KM,
    }


class World:
    def __init__(self, seed=21):
        random.seed(seed)
        alt = random.uniform(350, 850)
        self.a = R_EARTH_KM + alt
        self.e, self.inc = 0.001, random.uniform(40, 98)
        self.raan, self.argp, self.M0 = (random.uniform(0, 360) for _ in range(3))
        self.size_m = random.uniform(0.5, 2.5)
        self.mass = max(0.05, self.size_m ** 3 * 140.0)
        self.spin = random.uniform(0.5, 2.5)
        self.sf = random.uniform(0.96, 1.04)
        self.rot = [random.random() * 6.28 for _ in range(3)]
        v0 = math.sqrt(MU / self.a)
        B = self.mass / (2.2 * max(0.001, self.size_m ** 2))
        drag = 0.5 * rho_moderate(alt) * (v0 * 1000) ** 2 / B
        self.decay = min(8e-6, max(2e-7, -(-2 * (self.a * 1000) ** 2 * drag * (v0 * 1000) / 3.986004418e14) / 1000))
        self.hosts = [dict(a=R_EARTH_KM + random.uniform(160, 2000), e=0.001,
                           inc=random.uniform(0, 98), raan=random.uniform(0, 360),
                           argp=random.uniform(0, 360), M0=random.uniform(0, 360))
                      for _ in range(40)]

    def debris_at(self, t):
        a_t = self.a - self.decay * t
        pos, vel = eci_state(a_t, self.e, self.inc, self.raan, self.argp, self.M0, t, self.sf)
        speed = norm(vel)
        alt = a_t - R_EARTH_KM
        rho = rho_moderate(max(120, min(2000, alt)))
        area = max(0.001, self.size_m ** 2)
        B = self.mass / (2.2 * area)
        dragA = 0.5 * rho * (speed * 1000) ** 2 / B
        heat = 0.5 * rho * (speed * 1000) ** 3
        rot = [(r + self.spin * t * (0.6 + 0.2 * (i % 2))) % 6.28 for i, r in enumerate(self.rot)]
        hp = [eci_of(h["a"], h["e"], h["inc"], h["raan"], h["argp"],
                     (h["M0"] * RAD + math.sqrt(MU / h["a"] ** 3) * t) / RAD % 360.0)
              for h in self.hosts]
        return {"a": a_t, "pos": pos, "vel": vel, "speed": speed, "alt": alt,
                "h": norm(cross(pos, vel)), "rho": rho, "dragA": dragA, "heat": heat,
                "rot": rot, "hosts": hp,
                "decay": (self.decay, self.decay * 0.02, self.decay * 0.1, self.decay * 1.12)}


class CCSStore:
    def __init__(self):
        self.packets = []

    def store(self, p):
        self.packets.append(p)

    def save(self, path=None):
        if path is None:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapping_store.json")
        with open(path, "w") as f:
            json.dump(self.packets, f, indent=1)
        return path


def pass_packet(st, bot, t, host_pos, dist):
    """Bot records its exact pos + debris exact pos at detection instant. Raw send."""
    g = random.gauss
    x, y, z = st["pos"]
    hx, hy, hz = host_pos
    vx, vy, vz = st["vel"]
    return {
        "t": t, "bot": bot,
        "swarm_pos": (hx + g(0, 3.0), hy + g(0, 3.0), hz + g(0, 3.0)),
        "debris_pos": (x + g(0, 3.0), y + g(0, 3.0), z + g(0, 3.0)),
        "debris_coords": {"x": x + g(0, 3.0), "y": y + g(0, 3.0), "z": z + g(0, 3.0)},
        "altitude_sea": st["alt"] + g(0, 2.0),
        "speed": st["speed"] + g(0, 0.03),
        "trajectory": {"a": st["a"] + g(0, 3.0), "vx": vx + g(0, 0.02),
                       "vy": vy + g(0, 0.02), "vz": vz + g(0, 0.02)},
        "mass": max(0.05, W.mass + g(0, W.mass * 0.05)),
        "angular_momentum": st["h"] + g(0, 50),
        "perp_distance_km": dist + g(0, 2.0),
        "air_heat": {"rho": st["rho"], "drag": st["dragA"] + g(0, st["dragA"] * 0.1),
                     "heat": st["heat"] + g(0, st["heat"] * 0.1)},
        "size_m": max(0.05, W.size_m + g(0, 0.15)),
        "orientation": [a + g(0, 0.1) for a in st["rot"]],
        "decay": {"drag": st["decay"][0], "srp": st["decay"][1],
                  "tumble": st["decay"][2], "total": st["decay"][3]},
        "tumble_speed": W.spin + g(0, 0.2),
    }


if __name__ == "__main__":
    W = World()
    CCS = CCSStore()
    period = 2 * math.pi * math.sqrt(W.a ** 3 / MU) / W.sf
    total, step = 2 * period, 10.0
    last_seen = [-1e9] * 40
    print("STAGE 1 PASS-BASED — null info. Bots record only on flyby passes.")
    print(f"period {period / 60:.1f} min x2, range {DET_RANGE_KM:.0f} km, 40 hosts.\n")
    t, last_prop, last_state = 0.0, None, None
    n_steps = int(total / step)
    for k in range(n_steps):
        t += step
        st = W.debris_at(t)
        last_state = st
        for b, hp in enumerate(st["hosts"]):
            d = norm(sub(st["pos"], hp))
            if d < DET_RANGE_KM and t - last_seen[b] > COOLDOWN_S:
                last_seen[b] = t
                CCS.store(pass_packet(st, b, t, hp, d))
                print(f"  PASS bot-{b:02d} t+{t / 60:.1f}min d={d:.0f}km "
                      f"debris {tuple(round(v) for v in st['pos'])} Alt {st['alt']:.0f}km "
                      f"-> STORED ({len(CCS.packets)})")
        # live coordinate propagation between passes (last speed+trajectory)
        if last_prop is None and CCS.packets:
            lp = CCS.packets[-1]
            last_prop = (lp["t"], lp["debris_pos"],
                         (lp["trajectory"]["vx"], lp["trajectory"]["vy"], lp["trajectory"]["vz"]))
        if (k % 40 == 0 or k == n_steps - 1) and CCS.packets:
            lp = CCS.packets[-1]
            dt = t - lp["t"]
            px = lp["debris_pos"][0] + lp["trajectory"]["vx"] * dt
            py = lp["debris_pos"][1] + lp["trajectory"]["vy"] * dt
            pz = lp["debris_pos"][2] + lp["trajectory"]["vz"] * dt
            orb = t / period
            print(f"orbit {orb:.2f}/2  live-propagated coords {px:.0f},{py:.0f},{pz:.0f} "
                  f"from bot-{lp['bot']} speed {lp['speed']:.3f}km/s (stored {len(CCS.packets)}, info otherwise NULL)")
    print(f"\n2 ORBITS COMPLETE — {len(CCS.packets)} pass packets. PROCESSING (connect the dots)...")
    time.sleep(1.0)
    P = CCS.packets
    n = max(1, len(P))
    if not P:
        print("No passes in range — widen DET_RANGE_KM or add hosts.")
    else:
        dots = sorted([p["debris_pos"] for p in P], key=lambda q: 0)  # time order kept
        fA = sum(p["trajectory"]["a"] for p in P) / n
        d0 = sum(p["decay"]["total"] for p in P) / n
        print("UNIFIED DETAILS (passes connected):")
        print(f"  trajectory dots : {len(dots)} detections linked in time order")
        print(f"  final orbit a   : {fA:.1f} km (alt {fA - R_EARTH_KM:.1f} km)")
        print(f"  final decay     : {d0 * 86400:.4f} km/day (drag+SRP+tumble)")
        print(f"  speed           : {sum(p['speed'] for p in P) / n:.3f} km/s (live-updated between passes)")
        print(f"  mass            : {sum(p['mass'] for p in P) / n:.1f} kg  size {sum(p['size_m'] for p in P) / n:.2f} m")
        print(f"  ang momentum    : {sum(p['angular_momentum'] for p in P) / n:.0f} km2/s")
        print(f"  min perp dist   : {min(p['perp_distance_km'] for p in P):.1f} km")
        print(f"  tumble          : {sum(p['tumble_speed'] for p in P) / n:.2f} rad/s")
        path = CCS.save()
        print(f"  stored          : {len(P)} -> {path}")
        # NEW: Stage 2 detumble + push plan, sized FROM the mapped tumble speed.
        # More spin -> more ion-beam force/time; then thruster pushes stable debris to crash.
        m_avg = sum(p["mass"] for p in P) / n
        s_avg = sum(p["size_m"] for p in P) / n
        w_avg = sum(p["tumble_speed"] for p in P) / n
        alt_avg = fA - R_EARTH_KM
        plan = plan_detumble_and_push(m_avg, s_avg, w_avg, alt_avg)
        print("DETUMBLE PLAN (IBS ion-beam shepherd, from mapped tumble):")
        print(f"  spin {plan['tumble_rad_s']:.2f} rad/s -> bleed {plan['omega_bleed']:.2f} rad/s "
              f"(I={plan['inertia']:.2f} kg.m2, L={plan['ang_momentum']:.2f} N.m.s)")
        print(f"  ion force {plan['ion_force_applied_N'] * 1000.0:.2f} mN for "
              f"~{plan['beam_time_sized_s']:.0f}s (avail {plan['ion_thrust_avail_N'] * 1000.0:.0f} mN, "
              f"natural time ~{plan['beam_time_s']:.0f}s) -> spin 0, then STOP beam")
        print(f"THRUSTER PUSH (docked, stable debris only): dv {plan['thruster_dv_km_s']:.3f} km/s "
              f"retrograde over ~{plan['thruster_dur_s']:.0f}s -> perigee "
              f"{plan['target_perigee_km']:.0f}km -> crash/burnup")
        plan_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "detumble_plan.json")
        with open(plan_path, "w") as f:
            json.dump(plan, f, indent=1)
        print(f"  plan saved      : {plan_path}")
