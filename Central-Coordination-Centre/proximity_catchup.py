"""CCS Stage 3 — proximity catch-up after detumble (stdlib only).

Sequence (mirrors the sim's maneuver() physics — burns rebuild elements from
the new r/v vectors, so position never teleports):
  1. Ion-beam bleed stops the debris tumble (spin -> ~0, beam OFF).
  2. CCS waits SETTLE_S (~10 s) — debris left alone, no follow.
  3. Top proxy candidates are priced for REAL: plane-burn at their node +
     best Lambert intercept each; cheapest in-band job wins.
  4. Winner rotates into the debris plane at the node, ramps to ~9.5 km/s,
     and coasts a solved Lambert arc with live ETA.
  5. At intercept it brakes to rel-vel ZERO (matched), docks, slows down and
     pushes retrograde to a burnup perigee (operation complete).

Run:  python proximity_catchup.py [seed]
"""

import math
import random
import sys

from mapping_stage import (
    MU, R_EARTH_KM, RAD, eci_of, eci_state, norm, sub, cross,
    plan_detumble_and_push, World,
    ION_THRUST_N, PLUME_ETA, TUMBLE_STOP_RAD_S,
)

SETTLE_S = 10.0            # wait after bleed before anyone moves
TARGET_SPEED_KM_S = 9.5    # catch-up burn target (~9-10 km/s band)
CAPTURE_KM = 150.0         # intercept radius -> brake + dock
DV_BUDGET_KM_S = 6.0       # catch-up fuel budget per bot (plane turn included)


def circ_speed(r_km):
    return math.sqrt(MU / r_km)


def escape_speed(r_km):
    return math.sqrt(2.0 * MU / r_km)


def unit(v):
    n = norm(v)
    return (v[0] / n, v[1] / n, v[2] / n) if n > 0 else (1.0, 0.0, 0.0)


def dot(u, v):
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def rv2coe(pos, vel):
    """Kepler elements (deg units, like World/hosts) from r/v vectors."""
    r = norm(pos)
    v2 = dot(vel, vel)
    h = cross(pos, vel)
    hm = norm(h)
    nvec = (-h[1], h[0], 0.0)
    nm = norm(nvec)
    rv = dot(pos, vel)
    evec = tuple(((v2 - MU / r) * pos[i] - rv * vel[i]) / MU for i in range(3))
    e = norm(evec)
    a = -MU / (2.0 * (v2 / 2.0 - MU / r))
    inc = math.degrees(math.acos(max(-1.0, min(1.0, h[2] / hm))))
    raan = 0.0
    if nm > 1e-9:
        raan = math.degrees(math.acos(max(-1.0, min(1.0, nvec[0] / nm))))
        if nvec[1] < 0:
            raan = 360.0 - raan
    argp = 0.0
    if nm > 1e-9 and e > 1e-8:
        argp = math.degrees(math.acos(max(-1.0, min(1.0, dot(nvec, evec) / (nm * e)))))
        if evec[2] < 0:
            argp = 360.0 - argp
    if e > 1e-8 and e < 1.0:
        nu = math.degrees(math.acos(max(-1.0, min(1.0, dot(evec, pos) / (e * r)))))
        if rv < 0:
            nu = 360.0 - nu
        E = 2.0 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(math.radians(nu) / 2.0))
        M = E - e * math.sin(E)
    elif e <= 1e-8:
        M = math.atan2(pos[1], pos[0])  # circular fallback
    else:
        M = 0.0  # hyperbolic: anomaly unused (rejected by callers)
    n = math.sqrt(MU / abs(a) ** 3)
    return {"a": a, "e": e, "inc": inc, "raan": raan, "argp": argp,
            "M": (math.degrees(M) % 360.0 + 360.0) % 360.0, "n": n}


def _kepler_E(M, e):
    E = M if e < 0.8 else math.pi
    for _ in range(60):
        E -= (E - e * math.sin(E) - M) / (1.0 - e * math.cos(E))
    return E


def _eci_from_nu(a, e, inc_d, raan_d, argp_d, nu):
    inc, raan, argp = inc_d * RAD, raan_d * RAD, argp_d * RAD
    E = 2.0 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(nu / 2.0))
    r = a * (1 - e * math.cos(E))
    p, q = r * math.cos(nu), r * math.sin(nu)
    co, so = math.cos(raan), math.sin(raan)
    ci, si = math.cos(inc), math.sin(inc)
    cw, sw = math.cos(argp), math.sin(argp)
    return ((co * cw - so * sw * ci) * p + (-co * sw - so * cw * ci) * q,
            (so * cw + co * sw * ci) * p + (-so * sw + co * cw * ci) * q,
            (sw * si) * p + (cw * si) * q)


def bot_pos(h, t):
    n = math.sqrt(MU / h["a"] ** 3)
    M = (h["M0"] * RAD + n * t) % (2.0 * math.pi)
    E = _kepler_E(M, h["e"])
    nu = 2.0 * math.atan2(math.sqrt(1 + h["e"]) * math.sin(E / 2.0),
                          math.sqrt(1 - h["e"]) * math.cos(E / 2.0))
    return _eci_from_nu(h["a"], h["e"], h["inc"], h["raan"], h["argp"], nu)


def bot_state(h, t):
    p1 = bot_pos(h, t)
    p2 = bot_pos(h, t + 1.0)
    return p1, (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])


def next_plane_node(bot, h_d, t0):
    """First time after t0 the bot pierces the debris plane (r.hD changes
    sign) — the only efficient spot for a plane-change burn."""
    T = 2 * math.pi * math.sqrt(bot["a"] ** 3 / MU)
    step = 30.0
    t_prev, d_prev = t0, dot(bot_state(bot, t0)[0], h_d)
    t_c = t_prev + step
    while t_c <= t0 + T + step:
        d_c = dot(bot_state(bot, t_c)[0], h_d)
        if d_c == 0:
            return t_c
        if (d_prev < 0) != (d_c < 0):
            lo, hi, d_lo = t_prev, t_c, d_prev
            for _ in range(24):
                mid = 0.5 * (lo + hi)
                d_m = dot(bot_state(bot, mid)[0], h_d)
                if (d_lo < 0) != (d_m < 0):
                    hi = mid
                else:
                    lo, d_lo = mid, d_m
            return 0.5 * (lo + hi)
        t_prev, d_prev = t_c, d_c
        t_c += step
    return None


def impulsive_burn(h, t, v_new_vec):
    """Apply velocity v_new_vec at current position (position-continuous,
    like the sim's maneuver()). Returns dv cost in km/s."""
    p, v = bot_state(h, t)
    dv = norm(sub(v_new_vec, v))
    el = rv2coe(p, v_new_vec)
    h.update(a=el["a"], e=el["e"], inc=el["inc"], raan=el["raan"],
             argp=el["argp"], M0=(el["M"] - el["n"] * t / RAD) % 360.0)
    h["dv_used"] = h.get("dv_used", 0.0) + dv
    return dv


def run_detumble(world, mass, size, tumble0):
    """Bleed debris spin with the mapped ion-beam force. Returns bleed seconds."""
    plan = plan_detumble_and_push(mass, size, tumble0, world.a - R_EARTH_KM)
    force = plan["ion_force_applied_N"]
    torque = force * max(0.05, size / 2.0) * PLUME_ETA
    w, t = tumble0, 0.0
    while w > TUMBLE_STOP_RAD_S:
        w = max(0.0, w - torque / plan["inertia"] * 1.0)
        t += 1.0
        if t > 100000:
            break
    return t, plan


def fmt_eta(s):
    if s is None or not math.isfinite(s):
        return "--"
    s = max(0, int(round(s)))
    if s < 90:
        return "%ds" % s
    m, h = s // 60, s // 3600
    if h:
        return "%dh %dm" % (h, (m % 60))
    return "%dm %ds" % (m, s % 60)


def debris_period(w):
    return 2 * math.pi * math.sqrt(w.a ** 3 / MU) / w.sf


def _stumpff(z):
    if z > 1e-6:
        sz = math.sqrt(z)
        return (1.0 - math.cos(sz)) / z, (sz - math.sin(sz)) / (sz ** 3)
    if z < -1e-6:
        sz = math.sqrt(-z)
        return (math.cosh(sz) - 1.0) / (-z), (math.sinh(sz) - sz) / ((-z) ** 1.5)
    return 0.5, 1.0 / 6.0


def _lambert_dt(z, r1, r2, A):
    C, S = _stumpff(z)
    if C <= 1e-12:
        return None
    y = r1 + r2 + A * (z * S - 1.0) / math.sqrt(C)
    if y < 0:
        return None
    x = math.sqrt(y / C)
    return (x ** 3 * S + A * math.sqrt(y)) / math.sqrt(MU)


def lambert(r1v, r2v, dt, h_ref):
    """Intercept transfer r1 -> r2 in dt (universal variables, Newton with
    numeric derivative). Returns (v1_depart, v2_arrive) or None."""
    r1, r2 = norm(r1v), norm(r2v)
    cosd = max(-1.0, min(1.0, dot(r1v, r2v) / (r1 * r2)))
    dnu = math.acos(cosd)
    if dot(cross(r1v, r2v), h_ref) < 0:
        dnu = 2.0 * math.pi - dnu
    if abs(dnu - math.pi) < 0.05 or abs(math.sin(dnu)) < 1e-6:
        return None
    A = math.sin(dnu) * math.sqrt(r1 * r2 / (1.0 - cosd))
    if abs(A) < 1e-9:
        return None
    # dt(z) is monotonic increasing -> bracket + bisect (robust, no Newton).
    # (The y<0 gap, when present, sits at very negative z, so a None at the
    # low end means "move up toward 0", never down.)
    def F(z):
        return _lambert_dt(z, r1, r2, A)
    lo, hi = -20.0, 40.0
    flo = F(lo)
    while flo is None:
        lo /= 2.0
        if lo > -0.25:
            lo = 0.0
            flo = F(lo)
            break
        flo = F(lo)
    if flo is None:
        return None
    while flo > dt:
        lo = lo * 2.0 - 5.0
        if lo < -2000.0:
            return None
        flo = F(lo)
        if flo is None:
            return None
    fhi = F(hi)
    n = 0
    while (fhi is None or fhi < dt) and n < 12:
        hi = hi * 2.0 + 10.0
        if hi > 2000.0:
            return None
        fhi = F(hi)
        n += 1
    if fhi is None or flo is None:
        return None
    z = 0.0
    for _ in range(80):
        z = 0.5 * (lo + hi)
        f = F(z)
        if f is None:  # y<0 gap sits low -> push bottom up
            lo = z
            continue
        if abs(f - dt) < 0.25:
            break
        if f < dt:
            lo = z
        else:
            hi = z
    else:
        return None
    C, S = _stumpff(z)
    if C <= 1e-12:
        return None
    y = r1 + r2 + A * (z * S - 1.0) / math.sqrt(C)
    if y < 0 or abs(_lambert_dt(z, r1, r2, A) - dt) > 2.0:
        return None
    f = 1.0 - y / r1
    g = A * math.sqrt(y / MU)
    if abs(g) < 1e-9:
        return None
    gdot = 1.0 - y / r2
    v1 = tuple((r2v[i] - f * r1v[i]) / g for i in range(3))
    v2 = tuple((gdot * r2v[i] - r1v[i]) / g for i in range(3))
    return v1, v2


def find_transfer(bot, w, t0, h_ref):
    """All feasible intercepts over one rev of burn epochs. Returns up to 3
    best (closest departure to the 9-10 band, then cheapest), each a tuple
    (sp1, total_dv, t_burn, t_hit, v1_vec)."""
    T_deb = debris_period(w)
    feas = []
    tb = t0
    while tb < t0 + T_deb:
        p_b, v_b = bot_state(bot, tb)
        r_b = norm(p_b)
        v_esc = 0.98 * escape_speed(r_b)
        # short fast hops first (high-energy, minutes), then longer drift loops
        for frac in (0.08, 0.12, 0.18, 0.25, 0.35, 0.5, 0.75, 1.0, 1.25, 1.5):
            dt = frac * T_deb
            st2 = w.debris_at(tb + dt)
            sol = lambert(p_b, st2["pos"], dt, h_ref)
            if sol is None:
                continue
            v1, v2 = sol
            sp1 = norm(v1)
            if sp1 > v_esc:
                continue
            el = rv2coe(p_b, v1)
            if el["a"] <= 0 or el["e"] >= 1.0 or el["a"] * (1 - el["e"]) - R_EARTH_KM < 150.0:
                continue  # dives into air
            total = norm(sub(v1, v_b)) + norm(sub(v2, st2["vel"]))
            if total > 12.0:
                continue
            feas.append((abs(sp1 - TARGET_SPEED_KM_S), total, tb, tb + dt, v1))
        tb += 180.0
    feas.sort()
    return feas[:3]


def run_disposal(w, bot, bi, t_dock):
    """Docked push -> undock (bot survives) -> debris descent -> burnup."""
    st = w.debris_at(t_dock)
    push = plan_detumble_and_push(w.mass, w.size_m, 0.0, st["alt"])
    dv = push["thruster_dv_km_s"]
    u = unit(st["vel"])
    v_stack = (st["vel"][0] - dv * u[0], st["vel"][1] - dv * u[1], st["vel"][2] - dv * u[2])
    el = rv2coe(st["pos"], v_stack)
    for _ in range(60):
        if el["a"] <= 0 or el["e"] >= 1.0:
            break
        rp = el["a"] * (1 - el["e"]) - R_EARTH_KM
        if 45.0 <= rp <= 62.0:
            break
        step = max(-0.01, min(0.01, (rp - 55.0) * 0.00004))
        if abs(step) < 1e-6:
            break
        v_try = (v_stack[0] - step * u[0], v_stack[1] - step * u[1], v_stack[2] - step * u[2])
        el_try = rv2coe(st["pos"], v_try)
        if el_try["a"] <= 0 or el_try["e"] >= 1.0:
            break
        v_stack, el = v_try, el_try
    dv = norm(sub(st["vel"], v_stack))  # honest total retrograde cost
    n = math.sqrt(MU / el["a"] ** 3)
    deb = dict(a=el["a"], e=el["e"], inc=el["inc"], raan=el["raan"],
               argp=el["argp"], M0=(el["M"] - n * t_dock / RAD) % 360.0)
    rp = el["a"] * (1 - el["e"]) - R_EARTH_KM
    ra = el["a"] * (1 + el["e"]) - R_EARTH_KM
    T = 2 * math.pi * math.sqrt(el["a"] ** 3 / MU)
    print("  SLOW-DOWN + PUSH (docked stack): dv %.3fkm/s retrograde over ~%.0fs -> disposal orbit"
          " %.0fx%.0fkm, T~%s." % (dv, push["thruster_dur_s"], rp, ra, fmt_eta(T)))
    # metered push, then undock: bot raises to a safe shell and survives
    t_sep = t_dock + min(push["thruster_dur_s"], 600.0)
    p_s, v_s = bot_state(deb, t_sep)
    bot.update(a=deb["a"], e=deb["e"], inc=deb["inc"], raan=deb["raan"],
               argp=deb["argp"], M0=deb["M0"])  # rode the stack till sep
    us = unit(v_s)
    # raise until the bot's OWN perigee clears 250km — provably safe shell,
    # never a doomed ride-along.
    up, rp_b = 0.035, -1e9
    while True:
        v_try = (v_s[0] + up * us[0], v_s[1] + up * us[1], v_s[2] + up * us[2])
        el_b = rv2coe(p_s, v_try)
        if el_b["a"] > 0 and el_b["e"] < 1.0:
            rp_b = el_b["a"] * (1 - el_b["e"]) - R_EARTH_KM
        if rp_b >= 250.0 or up >= 0.5:
            break
        up += 0.01
    impulsive_burn(bot, t_sep, v_try)
    print("  UNDOCK t+%.0fs: BOT-%02d raise +%.3fkm/s -> perigee %.0fkm / %.0fkm shell — SURVIVES."
          % (t_sep, bi, up, rp_b, bot["a"] - R_EARTH_KM))
    # debris coasts alone into the atmosphere
    t, burning, step = t_sep, False, 10.0
    t_end, nxt = t_sep + 3 * T, t_sep
    while t < t_end:
        t += step
        p = bot_pos(deb, t)
        r = norm(p)
        alt = r - R_EARTH_KM
        if alt < 135.0 and not burning:
            burning = True
            print("  HEATING t+%.0fs: alt %.0fkm <135km — fireball, drag biting." % (t, alt))
        if t >= nxt:
            v = math.sqrt(max(1e-9, MU * (2.0 / r - 1.0 / deb["a"])))
            in_air = alt < 135.0
            print("  DESCENT t+%.0fs: alt %.0fkm v %.2fkm/s%s"
                  % (t, alt, v, " BURNING" if in_air else ""))
            nxt += max(60.0, T / 12.0)
        if alt < 66.0:
            print("  BURNED UP t+%.0fs at alt %.0fkm — debris DELETED. BOT-%02d SAFE. OP COMPLETE."
                  % (t, alt, bi))
            print("  CLOSEST-BOT OP DONE: BOT-%02d, mission dv %.3fkm/s." % (bi, bot["dv_used"]))
            return True
    print("  STILL COASTING (perigee %.0fkm) — no burnup in 3 revs." % rp)
    return False


def attempt_once(w, t, used, attempt):
    """One full try: pick (skip burnt bots) -> plane burn -> Lambert burn ->
    coast -> intercept. Returns ('docked', bot, bi, t) or ('retry', t_new)."""
    # 3. PROXIMITY PICK by ACTUAL transfer cost (burnt bots excluded).
    st = w.debris_at(t)
    h_d = unit(cross(st["pos"], st["vel"]))
    scored = []
    for i, h in enumerate(w.hosts):
        if i in used:
            continue
        p, v = bot_state(h, t)
        ang = math.acos(max(-1.0, min(1.0, dot(unit(cross(p, v)), h_d))))
        dist = norm(sub(st["pos"], p))
        d_alt = abs(norm(p) - norm(st["pos"]))
        scored.append((dist + 8000.0 * ang + 2.0 * d_alt, i, ang, d_alt, dist))
    scored.sort()
    T_deb0 = debris_period(w)
    options = []
    # cheapest sane geometry wins: in-band fast departures only when the
    # WHOLE job (plane turn + transfer) stays cheap, else cheapest drift.
    for _, i, ang, d_alt, dist in scored[:16]:
        cand = dict(w.hosts[i])
        cand["dv_used"] = 0.0
        t_nd = next_plane_node(cand, h_d, t)
        if t_nd is None or t_nd - t > T_deb0:
            continue
        stn = w.debris_at(t_nd)
        hd = unit(cross(stn["pos"], stn["vel"]))
        pn, vn = bot_state(cand, t_nd)
        uu = unit(cross(hd, pn))
        if dot(uu, stn["vel"]) < 0:
            uu = (-uu[0], -uu[1], -uu[2])
        spn = norm(vn)
        dv_pl = impulsive_burn(cand, t_nd, (uu[0] * spn, uu[1] * spn, uu[2] * spn))
        for (off_band, tot, t_b, t_hit, v1) in find_transfer(cand, w, t_nd, hd):
            sp1 = norm(v1)
            in_band = 8.8 <= sp1 <= 10.2
            combined = dv_pl + tot
            tier = (0 if (in_band and combined <= 5.0)
                    else (1 if combined <= 7.5 else 2))
            options.append(((tier, off_band, combined, t_b),
                            i, cand, t_nd, dv_pl, ang, d_alt, dist, t_b, t_hit, v1, tot))
    if not options:
        print("  [try %d] NO TRANSFER — waiting 10min for geometry, then re-pick." % attempt)
        return ("retry", t + 600.0)
    options.sort(key=lambda e: e[0])
    (_, bi, bot, t_node, dv_plane, pang, dalt, dist0, t_b, t_hit, v1, tot) = options[0]
    tier_note = "" if options[0][0][0] == 0 else (" (tier-%d fallback)" % options[0][0][0])
    p_b, v_b = bot_state(bot, t)
    print("  [try %d] PROXIMITY PICK: BOT-%02d (d=%.0fkm, shell off %.0fkm, plane off %.1f deg;"
          " transfer dv %.2fkm/s%s; bot %.3fkm/s vs debris %.3fkm/s)."
          % (attempt, bi, dist0, dalt, math.degrees(pang), dv_plane + tot, tier_note,
             norm(v_b), st["speed"]))
    if t_node > t:
        print("  NODE WAIT %s to plane crossing." % fmt_eta(t_node - t))
    t = t_node
    st = w.debris_at(t)
    print("  PLANE BURN: BOT-%02d turned %.1f deg into debris plane (dv %.3fkm/s)."
          % (bi, math.degrees(pang), dv_plane))
    if t_b > t:
        print("  PHASING WAIT %s for the window (burn at t+%.0fs)." % (fmt_eta(t_b - t), t_b))
        t = t_b
    st = w.debris_at(t)  # refresh: coast ETA must compare same-epoch states
    p_b, v_b = bot_state(bot, t)
    sp1 = norm(v1)
    u1 = (v1[0] / sp1, v1[1] / sp1, v1[2] / sp1)
    v0 = norm(v_b)
    dv_leg = norm(sub(v1, v_b))
    for k in range(1, 7):
        v_k = v0 + (sp1 - v0) * k / 6.0
        impulsive_burn(bot, t, (u1[0] * v_k, u1[1] * v_k, u1[2] * v_k))
        print("    ramp %d/6: %.3fkm/s" % (k, v_k))
    el = rv2coe(p_b, v1)
    print("  BURN: BOT-%02d %.3f -> %.3fkm/s (leg dv %.3fkm/s, cum %.3fkm/s of %.1f,"
          " a=%.0fkm, apogee %.0fkm). Now FASTER than debris." % (
              bi, v0, sp1, dv_leg, bot["dv_used"], DV_BUDGET_KM_S,
              el["a"], el["a"] * (1 + el["e"]) - R_EARTH_KM))
    print("  RE-ORBITED — intercept in ~%s." % fmt_eta(t_hit - t))

    # 5. COAST with live ETA + rel-vel, then BRAKE to rel-vel ZERO (matched).
    prev_d, prev_t = norm(sub(st["pos"], p_b)), t
    step = max(60.0, (t_hit - t) / 12.0)
    while t < t_hit:
        t = min(t + step, t_hit)
        st = w.debris_at(t)
        p_bt, v_bt = bot_state(bot, t)
        d = norm(sub(st["pos"], p_bt))
        rel = norm(sub(v_bt, st["vel"]))
        rate = (prev_d - d) / max(1e-9, t - prev_t)
        print("  COAST t+%.0fs d=%.0fkm closing %.2fkm/s rel-vel %.2fkm/s -> meet ~%s"
              % (t, d, max(0.0, rate), rel,
                 fmt_eta(d / rate if rate > 1e-6 else float("inf"))))
        prev_d, prev_t = d, t
    st = w.debris_at(t)
    p_f, v_f = bot_state(bot, t)
    d = norm(sub(st["pos"], p_f))
    if d > CAPTURE_KM:
        print("  MISSED (d=%.0fkm) — BOT-%02d stood down, HANDOFF to next-best." % (d, bi))
        used.add(bi)
        return ("retry", t)
    rel_before = norm(sub(v_f, st["vel"]))
    brake = impulsive_burn(bot, t, st["vel"])  # match debris velocity exactly
    rel_now = norm(sub(bot_state(bot, t)[1], st["vel"]))
    print("  INTERCEPT d=%.1fkm — BRAKE %.3fkm/s (rel-vel %.2f -> %.2f km/s ZERO) — MATCHED." % (d, brake, rel_before, rel_now))
    print("  DOCKED, holding formation. Now SLOWING DOWN for disposal...")
    return ("docked", bot, bi, t)


def run(seed=21):
    random.seed(seed)
    w = World(seed=seed)
    t = 0.0
    print("STAGE 3 PROXIMITY CATCH-UP — bleed -> %.0fs settle -> closest capable bot"
          " speeds up to catch it -> intercept -> dock -> dispose" % SETTLE_S)

    # 1. BLEED — ion beam until spin ~0, then beam OFF (debris left alone).
    tumble0 = w.spin
    bleed_s, _ = run_detumble(w, w.mass, w.size_m, tumble0)
    t += bleed_s
    st = w.debris_at(t)
    plan0 = plan_detumble_and_push(w.mass, w.size_m, tumble0, w.a - R_EARTH_KM)
    print("  BLEED done: spin %.2f -> <=%.2f rad/s in %.0fs (F=%.2fmN). Beam OFF."
          % (tumble0, TUMBLE_STOP_RAD_S, bleed_s, plan0["ion_force_applied_N"] * 1000.0))

    # 2. SETTLE — ~10 s, nobody follows.
    t += SETTLE_S
    print("  SETTLE %.0fs — debris coasts free, no follow." % SETTLE_S)

    # 3-5. Up to 3 tries: pick -> burn -> intercept. A miss or dead geometry
    # hands the job to the next-best bot (failed bots stood down) — the run
    # never dies on one miss.
    used = set()
    for attempt in range(1, 4):
        outcome = attempt_once(w, t, used, attempt)
        if outcome[0] == "docked":
            _, bot, bi, t = outcome
            return run_disposal(w, bot, bi, t)
        t = outcome[1]
    print("  3 ATTEMPTS SPENT — standing down (retarget).")
    return False


if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 21)
