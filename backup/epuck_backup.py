from controller import Robot
import os, json, time
import numpy as np

GRID_N = 6
START_ROW = 5
START_COL = 5
START_HEADING = "N"

DIR_VEC = {"N": (-1, 0), "E": (0, 1), "S": (1, 0), "W": (0, -1)}
LEFT_OF  = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def in_bounds(r, c):
    return 0 <= r < GRID_N and 0 <= c < GRID_N

def tile_key(r, c):
    return f"{r},{c}"

# -----------------------------
# Motion
# -----------------------------
MAX_SPEED = 6.28
FWD_SPEED = 4.5
TURN_SPEED = 3.4
SEARCH_TURN_SPEED = 2.4  # gentle

TURN_90_MS = 520
BACKOFF_MS = 170
FWD_COMMIT_MS = 220

# -----------------------------
# Proximity thresholds (front)
# -----------------------------
FRONT_BLOCK_TH = 220.0
HARD_JAM_TH = 420.0

# -----------------------------
# Auto-calibration (LEFT_OPEN_TH)
# -----------------------------
CAL_ROTATE_MS = 1200
KMEANS_ITERS = 12
LEFT_OPEN_TH_MIN = 60.0
LEFT_OPEN_TH_MAX = 450.0
LEFT_WALL_PRESENT_PAD = 35.0  # wall-present threshold = LEFT_OPEN_TH + pad

# -----------------------------
# Reacquire state machine (key fix)
# -----------------------------
WALL_LOST_STEPS_ENTER = 10      # if no wall for this many steps -> WALL_LOST
WALL_FOUND_STEPS_EXIT = 6       # if wall present for this many steps -> NORMAL
SEARCH_PULSE_MS = 80            # short pulses to avoid circles
SEARCH_FORWARD_MS = 140         # forward-biased search
LEFT_TURN_COOLDOWN_STEPS = 25   # after a left turn, ignore left-open for a bit

# -----------------------------
# Left-open detection stability
# -----------------------------
LEFT_OPEN_CONFIRM_STEPS = 6     # require stable left_open for N steps (reduces jitter)

# -----------------------------
# Hairpin logic (2nd left => 180)
# -----------------------------
HAIRPIN_ENABLE = True
HAIRPIN_PROBE_MS = 520
HAIRPIN_FRONT_TH = 0.75 * FRONT_BLOCK_TH
HAIRPIN_CONFIRM_STEPS = 1  # only after left, ok to be aggressive

# -----------------------------
# Ground tile tracking
# -----------------------------
GS_DEBOUNCE = 4
GS_MARGIN_FRAC = 0.10
EDGES_PER_TILE = 2

# -----------------------------
# Files
# -----------------------------
MAPFILE = "maze_map_6x6.json"
DEBUG_CSV = "epuck_debug_run.csv"
DEBUG_FLUSH_EVERY = 50


class DebugCSV:
    def __init__(self, path):
        self.f = open(path, "w", buffering=1)
        self.n = 0
        self.f.write(",".join([
            "sim_t","unix_t","row","col","heading","mode","action",
            "vL","vR",
            "ps0","ps5","ps6","ps7",
            "front","leftMax",
            "LEFT_OPEN_TH","LEFT_WALL_PRESENT_TH",
            "left_open","left_wall",
            "left_open_count","no_wall_count","wall_count","cooldown",
            "hairpin_flag","hairpin_count",
            "gs1","gs_th","edge_event","edge_events","tile_advanced"
        ]) + "\n")

    def log(self, rec: dict):
        keys = [
            "sim_t","unix_t","row","col","heading","mode","action",
            "vL","vR",
            "ps0","ps5","ps6","ps7",
            "front","leftMax",
            "LEFT_OPEN_TH","LEFT_WALL_PRESENT_TH",
            "left_open","left_wall",
            "left_open_count","no_wall_count","wall_count","cooldown",
            "hairpin_flag","hairpin_count",
            "gs1","gs_th","edge_event","edge_events","tile_advanced"
        ]
        self.f.write(",".join(str(rec.get(k,"")) for k in keys) + "\n")
        self.n += 1
        if self.n % DEBUG_FLUSH_EVERY == 0:
            self.f.flush()

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except:
            pass


def kmeans_1d_threshold(x: np.ndarray, iters=10):
    x = x.astype(np.float64)
    if len(x) < 10:
        return float(np.median(x))
    c1 = np.percentile(x, 20)
    c2 = np.percentile(x, 80)
    if c1 == c2:
        return float(c1)
    for _ in range(iters):
        d1 = np.abs(x - c1)
        d2 = np.abs(x - c2)
        a = x[d1 <= d2]
        b = x[d1 > d2]
        if len(a) > 0:
            c1 = float(np.mean(a))
        if len(b) > 0:
            c2 = float(np.mean(b))
    lo, hi = (c1, c2) if c1 < c2 else (c2, c1)
    return float((lo + hi) / 2.0)


class EPuckController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.dbg = DebugCSV(DEBUG_CSV)

        self.lm = self.robot.getDevice("left wheel motor")
        self.rm = self.robot.getDevice("right wheel motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        self.vL = 0.0
        self.vR = 0.0
        self.set_speed(0, 0)

        self.ps = []
        for i in range(8):
            s = self.robot.getDevice(f"ps{i}")
            s.enable(self.timestep)
            self.ps.append(s)

        self.gs = []
        for name in ("gs0","gs1","gs2"):
            g = self.robot.getDevice(name)
            g.enable(self.timestep)
            self.gs.append(g)

        self.row = START_ROW
        self.col = START_COL
        self.heading = START_HEADING

        self.gs_th = None
        self.edge_state = None
        self.edge_count = 0
        self.edge_events = 0

        # calibrated
        self.LEFT_OPEN_TH = None
        self.LEFT_WALL_PRESENT_TH = None

        # state machine
        self.mode = "NORMAL"  # NORMAL or WALL_LOST
        self.no_wall_count = 0
        self.wall_count = 0

        # left-open stability
        self.left_open_count = 0

        # cooldown to prevent repeated lefts while drifting
        self.left_turn_cooldown = 0

        # hairpin
        self.hairpin_count = 0

        self.map = self.load_or_init_map()

    # ---- stepping/motors ----
    def step(self, n=1):
        for _ in range(n):
            if self.robot.step(self.timestep) == -1:
                return False
        return True

    def set_speed(self, l, r):
        self.vL = clamp(l, -MAX_SPEED, MAX_SPEED)
        self.vR = clamp(r, -MAX_SPEED, MAX_SPEED)
        self.lm.setVelocity(self.vL)
        self.rm.setVelocity(self.vR)

    def stop(self, steps=2):
        self.set_speed(0,0)
        self.step(steps)

    def timed_drive(self, l, r, ms, action, do_tile_update=True, hairpin_flag=None):
        steps = max(1, int(ms / self.timestep))
        self.set_speed(l, r)
        for _ in range(steps):
            if not self.step(1):
                return False
            tile_adv, edge_evt = (False, False)
            if do_tile_update:
                tile_adv, edge_evt = self.maybe_advance_tile()
            self.log(action, tile_adv, edge_evt, hairpin_flag)
        self.stop()
        return True

    def backoff(self):
        self.timed_drive(-2.6, -2.6, BACKOFF_MS, "BACKOFF", do_tile_update=False)
        self.edge_events = 0

    def forward_commit(self):
        self.timed_drive(FWD_SPEED, FWD_SPEED, FWD_COMMIT_MS, "FWD_COMMIT", do_tile_update=True)

    def turn_left_90(self):
        self.timed_drive(-TURN_SPEED, TURN_SPEED, TURN_90_MS, "TURN_LEFT_90", do_tile_update=False)
        self.heading = LEFT_OF[self.heading]
        self.edge_events = 0
        self.forward_commit()
        self.left_turn_cooldown = LEFT_TURN_COOLDOWN_STEPS

    def turn_right_90(self):
        self.timed_drive(TURN_SPEED, -TURN_SPEED, TURN_90_MS, "TURN_RIGHT_90", do_tile_update=False)
        self.heading = RIGHT_OF[self.heading]
        self.edge_events = 0
        self.forward_commit()

    # ---- sensors ----
    def read_ps(self):
        return [s.getValue() for s in self.ps]

    def front(self, p):
        return max(p[7], p[0])

    def left_max(self, p):
        return max(p[6], p[5])

    def left_open(self, p):
        return self.left_max(p) < self.LEFT_OPEN_TH

    def left_wall_present(self, p):
        return self.left_max(p) >= self.LEFT_WALL_PRESENT_TH

    # ---- calibration ----
    def calibrate_all(self):
        left_samples = []
        gs1_samples = []

        self.set_speed(TURN_SPEED, -TURN_SPEED)
        ms = 0
        while ms < CAL_ROTATE_MS:
            if not self.step(1):
                break
            ms += self.timestep
            p = self.read_ps()
            left_samples.append(self.left_max(p))
            gs1_samples.append(self.gs[1].getValue())
        self.stop()

        left_samples = np.array(left_samples, dtype=np.float64)
        gs1_samples = np.array(gs1_samples, dtype=np.float64)

        th = kmeans_1d_threshold(left_samples, iters=KMEANS_ITERS)
        th = float(np.clip(th, LEFT_OPEN_TH_MIN, LEFT_OPEN_TH_MAX))
        self.LEFT_OPEN_TH = th
        self.LEFT_WALL_PRESENT_TH = float(th + LEFT_WALL_PRESENT_PAD)

        mn, mx = float(gs1_samples.min()), float(gs1_samples.max())
        if abs(mx - mn) < 1e-6:
            mn -= 1.0
            mx += 1.0
        self.gs_th = float(mn + (mx - mn) * (0.5 + GS_MARGIN_FRAC))
        cur = self.gs[1].getValue()
        self.edge_state = "HIGH" if cur > self.gs_th else "LOW"
        self.edge_count = 0
        self.edge_events = 0

        self.map["meta"]["calibration"] = {
            "LEFT_OPEN_TH": self.LEFT_OPEN_TH,
            "LEFT_WALL_PRESENT_TH": self.LEFT_WALL_PRESENT_TH,
            "left_samples_min": float(left_samples.min()),
            "left_samples_max": float(left_samples.max()),
            "gs_th": self.gs_th,
            "gs_mn": mn,
            "gs_mx": mx,
            "t_unix": time.time(),
        }
        self.save_map()

    # ---- hairpin (2nd left => 180) ----
    def hairpin_probe_and_maybe_left_again(self):
        if not HAIRPIN_ENABLE:
            return

        confirm = 0
        steps = max(1, int(HAIRPIN_PROBE_MS / self.timestep))
        self.set_speed(FWD_SPEED, FWD_SPEED)

        for _ in range(steps):
            if not self.step(1):
                return
            tile_adv, edge_evt = self.maybe_advance_tile()
            p = self.read_ps()
            f = self.front(p)
            hairpin_flag = self.left_open(p) or (f >= HAIRPIN_FRONT_TH)

            confirm = confirm + 1 if hairpin_flag else 0
            self.hairpin_count = confirm
            self.log("HAIRPIN_PROBE", tile_adv, edge_evt, hairpin_flag)

            if confirm >= HAIRPIN_CONFIRM_STEPS:
                self.stop()
                self.log("HAIRPIN_DO_SECOND_LEFT", False, False, True)
                self.turn_left_90()
                self.hairpin_count = 0
                return

        self.stop()
        self.hairpin_count = 0

    # ---- ground tile tracking ----
    def gs_edge_event(self):
        cur = self.gs[1].getValue()
        state = "HIGH" if cur > self.gs_th else "LOW"
        if state == self.edge_state:
            self.edge_count = 0
            return False
        self.edge_count += 1
        if self.edge_count >= GS_DEBOUNCE:
            self.edge_state = state
            self.edge_count = 0
            return True
        return False

    def maybe_advance_tile(self):
        edge_evt = self.gs_edge_event()
        tile_adv = False
        if edge_evt:
            self.edge_events += 1
            if self.edge_events >= EDGES_PER_TILE:
                self.edge_events = 0
                dr, dc = DIR_VEC[self.heading]
                nr, nc = self.row + dr, self.col + dc
                if in_bounds(nr, nc):
                    self.row, self.col = nr, nc
                self.log_tile_visit()
                tile_adv = True
        return tile_adv, edge_evt

    # ---- map ----
    def load_or_init_map(self):
        if os.path.exists(MAPFILE):
            try:
                with open(MAPFILE, "r") as f:
                    data = json.load(f)
                if "tiles" in data and "meta" in data:
                    return data
            except:
                pass
        return {
            "meta": {
                "grid_n": GRID_N,
                "start": {"row": START_ROW, "col": START_COL, "heading": START_HEADING},
                "created_unix": time.time(),
                "notes": "Auto-calibrated left-open + WALL_LOST reacquire to prevent circles."
            },
            "visit_order": [],
            "tiles": {}
        }

    def ensure_tile(self, r, c):
        k = tile_key(r, c)
        if k not in self.map["tiles"]:
            self.map["tiles"][k] = {"row": r, "col": c, "visited": 0, "t_last": None}
        return self.map["tiles"][k]

    def log_tile_visit(self):
        t = self.ensure_tile(self.row, self.col)
        t["visited"] += 1
        t["t_last"] = time.time()
        self.map["visit_order"].append({
            "row": self.row, "col": self.col, "heading": self.heading, "t_unix": time.time()
        })
        self.save_map()

    def save_map(self):
        tmp = MAPFILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.map, f, indent=2)
        os.replace(tmp, MAPFILE)

    # ---- debug (pure) ----
    def log(self, action, tile_adv, edge_evt, hairpin_flag):
        p = self.read_ps()
        f = self.front(p)
        lmax = self.left_max(p)
        gs1 = self.gs[1].getValue()

        left_open_now = (self.LEFT_OPEN_TH is not None) and (lmax < self.LEFT_OPEN_TH)
        left_wall_now = (self.LEFT_WALL_PRESENT_TH is not None) and (lmax >= self.LEFT_WALL_PRESENT_TH)

        rec = {
            "sim_t": f"{self.robot.getTime():.3f}",
            "unix_t": f"{time.time():.3f}",
            "row": int(self.row),
            "col": int(self.col),
            "heading": self.heading,
            "mode": self.mode,
            "action": action,
            "vL": f"{self.vL:.3f}",
            "vR": f"{self.vR:.3f}",
            "ps0": f"{p[0]:.1f}",
            "ps5": f"{p[5]:.1f}",
            "ps6": f"{p[6]:.1f}",
            "ps7": f"{p[7]:.1f}",
            "front": f"{f:.1f}",
            "leftMax": f"{lmax:.1f}",
            "LEFT_OPEN_TH": "" if self.LEFT_OPEN_TH is None else f"{self.LEFT_OPEN_TH:.1f}",
            "LEFT_WALL_PRESENT_TH": "" if self.LEFT_WALL_PRESENT_TH is None else f"{self.LEFT_WALL_PRESENT_TH:.1f}",
            "left_open": int(left_open_now),
            "left_wall": int(left_wall_now),
            "left_open_count": int(self.left_open_count),
            "no_wall_count": int(self.no_wall_count),
            "wall_count": int(self.wall_count),
            "cooldown": int(self.left_turn_cooldown),
            "hairpin_flag": "" if hairpin_flag is None else int(bool(hairpin_flag)),
            "hairpin_count": int(self.hairpin_count),
            "gs1": f"{gs1:.1f}",
            "gs_th": "" if self.gs_th is None else f"{self.gs_th:.2f}",
            "edge_event": int(bool(edge_evt)),
            "edge_events": int(self.edge_events),
            "tile_advanced": int(bool(tile_adv)),
        }
        self.dbg.log(rec)

    # ---- main loop ----
    def run(self):
        self.step(10)
        self.calibrate_all()
        self.log_tile_visit()
        self.log("START", False, False, None)

        try:
            while True:
                if self.left_turn_cooldown > 0:
                    self.left_turn_cooldown -= 1

                p = self.read_ps()
                f = self.front(p)
                lwall = self.left_wall_present(p)
                lopen = self.left_open(p)

                # update wall counters
                if lwall:
                    self.wall_count += 1
                    self.no_wall_count = 0
                else:
                    self.no_wall_count += 1
                    self.wall_count = 0

                # mode transitions
                if self.mode == "NORMAL" and self.no_wall_count >= WALL_LOST_STEPS_ENTER:
                    self.mode = "WALL_LOST"
                    self.left_open_count = 0
                    self.log("ENTER_WALL_LOST", False, False, None)

                if self.mode == "WALL_LOST" and self.wall_count >= WALL_FOUND_STEPS_EXIT:
                    self.mode = "NORMAL"
                    self.left_open_count = 0
                    self.log("EXIT_WALL_LOST", False, False, None)

                # hard jam
                if f > HARD_JAM_TH:
                    self.log("HARD_JAM_BACKOFF", False, False, None)
                    self.backoff()
                    continue

                # -------------------------
                # WALL_LOST: forward-biased left search (prevents circles)
                # -------------------------
                if self.mode == "WALL_LOST":
                    # If front blocked, turn right (standard escape)
                    if f > FRONT_BLOCK_TH:
                        self.log("WALL_LOST_FRONT_BLOCK_RIGHT", False, False, None)
                        self.turn_right_90()
                        continue

                    # Search pattern: forward a bit, then tiny left pulse, repeat
                    self.timed_drive(FWD_SPEED, FWD_SPEED, SEARCH_FORWARD_MS, "WALL_LOST_FORWARD", do_tile_update=True)
                    # tiny left “peek” without spinning in circles
                    self.timed_drive(-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, SEARCH_PULSE_MS, "WALL_LOST_LEFT_PEEK", do_tile_update=False)
                    continue

                # -------------------------
                # NORMAL: left-hand rule with stability + cooldown
                # -------------------------
                # stabilize left-open detection
                if lopen and self.left_turn_cooldown == 0:
                    self.left_open_count += 1
                else:
                    self.left_open_count = 0

                # TAKE LEFT only if:
                # - left wall is present (we're latched), and
                # - left_open is stable, and
                # - not in cooldown
                if lwall and (self.left_open_count >= LEFT_OPEN_CONFIRM_STEPS) and self.left_turn_cooldown == 0:
                    self.log("LEFT_OPEN_TURN_LEFT", False, False, None)
                    self.turn_left_90()
                    self.left_open_count = 0
                    self.hairpin_probe_and_maybe_left_again()
                    continue

                # if front blocked -> right
                if f > FRONT_BLOCK_TH:
                    self.log("FRONT_BLOCK_TURN_RIGHT", False, False, None)
                    self.turn_right_90()
                    continue

                # otherwise straight
                self.set_speed(FWD_SPEED, FWD_SPEED)
                if not self.step(1):
                    return
                tile_adv, edge_evt = self.maybe_advance_tile()
                self.log("DRIVE_STRAIGHT", tile_adv, edge_evt, None)

        finally:
            self.dbg.close()


if __name__ == "__main__":
    EPuckController().run()
