#!/usr/bin/env python3
"""Split one big C file into per-subsystem modules (repo reorganisation,
22 Nov 2026). One-off tool, kept for the record; nothing builds it.

    python3 tools/reorg/split_c.py mission   # app_program/subsystem5/mission.c
    python3 tools/reorg/split_c.py robot_app # app_program/robot_app.c

Steps:
1. Optionally resolve known preprocessor switches (unifdef), e.g. the robot
   build's MISSION_ROBOT 1 (the bench keeps its own frozen copy).
2. Cut the file's top level into items: directives, declarations (types,
   variables, prototypes) and function definitions, each with the comments
   above it and the #if conditions around it (its guard).
3. Assign functions to parts by name (CONFIG); data items (macros, types,
   variables) go where they are used. Anything used by more than one part
   goes to the private header: macros and types as they are, variables as
   extern declarations (defined once, in their first user's part),
   functions as prototypes (their definitions lose static).
4. Write the header and parts, each item in its original order, every
   item under its original guard.
The proof is outside: the sims' outputs must stay byte-identical.
"""
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
APP = REPO / "app_program"

# ---- configuration ---------------------------------------------------------
CONFIG = {
    "mission": {
        "source": APP / "subsystem5/mission.c",
        "header": APP / "mission_private.h",
        "guard": "MISSION_PRIVATE_H",
        "public": "mission.h",
        "defined": {"MISSION_ROBOT": 1, "BENCH_FRAMES": 1},
        "undefined": {"BENCH_IMAGE"},
        "rewrites": {"MISSION_ROBOT && !defined(MISSION_SIM)":
                     "!defined(MISSION_SIM)",
                     "BENCH_FRAMES && ROBOT_MQTT": "ROBOT_MQTT",
                     "BENCH_FRAMES && !ROBOT_MQTT": "!ROBOT_MQTT",
                     "ROBOT_MQTT || MISSION_ROBOT": True},
        # The board_* redirects apply from their definition on; the robot
        # glue above them (mission_* before that line) calls the real board.
        "redirect_owner": ("board", "MISSION_BOARD_OWNER"),
        "redirect_re": r"^#define board_\w+\s+mission_\w+",
        "core": "core",
        "parts": {
            "core": APP / "subsystem5/mission.c",
            "board": APP / "subsystem5/mission_board.c",
            "frames": APP / "subsystem1/mission_frames.c",
            "drive": APP / "subsystem2/mission_drive.c",
            "line": APP / "subsystem3/mission_line.c",
            "barcode": APP / "subsystem3/mission_barcode.c",
            "terrain": APP / "subsystem4/mission_terrain.c",
            "compass": APP / "subsystem4/mission_compass.c",
            "avoid": APP / "subsystem5/mission_avoid.c",
        },
        "rules": [
            (r"^mission_(read_|accel_waiting|accel_sample|sonar_|servo_|"
             r"publish|motor_|encoders_|watchdog_|wrap_|origin|"
             r"motion_output|no_console)", "board"),
            (r"^(bench_mag|mission_mag)", "compass"),
            (r"^(bench_frame|bench_text|bench_line$|bench_flush|bench_u$|"
             r"bench_ux|bench_s$|bench_i$|bench_dec|bench_fixed|"
             r"bench_signed|bench_name|bench_flag|bench_rec|bench_live|"
             r"bench_tasks_text|bench_send|bench_bits|bench_put|bench_kind|"
             r"bench_x$|bench_str$|bench_char$)", "frames"),
            (r"^(bench_drive|bench_move|bench_pid|bench_pivot|bench_sweep|"
             r"bench_gains|bench_motion|mission_motion|bench_step|"
             r"bench_rate|bench_settle|bench_pose_text|bench_wheel_summary|"
             r"bench_marks)", "drive"),
            (r"^(bench_code|bench_fit|bench_d0|bench_vote|bench_spec)",
             "barcode"),
            (r"^(bench_line|bench_centre|bench_smooth|bench_auto|bench_cal|"
             r"bench_raw|bench_nav|mission_ir|bench_is_line|bench_is_centre|"
             r"bench_straight|bench_edge)", "line"),
            (r"^(bench_ramp|bench_lean|bench_lift|mission_accel|"
             r"bench_rise_wheel_up|bench_hold_lets_go)", "terrain"),
            (r"^(bench_avoid|bench_front|bench_scan|mission_sonar|"
             r"bench_servo|bench_sonar|bench_beams_text)", "avoid"),
        ],
        "titles": {
            "core": "the mission's core: commands, the run's start and end, "
                    "the idle report, the step",
            "board": "the robot app's owners as the mission's board calls "
                     "(rings, requests, the Motion snapshot)",
            "frames": "subsystem 1: the mission's minimum-bit frames, text, "
                      "the line recorder and live lines",
            "drive": "subsystem 2: drive, pivot and wheel-controller "
                     "settings",
            "line": "subsystem 3: line following, AUTO-CENTRE, calibration "
                    "and junction moves",
            "barcode": "subsystem 3: the barcode store and decoders "
                       "(Decoder 1, Decoder 2, D0) and their vote",
            "terrain": "subsystem 4: the hump (ramp), side leans and the "
                       "lift stop",
            "compass": "subsystem 4: the compass (LSM303DLHC magnetometer)",
            "avoid": "subsystem 5: the front sonar and the obstacle "
                     "go-around",
        },
    },
    "robot_app": {
        "source": APP / "robot_app.c",
        "header": APP / "robot_app_private.h",
        "guard": "ROBOT_APP_PRIVATE_H",
        "public": "robot_app.h",
        "what": "The robot app",
        "whose": "the robot app's parts",
        # File-scope names the mission also has (once both static, now each
        # shared by its own parts): renamed here, whole words.
        "renames": {"g_line": "g_line_ctrl",
                    "g_line_runtime": "g_line_ctrl_runtime"},
        "defined": {}, "undefined": set(), "rewrites": {},
        "core": "core",
        "parts": {
            "core": APP / "robot_app.c",
            "comm": APP / "subsystem1/robot_comm.c",
            "motion": APP / "subsystem2/robot_motion.c",
            "line": APP / "subsystem3/robot_line.c",
            "imu": APP / "subsystem4/robot_imu.c",
            "scan": APP / "subsystem5/robot_scan.c",
        },
        "rules": [
            (r"^robot_app_(comm_step|telemetry_step|perception|"
             r"diagnostic_frame|count_drop|wire_signed)$", "comm"),
            (r"^robot_app_(motion_step|commit|motion_health|reset|"
             r"motion_init|signed_count|path_delta|count_delta|duty_pm)$",
             "motion"),
            (r"^robot_app_(line_step|optical_path|nav|codebook_ready|"
             r"original_line)$", "line"),
            (r"^robot_app_(imu_|terrain_run$|count8$)", "imu"),
            (r"^robot_app_(scan_step|scan_cycle|front_result|servo|"
             r"range_valid)$", "scan"),
        ],
        "titles": {
            "core": "the shared record and its locks, the fault latch and "
                    "noise control, Safety, Vehicle (the mission's glue), "
                    "task timing and the task entries",
            "comm": "subsystem 1: MQTT commands (Comm) and the status, "
                    "perception and diagnostic frames (Telemetry)",
            "motion": "subsystem 2: the Motion task (wheel controller, "
                      "encoders, stalls, the approved reset)",
            "line": "subsystem 3: the Line task (IR samples, line control)",
            "imu": "subsystem 4: the IMU task (accelerometer, terrain, tilt "
                   "and collision, its evidence)",
            "scan": "subsystem 5: the Scan task (servo and sonar)",
        },
    },
}


# ---- unifdef ---------------------------------------------------------------
def _cond_value(cond, cfg):
    """True/False if cond is decided by the configured switches, else None."""
    text = re.sub(r"/\*.*?\*/|//.*", "", cond).strip()
    if text in cfg["rewrites"]:
        value = cfg["rewrites"][text]
        return value if isinstance(value, bool) else ("rewrite", value)
    names = set(re.findall(r"[A-Za-z_]\w*", text)) - {"defined"}
    known = set(cfg["defined"]) | set(cfg["undefined"])
    if not names or not names <= known:
        return None
    expr = text
    expr = re.sub(r"defined\s*\(\s*(\w+)\s*\)|defined\s+(\w+)",
                  lambda m: "1" if (m.group(1) or m.group(2))
                  in cfg["defined"] else "0", expr)
    for name, value in cfg["defined"].items():
        expr = re.sub(rf"\b{name}\b", str(value), expr)
    for name in cfg["undefined"]:
        expr = re.sub(rf"\b{name}\b", "0", expr)
    expr = expr.replace("&&", " and ").replace("||", " or ")
    expr = re.sub(r"!(?!=)", " not ", expr)
    return bool(eval(expr, {"__builtins__": {}}))


def unifdef(lines, cfg):
    """Resolve #if/#ifdef/#ifndef/#else/#endif decided by cfg; keep others."""
    out, stack = [], []   # frames: [kind, taken, active_before]
    def active():
        return all(f[1] for f in stack if f[0] == "resolved")
    for line in lines:
        m = re.match(r"\s*#\s*(if|ifdef|ifndef|else|elif|endif)\b(.*)", line)
        if not m:
            if active():
                out.append(line)
            continue
        kind, rest = m.group(1), m.group(2).strip()
        if kind in ("if", "ifdef", "ifndef"):
            cond = rest if kind == "if" else (
                f"defined({rest.split()[0]})" if kind == "ifdef"
                else f"!defined({rest.split()[0]})")
            value = _cond_value(cond, cfg)
            if isinstance(value, tuple):
                stack.append(["kept"])
                if active():
                    out.append(f"#if {value[1]}\n")
            elif value is None:
                stack.append(["kept"])
                if active():
                    out.append(line)
            else:
                stack.append(["resolved", value])
        elif kind == "elif":
            raise SystemExit("unifdef: #elif not supported")
        elif kind == "else":
            frame = stack[-1]
            if frame[0] == "resolved":
                frame[1] = not frame[1]
            elif active():
                out.append(line)
        else:
            frame = stack.pop()
            if frame[0] == "kept" and active():
                out.append(line)
    assert not stack, "unbalanced #if"
    return out


# ---- scanning the top level ------------------------------------------------
IDENT = re.compile(r"[A-Za-z_]\w*")


def strip_comments_strings(text):
    """Code with comments, strings and char literals blanked."""
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"|'
                  r"'(?:\\.|[^'\\\n])*'", " ", text, flags=re.S)


class Item:
    def __init__(self, kind, lead, code, guard, line):
        self.kind = kind          # pp | decl | func
        self.lead = lead          # comments/blank lines before it
        self.code = code          # the item itself
        self.guard = guard        # tuple of #if conditions
        self.line = line          # first line of code (1-based)
        self.names = set()        # names it defines
        self.uses = set()         # identifiers it uses (code + guard)
        self.part = None
        self.external = False     # a func/var used outside its part
        self.proto_of = None      # prototype: the function's name

    def __repr__(self):
        return f"<{self.kind} {sorted(self.names)} L{self.line} {self.part}>"


def scan(text):
    """Top-level items of a preprocessed-switch-free C file."""
    items, guard = [], []
    i, n, line = 0, len(text), 1
    lead_start = 0
    while i < n:
        # Skip blanks and comments into the lead.
        j = i
        while j < n:
            if text[j] in " \t\r\n":
                j += 1
            elif text.startswith("/*", j):
                j = text.index("*/", j + 2) + 2
            elif text.startswith("//", j):
                j = text.find("\n", j)
                j = n if j < 0 else j
            else:
                break
        if j >= n:
            break
        start_line = text.count("\n", 0, j) + 1
        if text[j] == "#":
            k = j
            while True:
                e = text.find("\n", k)
                e = n if e < 0 else e
                if text[e - 1:e] == "\\" or text[e - 2:e] == "\\\r":
                    k = e + 1
                    continue
                # A trailing comment that runs on: up to its line's end.
                last = text.rfind("/*", j, e)
                if last >= 0 and text.find("*/", last, e) < 0:
                    close = text.index("*/", last) + 2
                    e = text.find("\n", close)
                    e = n if e < 0 else e
                break
            directive = text[j:e]
            m = re.match(r"#\s*(\w+)\s*(.*)", directive, re.S)
            word, rest = m.group(1), m.group(2).strip()
            if word in ("if", "ifdef", "ifndef"):
                cond = rest if word == "if" else (
                    f"defined({rest.split()[0]})" if word == "ifdef"
                    else f"!defined({rest.split()[0]})")
                guard.append(cond)
                lead_start = e
            elif word == "else":
                guard[-1] = f"!({guard[-1]})"
                lead_start = e
            elif word == "endif":
                guard.pop()
                lead_start = e
            else:
                items.append(Item("pp", text[lead_start:j], directive,
                                  tuple(guard), start_line))
                lead_start = e
            i = e
            continue
        # A declaration or a function definition.
        k, depth, brace_at = j, 0, None
        while k < n:
            c = text[k]
            if text.startswith("/*", k):
                k = text.index("*/", k + 2) + 2
                continue
            if text.startswith("//", k):
                k = text.find("\n", k)
                continue
            if c in "\"'":
                q = c
                k += 1
                while text[k] != q:
                    k += 2 if text[k] == "\\" else 1
                k += 1
                continue
            if c == "{":
                if depth == 0 and brace_at is None:
                    brace_at = k
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    head = strip_comments_strings(text[j:brace_at]).rstrip()
                    if head.endswith(")"):
                        k += 1
                        items.append(Item("func", text[lead_start:j],
                                          text[j:k], tuple(guard),
                                          start_line))
                        break
            elif c == ";" and depth == 0:
                k += 1
                items.append(Item("decl", text[lead_start:j], text[j:k],
                                  tuple(guard), start_line))
                break
            elif c == "\n" and depth == 0 and text[k + 1:k + 2] == "#":
                pass
            k += 1
        else:
            raise SystemExit(f"scan: unterminated item at line {start_line}")
        lead_start = k
        i = k
    tail = text[lead_start:]
    return items, tail


def analyse(item):
    code = strip_comments_strings(item.code)
    if item.kind == "pp":
        m = re.match(r"#\s*(define|undef)\s+(\w+)", item.code)
        if m and m.group(1) == "define":
            item.names.add(m.group(2))
            body = code[m.end():]
        elif m:
            item.undef = m.group(2)
            body = ""
        else:
            body = ""
        item.uses = set(IDENT.findall(body))
    elif item.kind == "func":
        head = code[:code.index("{")]
        name = re.findall(r"(\w+)\s*\(", head)[0]
        item.names.add(name)
        item.uses = set(IDENT.findall(code)) - {name}
    else:
        flat = re.sub(r"\s+", " ", code).strip()
        if flat.startswith("_Static_assert"):
            # A compile-time check: placed like a directive (the header,
            # with what it uses).
            item.kind = "pp"
            item.uses = set(IDENT.findall(code)) - {"_Static_assert"}
            for cond in item.guard:
                item.uses |= set(IDENT.findall(cond)) - {"defined"}
            return
        if flat.startswith("typedef"):
            outer = re.sub(r"\{.*\}", "{}", flat)
            item.names.add(re.findall(r"(\w+)\s*(?:\[[^\]]*\])?\s*;$",
                                      outer)[0])
            tag = re.match(r"typedef (?:struct|enum|union) (\w+)", flat)
            if tag:
                item.names.add(tag.group(1))
            if " enum" in f" {flat[:12]}":
                inner = flat[flat.index("{") + 1:flat.rindex("}")]
                for part in inner.split(","):
                    word = IDENT.match(part.strip())
                    if word:
                        item.names.add(word.group(0))
        elif re.match(r"(static |extern )?(inline )?[\w\s\*]+\b(\w+)\s*\("
                      r"[^=]*\)\s*(__attribute__\(\(.*\)\))?\s*;$", flat) \
                and "=" not in flat.split("(")[0]:
            name = re.findall(r"(\w+)\s*\(", flat)[0]
            item.proto_of = name
        else:
            head = flat.split("=")[0]
            head = re.sub(r"__attribute__\s*\(\(.*?\)\)", "", head)
            head = re.sub(r"\[[^\]]*\]", "", head).rstrip(" ;")
            for declarator in head.split(","):
                words = IDENT.findall(declarator)
                if words:
                    item.names.add(words[-1])
        item.uses = set(IDENT.findall(code)) - item.names
    for cond in item.guard:
        item.uses |= set(IDENT.findall(cond)) - {"defined"}


# ---- assignment ------------------------------------------------------------
def assign(items, cfg, redirect_line):
    by_name = {}
    for it in items:
        for name in it.names:
            by_name.setdefault(name, []).append(it)
    funcs = [it for it in items if it.kind == "func"]
    for it in funcs:
        name = next(iter(it.names))
        it.part = cfg["core"]
        for pattern, part in cfg["rules"]:
            if re.search(pattern, name):
                it.part = part
                break
        if (part_owner := cfg.get("redirect_owner")) and \
                it.part == part_owner[0] and redirect_line and \
                it.line > redirect_line:
            it.part = cfg["core"]
    # Who uses each item (by part), through data items transitively.
    users = {id(it): set() for it in items}
    def refs(it):
        for name in it.uses:
            for target in by_name.get(name, ()):
                if target is not it:
                    yield target
    changed = True
    for it in funcs:
        users[id(it)].add(it.part)
    while changed:
        changed = False
        for it in items:
            if not users[id(it)]:
                continue
            for target in refs(it):
                if target.kind == "func":
                    continue
                before = len(users[id(target)])
                users[id(target)] |= users[id(it)]
                changed |= len(users[id(target)]) != before
    # Direct function users (a call from another part makes it external).
    callers = {id(it): set() for it in funcs}
    for it in items:
        for target in refs(it):
            if target.kind == "func":
                callers[id(target)] |= (users[id(it)] if it.kind != "func"
                                        else {it.part})
    # Data placement.
    HEADER = "#header"
    for it in items:
        if it.kind == "func":
            continue
        if it.proto_of:
            continue
        u = users[id(it)]
        variable = it.kind == "decl" and not \
            it.code.lstrip().startswith("typedef")
        if it.kind == "pp" and not it.names and not hasattr(it, "undef"):
            it.part = HEADER          # includes, pragmas, errors: see below
        elif len(u) == 1:
            it.part = next(iter(u))
        elif not u:
            it.part = cfg["core"]
        elif variable:
            # Defined once (its first user, the core if it is one), extern
            # in the header.
            it.part = cfg["core"] if cfg["core"] in u else sorted(u)[0]
            it.external = True
        else:
            it.part = HEADER
    # Settings before the last of the file's first includes (e.g.
    # PROHIBIT_DEF_SIZE_T) must precede them: the header, in order.
    first_includes = [k for k, it in enumerate(items[:80])
                      if it.kind == "pp" and it.code.startswith("#include")]
    for it in items[:first_includes[-1] if first_includes else 0]:
        if it.kind == "pp":
            it.part = HEADER
    # #undef follows its macro's #define.
    for it in items:
        if hasattr(it, "undef"):
            defs = by_name.get(it.undef, [])
            it.part = defs[-1].part if defs else HEADER
    # Closure: what the header uses must be in the header (or external).
    changed = True
    while changed:
        changed = False
        for it in items:
            in_header = it.part == HEADER
            for target in refs(it):
                if target.kind == "func":
                    if (in_header or target.part != it.part) and \
                            not target.external:
                        target.external = True
                        changed = True
                    continue
                if target.proto_of:
                    continue
                if target.part == HEADER:
                    continue
                if in_header or (target.part != it.part and
                                 it.kind != "func") or \
                        (it.kind == "func" and target.part != it.part):
                    if target.kind == "decl" and not \
                            target.code.lstrip().startswith("typedef"):
                        if not target.external:
                            target.external = True
                            changed = True
                    else:
                        target.part = HEADER
                        changed = True
    # Prototypes follow their functions.
    func_by = {next(iter(f.names)): f for f in funcs}
    for it in items:
        if it.proto_of:
            f = func_by.get(it.proto_of)
            it.part = HEADER if (f is None or f.external) else f.part
    return HEADER


# ---- emission --------------------------------------------------------------
def unstatic(code):
    return re.sub(r"^(\s*)static\s+", r"\1", code, count=1)


def extern_decl(code):
    flat = strip_comments_strings(code)
    cut = len(code)
    depth = 0
    for k, c in enumerate(flat):
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "=" and depth == 0:
            cut = k
            break
    head = code[:cut].rstrip().rstrip(";").rstrip()
    head = re.sub(r"^\s*static\s+", "", head)
    if head.endswith("[]") and cut < len(code):
        # An unsized table: its size from the initializer, so sizeof works
        # where the header's macros use it.
        init = flat[cut:]
        body = init[init.index("{") + 1:init.rindex("}")]
        count, depth = 0, 0
        for c in body:
            if c in "([{":
                depth += 1
            elif c in ")]}":
                depth -= 1
            elif c == "," and depth == 0:
                count += 1
        if body.strip() and not body.rstrip().endswith(","):
            count += 1
        head = head[:-2] + f"[{count}]"
    return f"extern {head};"


def guarded(chunks):
    """Text for (guard, text) chunks, merging runs under the same guard."""
    out, current = [], ()
    for guard, text in chunks:
        if guard != current:
            if current:
                out.append("#endif\n")
            if guard:
                out.append("#if " + " && ".join(
                    f"({g})" if len(guard) > 1 else g for g in guard) + "\n")
            current = guard
        out.append(text)
    if current:
        out.append("#endif\n")
    return "".join(out)


def emit(items, cfg, header_key, tail):
    funcs = [it for it in items if it.kind == "func"]
    func_part = {next(iter(f.names)): f.part for f in funcs}
    parts = {key: [] for key in cfg["parts"]}
    # Each subsystem part's own header: the prototypes of its functions the
    # other parts call (the core's go in the private header).
    part_headers = {key: [] for key in cfg["parts"] if key != cfg["core"]}
    header = []
    for it in items:
        # One blank line if the original had one, then its comments.
        lead = (("\n" if it.lead.count("\n") >= 2 else "") +
                (it.lead.strip("\n") + "\n" if it.lead.strip() else ""))
        if it.part == header_key:
            code = unstatic(it.code) if it.proto_of else it.code
            owner_part = func_part.get(it.proto_of)
            if it.proto_of and owner_part in part_headers:
                part_headers[owner_part].append((it.guard,
                                                 lead + code + "\n"))
                continue
            header.append((it.guard, lead + code + "\n"))
        else:
            code = it.code
            if it.external and it.kind in ("func", "decl"):
                code = unstatic(code)
            parts[it.part].append((it.guard, lead + code + "\n"))
        if it.kind == "decl" and it.external and not it.proto_of and \
                it.part != header_key:
            header.append((it.guard, extern_decl(it.code) + "\n"))
    protos = {it.proto_of for it in items if it.proto_of}
    for f in funcs:
        name = next(iter(f.names))
        if f.external and name not in protos:
            head = f.code[:f.code.index("{")].rstrip()
            target = part_headers.get(f.part, header)
            target.append((f.guard, "\n" + unstatic(head) + ";\n"))
    owner = cfg.get("redirect_owner")
    redirect = re.compile(cfg.get("redirect_re", r"^$"))
    # Header text: the redirects only outside their owner's part.
    lines = []
    for guard, text in header:
        if owner and redirect.search(text.strip().splitlines()[-1]):
            lines.append(((f"!defined({owner[1]})",) + guard, text))
        else:
            lines.append((guard, text))
    import textwrap
    def brief(file_name, words):
        body = textwrap.wrap(f"@brief {words}", 74)
        return (f"/** @file {file_name}\n" +
                "".join(f" * {line}\n" for line in body) + " */\n")
    name = cfg["header"].name
    whose = cfg.get("whose", "the mission's parts")
    head = brief(name, f"Private to {whose} ({cfg['source'].name} and its "
                       f"subsystem files): what more than one part uses. "
                       f"Split out of the one file by tools/reorg/"
                       f"split_c.py (22 Nov 2026).")
    head += f"#ifndef {cfg['guard']}\n#define {cfg['guard']}\n"
    # Every part's header at the end: each part sees all the others' calls.
    includes = "".join(f'#include "{cfg["parts"][key].stem}.h"\n'
                       for key in part_headers)
    eof = "\n/* End of file. */\n\n"
    cfg["header"].write_text(head + guarded(lines) + "\n" + includes +
                             f"\n#endif /* {cfg['guard']} */\n" + eof)
    written = []
    what = cfg.get("what", "The mission")
    for key, protos_of in part_headers.items():
        path = cfg["parts"][key].with_suffix(".h")
        guard = path.stem.upper() + "_H"
        text = (brief(path.name, f"{what}: {cfg['titles'][key]}; the calls "
                                 f"the other parts make into "
                                 f"{path.stem}.c. Include {name} first.") +
                f"#ifndef {guard}\n#define {guard}\n" + guarded(protos_of) +
                f"\n#endif /* {guard} */\n" + eof)
        path.write_text(text)
        written.append((path, len(text.splitlines())))
    for key, chunks in parts.items():
        path = cfg["parts"][key]
        title = cfg["titles"][key]
        top = brief(path.name, f"{what}: {title}. Split out of the one file "
                               f"by tools/reorg/split_c.py (22 Nov 2026); "
                               f"what the parts share is in {name}.")
        if owner and key == owner[0]:
            top += f"#define {owner[1]} /* the real board calls here */\n"
        if key == cfg["core"]:
            top += f'#include "{cfg["public"]}"\n'
        top += f'#include "{name}"\n'
        if key in part_headers:
            top += f'#include "{path.stem}.h"\n'
        body = guarded(chunks)
        if key == cfg["core"]:
            body += tail.rstrip("\n") + "\n\n"
        else:
            body += eof
        path.write_text(top + body)
        written.append((path, len((top + body).splitlines())))
    return written


def main():
    key = sys.argv[1]
    cfg = CONFIG[key]
    text = cfg["source"].read_text()
    if "Split out of the one file" in text:
        raise SystemExit(f"{cfg['source'].name} is already split: run this "
                         f"on the one-file version")
    for old, new in cfg.get("renames", {}).items():
        text = re.sub(rf"\b{old}\b", new, text)
    lines = text.splitlines(keepends=True)
    if cfg.get("defined") or cfg.get("undefined"):
        lines = unifdef(lines, cfg)
    text = "".join(lines)
    redirect_line = None
    if cfg.get("redirect_re"):
        for number, line in enumerate(lines, 1):
            if re.match(cfg["redirect_re"], line):
                redirect_line = number
                break
    items, tail = scan(text)
    for it in items:
        analyse(it)
    header_key = assign(items, cfg, redirect_line)
    if "--dry" in sys.argv:
        from collections import Counter
        print(Counter(it.part for it in items))
        for it in items:
            if it.kind == "func" and it.part == cfg["core"]:
                print("core:", next(iter(it.names)))
        return
    for path, count in emit(items, cfg, header_key, tail):
        print(f"{path.relative_to(REPO)}: {count} lines")
    print(f"{cfg['header'].relative_to(REPO)}: "
          f"{len(cfg['header'].read_text().splitlines())} lines")


if __name__ == "__main__":
    main()
