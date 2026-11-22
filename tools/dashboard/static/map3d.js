/* IS29 dashboard: the live map in 3D (three.js 0.160, vendored in
 * static/vendor, no internet needed).
 *
 * Map frame (server.py, the car's odometry from START): x ahead at START,
 * y + left, mm; heading deg, + left (counter-clockwise from above).
 * three.js frame: metres, X = x, Y up, Z = -y, so a heading h is a rotation
 * of h about +Y and a car's local +X is its forward, local -Z its left.
 *
 * What is drawn, and from what:
 *   floor       white, like the track's foam tiles; 100 mm grid
 *   black tape  the path the car drove while following the line, 18 mm wide
 *               (it drives centred on the tape, so its path is the line)
 *   dashed      the path while measuring or going round an obstacle (off
 *               the line: no tape there)
 *   crossing    each junction the car detected: a tape across its path
 *   card        each card read, under the barcode sensor (80 mm left of the
 *               car's centre, ROBOT_BARCODE_LATERAL_OFFSET_M), its length as
 *               read, printed as the Code 39 the car decided on
 *   dome        each hump, under the right wheel (the track's hump is always
 *               there), its height as measured
 *   box         each obstacle: face and width as measured; depth and height
 *               assumed (the sonar sees the face only), drawn see-through
 *   car         the robot at its last pose: blue chassis, yellow body, the
 *               Robo Pico board, the HC-SR04 on its servo, rear wheels
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";

const MM = 0.001;
const TAPE_MM = 18;            /* track line width */
const CARD_LEFT_MM = 80;       /* barcode sensor left of the centre */
const CARD_WIDE_MM = 40;       /* card across the travel (drawn) */
const WHEEL_TRACK_MM = 130;    /* ROBOT_TRACK_WIDTH_M */
const OBST_H_MM = 100;         /* obstacle height: not measured */
const SONAR_MM = 70;           /* sonar ahead of the axle */
const SONAR_H = 0.125;         /* sonar height above the floor, m */
const C39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%";
const C39_PAT = [0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064, 0x109, 0x049, 0x148, 0x019,
  0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
  0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0, 0x085, 0x184, 0x0C4, 0x094, 0x0A8, 0x0A2, 0x08A, 0x02A];

/* map mm (x, y) and a height in m -> three.js */
const V = (x, y, up = 0) => new THREE.Vector3(x * MM, up, -y * MM);
const rad = (deg) => deg * Math.PI / 180;

/* An obstacle's own frame (OBST / the sweep: x0, y0, h0). */
function frameOf(f) {
  const h = rad(f.h0), c = Math.cos(h), s = Math.sin(h);
  return (x, y) => [f.x0 + x * c - y * s, f.y0 + x * s + y * c];
}

/* A flat strip of width w (mm) along 2D points (mm), lifted a little. */
function ribbon(points, w, lift, material) {
  const n = points.length;
  if (n < 2) return null;
  const pos = new Float32Array(n * 2 * 3), idx = [];
  for (let i = 0; i < n; i++) {
    const a = points[Math.max(0, i - 1)], b = points[Math.min(n - 1, i + 1)];
    let tx = b[0] - a[0], ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1;
    tx /= len; ty /= len;
    const lx = -ty * w / 2, ly = tx * w / 2;          /* left normal */
    const L = V(points[i][0] + lx, points[i][1] + ly, lift), R = V(points[i][0] - lx, points[i][1] - ly, lift);
    pos.set([L.x, L.y, L.z, R.x, R.y, R.z], i * 6);
    if (i) idx.push(2 * i - 2, 2 * i - 1, 2 * i, 2 * i - 1, 2 * i + 1, 2 * i);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  const m = new THREE.Mesh(g, material);
  m.receiveShadow = true;
  return m;
}

/* Smooth a pose path (one point every ~50 mm) for the tape: drop repeats
 * (pivots, stops), then a centripetal Catmull-Rom, sampled every ~8 mm. */
function smooth(pts) {
  const keep = [];
  pts.forEach((p) => {
    const q = keep[keep.length - 1];
    if (!q || Math.hypot(p[0] - q[0], p[1] - q[1]) > 3) keep.push(p);
  });
  if (keep.length < 3) return keep;
  const curve = new THREE.CatmullRomCurve3(keep.map((p) => new THREE.Vector3(p[0], p[1], 0)), false, "centripetal");
  let len = 0;
  for (let i = 1; i < keep.length; i++) len += Math.hypot(keep[i][0] - keep[i - 1][0], keep[i][1] - keep[i - 1][1]);
  return curve.getSpacedPoints(Math.min(6000, Math.max(8, Math.round(len / 8)))).map((v) => [v.x, v.y]);
}

/* Text on a sprite, about 22 mm tall. */
function label(text, color, bg) {
  const c = document.createElement("canvas"), g = c.getContext("2d"), px = 44;
  g.font = `600 ${px}px system-ui, sans-serif`;
  const w = Math.ceil(g.measureText(text).width) + 24;
  c.width = w; c.height = px + 18;
  g.font = `600 ${px}px system-ui, sans-serif`;
  g.fillStyle = bg; g.fillRect(0, 0, c.width, c.height);
  g.fillStyle = color; g.textBaseline = "middle"; g.fillText(text, 12, c.height / 2 + 2);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  /* the same size on screen however far the camera is (about 18 px tall) */
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false, sizeAttenuation: false }));
  const h = 0.036;
  s.scale.set(h * c.width / c.height, h, 1);
  s.renderOrder = 10;
  return s;
}

/* A card's face: the Code 39 of *letter* (narrow 1, wide 3), bars across
 * the travel, printed in the order the car crossed them. */
function cardTexture(letter, rev) {
  const text = letter.startsWith("*") ? letter : `*${letter}*`;
  const els = [];
  [...text].forEach((ch, i) => {
    const k = C39_CHARS.indexOf(ch);
    if (k < 0) return;
    if (i) els.push(1);
    for (let j = 0; j < 9; j++) els.push(C39_PAT[k] & (1 << (8 - j)) ? 3 : 1);
  });
  if (rev) els.reverse();
  const mods = els.reduce((a, b) => a + b, 0) || 1;
  const c = document.createElement("canvas"), g = c.getContext("2d");
  c.width = 1024; c.height = 128;
  g.fillStyle = "#ffffff"; g.fillRect(0, 0, c.width, c.height);
  const pad = 40, k = (c.width - 2 * pad) / mods;
  let x = pad;
  g.fillStyle = "#111111";
  els.forEach((m, i) => { if (i % 2 === 0) g.fillRect(x, 10, m * k, c.height - 20); x += m * k; });
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

/* The robot, axle centre on the floor at the origin, facing +X. From the
 * team's photos; sizes from robot_config.h where it has them. */
function buildCar() {
  const car = new THREE.Group();
  const mat = (color, extra = {}) => new THREE.MeshStandardMaterial(Object.assign({ color, roughness: .55, metalness: .05 }, extra));
  const box = (w, h, d, m, x, y, z) => {
    const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);
    b.position.set(x, y, z); b.castShadow = true; b.receiveShadow = true; car.add(b);
    return b;
  };
  const blue = mat(0x1f6fd6, { metalness: .45, roughness: .35 });
  const yellow = mat(0xf2c21a);
  const purple = mat(0x4a2c7d);
  const black = mat(0x151515, { roughness: .8 });
  const white = mat(0xf2f2f0);
  const pcb = mat(0x1d4fa0);
  /* blue aluminium chassis and its front bracket */
  box(0.165, 0.028, 0.112, blue, 0.025, 0.030, 0);
  box(0.012, 0.040, 0.112, blue, 0.105, 0.030, 0);
  /* yellow printed body */
  box(0.115, 0.058, 0.118, yellow, -0.005, 0.073, 0);
  /* Robo Pico board on top, the Pico on it */
  box(0.105, 0.006, 0.095, purple, -0.010, 0.106, 0);
  box(0.052, 0.004, 0.021, mat(0x1b7a3e), -0.010, 0.111, 0);
  /* HC-SR04 on its servo at the front */
  box(0.012, 0.018, 0.020, mat(0x2b2b2b), SONAR_MM * MM - 0.004, 0.098, 0);
  box(0.004, 0.021, 0.046, pcb, SONAR_MM * MM, SONAR_H, 0);
  [-0.013, 0.013].forEach((z) => {
    const eye = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.012, 20), mat(0xc9ccd1, { metalness: .8, roughness: .3 }));
    eye.rotation.z = Math.PI / 2; eye.position.set(SONAR_MM * MM + 0.008, SONAR_H, z); eye.castShadow = true; car.add(eye);
  });
  /* rear wheels on the axle (radius 32.5 mm, track 130 mm), white hubs */
  [-1, 1].forEach((side) => {
    const z = side * WHEEL_TRACK_MM / 2 * MM;
    const tyre = new THREE.Mesh(new THREE.CylinderGeometry(0.0325, 0.0325, 0.026, 32), black);
    tyre.rotation.x = Math.PI / 2; tyre.position.set(0, 0.0325, z); tyre.castShadow = true; car.add(tyre);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.019, 0.019, 0.027, 24), white);
    hub.rotation.x = Math.PI / 2; hub.position.set(0, 0.0325, z); car.add(hub);
  });
  /* front castor */
  const castor = new THREE.Mesh(new THREE.SphereGeometry(0.010, 16, 12), mat(0xdddddd, { metalness: .6 }));
  castor.position.set(0.085, 0.010, 0); car.add(castor);
  /* IR sensors: two on the line 130 mm ahead, 18 mm apart; the barcode one 80 mm left */
  [[0.130, -0.009], [0.130, 0.009], [0.100, -CARD_LEFT_MM * MM]].forEach(([x, z]) => {
    box(0.030, 0.003, 0.014, pcb, x, 0.016, z);
    box(0.004, 0.012, 0.004, black, x + 0.010, 0.008, z);
  });
  box(0.040, 0.004, 0.030, white, 0.110, 0.012, -0.050);     /* white sensor plate */
  return car;
}

export function createMap3d(host) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  host.appendChild(renderer.domElement);
  renderer.domElement.style.display = "block";

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 60);
  camera.position.set(-0.6, 0.55, 0.35);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI / 2 - 0.04;
  controls.minDistance = 0.12;
  controls.maxDistance = 20;
  controls.addEventListener("start", () => { api.follow = false; api.onFollow && api.onFollow(false); });

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8c8c86, 1.15));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -0.6, right: 0.6, top: 0.6, bottom: -0.6, near: 0.1, far: 4 });
  sun.shadow.bias = -0.0004;
  scene.add(sun, sun.target);

  const floorMat = new THREE.MeshStandardMaterial({ color: 0xf6f6f3, roughness: .95 });
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), floorMat);
  floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true;
  scene.add(floor);
  let grid = null;

  const car = buildCar();
  car.visible = false;
  scene.add(car);
  const world = new THREE.Group();          /* rebuilt on each update */
  scene.add(world);

  const tapeMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: .8 });
  const cardTex = new Map();

  const api = {
    follow: true, onFollow: null, visible: false, data: null,
    update(map) { api.data = map; build(map); api.redraw(); },
    setFollow(on) { api.follow = on; if (on) chase(true); api.redraw(); },
    fit() { api.follow = false; fitAll(); api.redraw(); },
    setTheme(dark, bg) { scene.background = new THREE.Color(bg); floorMat.color.set(dark ? 0xdcdcd6 : 0xf6f6f3); api.redraw(); },
    show(on) { api.visible = on; if (on) { resize(); api.redraw(); loop(); } },
  };

  let bounds = null, firstFit = true, runT0 = null;
  function build(map) {
    world.clear();
    /* a new run's map (its t0 changes): frame it afresh */
    if (map && map.t0 !== runT0) { runT0 = map.t0; firstFit = true; }
    const path = (map && map.path) || [];
    const marks = (map && map.marks) || [];
    const obst = (map && map.obst) || [];
    /* bounds: path, marks, obstacles, and START */
    const b = new THREE.Box2(new THREE.Vector2(-300, -300), new THREE.Vector2(300, 300));
    path.forEach((p) => b.expandByPoint(new THREE.Vector2(p[0], p[1])));
    marks.forEach((m) => b.expandByPoint(new THREE.Vector2(m.x, m.y)));
    obst.forEach((o) => { const tf = frameOf(o); [[o.face, -o.right], [o.face + o.depth, o.left]].forEach(([x, y]) => b.expandByPoint(new THREE.Vector2(...tf(x, y)))); });
    bounds = b.clone().expandByScalar(150);   /* what "Show whole run" frames */
    b.expandByScalar(400);                     /* the floor, a little wider */
    const cx = (b.min.x + b.max.x) / 2, cy = (b.min.y + b.max.y) / 2;
    const sx = (b.max.x - b.min.x) * MM, sy = (b.max.y - b.min.y) * MM;
    const size = Math.ceil(Math.max(sx, sy) * 10) / 10;   /* square, like the grid */
    floor.scale.set(size, size, 1);
    floor.position.copy(V(cx, cy, 0));
    if (grid) scene.remove(grid);
    grid = new THREE.GridHelper(size, Math.round(size / 0.1), 0xc9c9c2, 0xe2e2dc);
    grid.position.copy(V(cx, cy, 0.0004));
    grid.material.transparent = true; grid.material.opacity = .55;
    scene.add(grid);

    /* START */
    const start = new THREE.Mesh(new THREE.CircleGeometry(0.012, 24), new THREE.MeshBasicMaterial({ color: 0x0ca30c }));
    start.rotation.x = -Math.PI / 2; start.position.copy(V(0, 0, 0.0012)); world.add(start);
    const sl = label("START", "#0b0b0b", "rgba(255,255,255,.85)"); sl.position.copy(V(0, 0, 0.05)); world.add(sl);

    /* the path: tape where following the line, dashed off it */
    const runs = [];
    path.forEach((p) => {
      const off = p[3] === 1, last = runs[runs.length - 1];
      if (!last || last.off !== off) runs.push({ off, pts: last ? [last.pts[last.pts.length - 1]] : [] });
      runs[runs.length - 1].pts.push([p[0], p[1]]);
    });
    if (runs.length && runs[0].pts.length && (runs[0].pts[0][0] || runs[0].pts[0][1])) runs[0].pts.unshift([0, 0]);
    runs.forEach((r) => {
      if (r.pts.length < 2) return;
      if (!r.off) {
        const m = ribbon(smooth(r.pts), TAPE_MM, 0.0008, tapeMat);
        if (m) world.add(m);
      } else {
        const g = new THREE.BufferGeometry().setFromPoints(smooth(r.pts).map(([x, y]) => V(x, y, 0.002)));
        const l = new THREE.Line(g, new THREE.LineDashedMaterial({ color: 0x2a78d6, dashSize: 0.02, gapSize: 0.012 }));
        l.computeLineDistances(); world.add(l);
      }
    });

    /* marks: junction crossings, cards, humps, stops */
    marks.forEach((m) => {
      const h = rad(m.h || 0), ux = Math.cos(h), uy = Math.sin(h), lx = -uy, ly = ux;
      if (m.kind === "junction") {
        const half = 250;
        const t = ribbon([[m.x - lx * half, m.y - ly * half], [m.x + lx * half, m.y + ly * half]], TAPE_MM, 0.0009, tapeMat);
        if (t) world.add(t);
        const s = label(m.text, "#0b0b0b", "rgba(255,255,255,.9)"); s.position.copy(V(m.x + lx * 120, m.y + ly * 120, 0.05)); world.add(s);
      } else if (m.kind === "card") {
        const len = Math.max(60, Math.min(400, m.len || 200));
        const key = `${m.text}|${m.rev ? 1 : 0}`;
        if (!cardTex.has(key)) cardTex.set(key, cardTexture(m.text, m.rev));
        const g = new THREE.Group();
        g.position.copy(V(m.x + lx * CARD_LEFT_MM, m.y + ly * CARD_LEFT_MM, 0.0011));
        g.rotation.y = h;
        const p = new THREE.Mesh(new THREE.PlaneGeometry(len * MM, CARD_WIDE_MM * MM),
          new THREE.MeshStandardMaterial({ map: cardTex.get(key), roughness: .9 }));
        p.rotation.x = -Math.PI / 2; p.receiveShadow = true; g.add(p);
        world.add(g);
        const s = label(`*${m.text}*`, "#0b0b0b", "rgba(255,255,255,.9)");
        s.position.copy(V(m.x + lx * (CARD_LEFT_MM + 50), m.y + ly * (CARD_LEFT_MM + 50), 0.04)); world.add(s);
      } else if (m.kind === "hump") {
        const peak = parseFloat((m.text.match(/(\d+) mm/) || [])[1]) || 10;
        const d = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 12, 0, Math.PI * 2, 0, Math.PI / 2),
          new THREE.MeshStandardMaterial({ color: 0xb8b8b0, roughness: .9 }));
        d.scale.set(0.045, peak * MM, 0.035);
        d.position.copy(V(m.x - lx * WHEEL_TRACK_MM / 2, m.y - ly * WHEEL_TRACK_MM / 2, 0));
        d.rotation.y = h; d.castShadow = true; d.receiveShadow = true; world.add(d);
        const s = label(m.text, "#0b0b0b", "rgba(255,255,255,.9)");
        s.position.copy(V(m.x - lx * 150, m.y - ly * 150, 0.04)); world.add(s);
      } else {
        const s = label("stop", "#ffffff", "rgba(208,59,59,.92)"); s.position.copy(V(m.x, m.y, 0.17)); world.add(s);
      }
    });

    /* obstacles: measured face and width, assumed depth and height */
    obst.forEach((o) => {
      const tf = frameOf(o), w = o.left + o.right;
      const [cx2, cy2] = tf(o.face + o.depth / 2, (o.left - o.right) / 2);
      const g = new THREE.Group();
      g.position.copy(V(cx2, cy2, OBST_H_MM * MM / 2)); g.rotation.y = rad(o.h0);
      const geo = new THREE.BoxGeometry(o.depth * MM, OBST_H_MM * MM, w * MM);
      const body = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0xe09b00, transparent: true, opacity: .35, roughness: .7 }));
      body.castShadow = true; g.add(body);
      g.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: 0x8a5a00 })));
      world.add(g);
      const s = label(`${w}${o.left_open || o.right_open ? "+" : ""} mm wide · depth, height assumed`, "#0b0b0b", "rgba(255,255,255,.9)");
      s.position.copy(V(...tf(o.face, (o.left - o.right) / 2), OBST_H_MM * MM + 0.03)); world.add(s);
      /* sonar echoes, and the planned way round */
      const hits = [];
      (o.beams || []).forEach(([back, deg, mm]) => {
        if (!(mm > 0)) return;
        const a = rad(deg);
        hits.push(V(...tf((SONAR_MM + mm) * Math.cos(a) - back, (SONAR_MM + mm) * Math.sin(a)), SONAR_H));
      });
      if (hits.length) world.add(new THREE.Points(new THREE.BufferGeometry().setFromPoints(hits),
        new THREE.PointsMaterial({ color: 0xec835a, size: 0.012 })));
      if (o.go !== "none") {
        const y = (o.go === "R" ? -1 : 1) * o.out, bk = -(o.back || 0);
        const plan = [[bk, 0], [bk, y], [bk + o.along, y], [bk + o.along, 0]].map(([a, c]) => V(...tf(a, c), 0.003));
        const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(plan), new THREE.LineDashedMaterial({ color: 0x0ca30c, dashSize: 0.03, gapSize: 0.015 }));
        l.computeLineDistances(); world.add(l);
      }
    });

    /* the car at its last pose */
    const last = path[path.length - 1];
    car.visible = !!last;
    if (last) { car.position.copy(V(last[0], last[1], 0)); car.rotation.y = rad(last[2]); }
    /* First sight of a run, live or ended: centred on the car, following it
     * (19 Nov: the car in the middle, not START). */
    if (firstFit && last) {
      firstFit = false;
      api.follow = true;
      if (api.onFollow) api.onFollow(true);
      chase(true);
    } else if (api.follow) chase(false);
  }

  /* Follow: keep the camera's offset from the car, move with it. */
  function chase(reset) {
    if (!car.visible) { fitAll(); return; }
    const target = car.position.clone().setY(0.05);
    if (reset) {
      const back = new THREE.Vector3(-0.55, 0, 0).applyAxisAngle(new THREE.Vector3(0, 1, 0), car.rotation.y);
      camera.position.copy(target).add(back).add(new THREE.Vector3(0, 0.42, 0)).add(new THREE.Vector3(0, 0, 0.18).applyAxisAngle(new THREE.Vector3(0, 1, 0), car.rotation.y));
    } else {
      camera.position.add(target.clone().sub(controls.target));
    }
    controls.target.copy(target);
  }
  /* The whole run in view, still centred on the car (on the run's middle
   * only before there is a car): far enough out that the run's furthest
   * corner from the car fits, looking from behind the car. */
  function fitAll() {
    if (!bounds) return;
    let c, span;
    if (car.visible) {
      c = car.position.clone();
      const far = Math.max(...[[bounds.min.x, bounds.min.y], [bounds.min.x, bounds.max.y],
        [bounds.max.x, bounds.min.y], [bounds.max.x, bounds.max.y]].map(([x, y]) => V(x, y, 0).distanceTo(c)));
      span = 2 * far;
    } else {
      c = V((bounds.min.x + bounds.max.x) / 2, (bounds.min.y + bounds.max.y) / 2, 0);
      span = Math.max(bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y) * MM;
    }
    controls.target.copy(c);
    const back = new THREE.Vector3(-span * 0.45, 0, span * 0.2).applyAxisAngle(new THREE.Vector3(0, 1, 0), car.visible ? car.rotation.y : 0);
    camera.position.copy(c).add(back).add(new THREE.Vector3(0, span * 0.8, 0));
  }

  function resize() {
    const w = host.clientWidth || 600, h = host.clientHeight || 500;
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = w + "px"; renderer.domElement.style.height = h + "px";
    camera.aspect = w / h; camera.updateProjectionMatrix();
    if (api.redraw) api.redraw();
  }
  new ResizeObserver(() => { if (api.visible) resize(); }).observe(host);

  /* Draw only when something changed: the view moved (or is still easing
   * to a stop), new data, a resize or a theme change. */
  let looping = false, dirty = true;
  controls.addEventListener("change", () => { dirty = true; });
  api.redraw = () => { dirty = true; };
  function loop() {
    if (looping) return;
    looping = true;
    const tick = () => {
      if (!api.visible) { looping = false; return; }
      const moved = controls.update();
      if (moved || dirty) {
        dirty = false;
        const t = controls.target;
        sun.position.set(t.x + 0.5, 1.2, t.z + 0.35); sun.target.position.copy(t);
        renderer.render(scene, camera);
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }
  return api;
}
