// The UR5 of the home page in 3D, on <div data-ur5>: its tool aims at the pointer.
// Model: assets/models/ur5.glb (bin/export-ur5-glb.py). The viewer and its cartoon
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const MODELS = new URL('../models/', import.meta.url);
const still = matchMedia('(prefers-reduced-motion: reduce)').matches;
const dark = () => document.documentElement.dataset.theme === 'dark'; // al-folio's attribute (chroma: bsTheme)
const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const _v = new THREE.Vector3(), _w = new THREE.Vector3();

// Normals creased at 30° (the case's edges sharp, its curves smooth) from the
// welded mesh the GLB holds (it has no normals: they would quadruple it). Each
// corner gets the sum of its vertex's faces within 30° of its own; the corners
// of a vertex with the same normal share one. Run in a worker (a few hundred
// ms for the robot): the model is drawn flat meanwhile.
function crease(P, idx, cos = Math.cos(Math.PI / 6)) {
  const nv = P.length / 3, nf = idx.length / 3, nc = idx.length;
  // Each face's normal, area-weighted (F) and unit (U).
  const F = new Float32Array(nf * 3), U = new Float32Array(nf * 3);
  for (let f = 0; f < nf; f++) {
    const a = 3 * idx[3 * f], b = 3 * idx[3 * f + 1], c = 3 * idx[3 * f + 2];
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    const x = uy * vz - uz * vy, y = uz * vx - ux * vz, z = ux * vy - uy * vx, l = Math.hypot(x, y, z) || 1;
    (F[3 * f] = x), (F[3 * f + 1] = y), (F[3 * f + 2] = z);
    (U[3 * f] = x / l), (U[3 * f + 1] = y / l), (U[3 * f + 2] = z / l);
  }
  // The corners around each vertex.
  const start = new Uint32Array(nv + 1);
  for (let i = 0; i < nc; i++) start[idx[i] + 1]++;
  for (let v = 0; v < nv; v++) start[v + 1] += start[v];
  const fill = start.slice(0, nv), around = new Uint32Array(nc);
  for (let i = 0; i < nc; i++) around[fill[idx[i]]++] = i;
  // The new vertices; those of an old vertex chained from head[v] through next.
  const outP = new Float32Array(nc * 3), outN = new Float32Array(nc * 3), outI = new Uint32Array(nc);
  const head = new Int32Array(nv).fill(-1), next = new Int32Array(nc);
  let n = 0;
  for (let v = 0; v < nv; v++)
    for (let k = start[v]; k < start[v + 1]; k++) {
      const i = around[k], f = 3 * ((i / 3) | 0);
      // A corner of this vertex on a parallel face (a flat fan's): its vertex.
      let q = start[v];
      for (; q < k; q++) {
        const h = 3 * ((around[q] / 3) | 0);
        if (U[f] * U[h] + U[f + 1] * U[h + 1] + U[f + 2] * U[h + 2] > 0.9999) break;
      }
      if (q < k) {
        outI[i] = outI[around[q]];
        continue;
      }
      let x = 0, y = 0, z = 0;
      for (let m = start[v]; m < start[v + 1]; m++) {
        const h = 3 * ((around[m] / 3) | 0);
        if (U[f] * U[h] + U[f + 1] * U[h + 1] + U[f + 2] * U[h + 2] >= cos) (x += F[h]), (y += F[h + 1]), (z += F[h + 2]);
      }
      const l = Math.hypot(x, y, z) || 1;
      (x /= l), (y /= l), (z /= l);
      let j = head[v];
      while (j >= 0 && outN[3 * j] * x + outN[3 * j + 1] * y + outN[3 * j + 2] * z <= 0.9999) j = next[j];
      if (j < 0) {
        j = n++;
        (next[j] = head[v]), (head[v] = j);
        (outP[3 * j] = P[3 * v]), (outP[3 * j + 1] = P[3 * v + 1]), (outP[3 * j + 2] = P[3 * v + 2]);
        (outN[3 * j] = x), (outN[3 * j + 1] = y), (outN[3 * j + 2] = z);
      }
      outI[i] = j;
    }
  return { P: outP.slice(0, 3 * n), N: outN.slice(0, 3 * n), I: outI };
}

const worker = new Worker(URL.createObjectURL(new Blob([`${crease}\nonmessage = ({ data: [id, P, I] }) => { const r = crease(P, I); postMessage([id, r], [r.P.buffer, r.N.buffer, r.I.buffer]); };`], { type: 'text/javascript' })));
const pending = new Map();
worker.onmessage = ({ data: [id, r] }) => (pending.get(id)(r), pending.delete(id));
function creased(g) {
  const pos = g.attributes.position, P = new Float32Array(pos.count * 3);
  for (let i = 0; i < pos.count; i++) (P[3 * i] = pos.getX(i)), (P[3 * i + 1] = pos.getY(i)), (P[3 * i + 2] = pos.getZ(i));
  const id = pending.size + Math.random();
  worker.postMessage([id, P, Uint32Array.from(g.index.array)], [P.buffer]);
  return new Promise(resolve => pending.set(id, resolve)).then(({ P, N, I }) => {
    const r = new THREE.BufferGeometry();
    r.setAttribute('position', new THREE.BufferAttribute(P, 3));
    r.setAttribute('normal', new THREE.BufferAttribute(N, 3));
    r.setIndex(new THREE.BufferAttribute(I, 1));
    return Object.assign(r, { welded: g });
  });
}

// A model: drawn at once (flat), smooth once its normals are made (the welded
// mesh kept for the cartoon outline).
const loads = {};
const load = file => (loads[file] ??= loader.loadAsync(new URL(file, MODELS).href).then(({ scene: model }) => {
  const users = new Map();
  model.traverse(o => {
    if (o.isMesh && !o.geometry.attributes.normal) users.set(o.geometry, [...(users.get(o.geometry) ?? []), o]);
  });
  // All at once: one change of look, not a flicker part by part.
  Promise.all([...users.keys()].map(creased)).then(gs => {
    const mats = new Set();
    [...users.values()].forEach((ms, i) => ms.forEach(o => ((o.geometry = gs[i]), mats.add(o.material))));
    for (const m of mats) (m.flatShading = false), (m.needsUpdate = true);
  });
  return model;
}));

// ---- stage: renderer, light, camera, a loop running only while on screen ----

function stage(el) {
  const canvas = document.createElement('canvas');
  el.append(canvas);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, stencil: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.toneMapping = THREE.NeutralToneMapping;
  const scene = new THREE.Scene();
  scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.6;
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  scene.add(key, key.target);
  const camera = new THREE.PerspectiveCamera(28, canvas.clientWidth / canvas.clientHeight || 1.6, 0.005, 20);
  const s = { el, canvas, renderer, scene, camera, key, tick: () => {}, visible: false };
  const resize = () => {
    const { clientWidth: w, clientHeight: h } = canvas;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(canvas);
  new IntersectionObserver(([e]) => (s.visible = e.isIntersecting)).observe(canvas);
  let last = 0;
  renderer.setAnimationLoop(t => {
    if (!s.visible) return (last = 0);
    const dt = last ? Math.min((t - last) / 1000, 0.1) : 0;
    last = t;
    s.tick(t / 1000, dt);
    renderer.render(scene, camera);
  });
  return s;
}

// The model on the floor (y = 0), centred; the camera looking at it from
// `dir`, as close as `box` (default: the model's) allows inside padX, padY of
// the frame (NDC); the light placed to it.
function frame(s, model, dir, { box = new THREE.Box3().setFromObject(model), padX = 0.9, padY = 0.85 } = {}) {
  const c = box.getCenter(new THREE.Vector3());
  model.position.x -= c.x;
  model.position.z -= c.z;
  model.position.y -= box.min.y;
  box.translate(_v.set(-c.x, -box.min.y, -c.z));
  const target = box.getCenter(new THREE.Vector3());
  const radius = box.getSize(_v).length() / 2;
  const corners = [0, 1, 2, 3, 4, 5, 6, 7].map(i => new THREE.Vector3(i & 1 ? box.max.x : box.min.x, i & 2 ? box.max.y : box.min.y, i & 4 ? box.max.z : box.min.z));
  dir = dir.clone().normalize();
  let d = radius * 3;
  // Perspective: the extent on screen goes about as 1 / distance.
  for (let i = 0; i < 6; i++) {
    s.camera.position.copy(target).addScaledVector(dir, d);
    s.camera.lookAt(target);
    s.camera.updateMatrixWorld();
    let r = 0;
    for (const p of corners) {
      _v.copy(p).project(s.camera);
      r = Math.max(r, Math.abs(_v.x) / padX, Math.abs(_v.y) / padY);
    }
    d *= r;
  }
  s.key.position.set(0.6, 1.6, 0.9).multiplyScalar(radius * 3);
  s.key.target.position.copy(target);
  return target;
}

// ---- the cartoon look: the silhouette ringed in a dark shade of each part's
// colour (a light one on the dark theme) ----

const LOOK = { width: { value: 7 }, light: { value: 0 } };
const STENCIL = { stencilWrite: true, stencilRef: 1, stencilZPass: THREE.ReplaceStencilOp };
const HULL_VS = `uniform float uWidth;
  void main() {
    vec4 p = modelViewMatrix * vec4(position, 1.0);
    p.xyz += normalize(normalMatrix * normal) * uWidth / 540.0 * -p.z / projectionMatrix[1][1];
    gl_Position = projectionMatrix * p;
  }`;
const HULL_FS = `uniform vec3 uDark; uniform vec3 uLite; uniform float uLight;
  void main() { gl_FragColor = vec4(mix(uDark, uLite, uLight), 0.88);
  #include <colorspace_fragment>
  }`;
const hullMats = new Map();
function hullMat(mat) {
  if (!hullMats.has(mat)) {
    const hsl = (mat.color ?? new THREE.Color(0.5, 0.5, 0.5)).getHSL({});
    hullMats.set(mat, new THREE.ShaderMaterial({
      uniforms: {
        uWidth: LOOK.width,
        uLight: LOOK.light,
        uDark: { value: new THREE.Color().setHSL(hsl.h, Math.min(1, hsl.s * 1.3), 0.06 + hsl.l * 0.12) },
        uLite: { value: new THREE.Color().setHSL(hsl.h, hsl.s * 0.5, 0.82) },
      },
      vertexShader: HULL_VS,
      fragmentShader: HULL_FS,
      side: THREE.BackSide,
      transparent: true,
      depthWrite: false,
      toneMapped: false,
      ...STENCIL,
      stencilFunc: THREE.NotEqualStencilFunc,
    }));
  }
  return hullMats.get(mat);
}
// The welded mesh with smoothed normals: the hull stays closed at the corners.
const smooth = new Map();
function smoothOf(g) {
  if (!smooth.has(g)) {
    const m = new THREE.BufferGeometry().setAttribute('position', g.attributes.position).setIndex(g.index);
    m.computeVertexNormals();
    smooth.set(g, m);
  }
  return smooth.get(g);
}
function toon(model) {
  const meshes = [];
  model.traverse(o => o.isMesh && meshes.push(o));
  for (const o of meshes) {
    // Stencil: the hull only shows outside the silhouette.
    Object.assign(o.material, STENCIL, { stencilFunc: THREE.AlwaysStencilFunc });
    const h = new THREE.Mesh(smoothOf(o.geometry.welded ?? o.geometry), hullMat(o.material));
    h.raycast = () => {};
    h.renderOrder = 1;
    o.add(h);
  }
}

// ---- ur5: the arm of the home page, its tool on the pointer (inverse kinematics) ----

const UR5_AXES = { shoulder_pan_joint: [0, 0, 1], shoulder_lift_joint: [0, 1, 0], elbow_joint: [0, 1, 0], wrist_1_joint: [0, 1, 0], wrist_2_joint: [0, 0, 1], wrist_3_joint: [0, 1, 0] };
// Calibration knobs: where the arm rests (URDF metres); how far toward the camera
// the plane the pointer moves on stands from the base; how far behind the base the
// tool's line of sight starts; how far short of the pointer the tool stops.
const UR5_REST = [0.35, -0.3, 0.35];
const UR5_PLANE = 0.4;
const UR5_BACK = 0.4;
const UR5_STANDOFF = 0.1;
// How far past the view (1 = its edge) the pointer is still followed, and how
// far the tool may be moved along its line to stay reachable (fraction).
const UR5_MARGIN = 1.4;
const UR5_SLACK = 0.5;
// How much of the view (NDC) the joints may reach: the links and tool need the rest.
const UR5_VIEW = 0.84;
const UR5_SHOULDER = 0.089159;
const norm3 = v => { const l = Math.hypot(...v) || 1; return v.map(c => c / l); };
// Closed-form IK, elbow up, wrist 2 in (0, pi): the 6 joint angles putting the tool
// tip at p with its axis along a and its x along u (unit, a perpendicular to u),
// all in the base frame (URDF); null outside the workspace.
function ur5IK(p, a, u) {
  const L1 = 0.425, L2 = 0.39225, d = 0.13585 - 0.1197 + 0.093, w3 = 0.09465, tool = 0.0823;
  // The wrist 3 joint; the arm's plane (x', z) is at d from it, sideways (y').
  const px = p[0] - tool * a[0], py = p[1] - tool * a[1], pz = p[2] - tool * a[2], rho = Math.hypot(px, py);
  if (p[2] < 0.02 || rho < d + 0.03) return null;
  const pan = Math.atan2(py, px) - Math.asin(d / rho), c = Math.cos(pan), s = Math.sin(pan);
  // The tool axis in that plane gives wrist 2 and the wrist's pitch A.
  const ax = a[0] * c + a[1] * s, ay = -a[0] * s + a[1] * c;
  const q5 = Math.acos(clamp(ay, -1, 1)), sq5 = Math.sin(q5);
  if (sq5 < 0.05) return null;
  const A = Math.atan2(a[2], -ax), sA = Math.sin(A), cA = Math.cos(A);
  // The wrist centre, from the shoulder: a 2-link arm.
  const wr = px * c + py * s - w3 * sA, wz = pz - w3 * cA - UR5_SHOULDER, D = Math.hypot(wr, wz);
  if (D > L1 + L2 - 0.01 || D < 0.2 || wr < 0.1) return null;
  const a1 = Math.atan2(wr, wz) - Math.acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D));
  const q3 = Math.acos((D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2));
  // The roll about the tool axis, from wrist 3's x and z at q6 = 0.
  const zw = [sA * c, sA * s, cA], xw = [cA * c, cA * s, -sA], yl = [-s, c, 0], cq5 = Math.cos(q5);
  const x2 = xw.map((v, i) => cq5 * v + sq5 * yl[i]);
  const dot = (m, n) => m[0] * n[0] + m[1] * n[1] + m[2] * n[2];
  return [pan, a1 - Math.PI / 2, q3, A - Math.PI / 2 - a1 - q3, q5, Math.atan2(-dot(u, zw), dot(u, x2))];
}
// The tool aimed at P along the line from `from`, stopping just short of it; moved
// along that line while out of reach or out of `fits`, still aimed at P. Its roll keeps x level.
function ur5Aim(P, from, fits = () => true) {
  const S = [0, 0, UR5_SHOULDER], line = norm3(P.map((v, i) => v - from[i]));
  const tip = P.map((v, i) => v - UR5_STANDOFF * line[i]);
  for (let i = 0; i <= UR5_SLACK * 50; i++)
    for (const k of [1 - 0.02 * i, 1 + 0.02 * i]) {
      const T = tip.map((v, n) => S[n] + (v - S[n]) * k);
      T[2] = Math.max(T[2], 0.02);
      const a = norm3(P.map((v, n) => v - T[n]));
      if (a[0] * line[0] + a[1] * line[1] + a[2] * line[2] < 0.3) continue;
      const q = ur5IK(T, a, norm3(Math.hypot(a[0], a[1]) > 1e-3 ? [-a[1], a[0], 0] : [1, 0, 0]));
      if (q && fits(q)) return q;
    }
  return null;
}

async function ur5(el) {
  const s = stage(el);
  const model = await load('ur5.glb');
  s.scene.add(model);
  s.scene.add(new THREE.HemisphereLight(0xffffff, 0x888888, 0.6));
  toon(model);
  const joints = Object.entries(UR5_AXES).map(([name, axis]) => ({ o: model.getObjectByName(name), ax: new THREE.Vector3(...axis) }));
  const rest = ur5IK(UR5_REST, [0, 0, -1], [1, 0, 0]), q = [...rest];
  const pose = (at = q) => joints.forEach((j, i) => j.o.quaternion.setFromAxisAngle(j.ax, at[i]));
  pose();
  // Framed on the part of the workspace it reaches, not on the resting arm.
  const box = new THREE.Box3(new THREE.Vector3(-0.3, 0, -0.3), new THREE.Vector3(0.3, 0.72, 0.3));
  frame(s, model, new THREE.Vector3(1, 0.45, 1), { box, padX: 0.95, padY: 0.98 });
  model.updateMatrixWorld(true);
  const root = model.getObjectByName('ur5');
  const ray = new THREE.Raycaster(), plane = new THREE.Plane(), ndc = new THREE.Vector2(), hit = new THREE.Vector3(), toward = new THREE.Vector3();
  let goal = rest;
  // Never cut by the view: every joint inside it, with room for the links around.
  const inView = c => {
    pose(c);
    model.updateMatrixWorld(true);
    return joints.slice(1).every(j => Math.max(...j.o.getWorldPosition(_v).project(s.camera).toArray().slice(0, 2).map(Math.abs)) <= UR5_VIEW);
  };
  // The pointer, cast on a vertical plane facing the camera, in front of the base.
  const follow = e => {
    const r = s.canvas.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    toward.copy(s.camera.position).setY(0).normalize();
    plane.setFromNormalAndCoplanarPoint(toward, _v.copy(toward).multiplyScalar(UR5_PLANE).add(model.position));
    ray.setFromCamera(ndc, s.camera);
    const on = Math.max(Math.abs(ndc.x), Math.abs(ndc.y)) <= UR5_MARGIN && ray.ray.intersectPlane(plane, hit);
    if (!on) return (goal = rest);
    const P = root.worldToLocal(hit).toArray();
    // The tool's line of sight starts behind the base, seen from the camera.
    _w.copy(toward).multiplyScalar(-UR5_BACK).add(model.position).setY(model.position.y + UR5_SHOULDER);
    goal = ur5Aim(P, root.worldToLocal(_w).toArray(), inView) || rest;
  };
  if (!still) addEventListener('pointermove', follow, { passive: true });
  s.tick = (t, dt) => {
    LOOK.light.value = dark() ? 1 : 0;
    // Each joint eased to its goal, the short way round.
    q.forEach((v, i) => (q[i] = v + Math.atan2(Math.sin(goal[i] - v), Math.cos(goal[i] - v)) * (1 - Math.exp(-dt * 8))));
    pose();
  };
}

for (const el of document.querySelectorAll('[data-ur5]')) ur5(el).catch(err => console.error('ur5', err));
