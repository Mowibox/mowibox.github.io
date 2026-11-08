// ur5IK of assets/js/ur5.js against forward kinematics from ur5_robot.urdf.
//   node bin/check-ur5-ik.mjs
import { readFileSync } from 'node:fs';
const src = readFileSync(new URL('../assets/js/ur5.js', import.meta.url), 'utf8');
const pick = re => src.match(re)[0];
const { ur5IK, ur5Aim } = new Function(`const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
${pick(/const UR5_REST[\s\S]*?const norm3.*\n/)}${pick(/function ur5IK[\s\S]*?\n}\n/)}${pick(/function ur5Aim[\s\S]*?\n}\n/)}return { ur5IK, ur5Aim };`)();

const mul = (A, B) => A.map(r => [0, 1, 2, 3].map(j => r.reduce((s, v, i) => s + v * B[i][j], 0)));
const trans = ([x, y, z]) => [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]];
const rotY = t => [[Math.cos(t), 0, Math.sin(t), 0], [0, 1, 0, 0], [-Math.sin(t), 0, Math.cos(t), 0], [0, 0, 0, 1]];
const rotZ = t => [[Math.cos(t), -Math.sin(t), 0, 0], [Math.sin(t), Math.cos(t), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
const CHAIN = [[[0, 0, 0.089159], 0, rotZ], [[0, 0.13585, 0], Math.PI / 2, rotY], [[0, -0.1197, 0.425], 0, rotY], [[0, 0, 0.39225], Math.PI / 2, rotY], [[0, 0.093, 0], 0, rotZ], [[0, 0, 0.09465], 0, rotY]];
const fk = q => CHAIN.reduce((T, [xyz, pitch, rot], i) => mul(mul(mul(T, trans(xyz)), rotY(pitch)), rot(q[i])), trans([0, 0, 0]));
const near = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-9);
const col = (T, j) => T.slice(0, 3).map(r => r[j]);
const rnd = () => Math.random() * 2 - 1;

// Reachable poses, from random joint angles of the elbow-up branch's range.
let solved = 0;
for (let i = 0; i < 20000; i++) {
  const T = fk([rnd() * 3, -1.5 + rnd(), 1.2 + rnd(), rnd() * 3, 1.57 + rnd() * 1.2, rnd() * 3]);
  const tip = col(T, 3).map((v, n) => v + 0.0823 * col(T, 1)[n]);
  const q = ur5IK(tip, col(T, 1), col(T, 0));
  if (!q) continue;
  solved++;
  const R = fk(q);
  const got = col(R, 3).map((v, n) => v + 0.0823 * col(R, 1)[n]);
  if (!near(got, tip) || !near(col(R, 1), col(T, 1)) || !near(col(R, 0), col(T, 0))) throw Error(`bad pose at ${q}`);
}
if (solved < 2000) throw Error(`only ${solved} solved`);

// Over the pointer's plane: nearly always solved, and aimed at the pointer.
const toward = [Math.SQRT1_2, -Math.SQRT1_2], side = [Math.SQRT1_2, Math.SQRT1_2];
let n = 0, ok = 0;
for (let s = -0.7; s <= 0.7; s += 0.05)
  for (let z = 0; z <= 0.9; z += 0.05) {
    const P = [toward[0] * 0.4 + side[0] * s, toward[1] * 0.4 + side[1] * s, z];
    const q = ur5Aim(P, [-toward[0] * 0.4, -toward[1] * 0.4, 0.089159]);
    n++;
    if (!q) continue;
    ok++;
    const R = fk(q), tip = col(R, 3).map((v, i) => v + 0.0823 * col(R, 1)[i]);
    const to = P.map((v, i) => v - tip[i]), l = Math.hypot(...to);
    if (Math.abs(to[0] * R[0][1] + to[1] * R[1][1] + to[2] * R[2][1] - l) > 1e-6) throw Error(`not aimed at ${P}`);
  }
if (ok / n < 0.9) throw Error(`pointer plane covered at ${ok}/${n}`);
console.log(`ok: ${solved} poses, pointer plane ${ok}/${n}`);
