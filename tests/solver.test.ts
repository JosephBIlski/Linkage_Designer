import { describe, expect, it } from 'vitest';
import { DAMPING_FLOOR, choleskySolve, matrixRank, nullSpace, pivotedQR, solve } from '../src/core/solver';
import { symmetricEigen3 } from '../src/core/synthesis';
import { constRef, distance, type PRef } from '../src/core/constraints';

describe('dense linear algebra', () => {
  it('solves SPD systems with Cholesky', () => {
    const A = new Float64Array([4, 1, 0, 1, 3, 1, 0, 1, 2]);
    const b = new Float64Array([1, 2, 3]);
    const x = choleskySolve(A, b, 3)!;
    expect(x).not.toBeNull();
    const r = [4 * x[0] + x[1] - 1, x[0] + 3 * x[1] + x[2] - 2, x[1] + 2 * x[2] - 3];
    for (const v of r) expect(Math.abs(v)).toBeLessThan(1e-12);
  });
  it('computes rank and null space', () => {
    // rows: [1,0,0],[0,1,0],[1,1,0] -> rank 2, null space = z
    const J = new Float64Array([1, 0, 0, 0, 1, 0, 1, 1, 0]);
    expect(matrixRank(J, 3, 3)).toBe(2);
    const { N, k } = nullSpace(J, 3, 3);
    expect(k).toBe(1);
    expect(Math.abs(Math.abs(N[2]) - 1)).toBeLessThan(1e-12);
    const qr = pivotedQR(J, 3, 3);
    expect(qr.rank).toBe(2);
  });
  it('projects onto a nearly vertical distance in a few steps (damping floored at a fraction of the largest diagonal entry)', () => {
    // one distance between two free points whose edge is almost parallel to y: the x columns of JᵀJ are ~1e-9, and
    // undamped they would take a step of order 1e-2 for a residual of 1e-5 (rejected, then stalled)
    const p: PRef = { cols: [0, 1, -1], c: [0, 0, 0] };
    const q: PRef = { cols: [2, 3, -1], c: [0, 0, 0] };
    const sys = { n: 4, constraints: [distance(p, q, 1)] };
    const x0 = new Float64Array([2 + 3.3e-5, 0, 2, 1 + 1e-5]);
    const res = solve(sys, x0, { maxIter: 10, tol: 1e-8 });
    expect(res.converged).toBe(true);
    expect(res.iterations).toBeLessThanOrEqual(3);
    expect(Math.abs(res.x[0] - x0[0])).toBeLessThan(1e-6); // the minimum-norm step hardly moves x
    expect(DAMPING_FLOOR).toBe(1e-3);
    // a system with no variables is converged only when its (constant) residual is small
    const stretched = { n: 0, constraints: [distance(constRef([0, 0, 0]), constRef([1, 0, 0]), 1.5)] };
    const r0 = solve(stretched, new Float64Array(0), { tol: 1e-8 });
    expect(r0.converged).toBe(false);
    expect(r0.hardResidual).toBeCloseTo(0.5, 12);
    expect(solve({ n: 0, constraints: [distance(constRef([0, 0, 0]), constRef([1, 0, 0]), 1)] }, new Float64Array(0)).converged).toBe(true);
  });

  it('eigen-decomposes symmetric 3x3', () => {
    const A = new Float64Array([2, 0, 0, 0, 3, 0, 0, 0, 1]);
    const e = symmetricEigen3(A);
    expect(e.values[0]).toBeCloseTo(3);
    expect(e.values[2]).toBeCloseTo(1);
    expect(Math.abs(e.vectors[0][1])).toBeCloseTo(1);
  });
});
