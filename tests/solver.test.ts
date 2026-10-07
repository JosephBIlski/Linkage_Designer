import { describe, expect, it } from 'vitest';
import { choleskySolve, matrixRank, nullSpace, pivotedQR } from '../src/core/solver';
import { symmetricEigen3 } from '../src/core/synthesis';

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
  it('eigen-decomposes symmetric 3x3', () => {
    const A = new Float64Array([2, 0, 0, 0, 3, 0, 0, 0, 1]);
    const e = symmetricEigen3(A);
    expect(e.values[0]).toBeCloseTo(3);
    expect(e.values[2]).toBeCloseTo(1);
    expect(Math.abs(e.vectors[0][1])).toBeCloseTo(1);
  });
});
