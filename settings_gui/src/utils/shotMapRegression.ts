export interface XYPoint {
  x: number;
  y: number;
}

export interface ChartPoint {
  distance: number;
  [key: string]: number | null;
}

interface SplineSegment {
  x0: number;
  x1: number;
  a: number;
  b: number;
  c: number;
  d: number;
}

function collapseDuplicateX(points: XYPoint[]): XYPoint[] {
  const buckets = new Map<number, { sum: number; count: number }>();

  for (const point of points) {
    const bucket = buckets.get(point.x);
    if (bucket) {
      bucket.sum += point.y;
      bucket.count += 1;
    } else {
      buckets.set(point.x, { sum: point.y, count: 1 });
    }
  }

  return [...buckets.entries()]
    .map(([x, bucket]) => ({ x, y: bucket.sum / bucket.count }))
    .sort((a, b) => a.x - b.x);
}

function solveLinearSystem(matrix: number[][], values: number[]): number[] | null {
  const size = values.length;
  const augmented = matrix.map((row, index) => [...row, values[index]]);

  for (let pivot = 0; pivot < size; pivot += 1) {
    let maxRow = pivot;
    for (let row = pivot + 1; row < size; row += 1) {
      if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[maxRow][pivot])) {
        maxRow = row;
      }
    }

    if (Math.abs(augmented[maxRow][pivot]) < 1e-10) {
      return null;
    }

    if (maxRow !== pivot) {
      [augmented[pivot], augmented[maxRow]] = [augmented[maxRow], augmented[pivot]];
    }

    const divisor = augmented[pivot][pivot];
    for (let column = pivot; column <= size; column += 1) {
      augmented[pivot][column] /= divisor;
    }

    for (let row = 0; row < size; row += 1) {
      if (row === pivot) {
        continue;
      }

      const factor = augmented[row][pivot];
      for (let column = pivot; column <= size; column += 1) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }

  return augmented.map((row) => row[size]);
}

function fitPolynomial(points: XYPoint[], degree: number): number[] | null {
  const deduped = collapseDuplicateX(points);

  if (deduped.length < degree + 1) {
    return null;
  }

  const matrixSize = degree + 1;
  const sums = Array.from({ length: degree * 2 + 1 }, () => 0);
  const rhs = Array.from({ length: matrixSize }, () => 0);

  for (const point of deduped) {
    const powers = Array.from({ length: degree * 2 + 1 }, (_, index) => point.x ** index);
    for (let power = 0; power < powers.length; power += 1) {
      sums[power] += powers[power];
    }
    for (let row = 0; row < matrixSize; row += 1) {
      rhs[row] += point.y * powers[row];
    }
  }

  const matrix = Array.from({ length: matrixSize }, (_, row) =>
    Array.from({ length: matrixSize }, (_, column) => sums[row + column]),
  );

  return solveLinearSystem(matrix, rhs);
}

function evaluatePolynomial(coefficients: number[], x: number): number {
  return coefficients.reduce((sum, coefficient, index) => sum + coefficient * x ** index, 0);
}

function buildPolynomialSeries(points: XYPoint[], degree: number, sampleCount = 100): XYPoint[] {
  const coefficients = fitPolynomial(points, degree);
  const deduped = collapseDuplicateX(points);

  if (!coefficients || deduped.length === 0) {
    return [];
  }

  const minX = deduped[0].x;
  const maxX = deduped[deduped.length - 1].x;
  if (Math.abs(maxX - minX) < 1e-10) {
    return [{ x: minX, y: evaluatePolynomial(coefficients, minX) }];
  }

  return Array.from({ length: sampleCount }, (_, index) => {
    const x = minX + ((maxX - minX) * index) / (sampleCount - 1);
    return { x, y: evaluatePolynomial(coefficients, x) };
  });
}

function fitNaturalCubicSpline(points: XYPoint[]): SplineSegment[] | null {
  const deduped = collapseDuplicateX(points);

  if (deduped.length < 2) {
    return null;
  }

  if (deduped.length === 2) {
    const [first, second] = deduped;
    const slope = (second.y - first.y) / (second.x - first.x);
    return [{
      x0: first.x,
      x1: second.x,
      a: first.y,
      b: slope,
      c: 0,
      d: 0,
    }];
  }

  const pointCount = deduped.length;
  const h = Array.from({ length: pointCount - 1 }, (_, index) => deduped[index + 1].x - deduped[index].x);
  if (h.some((value) => value <= 0)) {
    return null;
  }

  const alpha = Array.from({ length: pointCount }, () => 0);
  for (let index = 1; index < pointCount - 1; index += 1) {
    alpha[index] =
      (3 / h[index]) * (deduped[index + 1].y - deduped[index].y) -
      (3 / h[index - 1]) * (deduped[index].y - deduped[index - 1].y);
  }

  const l = Array.from({ length: pointCount }, () => 0);
  const mu = Array.from({ length: pointCount }, () => 0);
  const z = Array.from({ length: pointCount }, () => 0);
  const c = Array.from({ length: pointCount }, () => 0);
  const b = Array.from({ length: pointCount - 1 }, () => 0);
  const d = Array.from({ length: pointCount - 1 }, () => 0);

  l[0] = 1;
  for (let index = 1; index < pointCount - 1; index += 1) {
    l[index] = 2 * (deduped[index + 1].x - deduped[index - 1].x) - h[index - 1] * mu[index - 1];
    if (Math.abs(l[index]) < 1e-10) {
      return null;
    }
    mu[index] = h[index] / l[index];
    z[index] = (alpha[index] - h[index - 1] * z[index - 1]) / l[index];
  }
  l[pointCount - 1] = 1;

  const segments: SplineSegment[] = [];
  for (let index = pointCount - 2; index >= 0; index -= 1) {
    c[index] = z[index] - mu[index] * c[index + 1];
    b[index] =
      (deduped[index + 1].y - deduped[index].y) / h[index] -
      (h[index] * (c[index + 1] + 2 * c[index])) / 3;
    d[index] = (c[index + 1] - c[index]) / (3 * h[index]);

    segments.unshift({
      x0: deduped[index].x,
      x1: deduped[index + 1].x,
      a: deduped[index].y,
      b: b[index],
      c: c[index],
      d: d[index],
    });
  }

  return segments;
}

function evaluateSpline(segment: SplineSegment, x: number): number {
  const delta = x - segment.x0;
  return segment.a + segment.b * delta + segment.c * delta ** 2 + segment.d * delta ** 3;
}

function buildSplineSeries(points: XYPoint[], samplesPerSegment = 16): XYPoint[] {
  const segments = fitNaturalCubicSpline(points);
  if (!segments) {
    return [];
  }

  const sampled: XYPoint[] = [];
  for (const segment of segments) {
    for (let index = 0; index < samplesPerSegment; index += 1) {
      const x = segment.x0 + ((segment.x1 - segment.x0) * index) / samplesPerSegment;
      sampled.push({ x, y: evaluateSpline(segment, x) });
    }
  }

  const lastSegment = segments[segments.length - 1];
  sampled.push({ x: lastSegment.x1, y: evaluateSpline(lastSegment, lastSegment.x1) });
  return sampled;
}

export function buildChartData(
  actualPoints: Record<string, XYPoint[]>,
  fitPoints: Record<string, XYPoint[]>,
): ChartPoint[] {
  const merged = new Map<string, ChartPoint>();

  const insertPoint = (distance: number, key: string, value: number) => {
    const mapKey = distance.toFixed(6);
    const existing = merged.get(mapKey) ?? { distance };
    existing[key] = value;
    merged.set(mapKey, existing);
  };

  for (const [key, points] of Object.entries(actualPoints)) {
    for (const point of points) {
      insertPoint(point.x, key, point.y);
    }
  }

  for (const [key, points] of Object.entries(fitPoints)) {
    for (const point of points) {
      insertPoint(point.x, key, point.y);
    }
  }

  return [...merged.values()].sort((a, b) => a.distance - b.distance);
}

export function buildQuadraticSeries(points: XYPoint[]): XYPoint[] {
  return buildPolynomialSeries(points, 2);
}

export function buildCubicSeries(points: XYPoint[]): XYPoint[] {
  return buildPolynomialSeries(points, 3);
}

export function buildSplineInterpolationSeries(points: XYPoint[]): XYPoint[] {
  return buildSplineSeries(points);
}
