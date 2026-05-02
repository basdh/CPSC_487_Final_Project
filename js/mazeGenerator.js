/**
 * Grid mazes using the **recursive backtracker** (randomized depth-first search).
 *
 * This matches the standard description of “randomized depth-first search” /
 * “recursive backtracking” in maze literature: carve passages until every cell is
 * visited. The result is a **perfect maze** — a spanning tree of the grid graph —
 * so the maze is *simply connected*: there is exactly one path between any two
 * cells, and in particular the start cell can always reach every other cell
 * (including a chosen goal corner).
 *
 * Reference: https://en.wikipedia.org/wiki/Maze_generation_algorithm (section on
 * randomized depth-first search / recursive backtracker).
 *
 * Walls: n, e, s, w per cell (true = closed). Grid indices: x = 0..cols-1 (east),
 * z = 0..rows-1 (south); world Z increases northward.
 *
 * Do not build mazes by independently randomizing each wall — that can disconnect regions.
 * This module only carves passages by removing matched wall pairs between adjacent cells during DFS.
 */

/** Mulberry32 PRNG */
export function createRng(seed) {
  let t = seed >>> 0;
  return function rng() {
    t += 0x6d2b79f5;
    let r = Math.imul(t ^ (t >>> 15), 1 | t);
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * @param {{ n:boolean,e:boolean,s:boolean,w:boolean }[][]} grid
 * @param {number} cols
 * @param {number} rows
 */
function validateWallConsistency(grid, cols, rows) {
  for (let z = 0; z < rows; z++) {
    for (let x = 0; x < cols; x++) {
      const c = grid[z][x];
      if (x < cols - 1 && c.e !== grid[z][x + 1].w) return false;
      if (z < rows - 1 && c.s !== grid[z + 1][x].n) return false;
      if (z > 0 && c.n !== grid[z - 1][x].s) return false;
      if (x > 0 && c.w !== grid[z][x - 1].e) return false;
    }
  }
  return true;
}

/**
 * Perfect maze via recursive backtracker (randomized depth-first search).
 * Never randomly toggles walls; only removes paired walls between current cell and a chosen neighbor.
 *
 * @param {number} cols
 * @param {number} rows
 * @param {() => number} rng
 * @param {number} [startX=0]
 * @param {number} [startZ=0]
 */
export function generateMaze(cols, rows, rng, startX = 0, startZ = 0) {
  if (cols < 1 || rows < 1) throw new Error('generateMaze: cols and rows must be >= 1');
  if (startX < 0 || startX >= cols || startZ < 0 || startZ >= rows) {
    throw new Error('generateMaze: start cell out of bounds');
  }

  /** @type {{ n:boolean,e:boolean,s:boolean,w:boolean,visited?:boolean }[][]} */
  const grid = [];
  for (let z = 0; z < rows; z++) {
    grid[z] = [];
    for (let x = 0; x < cols; x++) {
      grid[z][x] = { n: true, e: true, s: true, w: true };
    }
  }

  const stack = [];
  grid[startZ][startX].visited = true;
  stack.push([startX, startZ]);

  function unvisitedNeighbors(x, z) {
    const out = [];
    if (z > 0 && !grid[z - 1][x].visited) out.push([x, z - 1, 'n']);
    if (x < cols - 1 && !grid[z][x + 1].visited) out.push([x + 1, z, 'e']);
    if (z < rows - 1 && !grid[z + 1][x].visited) out.push([x, z + 1, 's']);
    if (x > 0 && !grid[z][x - 1].visited) out.push([x - 1, z, 'w']);
    return out;
  }

  while (stack.length > 0) {
    const [x, z] = stack[stack.length - 1];
    const nbs = unvisitedNeighbors(x, z);
    if (nbs.length === 0) {
      stack.pop();
      continue;
    }
    const pick = nbs[Math.floor(rng() * nbs.length)];
    const [nx, nz, dir] = pick;

    if (dir === 'n') {
      grid[z][x].n = false;
      grid[nz][nx].s = false;
    } else if (dir === 'e') {
      grid[z][x].e = false;
      grid[nz][nx].w = false;
    } else if (dir === 's') {
      grid[z][x].s = false;
      grid[nz][nx].n = false;
    } else {
      grid[z][x].w = false;
      grid[nz][nx].e = false;
    }

    grid[nz][nx].visited = true;
    stack.push([nx, nz]);
  }

  for (let z = 0; z < rows; z++) {
    for (let x = 0; x < cols; x++) {
      delete grid[z][x].visited;
    }
  }

  return { grid, cols, rows };
}

/**
 * Randomly opens additional **interior** edges (already represented twice in the grid).
 * Turns a perfect maze into an imperfect one with many loops — still fully connected,
 * but far fewer blocking walls (lower perceived density).
 *
 * @param {number} openClosedEdgeChance — per closed interior edge, probability to carve open (0..1)
 */
function thinInteriorWalls(grid, cols, rows, rng, openClosedEdgeChance) {
  for (let z = 0; z < rows; z++) {
    for (let x = 0; x < cols - 1; x++) {
      if (grid[z][x].e && rng() < openClosedEdgeChance) {
        grid[z][x].e = false;
        grid[z][x + 1].w = false;
      }
    }
  }
  for (let z = 0; z < rows - 1; z++) {
    for (let x = 0; x < cols; x++) {
      if (grid[z][x].s && rng() < openClosedEdgeChance) {
        grid[z][x].s = false;
        grid[z + 1][x].n = false;
      }
    }
  }
}

/**
 * BFS through open passages only — verifies goal reachable from start.
 */
export function hasPathThroughMaze(maze, sx, sz, gx, gz) {
  const { grid, cols, rows } = maze;
  const key = (x, z) => `${x},${z}`;
  const q = [[sx, sz]];
  const seen = new Set([key(sx, sz)]);
  while (q.length) {
    const [x, z] = q.shift();
    if (x === gx && z === gz) return true;
    const c = grid[z][x];
    if (z > 0 && !c.n && !seen.has(key(x, z - 1))) {
      seen.add(key(x, z - 1));
      q.push([x, z - 1]);
    }
    if (z < rows - 1 && !c.s && !seen.has(key(x, z + 1))) {
      seen.add(key(x, z + 1));
      q.push([x, z + 1]);
    }
    if (x < cols - 1 && !c.e && !seen.has(key(x + 1, z))) {
      seen.add(key(x + 1, z));
      q.push([x + 1, z]);
    }
    if (x > 0 && !c.w && !seen.has(key(x - 1, z))) {
      seen.add(key(x - 1, z));
      q.push([x - 1, z]);
    }
  }
  return false;
}

/**
 * One shortest path through open passages (same moves as BFS), or null if unreachable.
 * @returns {number[][] | null} [[x,z], ...] from start to goal inclusive.
 */
export function bfsShortestPathCells(maze, sx, sz, gx, gz) {
  const { grid, cols, rows } = maze;
  const key = (x, z) => `${x},${z}`;
  const prev = new Map();
  const q = [[sx, sz]];
  prev.set(key(sx, sz), null);
  while (q.length) {
    const [x, z] = q.shift();
    if (x === gx && z === gz) {
      /** @type {number[][]} */
      const path = [];
      let k = key(x, z);
      while (k !== null) {
        const [cx, cz] = k.split(',').map(Number);
        path.push([cx, cz]);
        k = prev.get(k);
      }
      path.reverse();
      return path;
    }
    const c = grid[z][x];
    const tryPush = (nx, nz, can) => {
      if (!can) return;
      const nk = key(nx, nz);
      if (prev.has(nk)) return;
      prev.set(nk, key(x, z));
      q.push([nx, nz]);
    };
    tryPush(x, z - 1, z > 0 && !c.n);
    tryPush(x, z + 1, z < rows - 1 && !c.s);
    tryPush(x + 1, z, x < cols - 1 && !c.e);
    tryPush(x - 1, z, x > 0 && !c.w);
  }
  return null;
}

/**
 * DFS maze, then wall consistency + BFS start→goal. Regenerates on failure (defensive).
 *
 * @param {number} cols
 * @param {number} rows
 * @param {number} seed - passed to {@link createRng}
 * @param {number} [goalX]
 * @param {number} [goalZ]
 * @param {number} [wallThinChance] — extra openings on interior edges (default strong thinning)
 */
export function generateVerifiedMaze(cols, rows, seed, goalX, goalZ, wallThinChance = 0.72) {
  const gx = goalX !== undefined ? goalX : cols - 1;
  const gz = goalZ !== undefined ? goalZ : rows - 1;
  const maxAttempts = 64;
  for (let attempt = 0; attempt < maxAttempts; attempt++) {
    const s = (seed + Math.imul(attempt, 0x9e3779b9)) >>> 0;
    const maze = generateMaze(cols, rows, createRng(s), 0, 0);
    const thinRng = createRng((s ^ 0xdeadbeef) >>> 0);
    thinInteriorWalls(maze.grid, cols, rows, thinRng, wallThinChance);
    const consistent = validateWallConsistency(maze.grid, cols, rows);
    const reachable = hasPathThroughMaze(maze, 0, 0, gx, gz);
    if (consistent && reachable) {
      console.assert(
        hasPathThroughMaze(maze, 0, 0, gx, gz) === true,
        'hasPathThroughMaze(maze, 0, 0, cols - 1, rows - 1) === true before use'
      );
      return maze;
    }
    console.error('[mazeGenerator] Maze failed verification; regenerating', {
      attempt: attempt + 1,
      cols,
      rows,
      consistent,
      reachable,
    });
  }
  throw new Error(`generateVerifiedMaze: failed after ${maxAttempts} attempts`);
}

/**
 * Axis-aligned wall segments in world XZ for collision — derived **only** from cell wall flags.
 *
 * Grid/world Z increases with row index z. Logical **north** is toward z − 1 (smaller world Z),
 * so the north wall segment sits on the **low-Z** face of the cell (`cz - hs`). Logical **south**
 * is toward z + 1 (**high-Z** face `cz + hs`).
 *
 * Unique coverage (no doubled segments):
 * - **North** (`c.n`): edge to (x, z − 1) — draw at low-Z for every cell (includes outer south rim).
 * - **East** (`c.e`): edge to (x + 1, z) — draw at high-X for every cell (includes outer east rim).
 * - **South** (`c.s`): only for z === rows − 1 — outer north rim (interior horizontal edges use
 *   `grid[z+1][x].n`, i.e. north face of the cell below).
 * - **West** (`c.w`): only for x === 0 — outer west rim (interior vertical edges use east of x − 1).
 *
 * Do **not** add full-width perimeter slabs; they collide with playable space even when the graph is connected.
 */
export function mazeToWallRects(maze, cellSize, wallT) {
  const { grid, cols, rows } = maze;
  const W = cols * cellSize;
  const H = rows * cellSize;
  const halfW = W / 2;
  const halfH = H / 2;
  const rects = [];
  const wt = wallT;

  for (let z = 0; z < rows; z++) {
    for (let x = 0; x < cols; x++) {
      const c = grid[z][x];
      const cx = (x + 0.5) * cellSize - halfW;
      const cz = (z + 0.5) * cellSize - halfH;
      const hs = cellSize / 2;

      if (c.n) {
        rects.push({
          minX: cx - hs - wt,
          maxX: cx + hs + wt,
          minZ: cz - hs - wt,
          maxZ: cz - hs + wt,
        });
      }
      if (c.e) {
        rects.push({
          minX: cx + hs - wt,
          maxX: cx + hs + wt,
          minZ: cz - hs - wt,
          maxZ: cz + hs + wt,
        });
      }
      if (z === rows - 1 && c.s) {
        rects.push({
          minX: cx - hs - wt,
          maxX: cx + hs + wt,
          minZ: cz + hs - wt,
          maxZ: cz + hs + wt,
        });
      }
      if (x === 0 && c.w) {
        rects.push({
          minX: cx - hs - wt,
          maxX: cx - hs + wt,
          minZ: cz - hs - wt,
          maxZ: cz + hs + wt,
        });
      }
    }
  }

  return rects;
}

export function cellCenterWorld(cx, cz, cols, rows, cellSize) {
  const halfW = (cols * cellSize) / 2;
  const halfH = (rows * cellSize) / 2;
  return {
    x: (cx + 0.5) * cellSize - halfW,
    y: 0.12,
    z: (cz + 0.5) * cellSize - halfH,
  };
}
