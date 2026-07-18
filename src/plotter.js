/**
 * ImagePlotter — heavy-duty outline extraction for pen plotters.
 *
 * Pipeline:
 *  A. Render image to canvas at chosen resolution
 *  B. Perceptual grayscale (Rec.709)
 *  C. CLAHE (adaptive per-tile histogram equalization, lighting-robust)
 *  D. Multi-scale Canny at two sigma values → union of edge maps
 *     D1. sigma=1.0  (fine details: text, sharp edges)
 *     D2. sigma=2.5  (medium structure: objects, faces)
 *  E. Degree-aware connected path tracing
 *     — endpoints (deg=1) are traced first, junctions (deg≥3) are breakpoints
 *  F. Douglas-Peucker path simplification (removes redundant co-linear waypoints)
 *  G. Greedy nearest-neighbor stroke sort (minimises pen travel between strokes)
 *  H. Map to printer coordinates, emit waypoints
 */

export class ImagePlotter {
  constructor() {
    this.waypoints   = [];
    this.processCanvas = document.createElement('canvas');
    this.previewCanvas = null;
    this.imageBitmap   = null;
    this._lastPenUpZ   = 15;
  }

  // ── Image ingest ───────────────────────────────────────────────────────────

  loadFromFile(file) {
    return new Promise((resolve, reject) => {
      const url = URL.createObjectURL(file);
      const img = new Image();
      img.onload  = () => { this.imageBitmap = img; URL.revokeObjectURL(url); resolve(); };
      img.onerror = reject;
      img.src = url;
    });
  }

  loadFromVideo(videoEl) {
    return new Promise((resolve) => {
      const snap = document.createElement('canvas');
      snap.width  = videoEl.videoWidth  || 640;
      snap.height = videoEl.videoHeight || 480;
      snap.getContext('2d').drawImage(videoEl, 0, 0);
      const img = new Image();
      img.onload = () => { this.imageBitmap = img; resolve(); };
      img.src = snap.toDataURL('image/jpeg', 0.92);
    });
  }

  // ── Core process ───────────────────────────────────────────────────────────

  /**
   * @param {number} resolution  — image processing height in pixels (80–400)
   * @param {number} threshold   — Canny high-threshold (5–150; lower = more edges)
   * @param {object} workspace   — { centerX, centerY, workspaceWidth, workspaceHeight }
   * @param {number} penDownZ
   * @param {number} penUpZ
   * @returns {number} waypoint count
   */
  process(resolution, threshold, workspace, penDownZ, penUpZ) {
    if (!this.imageBitmap) throw new Error('No image loaded');
    this._lastPenUpZ = penUpZ;

    // ── A: Render to processing canvas ──────────────────────────────────────
    const aspect = this.imageBitmap.width / this.imageBitmap.height;
    const resH = Math.max(20, resolution);
    const resW = Math.max(8, Math.round(resH * aspect));

    const pc  = this.processCanvas;
    pc.width  = resW;
    pc.height = resH;
    const ctx = pc.getContext('2d', { willReadFrequently: true });
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, resW, resH);
    ctx.drawImage(this.imageBitmap, 0, 0, resW, resH);
    const px = ctx.getImageData(0, 0, resW, resH).data;

    // ── B: Perceptual grayscale ──────────────────────────────────────────────
    const gray = new Float32Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) {
      gray[i] = 0.2126 * px[i*4] + 0.7152 * px[i*4+1] + 0.0722 * px[i*4+2];
    }

    // ── C: CLAHE (adaptive per-tile) ─────────────────────────────────────────
    //   8×8 tiles, clip limit 3.0 — equalises contrast even in bright/dark areas
    const clahe = this._clahe(gray, resW, resH, 8, 8, 3.0);

    // ── D: Multi-scale Canny edge detection ─────────────────────────────────
    //   highT = user threshold (gradient magnitude units after CLAHE)
    //   lowT  = 40% of highT (hysteresis lower bound)
    const highT = Math.max(5, threshold);
    const lowT  = Math.round(highT * 0.4);

    const edges1 = this._canny(clahe, resW, resH, 1.0, highT,       lowT);       // fine
    const edges2 = this._canny(clahe, resW, resH, 2.5, highT * 0.7, lowT * 0.7); // medium

    // Union: a pixel is an edge if either scale detected it
    const edges = new Uint8Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) edges[i] = edges1[i] | edges2[i];

    // ── E: Degree-aware path tracing ─────────────────────────────────────────
    const rawStrokes = this._traceStrokes(edges, resW, resH);

    // ── F: Douglas-Peucker simplification (epsilon = 0.8px) ──────────────────
    //   Removes intermediate waypoints that are close to the line — fewer G-code moves
    const simplified = rawStrokes.map(s => this._douglasPeucker(s, 0.8)).filter(s => s.length >= 2);

    // ── G: Greedy nearest-neighbour stroke sort ───────────────────────────────
    //   Picks each next stroke by whichever end is closest to the current pen position
    const sorted = this._sortStrokes(simplified);

    // ── H: Map to printer coordinates ────────────────────────────────────────
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth  / 2.0;
    const minY = centerY - workspaceHeight / 2.0;
    const toX  = col => minX + (col / (resW - 1)) * workspaceWidth;
    const toY  = row => minY + (row / (resH - 1)) * workspaceHeight;

    const waypoints = [];
    for (const stroke of sorted) {
      if (!stroke || stroke.length < 2) continue;
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penUpZ   }); // travel
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penDownZ }); // pen down
      for (let i = 1; i < stroke.length; i++) {
        waypoints.push({ x: toX(stroke[i].x), y: toY(stroke[i].y), z: penDownZ });
      }
      const last = stroke[stroke.length - 1];
      waypoints.push({ x: toX(last.x), y: toY(last.y), z: penUpZ }); // pen up
    }

    this.waypoints = waypoints;
    if (this.previewCanvas) this._renderPreview(sorted, resW, resH);
    return waypoints.length;
  }

  // ── CLAHE (Contrast Limited Adaptive Histogram Equalization) ──────────────

  _clahe(gray, w, h, numTX, numTY, clipLimit) {
    const twPx = w / numTX;
    const thPx = h / numTY;

    // Compute per-tile LUTs
    const luts = [];
    for (let ty = 0; ty < numTY; ty++) {
      for (let tx = 0; tx < numTX; tx++) {
        const x0 = Math.round(tx * twPx),     x1 = Math.min(w, Math.round((tx+1)*twPx));
        const y0 = Math.round(ty * thPx),     y1 = Math.min(h, Math.round((ty+1)*thPx));
        const hist = new Float32Array(256);
        let count = 0;
        for (let y = y0; y < y1; y++) {
          for (let x = x0; x < x1; x++) {
            hist[Math.min(255, Math.max(0, Math.round(gray[y*w+x])))]++;
            count++;
          }
        }
        // Clip and redistribute excess
        const clip = clipLimit * count / 256;
        let excess = 0;
        for (let i = 0; i < 256; i++) { if (hist[i] > clip) { excess += hist[i] - clip; hist[i] = clip; } }
        const add = excess / 256;
        for (let i = 0; i < 256; i++) hist[i] += add;
        // Build CDF -> LUT
        const lut = new Float32Array(256);
        let cum = 0;
        for (let i = 0; i < 256; i++) { cum += hist[i]; lut[i] = Math.min(255, (cum / count) * 255); }
        luts.push(lut);
      }
    }

    // Bilinear interpolation between tile LUTs
    const out = new Float32Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const val = Math.min(255, Math.max(0, Math.round(gray[y*w+x])));
        // Fractional tile coords (LUT centres are at tile midpoints)
        const txf = (x / twPx) - 0.5;
        const tyf = (y / thPx) - 0.5;
        const tx0 = Math.max(0, Math.min(numTX-2, Math.floor(txf)));
        const ty0 = Math.max(0, Math.min(numTY-2, Math.floor(tyf)));
        const tx1 = tx0 + 1, ty1 = ty0 + 1;
        const wx = Math.max(0, Math.min(1, txf - tx0));
        const wy = Math.max(0, Math.min(1, tyf - ty0));
        const v00 = luts[ty0*numTX+tx0][val];
        const v10 = luts[ty0*numTX+tx1][val];
        const v01 = luts[ty1*numTX+tx0][val];
        const v11 = luts[ty1*numTX+tx1][val];
        out[y*w+x] = v00*(1-wx)*(1-wy) + v10*wx*(1-wy) + v01*(1-wx)*wy + v11*wx*wy;
      }
    }
    return out;
  }

  // ── Separable Gaussian blur with configurable sigma ───────────────────────

  _gaussianBlur(src, w, h, sigma) {
    const radius = Math.ceil(sigma * 3);
    const size   = 2 * radius + 1;
    const k = new Float32Array(size);
    let sum = 0;
    for (let i = 0; i < size; i++) { const x = i-radius; k[i] = Math.exp(-(x*x)/(2*sigma*sigma)); sum += k[i]; }
    for (let i = 0; i < size; i++) k[i] /= sum;

    const tmp = new Float32Array(w * h);
    const dst = new Float32Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        let acc = 0;
        for (let ki = 0; ki < size; ki++) {
          const sx = Math.max(0, Math.min(w-1, x+ki-radius));
          acc += src[y*w+sx] * k[ki];
        }
        tmp[y*w+x] = acc;
      }
    }
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        let acc = 0;
        for (let ki = 0; ki < size; ki++) {
          const sy = Math.max(0, Math.min(h-1, y+ki-radius));
          acc += tmp[sy*w+x] * k[ki];
        }
        dst[y*w+x] = acc;
      }
    }
    return dst;
  }

  // ── Full Canny pipeline at one scale ─────────────────────────────────────

  _canny(gray, w, h, sigma, highT, lowT) {
    const blurred = this._gaussianBlur(gray, w, h, sigma);

    // Sobel gradients
    const mag = new Float32Array(w * h);
    const dir = new Float32Array(w * h);
    for (let y = 1; y < h-1; y++) {
      for (let x = 1; x < w-1; x++) {
        const tl=blurred[(y-1)*w+(x-1)], tc=blurred[(y-1)*w+x], tr=blurred[(y-1)*w+(x+1)];
        const ml=blurred[    y*w+(x-1)],                          mr=blurred[    y*w+(x+1)];
        const bl=blurred[(y+1)*w+(x-1)], bc=blurred[(y+1)*w+x], br=blurred[(y+1)*w+(x+1)];
        const gx = -tl - 2*ml - bl + tr + 2*mr + br;
        const gy = -tl - 2*tc - tr + bl + 2*bc + br;
        mag[y*w+x] = Math.sqrt(gx*gx + gy*gy);
        dir[y*w+x] = Math.atan2(gy, gx);
      }
    }

    // Non-maximum suppression
    const nms = new Float32Array(w * h);
    for (let y = 1; y < h-1; y++) {
      for (let x = 1; x < w-1; x++) {
        const ang = ((dir[y*w+x]*180/Math.PI)+180) % 180;
        let n1, n2;
        if      (ang <  22.5 || ang >= 157.5) { n1=mag[y*w+(x-1)];         n2=mag[y*w+(x+1)]; }
        else if (ang <  67.5)                 { n1=mag[(y-1)*w+(x+1)];     n2=mag[(y+1)*w+(x-1)]; }
        else if (ang < 112.5)                 { n1=mag[(y-1)*w+x];         n2=mag[(y+1)*w+x]; }
        else                                  { n1=mag[(y-1)*w+(x-1)];     n2=mag[(y+1)*w+(x+1)]; }
        const m = mag[y*w+x];
        nms[y*w+x] = (m >= n1 && m >= n2) ? m : 0;
      }
    }

    // Hysteresis double-threshold
    const state = new Uint8Array(w * h);
    for (let i = 0; i < w*h; i++) {
      if (nms[i] >= highT) state[i] = 2;
      else if (nms[i] >= lowT) state[i] = 1;
    }
    const edges   = new Uint8Array(w * h);
    const visited = new Uint8Array(w * h);
    const queue   = [];
    for (let i = 0; i < w*h; i++) { if (state[i]===2) { queue.push(i); visited[i]=1; edges[i]=1; } }
    const dx8=[-1,0,1,-1,1,-1,0,1], dy8=[-1,-1,-1,0,0,1,1,1];
    let qi = 0;
    while (qi < queue.length) {
      const idx=queue[qi++], ex=idx%w, ey=Math.floor(idx/w);
      for (let d=0; d<8; d++) {
        const nx=ex+dx8[d], ny=ey+dy8[d];
        if (nx<0||nx>=w||ny<0||ny>=h) continue;
        const ni=ny*w+nx;
        if (!visited[ni] && state[ni]>=1) { visited[ni]=1; edges[ni]=1; queue.push(ni); }
      }
    }
    return edges;
  }

  // ── Degree-aware path tracing ─────────────────────────────────────────────
  //
  // Degree = number of edge-pixel neighbours.
  //  deg=0  isolated pixel  (skip)
  //  deg=1  endpoint / tip of a stroke
  //  deg=2  interior of a stroke
  //  deg≥3  junction (T or X crossing) — we break here to avoid scrambled paths

  _traceStrokes(edges, w, h) {
    const dx8=[-1,0,1,-1,1,-1,0,1], dy8=[-1,-1,-1,0,0,1,1,1];

    // Pre-compute degree of every edge pixel
    const degree  = new Uint8Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        if (!edges[y*w+x]) continue;
        let deg = 0;
        for (let d=0; d<8; d++) {
          const nx=x+dx8[d], ny=y+dy8[d];
          if (nx>=0&&nx<w&&ny>=0&&ny<h&&edges[ny*w+nx]) deg++;
        }
        degree[y*w+x] = deg;
      }
    }

    const visited = new Uint8Array(w * h);
    const strokes = [];

    const followStroke = (startX, startY) => {
      const stroke = [];
      let cx=startX, cy=startY, prevDx=0, prevDy=0;
      while (true) {
        const ci = cy*w+cx;
        if (visited[ci]) break;
        visited[ci] = 1;
        stroke.push({ x: cx, y: cy });
        // Stop at junctions after first step to keep strokes clean
        if (stroke.length > 1 && degree[ci] >= 3) break;
        // Find best next unvisited neighbour (prefer continuing same direction)
        let bestX=-1, bestY=-1, bestScore=-Infinity;
        for (let d=0; d<8; d++) {
          const nx=cx+dx8[d], ny=cy+dy8[d];
          if (nx<0||nx>=w||ny<0||ny>=h) continue;
          const ni=ny*w+nx;
          if (!edges[ni]||visited[ni]) continue;
          const score = (prevDx!==0||prevDy!==0) ? dx8[d]*prevDx+dy8[d]*prevDy : 0;
          if (score > bestScore) { bestScore=score; bestX=nx; bestY=ny; }
        }
        if (bestX===-1) break;
        prevDx=bestX-cx; prevDy=bestY-cy;
        cx=bestX; cy=bestY;
      }
      return stroke;
    };

    // First pass: endpoints (degree=1) — stroke tips, start here for natural traces
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        const i=y*w+x;
        if (!edges[i]||visited[i]||degree[i]!==1) continue;
        const s = followStroke(x, y);
        if (s.length >= 2) strokes.push(s);
      }

    // Second pass: remaining unvisited (loops, junctions, isolated segments)
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) {
        const i=y*w+x;
        if (!edges[i]||visited[i]) continue;
        const s = followStroke(x, y);
        if (s.length >= 2) strokes.push(s);
      }

    return strokes;
  }

  // ── Douglas-Peucker path simplification ───────────────────────────────────

  _douglasPeucker(points, epsilon) {
    if (points.length <= 2) return points;
    const first = points[0], last = points[points.length-1];
    const lineLen = Math.hypot(last.x-first.x, last.y-first.y) || 1;
    let maxDist=0, maxIdx=0;
    for (let i=1; i<points.length-1; i++) {
      // Perpendicular distance from point to line first->last
      const dist = Math.abs(
        (last.y-first.y)*points[i].x - (last.x-first.x)*points[i].y +
        last.x*first.y - last.y*first.x
      ) / lineLen;
      if (dist > maxDist) { maxDist=dist; maxIdx=i; }
    }
    if (maxDist > epsilon) {
      const L = this._douglasPeucker(points.slice(0, maxIdx+1), epsilon);
      const R = this._douglasPeucker(points.slice(maxIdx),      epsilon);
      return L.slice(0,-1).concat(R);
    }
    return [first, last];
  }

  // ── Greedy nearest-neighbour stroke sort ──────────────────────────────────
  // Picks the next stroke by whichever endpoint is closest to the current pen.
  // Reverses the stroke if its end-point is closer than its start-point.

  _sortStrokes(strokes) {
    if (!strokes.length) return strokes;
    const sorted = [];
    const used   = new Uint8Array(strokes.length);
    let curX = strokes[0][0].x, curY = strokes[0][0].y;

    for (let iter=0; iter<strokes.length; iter++) {
      let bestIdx=-1, bestDist=Infinity, bestReverse=false;
      for (let i=0; i<strokes.length; i++) {
        if (used[i]||!strokes[i]||strokes[i].length<2) continue;
        const s  = strokes[i];
        const ds = Math.hypot(s[0].x-curX,           s[0].y-curY);
        const de = Math.hypot(s[s.length-1].x-curX,  s[s.length-1].y-curY);
        const d  = Math.min(ds, de);
        if (d < bestDist) { bestDist=d; bestIdx=i; bestReverse=(de<ds); }
      }
      if (bestIdx===-1) break;
      used[bestIdx]=1;
      const stroke = bestReverse ? strokes[bestIdx].slice().reverse() : strokes[bestIdx];
      sorted.push(stroke);
      const last = stroke[stroke.length-1];
      curX=last.x; curY=last.y;
    }
    return sorted;
  }

  // ── Preview render ────────────────────────────────────────────────────────

  _renderPreview(strokes, resW, resH) {
    const cv = this.previewCanvas;
    // Scale preview up so even small images look detailed
    const maxDim  = 400;
    const SCALE   = Math.max(1, Math.floor(Math.min(maxDim / resW, maxDim / resH)));
    cv.width      = resW * SCALE;
    cv.height     = resH * SCALE;
    const ctx     = cv.getContext('2d');

    ctx.fillStyle = '#0c0f16';
    ctx.fillRect(0, 0, cv.width, cv.height);

    // Each stroke gets a colour from a neon palette for easy visual inspection
    const palette = ['#39ff14', '#00e6ff', '#ff6e00', '#ff0090', '#ffe600', '#a855f7', '#06b6d4'];
    ctx.lineCap   = 'round';
    ctx.lineJoin  = 'round';
    ctx.lineWidth = Math.max(1, SCALE * 0.6);

    strokes.forEach((stroke, si) => {
      if (!stroke || stroke.length < 2) return;
      ctx.strokeStyle = palette[si % palette.length];
      ctx.beginPath();
      ctx.moveTo(stroke[0].x * SCALE + SCALE/2, stroke[0].y * SCALE + SCALE/2);
      for (let i=1; i<stroke.length; i++) {
        ctx.lineTo(stroke[i].x * SCALE + SCALE/2, stroke[i].y * SCALE + SCALE/2);
      }
      ctx.stroke();
    });
  }

  // ── G-code streaming ─────────────────────────────────────────────────────

  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;

    for (let i=0; i<wps.length; i++) {
      if (isCancelled && isCancelled()) break;
      const wp = wps[i];
      // Use slower Z feedrate for purely vertical moves (pen lifts/drops)
      const isZOnly = i > 0 &&
        Math.abs(wps[i-1].x - wp.x) < 0.01 &&
        Math.abs(wps[i-1].y - wp.y) < 0.01;
      const f = isZOnly ? feedrateZ : feedrateXY;
      socket.send(`gcode-plot:G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${f}`);
      if (onProgress) onProgress(i+1, wps.length);
      if (i % 5 === 0) await new Promise(r => setTimeout(r, 0));
    }

    // Safe final lift
    const last = wps[wps.length-1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z${this._lastPenUpZ + 5} F${feedrateXY}`);
  }
}
