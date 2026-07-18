/**
 * ImagePlotter — converts an image to marker-friendly outline pen paths.
 *
 * Pipeline:
 *  1. Load image -> draw to processing canvas (low resolution)
 *  2. Perceptual grayscale (Rec.709)
 *  3. CLAHE-lite contrast normalization (handles bright/uneven lighting)
 *  4. Gaussian blur (kill noise before edge detection)
 *  5. Sobel edge detection -> gradient magnitude map
 *  6. Non-maximum suppression (thin edges to 1px wide)
 *  7. Hysteresis double-threshold (strong + weak edges)
 *  8. Connected 8-path tracing -> ordered pen stroke chains
 *  9. Map strokes to printer coordinates, generate G-code waypoints
 * 10. Render preview
 */
export class ImagePlotter {
  constructor() {
    this.waypoints = [];       // [{x, y, z}, ...]
    this.processCanvas = document.createElement('canvas');
    this.previewCanvas = null;
    this.imageBitmap = null;
    this._lastPenDownZ = 0;
    this._lastPenUpZ = 15;
  }

  // -- 1. Image ingest -------------------------------------------------------

  loadFromFile(file) {
    return new Promise((resolve, reject) => {
      const url = URL.createObjectURL(file);
      const img = new Image();
      img.onload = () => { this.imageBitmap = img; URL.revokeObjectURL(url); resolve(); };
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

  // -- 2. Core processing ----------------------------------------------------

  /**
   * @param {number} numLines   - controls render resolution (height in px)
   * @param {number} threshold  - Canny high threshold hint [0-255]
   * @param {object} workspace  - { centerX, centerY, workspaceWidth, workspaceHeight }
   * @param {number} penDownZ
   * @param {number} penUpZ
   * @returns {number} total waypoint count
   */
  process(numLines, threshold, workspace, penDownZ, penUpZ) {
    if (!this.imageBitmap) throw new Error('No image loaded');
    this._lastPenDownZ = penDownZ;
    this._lastPenUpZ = penUpZ;

    // -- A: render to low-res canvas -----------------------------------------
    const aspect = this.imageBitmap.width / this.imageBitmap.height;
    const resH = numLines;
    const resW = Math.max(8, Math.round(numLines * aspect));

    const pc = this.processCanvas;
    pc.width  = resW;
    pc.height = resH;
    const ctx = pc.getContext('2d', { willReadFrequently: true });
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, resW, resH);
    ctx.drawImage(this.imageBitmap, 0, 0, resW, resH);

    const imgData = ctx.getImageData(0, 0, resW, resH);
    const px = imgData.data;

    // -- B: perceptual grayscale (Rec.709) ------------------------------------
    const lum = new Uint8Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) {
      lum[i] = Math.round(0.2126 * px[i*4] + 0.7152 * px[i*4+1] + 0.0722 * px[i*4+2]);
    }

    // -- C: CLAHE-lite contrast stretch (handles bright lighting) ------------
    const lumSorted = Float32Array.from(lum).sort();
    const clipLow  = lumSorted[Math.floor(lumSorted.length * 0.02)] || 0;
    const clipHigh = lumSorted[Math.floor(lumSorted.length * 0.98)] || 255;
    const lumRange = Math.max(1, clipHigh - clipLow);
    const norm = new Uint8Array(resW * resH);
    for (let i = 0; i < lum.length; i++) {
      norm[i] = Math.round(Math.max(0, Math.min(255, ((lum[i] - clipLow) / lumRange) * 255)));
    }

    // -- D: 5x5 Gaussian blur (stronger than 3x3 to reduce noise before Sobel)
    const blurred = this._gaussianBlur5(norm, resW, resH);

    // -- E: Sobel gradient magnitude + direction -----------------------------
    const { mag, dir } = this._sobel(blurred, resW, resH);

    // -- F: Non-maximum suppression (thin edges to 1px wide) ----------------
    const suppressed = this._nms(mag, dir, resW, resH);

    // -- G: Hysteresis double-threshold (Canny-style) ------------------------
    //   highT = user threshold, lowT = 40% of that
    const highT = Math.max(10, Math.min(255, threshold));
    const lowT  = Math.round(highT * 0.4);
    const edges = this._hysteresis(suppressed, resW, resH, lowT, highT);

    // -- H: Trace connected 8-paths into ordered pen stroke chains -----------
    const strokes = this._traceStrokes(edges, resW, resH);

    // -- I: Map strokes to printer coordinates, build waypoints --------------
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth  / 2.0;
    const minY = centerY - workspaceHeight / 2.0;

    const toX = (col) => minX + (col / (resW - 1)) * workspaceWidth;
    const toY = (row) => minY + (row / (resH - 1)) * workspaceHeight;

    const waypoints = [];
    for (const stroke of strokes) {
      if (stroke.length === 0) continue;
      // Travel to stroke start with pen UP
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penUpZ });
      // Pen DOWN at stroke start
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penDownZ });
      // Draw the stroke
      for (let i = 1; i < stroke.length; i++) {
        waypoints.push({ x: toX(stroke[i].x), y: toY(stroke[i].y), z: penDownZ });
      }
      // Pen UP at stroke end
      const last = stroke[stroke.length - 1];
      waypoints.push({ x: toX(last.x), y: toY(last.y), z: penUpZ });
    }

    this.waypoints = waypoints;

    if (this.previewCanvas) {
      this._renderPreview(strokes, resW, resH);
    }

    return waypoints.length;
  }

  // -- Internal DSP helpers --------------------------------------------------

  _gaussianBlur5(src, w, h) {
    // Separable 5-tap Gaussian kernel [1,4,6,4,1]/16
    const tmp = new Float32Array(w * h);
    const dst = new Uint8Array(w * h);
    const k = [1, 4, 6, 4, 1];
    const kSum = 16;
    // Horizontal pass
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        let acc = 0;
        for (let ki = 0; ki < 5; ki++) {
          const sx = Math.max(0, Math.min(w - 1, x + ki - 2));
          acc += src[y * w + sx] * k[ki];
        }
        tmp[y * w + x] = acc / kSum;
      }
    }
    // Vertical pass
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        let acc = 0;
        for (let ki = 0; ki < 5; ki++) {
          const sy = Math.max(0, Math.min(h - 1, y + ki - 2));
          acc += tmp[sy * w + x] * k[ki];
        }
        dst[y * w + x] = Math.round(acc / kSum);
      }
    }
    return dst;
  }

  _sobel(src, w, h) {
    const mag = new Float32Array(w * h);
    const dir = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) {
      for (let x = 1; x < w - 1; x++) {
        const tl = src[(y-1)*w + (x-1)], tc = src[(y-1)*w + x], tr = src[(y-1)*w + (x+1)];
        const ml = src[    y*w + (x-1)],                         mr = src[    y*w + (x+1)];
        const bl = src[(y+1)*w + (x-1)], bc = src[(y+1)*w + x], br = src[(y+1)*w + (x+1)];
        const gx = -tl - 2*ml - bl + tr + 2*mr + br;
        const gy = -tl - 2*tc - tr + bl + 2*bc + br;
        mag[y * w + x] = Math.sqrt(gx * gx + gy * gy);
        dir[y * w + x] = Math.atan2(gy, gx);
      }
    }
    return { mag, dir };
  }

  _nms(mag, dir, w, h) {
    // Keep a pixel only if it is a local maximum along its gradient direction
    const out = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) {
      for (let x = 1; x < w - 1; x++) {
        const angle = ((dir[y * w + x] * 180 / Math.PI) + 180) % 180;
        let n1, n2;
        if (angle < 22.5 || angle >= 157.5) {
          n1 = mag[y * w + (x - 1)]; n2 = mag[y * w + (x + 1)];
        } else if (angle < 67.5) {
          n1 = mag[(y - 1) * w + (x + 1)]; n2 = mag[(y + 1) * w + (x - 1)];
        } else if (angle < 112.5) {
          n1 = mag[(y - 1) * w + x]; n2 = mag[(y + 1) * w + x];
        } else {
          n1 = mag[(y - 1) * w + (x - 1)]; n2 = mag[(y + 1) * w + (x + 1)];
        }
        const m = mag[y * w + x];
        out[y * w + x] = (m >= n1 && m >= n2) ? m : 0;
      }
    }
    return out;
  }

  _hysteresis(mag, w, h, lowT, highT) {
    // state: 0=none, 1=weak, 2=strong
    const state = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) {
      if (mag[i] >= highT) state[i] = 2;
      else if (mag[i] >= lowT) state[i] = 1;
    }
    // BFS: promote weak edges connected to strong edges
    const edges    = new Uint8Array(w * h);
    const visited  = new Uint8Array(w * h);
    const queue    = [];
    for (let i = 0; i < w * h; i++) {
      if (state[i] === 2) { queue.push(i); visited[i] = 1; edges[i] = 1; }
    }
    const dx8 = [-1, 0, 1, -1, 1, -1, 0, 1];
    const dy8 = [-1, -1, -1, 0, 0, 1, 1, 1];
    let qi = 0;
    while (qi < queue.length) {
      const idx = queue[qi++];
      const ex = idx % w;
      const ey = Math.floor(idx / w);
      for (let d = 0; d < 8; d++) {
        const nx = ex + dx8[d];
        const ny = ey + dy8[d];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const ni = ny * w + nx;
        if (!visited[ni] && state[ni] >= 1) {
          visited[ni] = 1; edges[ni] = 1; queue.push(ni);
        }
      }
    }
    return edges;
  }

  _traceStrokes(edges, w, h) {
    // Follow 8-connected edge pixels into ordered stroke chains.
    // Greedy: prefer continuing in the same direction to avoid zigzag paths.
    const visited = new Uint8Array(w * h);
    const strokes = [];
    const dx8 = [-1, 0, 1, -1, 1, -1, 0, 1];
    const dy8 = [-1, -1, -1, 0, 0, 1, 1, 1];

    for (let startY = 0; startY < h; startY++) {
      for (let startX = 0; startX < w; startX++) {
        const si = startY * w + startX;
        if (!edges[si] || visited[si]) continue;

        const stroke = [];
        let cx = startX, cy = startY;
        let prevDx = 0, prevDy = 0;

        while (true) {
          const ci = cy * w + cx;
          if (visited[ci]) break;
          visited[ci] = 1;
          stroke.push({ x: cx, y: cy });

          // Find best unvisited 8-connected neighbour
          let bestX = -1, bestY = -1, bestScore = -Infinity;
          for (let d = 0; d < 8; d++) {
            const nx = cx + dx8[d];
            const ny = cy + dy8[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            const ni = ny * w + nx;
            if (!edges[ni] || visited[ni]) continue;
            // Prefer direction that continues previous travel (minimises turns)
            const score = (prevDx !== 0 || prevDy !== 0)
              ? (dx8[d] * prevDx + dy8[d] * prevDy)
              : 0;
            if (score > bestScore) { bestScore = score; bestX = nx; bestY = ny; }
          }

          if (bestX === -1) break;
          prevDx = bestX - cx;
          prevDy = bestY - cy;
          cx = bestX;
          cy = bestY;
        }

        if (stroke.length >= 2) strokes.push(stroke);
      }
    }
    return strokes;
  }

  // -- Preview render --------------------------------------------------------

  _renderPreview(strokes, resW, resH) {
    const cv = this.previewCanvas;
    const SCALE = 4;
    cv.width  = resW * SCALE;
    cv.height = resH * SCALE;
    const ctx = cv.getContext('2d');

    ctx.fillStyle = '#0c0f16';
    ctx.fillRect(0, 0, cv.width, cv.height);

    // Cycle through neon colours so individual strokes are distinguishable
    const palette = ['#39ff14', '#00e6ff', '#ff6e00', '#ff0090', '#ffe600'];
    ctx.lineWidth = 1.5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    strokes.forEach((stroke, si) => {
      if (stroke.length < 2) return;
      ctx.strokeStyle = palette[si % palette.length];
      ctx.beginPath();
      ctx.moveTo(stroke[0].x * SCALE + SCALE / 2, stroke[0].y * SCALE + SCALE / 2);
      for (let i = 1; i < stroke.length; i++) {
        ctx.lineTo(stroke[i].x * SCALE + SCALE / 2, stroke[i].y * SCALE + SCALE / 2);
      }
      ctx.stroke();
    });
  }

  // -- G-code streaming -----------------------------------------------------

  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;

    for (let i = 0; i < wps.length; i++) {
      if (isCancelled && isCancelled()) break;
      const wp = wps[i];
      // Use Z feedrate for Z-only moves, XY feedrate for travel/draw
      const isZOnly = (i > 0 &&
        Math.abs(wps[i-1].x - wp.x) < 0.01 &&
        Math.abs(wps[i-1].y - wp.y) < 0.01);
      const f = isZOnly ? feedrateZ : feedrateXY;
      socket.send(`gcode-plot:G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${f}`);
      if (onProgress) onProgress(i + 1, wps.length);
      if (i % 5 === 0) await new Promise(r => setTimeout(r, 0));
    }

    // Final safe lift
    const last = wps[wps.length - 1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z${this._lastPenUpZ + 5} F${feedrateXY}`);
  }
}
