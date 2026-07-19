/**
 * ImagePlotter — perceptual sketch extraction for pen plotters.
 *
 * Pipeline:
 *  A. Render to processing canvas
 *  B. Perceptual grayscale (Rec.709)
 *  C. CLAHE per-tile (lighting robustness)
 *  D. Bilateral filter (smooth texture inside regions, keep object borders)
 *  E. XDoG x2 scales — finds artist-quality structural lines, not pixel noise
 *       sigma=0.8  (fine: eyes, lips, sharp borders)
 *       sigma=1.6  (medium: face outline, hair, objects)
 *       union of both → complete sketch map
 *  F. Minimum stroke length filter (discard texture fragments < minLen px)
 *  G. Degree-aware path tracing
 *  H. Douglas-Peucker simplification
 *  I. Greedy nearest-neighbour stroke sort (minimise travel)
 *  J. Emit waypoints
 *
 * Pause/Resume/Stop:
 *  imagePlotter.pause()   — pause between waypoints
 *  imagePlotter.resume()  — resume from pause
 *  imagePlotter.stop()    — cancel plot
 */

export class ImagePlotter {
  constructor() {
    this.waypoints     = [];
    this.processCanvas = document.createElement('canvas');
    this.previewCanvas = null;
    this.imageBitmap   = null;
    this._lastPenUpZ   = 15;
    // Streaming control
    this._paused        = false;
    this._stopped       = false;
    this._resumeResolve = null;
    this._pendingOkResolvers = [];
  }

  // ── Streaming control API ──────────────────────────────────────────────────
  pause()  { this._paused = true; }
  resume() {
    this._paused = false;
    if (this._resumeResolve) { this._resumeResolve(); this._resumeResolve = null; }
  }
  stop()   { 
    this._stopped = true; 
    this.resume(); 
    const resolvers = this._pendingOkResolvers;
    this._pendingOkResolvers = [];
    for (const r of resolvers) r();
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
   * @param {number} resolution — processing height in pixels (80–400)
   * @param {number} threshold  — edge threshold; lower = more edges (1–50)
   * @param {number} minStroke  — minimum stroke length in pixels to keep (2–20)
   * @param {number} simplification — Douglas-Peucker epsilon tolerance (0.1–10.0)
   * @param {number} mergeGapMM — maximum physical gap in mm to bridge without lifting the pen (0.0-20.0)
   * @param {string} processingMode — 'lineart' or 'photo'
   * @param {number} lineWidthMM — pen stroke width in mm (0.1–10.0)
   * @param {string} drawingStyle — 'outlines', 'hatch', or 'crosshatch'
   * @param {number} shadingDensity — spacing between shading lines in pixels (3–25)
   * @param {object} workspace
   * @param {number} penDownZ
   * @param {number} penUpZ
   */
  process(resolution, threshold, minStroke, simplification, mergeGapMM, processingMode, lineWidthMM, drawingStyle, shadingDensity, workspace, penDownZ, penUpZ) {
    if (!this.imageBitmap) throw new Error('No image loaded');
    this._lastPenUpZ = penUpZ;
    this._lineWidthMM = lineWidthMM || 0.8;

    // ── A: Render ──────────────────────────────────────────────────────────
    const aspect = this.imageBitmap.width / this.imageBitmap.height;
    const resH   = Math.max(20, resolution);
    const resW   = Math.max(8,  Math.round(resH * aspect));
    const pc     = this.processCanvas;
    pc.width = resW; pc.height = resH;
    const ctx = pc.getContext('2d', { willReadFrequently: true });
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, resW, resH);
    ctx.drawImage(this.imageBitmap, 0, 0, resW, resH);
    const px = ctx.getImageData(0, 0, resW, resH).data;

    // ── B: Grayscale ───────────────────────────────────────────────────────
    const gray = new Float32Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) {
      gray[i] = 0.2126 * px[i*4] + 0.7152 * px[i*4+1] + 0.0722 * px[i*4+2];
    }

    let sketch;
    let bilateral;

    if (processingMode === 'lineart') {
      // Line Art Mode: Adaptive Binarization + Zhang-Suen Medial Axis Thinning
      // Extracts single-pixel centerlines to avoid double hollow outlines!
      const binThresh = Math.max(30, Math.min(230, 255 - threshold * 4.0));
      const bin = new Uint8Array(resW * resH);
      for (let i = 0; i < resW * resH; i++) {
        bin[i] = gray[i] < binThresh ? 1 : 0;
      }
      sketch = this._zhangSuenThinning(bin, resW, resH);
      bilateral = gray; // fallback for shading calculation
    } else {
      // Photo Mode: CLAHE -> Bilateral filter -> XDoG Contours -> Zhang-Suen Medial Axis Thinning
      // Converts raw XDoG contour meshes into crisp 2D vector portrait lines!
      const clahe = this._clahe(gray, resW, resH, 8, 8, 3.0);
      bilateral = this._bilateralFilter(clahe, resW, resH, 2.0, 30.0);
      const xdogThresh = Math.max(0.001, threshold / 1000.0);
      const sketchFine   = this._xdog(bilateral, resW, resH, 0.8, 1.6, 0.97, 100.0, xdogThresh);
      const sketchMedium = this._xdog(bilateral, resW, resH, 1.6, 1.6, 0.97,  80.0, xdogThresh * 0.7);
      const rawContour = new Uint8Array(resW * resH);
      for (let i = 0; i < resW * resH; i++) rawContour[i] = sketchFine[i] | sketchMedium[i];
      // Thin photo contours down to single 2D vector lines!
      sketch = this._zhangSuenThinning(rawContour, resW, resH);
    }

    // ── F–I: Trace, chain/connect, filter, simplify, smooth, sort ──────────
    const rawOutlines  = this._traceStrokes(sketch, resW, resH);
    // Connect nearby polyline endpoints to bridge gaps and create long continuous 2D vector strokes
    const connectedOutlines = this._connectNearbyStrokes(rawOutlines, 6);
    const filteredOutlines  = connectedOutlines.filter(s => s.length >= Math.max(2, minStroke));
    const simplifiedOutlines = filteredOutlines.map(s => this._douglasPeucker(s, simplification)).filter(s => s.length >= 2);
    // Apply Chaikin vector curve smoothing for silky smooth 2D lineart
    const smoothedOutlines   = simplifiedOutlines.map(s => this._chaikinSmooth(s, 1));

    // Multi-pass stroke thickening with blur / overlap prevention
    const extraPasses = Math.floor(lineWidthMM / 0.8);
    let finalOutlines = smoothedOutlines;
    if (extraPasses > 1) {
      finalOutlines = [];
      const passOffsetPx = 0.6; // pixel offset spacing for multi-pass thickening
      for (const stroke of smoothedOutlines) {
        finalOutlines.push(stroke);
        for (let p = 1; p < extraPasses; p++) {
          const offsetDist = (p % 2 === 1 ? 1 : -1) * Math.ceil(p / 2) * passOffsetPx;
          const offsetStroke = [];
          for (let i = 0; i < stroke.length; i++) {
            const pt = stroke[i];
            let dx = 0, dy = 0;
            if (i < stroke.length - 1) { dx += stroke[i+1].x - pt.x; dy += stroke[i+1].y - pt.y; }
            if (i > 0) { dx += pt.x - stroke[i-1].x; dy += pt.y - stroke[i-1].y; }
            const len = Math.hypot(dx, dy) || 1;
            const ox = pt.x + (-dy / len) * offsetDist;
            const oy = pt.y + (dx / len) * offsetDist;
            // Ensure offset stays within image bounds
            if (ox >= 0 && ox < resW && oy >= 0 && oy < resH) {
              offsetStroke.push({ x: ox, y: oy });
            }
          }
          if (offsetStroke.length >= 2) finalOutlines.push(offsetStroke);
        }
      }
    }

    const shadingStrokes = [];
    const step = Math.max(3, shadingDensity);

    // Style 1: Hatch (diagonals at 45 deg: x + y = C)
    if (drawingStyle === 'hatch' || drawingStyle === 'crosshatch') {
      for (let c = 0; c < resW + resH; c += step) {
        let currentStroke = [];
        for (let x = 0; x < resW; x++) {
          const y = c - x;
          if (y >= 0 && y < resH) {
            const isDark = bilateral[y * resW + x] < 128;
            if (isDark) {
              currentStroke.push({ x, y });
            } else {
              if (currentStroke.length >= 2) shadingStrokes.push(currentStroke);
              currentStroke = [];
            }
          }
        }
        if (currentStroke.length >= 2) shadingStrokes.push(currentStroke);
      }
    }

    // Style 2: Cross-hatch (add diagonals at -45 deg: x - y = C)
    if (drawingStyle === 'crosshatch') {
      for (let c = -resH; c < resW; c += step) {
        let currentStroke = [];
        for (let x = 0; x < resW; x++) {
          const y = x - c;
          if (y >= 0 && y < resH) {
            const isDark = bilateral[y * resW + x] < 128;
            if (isDark) {
              currentStroke.push({ x, y });
            } else {
              if (currentStroke.length >= 2) shadingStrokes.push(currentStroke);
              currentStroke = [];
            }
          }
        }
        if (currentStroke.length >= 2) shadingStrokes.push(currentStroke);
      }
    }

    // Simplify shading strokes (straight lines -> 2 points)
    const simplifiedShading = shadingStrokes.map(s => this._douglasPeucker(s, simplification)).filter(s => s.length >= 2);

    // Combine outlines and shading
    const allStrokes = finalOutlines.concat(simplifiedShading);

    // Sort together to minimize travel path
    const sorted = this._sortStrokes(allStrokes);

    // ── J: Map to printer coordinates ─────────────────────────────────────
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth  / 2;
    const minY = centerY - workspaceHeight / 2;
    const toX  = col => minX + (col / (resW - 1)) * workspaceWidth;
    const toY  = row => minY + (row / (resH - 1)) * workspaceHeight;

    const waypoints = [];
    let penDown = false;
    let lastWp = null;

    for (const stroke of sorted) {
      if (!stroke || stroke.length < 2) continue;

      const startPt = { x: toX(stroke[0].x), y: toY(stroke[0].y) };

      // Check if the gap from the end of the previous stroke is small enough to bridge
      let bridge = false;
      if (penDown && lastWp) {
        const dist = Math.hypot(startPt.x - lastWp.x, startPt.y - lastWp.y);
        if (dist <= mergeGapMM) {
          bridge = true;
        }
      }

      if (bridge) {
        // Draw a straight line to the start of this stroke (pen stays DOWN)
        waypoints.push({ x: startPt.x, y: startPt.y, z: penDownZ });
      } else {
        // If pen was down, lift it first
        if (penDown && lastWp) {
          waypoints.push({ x: lastWp.x, y: lastWp.y, z: penUpZ });
          penDown = false;
        }
        // Travel to stroke start (pen UP)
        waypoints.push({ x: startPt.x, y: startPt.y, z: penUpZ });
        // Drop pen at start of this stroke
        waypoints.push({ x: startPt.x, y: startPt.y, z: penDownZ });
        penDown = true;
      }

      // Draw the stroke
      for (let i = 1; i < stroke.length; i++) {
        waypoints.push({ x: toX(stroke[i].x), y: toY(stroke[i].y), z: penDownZ });
      }

      // Record the last point of this stroke
      const lastStrokePt = stroke[stroke.length - 1];
      lastWp = { x: toX(lastStrokePt.x), y: toY(lastStrokePt.y) };
    }

    // Final lift
    if (penDown && lastWp) {
      waypoints.push({ x: lastWp.x, y: lastWp.y, z: penUpZ });
    }

    this.resW = resW;
    this.resH = resH;
    this.strokes = sorted;
    this.minX = minX;
    this.minY = minY;
    this.wW = workspaceWidth;
    this.wH = workspaceHeight;
    this._lastPenDownZ = penDownZ;

    this.waypoints = waypoints;
    if (this.previewCanvas) this._renderPreview(sorted, resW, resH);
    return waypoints.length;
  }

  // ── CLAHE ─────────────────────────────────────────────────────────────────

  _clahe(gray, w, h, numTX, numTY, clipLimit) {
    const twPx = w / numTX, thPx = h / numTY;
    const luts = [];
    for (let ty = 0; ty < numTY; ty++) {
      for (let tx = 0; tx < numTX; tx++) {
        const x0=Math.round(tx*twPx), x1=Math.min(w,Math.round((tx+1)*twPx));
        const y0=Math.round(ty*thPx), y1=Math.min(h,Math.round((ty+1)*thPx));
        const hist = new Float32Array(256); let count=0;
        for (let y=y0;y<y1;y++) for (let x=x0;x<x1;x++) { hist[Math.min(255,Math.max(0,Math.round(gray[y*w+x])))]++; count++; }
        const clip=clipLimit*count/256; let excess=0;
        for (let i=0;i<256;i++) { if(hist[i]>clip){excess+=hist[i]-clip;hist[i]=clip;} }
        const add=excess/256; for (let i=0;i<256;i++) hist[i]+=add;
        const lut=new Float32Array(256); let cum=0;
        for (let i=0;i<256;i++) { cum+=hist[i]; lut[i]=Math.min(255,(cum/count)*255); }
        luts.push(lut);
      }
    }
    const out = new Float32Array(w*h);
    for (let y=0;y<h;y++) for (let x=0;x<w;x++) {
      const val=Math.min(255,Math.max(0,Math.round(gray[y*w+x])));
      const txf=(x/twPx)-0.5, tyf=(y/thPx)-0.5;
      const tx0=Math.max(0,Math.min(numTX-2,Math.floor(txf))), ty0=Math.max(0,Math.min(numTY-2,Math.floor(tyf)));
      const wx=Math.max(0,Math.min(1,txf-tx0)), wy=Math.max(0,Math.min(1,tyf-ty0));
      const v00=luts[ty0*numTX+tx0][val], v10=luts[ty0*numTX+(tx0+1)][val];
      const v01=luts[(ty0+1)*numTX+tx0][val], v11=luts[(ty0+1)*numTX+(tx0+1)][val];
      out[y*w+x]=v00*(1-wx)*(1-wy)+v10*wx*(1-wy)+v01*(1-wx)*wy+v11*wx*wy;
    }
    return out;
  }

  // ── Zhang-Suen Thinning Algorithm (Medial Axis Skeletonization) ────────────
  // Converts thick black line art / sketch strokes down to 1-pixel centerlines.
  // Eliminates double-edges/hollow outlines when processing drawings and lineart.

  _zhangSuenThinning(bin, w, h) {
    const grid = new Uint8Array(bin);
    let changed = true;
    let iter = 0;

    const getP = (x, y) => (x >= 0 && x < w && y >= 0 && y < h) ? grid[y * w + x] : 0;

    while (changed && iter < 100) {
      changed = false;
      iter++;
      
      // Pass 1
      const del1 = [];
      for (let y = 1; y < h - 1; y++) {
        for (let x = 1; x < w - 1; x++) {
          if (grid[y * w + x] === 0) continue;

          const p2 = getP(x, y - 1);
          const p3 = getP(x + 1, y - 1);
          const p4 = getP(x + 1, y);
          const p5 = getP(x + 1, y + 1);
          const p6 = getP(x, y + 1);
          const p7 = getP(x - 1, y + 1);
          const p8 = getP(x - 1, y);
          const p9 = getP(x - 1, y - 1);

          const B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
          if (B < 2 || B > 6) continue;

          let A = 0;
          if (p2 === 0 && p3 === 1) A++;
          if (p3 === 0 && p4 === 1) A++;
          if (p4 === 0 && p5 === 1) A++;
          if (p5 === 0 && p6 === 1) A++;
          if (p6 === 0 && p7 === 1) A++;
          if (p7 === 0 && p8 === 1) A++;
          if (p8 === 0 && p9 === 1) A++;
          if (p9 === 0 && p2 === 1) A++;
          if (A !== 1) continue;

          if (p2 * p4 * p6 !== 0) continue;
          if (p4 * p6 * p8 !== 0) continue;

          del1.push(y * w + x);
        }
      }
      for (let i = 0; i < del1.length; i++) {
        grid[del1[i]] = 0;
        changed = true;
      }

      // Pass 2
      const del2 = [];
      for (let y = 1; y < h - 1; y++) {
        for (let x = 1; x < w - 1; x++) {
          if (grid[y * w + x] === 0) continue;

          const p2 = getP(x, y - 1);
          const p3 = getP(x + 1, y - 1);
          const p4 = getP(x + 1, y);
          const p5 = getP(x + 1, y + 1);
          const p6 = getP(x, y + 1);
          const p7 = getP(x - 1, y + 1);
          const p8 = getP(x - 1, y);
          const p9 = getP(x - 1, y - 1);

          const B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
          if (B < 2 || B > 6) continue;

          let A = 0;
          if (p2 === 0 && p3 === 1) A++;
          if (p3 === 0 && p4 === 1) A++;
          if (p4 === 0 && p5 === 1) A++;
          if (p5 === 0 && p6 === 1) A++;
          if (p6 === 0 && p7 === 1) A++;
          if (p7 === 0 && p8 === 1) A++;
          if (p8 === 0 && p9 === 1) A++;
          if (p9 === 0 && p2 === 1) A++;
          if (A !== 1) continue;

          if (p2 * p4 * p8 !== 0) continue;
          if (p2 * p6 * p8 !== 0) continue;

          del2.push(y * w + x);
        }
      }
      for (let i = 0; i < del2.length; i++) {
        grid[del2[i]] = 0;
        changed = true;
      }
    }

    return grid;
  }

  // ── Bilateral filter ───────────────────────────────────────────────────────
  // Edge-preserving smoothing: blurs uniformly-coloured regions (skin, sky, paper)
  // while KEEPING sharp edges (face outline, eyelids, lips, hair boundary).
  // This is what prevents a face from being covered in texture edges.

  _bilateralFilter(src, w, h, sigmaS, sigmaR) {
    const radius = Math.ceil(sigmaS * 2);
    const twoSS  = 2 * sigmaS * sigmaS;
    const twoSR  = 2 * sigmaR * sigmaR;
    const dst    = new Float32Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const centre = src[y * w + x];
        let acc = 0, wSum = 0;
        for (let ky = -radius; ky <= radius; ky++) {
          for (let kx = -radius; kx <= radius; kx++) {
            const nx = Math.max(0, Math.min(w-1, x+kx));
            const ny = Math.max(0, Math.min(h-1, y+ky));
            const val = src[ny*w+nx];
            const wS  = Math.exp(-(kx*kx + ky*ky) / twoSS);
            const wR  = Math.exp(-(centre-val)*(centre-val) / twoSR);
            const wt  = wS * wR;
            acc  += val * wt;
            wSum += wt;
          }
        }
        dst[y*w+x] = acc / wSum;
      }
    }
    return dst;
  }

  // ── Separable Gaussian blur ────────────────────────────────────────────────

  _gaussianBlur(src, w, h, sigma) {
    const radius = Math.ceil(sigma * 3);
    const size   = 2 * radius + 1;
    const k = new Float32Array(size);
    let sum = 0;
    for (let i=0;i<size;i++) { const d=i-radius; k[i]=Math.exp(-(d*d)/(2*sigma*sigma)); sum+=k[i]; }
    for (let i=0;i<size;i++) k[i]/=sum;
    const tmp = new Float32Array(w*h), dst = new Float32Array(w*h);
    for (let y=0;y<h;y++) for (let x=0;x<w;x++) {
      let a=0; for (let ki=0;ki<size;ki++) { a+=src[y*w+Math.max(0,Math.min(w-1,x+ki-radius))]*k[ki]; } tmp[y*w+x]=a;
    }
    for (let y=0;y<h;y++) for (let x=0;x<w;x++) {
      let a=0; for (let ki=0;ki<size;ki++) { a+=tmp[Math.max(0,Math.min(h-1,y+ki-radius))*w+x]*k[ki]; } dst[y*w+x]=a;
    }
    return dst;
  }

  // ── XDoG (eXtended Difference of Gaussians) ───────────────────────────────
  // Produces output that resembles how an artist would sketch an image.
  // Unlike Canny which treats all gradients equally, XDoG is tuned to:
  //   • suppress fine texture (noise, skin pores, fabric weave)
  //   • preserve and strengthen contour-level boundaries (face outline, eyes, etc)
  //   • produce clean, connected strokes naturally

  _xdog(src, w, h, sigma, k, tau, phi, epsilon) {
    const g1  = this._gaussianBlur(src, w, h, sigma);
    const g2  = this._gaussianBlur(src, w, h, sigma * k);
    const out = new Uint8Array(w * h);
    for (let i = 0; i < w * h; i++) {
      // Normalised difference (DoG)
      const dog = (g1[i] - tau * g2[i]) / 255.0;
      // Soft threshold via tanh — creates an "ink" quality
      let val;
      if (dog >= epsilon) {
        val = 1.0;
      } else {
        val = 1.0 + Math.tanh(phi * (dog - epsilon));
      }
      // Edge pixel = value below 0.5 on the "paper" scale
      // (XDoG marks edges as dark strokes on white background)
      out[i] = val < 0.5 ? 1 : 0;
    }
    return out;
  }

  // ── Degree-aware path tracing ─────────────────────────────────────────────

  _traceStrokes(edges, w, h) {
    const dx8=[-1,0,1,-1,1,-1,0,1], dy8=[-1,-1,-1,0,0,1,1,1];
    const degree = new Uint8Array(w*h);
    for (let y=0;y<h;y++) for (let x=0;x<w;x++) {
      if (!edges[y*w+x]) continue;
      let deg=0;
      for (let d=0;d<8;d++) { const nx=x+dx8[d],ny=y+dy8[d]; if(nx>=0&&nx<w&&ny>=0&&ny<h&&edges[ny*w+nx]) deg++; }
      degree[y*w+x]=deg;
    }
    const visited=new Uint8Array(w*h), strokes=[];
    const follow=(startX,startY)=>{
      const stroke=[]; let cx=startX,cy=startY,prevDx=0,prevDy=0;
      while(true){
        const ci=cy*w+cx; if(visited[ci]) break;
        visited[ci]=1; stroke.push({x:cx,y:cy});
        let bestX=-1,bestY=-1,bestScore=-Infinity;
        for(let d=0;d<8;d++){
          const nx=cx+dx8[d],ny=cy+dy8[d];
          if(nx<0||nx>=w||ny<0||ny>=h) continue;
          const ni=ny*w+nx; if(!edges[ni]||visited[ni]) continue;
          const score=(prevDx!==0||prevDy!==0)?dx8[d]*prevDx+dy8[d]*prevDy:0;
          if(score>bestScore){bestScore=score;bestX=nx;bestY=ny;}
        }
        if(bestX===-1) break;
        prevDx=bestX-cx; prevDy=bestY-cy; cx=bestX; cy=bestY;
      }
      return stroke;
    };
    // Endpoints first (natural stroke tips)
    for(let y=0;y<h;y++) for(let x=0;x<w;x++){
      const i=y*w+x; if(!edges[i]||visited[i]||degree[i]!==1) continue;
      const s=follow(x,y); if(s.length>=2) strokes.push(s);
    }
    // Remaining (loops, junctions)
    for(let y=0;y<h;y++) for(let x=0;x<w;x++){
      const i=y*w+x; if(!edges[i]||visited[i]) continue;
      const s=follow(x,y); if(s.length>=2) strokes.push(s);
    }
    return strokes;
  }

  // ── Douglas-Peucker ───────────────────────────────────────────────────────

  _douglasPeucker(pts, eps) {
    if (pts.length<=2) return pts;
    const f=pts[0],l=pts[pts.length-1];
    const len=Math.hypot(l.x-f.x,l.y-f.y)||1;
    let maxD=0,maxI=0;
    for(let i=1;i<pts.length-1;i++){
      const d=Math.abs((l.y-f.y)*pts[i].x-(l.x-f.x)*pts[i].y+l.x*f.y-l.y*f.x)/len;
      if(d>maxD){maxD=d;maxI=i;}
    }
    if(maxD>eps){
      return this._douglasPeucker(pts.slice(0,maxI+1),eps).slice(0,-1)
        .concat(this._douglasPeucker(pts.slice(maxI),eps));
    }
    return [f,l];
  }

  // ── Polyline Endpoint Chaining (Gaps & Fragment Bridging) ──────────────────
  // Glues nearby endpoint tips together into long continuous vector strokes,
  // completely eliminating broken "chicken scratch" fragments.

  _connectNearbyStrokes(strokes, maxDistPx) {
    if (!strokes || !strokes.length) return strokes;
    let list = strokes.filter(s => s && s.length >= 2);
    let merged = true;

    while (merged) {
      merged = false;
      for (let i = 0; i < list.length; i++) {
        if (!list[i]) continue;
        const s1 = list[i];
        const p1_end = s1[s1.length - 1];

        for (let j = 0; j < list.length; j++) {
          if (i === j || !list[j]) continue;
          const s2 = list[j];
          const p2_start = s2[0];
          const p2_end = s2[s2.length - 1];

          // Check End(s1) -> Start(s2)
          if (Math.hypot(p1_end.x - p2_start.x, p1_end.y - p2_start.y) <= maxDistPx) {
            list[i] = s1.concat(s2.slice(1));
            list[j] = null;
            merged = true;
            break;
          }
          // Check End(s1) -> End(s2)
          if (Math.hypot(p1_end.x - p2_end.x, p1_end.y - p2_end.y) <= maxDistPx) {
            const s2_rev = s2.slice().reverse();
            list[i] = s1.concat(s2_rev.slice(1));
            list[j] = null;
            merged = true;
            break;
          }
        }
        if (merged) break;
      }
      list = list.filter(Boolean);
    }

    return list;
  }

  // ── Chaikin Corner Smoothing (2D Vector Curve Fitting) ───────────────────
  // Smooths polygonal line steps into silky-smooth continuous 2D vector curves.

  _chaikinSmooth(stroke, iterations = 1) {
    if (!stroke || stroke.length <= 2) return stroke;
    let current = stroke;

    for (let it = 0; it < iterations; it++) {
      const smoothed = [];
      smoothed.push(current[0]);

      for (let i = 0; i < current.length - 1; i++) {
        const p0 = current[i];
        const p1 = current[i + 1];

        const q = { x: 0.75 * p0.x + 0.25 * p1.x, y: 0.75 * p0.y + 0.25 * p1.y };
        const r = { x: 0.25 * p0.x + 0.75 * p1.x, y: 0.25 * p0.y + 0.75 * p1.y };

        smoothed.push(q);
        smoothed.push(r);
      }

      smoothed.push(current[current.length - 1]);
      current = smoothed;
    }

    return current;
  }

  // ── Greedy nearest-neighbour stroke sort ─────────────────────────────────

  _sortStrokes(strokes) {
    if (!strokes.length) return strokes;
    const sorted=[],used=new Uint8Array(strokes.length);
    let curX=strokes[0][0].x,curY=strokes[0][0].y;
    for(let iter=0;iter<strokes.length;iter++){
      let bestIdx=-1,bestDist=Infinity,bestRev=false;
      for(let i=0;i<strokes.length;i++){
        if(used[i]||!strokes[i]||strokes[i].length<2) continue;
        const s=strokes[i];
        const ds=Math.hypot(s[0].x-curX,s[0].y-curY);
        const de=Math.hypot(s[s.length-1].x-curX,s[s.length-1].y-curY);
        const d=Math.min(ds,de);
        if(d<bestDist){bestDist=d;bestIdx=i;bestRev=(de<ds);}
      }
      if(bestIdx===-1) break;
      used[bestIdx]=1;
      const stroke=bestRev?strokes[bestIdx].slice().reverse():strokes[bestIdx];
      sorted.push(stroke);
      const last=stroke[stroke.length-1]; curX=last.x; curY=last.y;
    }
    return sorted;
  }

  // ── Preview ───────────────────────────────────────────────────────────────

  _renderPreview(strokes, resW, resH) {
    const cv=this.previewCanvas;
    const maxDim=500, SCALE=Math.max(1,Math.floor(Math.min(maxDim/resW,maxDim/resH)));
    cv.width=resW*SCALE; cv.height=resH*SCALE;
    const ctx=cv.getContext('2d');
    // White background (like paper) so the preview matches what will be drawn
    ctx.fillStyle='#f5f0e8';
    ctx.fillRect(0,0,cv.width,cv.height);
    ctx.strokeStyle='#1a1a2e';
    const strokeW = Math.max(1, (this._lineWidthMM || 0.8) * SCALE * 0.5);
    ctx.lineWidth = strokeW;
    ctx.lineCap='round'; ctx.lineJoin='round';
    for(const stroke of strokes){
      if(!stroke||stroke.length<2) continue;
      ctx.beginPath();
      ctx.moveTo(stroke[0].x*SCALE+SCALE/2,stroke[0].y*SCALE+SCALE/2);
      for(let i=1;i<stroke.length;i++) ctx.lineTo(stroke[i].x*SCALE+SCALE/2,stroke[i].y*SCALE+SCALE/2);
      ctx.stroke();
    }
  }

  renderLivePosition(currentWpIndex) {
    if (!this.previewCanvas || !this.strokes) return;
    const cv = this.previewCanvas;
    const ctx = cv.getContext('2d');
    const w = this.resW, h = this.resH;
    const maxDim = 500, SCALE = Math.max(1, Math.floor(Math.min(maxDim/w, maxDim/h)));

    ctx.fillStyle = '#f5f0e8';
    ctx.fillRect(0, 0, cv.width, cv.height);

    // 1. Draw light grey phantom strokes (what needs to be drawn)
    ctx.lineWidth = Math.max(1, SCALE * 0.5);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.strokeStyle = 'rgba(26, 26, 46, 0.12)';
    for (const stroke of this.strokes) {
      if (!stroke || stroke.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(stroke[0].x * SCALE + SCALE/2, stroke[0].y * SCALE + SCALE/2);
      for (let i = 1; i < stroke.length; i++) {
        ctx.lineTo(stroke[i].x * SCALE + SCALE/2, stroke[i].y * SCALE + SCALE/2);
      }
      ctx.stroke();
    }

    // 2. Draw what was drawn so far in black
    const wps = this.waypoints;
    ctx.strokeStyle = '#1a1a2e';
    ctx.lineWidth = Math.max(1.5, SCALE * 0.6);
    
    let pathStarted = false;
    const minX = this.minX, minY = this.minY;
    const wW = this.wW, wH = this.wH;

    const map = (wp) => {
      const col = ((wp.x - minX) / wW) * (w - 1);
      const row = ((wp.y - minY) / wH) * (h - 1);
      return { x: col * SCALE + SCALE/2, y: row * SCALE + SCALE/2 };
    };

    for (let i = 0; i <= currentWpIndex && i < wps.length; i++) {
      const wp = wps[i];
      const pt = map(wp);
      if (wp.z <= this._lastPenDownZ + 0.1) {
        if (!pathStarted) {
          ctx.beginPath();
          ctx.moveTo(pt.x, pt.y);
          pathStarted = true;
        } else {
          ctx.lineTo(pt.x, pt.y);
        }
      } else {
        if (pathStarted) {
          ctx.stroke();
          pathStarted = false;
        }
      }
    }
    if (pathStarted) ctx.stroke();

    // 3. Draw crosshair cursor at current location
    if (currentWpIndex >= 0 && currentWpIndex < wps.length) {
      const curWp = wps[currentWpIndex];
      const pt = map(curWp);
      const penDown = curWp.z <= this._lastPenDownZ + 0.1;
      const color = penDown ? '#ff0055' : '#00bfff';

      // Ring
      const pulse = 6 + Math.sin(performance.now() / 80) * 1.5;
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, pulse, 0, 2 * Math.PI);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Dot
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, 2, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();
    }
  }

  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;
    this._stopped = false;
    this._pendingOkResolvers = [];
    let inFlight = 0;
    const maxInFlight = 2; // Allow printer lookahead for smooth movement

    // Helper function to wait until an 'ok' frees a slot
    const waitOk = () => new Promise(resolve => this._pendingOkResolvers.push(resolve));

    for (let i = 0; i < wps.length; i++) {
      if (this._stopped || (isCancelled && isCancelled())) break;

      if (this._paused) {
        await new Promise(resolve => { this._resumeResolve = resolve; });
      }
      if (this._stopped) break;

      // Wait if the queue of in-flight commands is full
      while (inFlight >= maxInFlight && !this._stopped) {
        await waitOk();
        inFlight = Math.max(0, inFlight - 1);
      }
      if (this._stopped) break;

      const wp = wps[i];
      const isZOnly = i > 0 &&
        Math.abs(wps[i-1].x - wp.x) < 0.01 &&
        Math.abs(wps[i-1].y - wp.y) < 0.01;
      const f = isZOnly ? feedrateZ : feedrateXY;

      inFlight++;
      socket.send(`gcode-plot:G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${f}`);
      
      if (onProgress) onProgress(i + 1, wps.length, i);
    }

    // Wait for all remaining in-flight moves to finish executing
    while (inFlight > 0 && !this._stopped) {
      await waitOk();
      inFlight = Math.max(0, inFlight - 1);
    }

    // Lift pen safely at end regardless of how we stopped
    const last = wps[wps.length-1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z${this._lastPenUpZ+5} F${feedrateXY}`);
  }
}

export class GenerativePlotter {
  constructor() {
    this.waypoints = [];
    this.previewCanvas = null;
    this._paused = false;
    this._stopped = false;
    this._resumeResolve = null;
    this._pendingOkResolvers = [];
    this._lastPenUpZ = 15;
    this._lastPenDownZ = 0;
    this.strokes = [];
  }

  pause()  { this._paused = true; }
  resume() {
    this._paused = false;
    if (this._resumeResolve) { this._resumeResolve(); this._resumeResolve = null; }
  }
  stop()   { 
    this._stopped = true; 
    this.resume(); 
    const resolvers = this._pendingOkResolvers;
    this._pendingOkResolvers = [];
    for (const r of resolvers) r();
  }

  generate(patternType, p1, p2, workspace, penDownZ, penUpZ) {
    this._lastPenUpZ = penUpZ;
    this._lastPenDownZ = penDownZ;
    this.strokes = [];
    
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const maxR = Math.min(workspaceWidth, workspaceHeight) / 2.0;

    if (patternType === 'archimedean') {
      const revs = p1; // e.g. 20
      const scale = p2; // e.g. 90 (mm outer radius)
      const stroke = [];
      const steps = 1000;
      const maxTheta = revs * 2 * Math.PI;
      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * maxTheta;
        const r = (theta / maxTheta) * scale;
        const x = centerX + r * Math.cos(theta);
        const y = centerY + r * Math.sin(theta);
        stroke.push({ x, y });
      }
      this.strokes.push(stroke);
    } 
    else if (patternType === 'fermat') {
      const revs = p1;
      const scale = p2;
      const maxTheta = revs * 2 * Math.PI;
      const steps = 800;
      
      // Positive branch
      const s1 = [];
      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * maxTheta;
        const r = scale * Math.sqrt(theta / maxTheta);
        const x = centerX + r * Math.cos(theta);
        const y = centerY + r * Math.sin(theta);
        s1.push({ x, y });
      }
      this.strokes.push(s1);

      // Negative branch
      const s2 = [];
      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * maxTheta;
        const r = -scale * Math.sqrt(theta / maxTheta);
        const x = centerX + r * Math.cos(theta);
        const y = centerY + r * Math.sin(theta);
        s2.push({ x, y });
      }
      this.strokes.push(s2);
    } 
    else if (patternType === 'spirograph') {
      // Spirograph: p1 = inner gear radius (r), p2 = pen offset (d)
      // R = fixed to outer ring radius (e.g. 90mm)
      const R = maxR * 0.95;
      const r = p1;
      const d = p2;
      const stroke = [];
      
      const gcd = (a, b) => b ? gcd(b, a % b) : a;
      const common = gcd(Math.round(r * 10), Math.round(R * 10)) / 10;
      const lcm = (r * R) / common;
      const maxTheta = (2 * Math.PI * lcm) / R;
      const steps = Math.min(3000, Math.max(500, Math.round(maxTheta * 20)));

      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * maxTheta;
        const x = centerX + (R - r) * Math.cos(theta) + d * Math.cos((R - r) * theta / r);
        const y = centerY + (R - r) * Math.sin(theta) - d * Math.sin((R - r) * theta / r);
        stroke.push({ x, y });
      }
      this.strokes.push(stroke);
    } 
    else if (patternType === 'rose') {
      const n = p1; // multiplier
      const scale = p2; // Scale
      const stroke = [];
      const steps = 1000;
      const limitTheta = Math.PI * (n % 2 === 0 ? 2 : 1);
      
      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * limitTheta;
        const r = scale * Math.cos(n * theta);
        const x = centerX + r * Math.cos(theta);
        const y = centerY + r * Math.sin(theta);
        stroke.push({ x, y });
      }
      this.strokes.push(stroke);
    } 
    else if (patternType === 'lissajous') {
      const fX = p1;
      const fY = p2;
      const scaleX = maxR * 0.9;
      const scaleY = maxR * 0.9;
      const stroke = [];
      const steps = 1000;
      const maxTheta = 2 * Math.PI;

      for (let i = 0; i <= steps; i++) {
        const theta = (i / steps) * maxTheta;
        const x = centerX + scaleX * Math.sin(fX * theta + Math.PI / 4);
        const y = centerY + scaleY * Math.sin(fY * theta);
        stroke.push({ x, y });
      }
      this.strokes.push(stroke);
    }
    else if (patternType.startsWith('rotate_')) {
      const numCopies = Math.max(3, Math.min(120, Math.round(p1)));
      const scale = p2;
      const baseShape = patternType.replace('rotate_', '');
      
      // Generate base 2D shape points around (0,0)
      const basePts = [];
      if (baseShape === 'square') {
        for (let i = 0; i <= 4; i++) {
          const a = (i / 4) * 2 * Math.PI + Math.PI / 4;
          basePts.push({ x: scale * Math.cos(a), y: scale * Math.sin(a) });
        }
      } else if (baseShape === 'triangle') {
        for (let i = 0; i <= 3; i++) {
          const a = (i / 3) * 2 * Math.PI - Math.PI / 2;
          basePts.push({ x: scale * Math.cos(a), y: scale * Math.sin(a) });
        }
      } else if (baseShape === 'hexagon') {
        for (let i = 0; i <= 6; i++) {
          const a = (i / 6) * 2 * Math.PI;
          basePts.push({ x: scale * Math.cos(a), y: scale * Math.sin(a) });
        }
      } else if (baseShape === 'star') {
        for (let i = 0; i <= 10; i++) {
          const a = (i / 10) * 2 * Math.PI - Math.PI / 2;
          const r = (i % 2 === 0) ? scale : scale * 0.45;
          basePts.push({ x: r * Math.cos(a), y: r * Math.sin(a) });
        }
      } else if (baseShape === 'ellipse') {
        const steps = 60;
        for (let i = 0; i <= steps; i++) {
          const a = (i / steps) * 2 * Math.PI;
          basePts.push({ x: scale * Math.cos(a), y: (scale * 0.4) * Math.sin(a) });
        }
      } else if (baseShape === 'heart') {
        const steps = 80;
        for (let i = 0; i <= steps; i++) {
          const t = (i / steps) * 2 * Math.PI;
          const hx = 16 * Math.pow(Math.sin(t), 3);
          const hy = -(13 * Math.cos(t) - 5 * Math.cos(2*t) - 2 * Math.cos(3*t) - Math.cos(4*t));
          basePts.push({ x: (hx / 16) * scale, y: (hy / 17) * scale });
        }
      }

      // Generate N rotated copies around center
      for (let k = 0; k < numCopies; k++) {
        const phi = (k / numCopies) * 2 * Math.PI;
        const cosP = Math.cos(phi);
        const sinP = Math.sin(phi);
        const stroke = [];
        for (const pt of basePts) {
          const rx = pt.x * cosP - pt.y * sinP + centerX;
          const ry = pt.x * sinP + pt.y * cosP + centerY;
          stroke.push({ x: rx, y: ry });
        }
        this.strokes.push(stroke);
      }
    }

    // Now map strokes to waypoints
    this.waypoints = [];
    for (const stroke of this.strokes) {
      if (stroke.length < 2) continue;
      this.waypoints.push({ x: stroke[0].x, y: stroke[0].y, z: penUpZ });
      this.waypoints.push({ x: stroke[0].x, y: stroke[0].y, z: penDownZ });
      for (let i = 1; i < stroke.length; i++) {
        this.waypoints.push({ x: stroke[i].x, y: stroke[i].y, z: penDownZ });
      }
      const last = stroke[stroke.length - 1];
      this.waypoints.push({ x: last.x, y: last.y, z: penUpZ });
    }

    if (this.previewCanvas) {
      this._renderPreview(workspace);
    }

    return this.waypoints.length;
  }

  _renderPreview(workspace) {
    const cv = this.previewCanvas;
    const ctx = cv.getContext('2d');
    cv.width = 400;
    cv.height = 400;

    ctx.fillStyle = '#0c0f16';
    ctx.fillRect(0, 0, cv.width, cv.height);

    ctx.strokeStyle = '#39ff14'; // neon green
    ctx.lineWidth = 1.5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth / 2;
    const minY = centerY - workspaceHeight / 2;

    const toCanvasX = x => ((x - minX) / workspaceWidth) * cv.width;
    const toCanvasY = y => ((y - minY) / workspaceHeight) * cv.height;

    for (const stroke of this.strokes) {
      if (stroke.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(toCanvasX(stroke[0].x), toCanvasY(stroke[0].y));
      for (let i = 1; i < stroke.length; i++) {
        ctx.lineTo(toCanvasX(stroke[i].x), toCanvasY(stroke[i].y));
      }
      ctx.stroke();
    }
  }

  renderLivePosition(currentWpIndex, workspace) {
    if (!this.previewCanvas || !this.strokes) return;
    const cv = this.previewCanvas;
    const ctx = cv.getContext('2d');
    
    ctx.fillStyle = '#0c0f16';
    ctx.fillRect(0, 0, cv.width, cv.height);

    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth / 2;
    const minY = centerY - workspaceHeight / 2;
    const toCanvasX = x => ((x - minX) / workspaceWidth) * cv.width;
    const toCanvasY = y => ((y - minY) / workspaceHeight) * cv.height;

    // 1. Draw target outlines in faint neon green
    ctx.strokeStyle = 'rgba(57, 255, 20, 0.15)';
    ctx.lineWidth = 1.5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    for (const stroke of this.strokes) {
      if (stroke.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(toCanvasX(stroke[0].x), toCanvasY(stroke[0].y));
      for (let i = 1; i < stroke.length; i++) {
        ctx.lineTo(toCanvasX(stroke[i].x), toCanvasY(stroke[i].y));
      }
      ctx.stroke();
    }

    // 2. Draw drawn segment so far in bright neon green
    ctx.strokeStyle = '#39ff14';
    ctx.lineWidth = 2.0;
    const wps = this.waypoints;
    let pathStarted = false;
    for (let i = 0; i <= currentWpIndex && i < wps.length; i++) {
      const wp = wps[i];
      const cx = toCanvasX(wp.x);
      const cy = toCanvasY(wp.y);
      if (wp.z <= this._lastPenDownZ + 0.1) {
        if (!pathStarted) {
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          pathStarted = true;
        } else {
          ctx.lineTo(cx, cy);
        }
      } else {
        if (pathStarted) {
          ctx.stroke();
          pathStarted = false;
        }
      }
    }
    if (pathStarted) ctx.stroke();

    // 3. Draw pulsing cursor
    if (currentWpIndex >= 0 && currentWpIndex < wps.length) {
      const curWp = wps[currentWpIndex];
      const cx = toCanvasX(curWp.x);
      const cy = toCanvasY(curWp.y);
      const penDown = curWp.z <= this._lastPenDownZ + 0.1;
      const color = penDown ? '#ff0055' : '#00bfff';

      const pulse = 6 + Math.sin(performance.now() / 80) * 1.5;
      ctx.beginPath();
      ctx.arc(cx, cy, pulse, 0, 2 * Math.PI);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(cx, cy, 2, 0, 2 * Math.PI);
      ctx.fillStyle = color;
      ctx.fill();
    }
  }

  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;
    this._stopped = false;
    this._pendingOkResolvers = [];
    let inFlight = 0;
    const maxInFlight = 2;

    const waitOk = () => new Promise(resolve => this._pendingOkResolvers.push(resolve));

    for (let i = 0; i < wps.length; i++) {
      if (this._stopped || (isCancelled && isCancelled())) break;

      if (this._paused) {
        await new Promise(resolve => { this._resumeResolve = resolve; });
      }
      if (this._stopped) break;

      while (inFlight >= maxInFlight && !this._stopped) {
        await waitOk();
        inFlight = Math.max(0, inFlight - 1);
      }
      if (this._stopped) break;

      const wp = wps[i];
      const isZOnly = i > 0 &&
        Math.abs(wps[i-1].x - wp.x) < 0.01 &&
        Math.abs(wps[i-1].y - wp.y) < 0.01;
      const f = isZOnly ? feedrateZ : feedrateXY;

      inFlight++;
      socket.send(`gcode-plot:G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${f}`);
      
      if (onProgress) onProgress(i + 1, wps.length, i);
    }

    while (inFlight > 0 && !this._stopped) {
      await waitOk();
      inFlight = Math.max(0, inFlight - 1);
    }

    const last = wps[wps.length-1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z${this._lastPenUpZ+5} F${feedrateXY}`);
  }
}
