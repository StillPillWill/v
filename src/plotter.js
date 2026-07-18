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
  }

  // ── Streaming control API ──────────────────────────────────────────────────
  pause()  { this._paused = true; }
  resume() {
    this._paused = false;
    if (this._resumeResolve) { this._resumeResolve(); this._resumeResolve = null; }
  }
  stop()   { this._stopped = true; this.resume(); } // unblock if paused

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
   * @param {number} threshold  — XDoG edge threshold; lower = more edges (1–50)
   * @param {number} minStroke  — minimum stroke length in pixels to keep (2–20)
   * @param {number} simplification — Douglas-Peucker epsilon tolerance (0.1–10.0)
   * @param {object} workspace
   * @param {number} penDownZ
   * @param {number} penUpZ
   */
  process(resolution, threshold, minStroke, simplification, workspace, penDownZ, penUpZ) {
    if (!this.imageBitmap) throw new Error('No image loaded');
    this._lastPenUpZ = penUpZ;

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

    // ── C: CLAHE ───────────────────────────────────────────────────────────
    const clahe = this._clahe(gray, resW, resH, 8, 8, 3.0);

    // ── D: Bilateral filter ────────────────────────────────────────────────
    // sigmaS=2 (spatial radius), sigmaR=30 (color range)
    // Key: preserves OBJECT EDGES while destroying skin texture / noise
    const bilateral = this._bilateralFilter(clahe, resW, resH, 2.0, 30.0);

    // ── E: XDoG at two scales → union ─────────────────────────────────────
    // XDoG is designed for artistic sketch output — understands perceptual
    // structure rather than just pixel gradients.
    const xdogThresh = Math.max(0.001, threshold / 1000.0); // normalise to [0-0.1]
    const sketchFine   = this._xdog(bilateral, resW, resH, 0.8, 1.6, 0.97, 100.0, xdogThresh);
    const sketchMedium = this._xdog(bilateral, resW, resH, 1.6, 1.6, 0.97,  80.0, xdogThresh * 0.7);

    const sketch = new Uint8Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) sketch[i] = sketchFine[i] | sketchMedium[i];

    // ── F–I: Trace, filter, simplify, sort ────────────────────────────────
    const rawStrokes  = this._traceStrokes(sketch, resW, resH);
    const filtered    = rawStrokes.filter(s => s.length >= Math.max(2, minStroke));
    const simplified  = filtered.map(s => this._douglasPeucker(s, simplification)).filter(s => s.length >= 2);
    const sorted      = this._sortStrokes(simplified);

    // ── J: Map to printer coordinates ─────────────────────────────────────
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth  / 2;
    const minY = centerY - workspaceHeight / 2;
    const toX  = col => minX + (col / (resW - 1)) * workspaceWidth;
    const toY  = row => minY + (row / (resH - 1)) * workspaceHeight;

    const waypoints = [];
    for (const stroke of sorted) {
      if (!stroke || stroke.length < 2) continue;
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penUpZ   });
      waypoints.push({ x: toX(stroke[0].x), y: toY(stroke[0].y), z: penDownZ });
      for (let i = 1; i < stroke.length; i++) {
        waypoints.push({ x: toX(stroke[i].x), y: toY(stroke[i].y), z: penDownZ });
      }
      const last = stroke[stroke.length - 1];
      waypoints.push({ x: toX(last.x), y: toY(last.y), z: penUpZ });
    }

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
        if(stroke.length>1 && degree[ci]>=3) break;
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
    ctx.lineWidth=Math.max(1,SCALE*0.5);
    ctx.lineCap='round'; ctx.lineJoin='round';
    for(const stroke of strokes){
      if(!stroke||stroke.length<2) continue;
      ctx.beginPath();
      ctx.moveTo(stroke[0].x*SCALE+SCALE/2,stroke[0].y*SCALE+SCALE/2);
      for(let i=1;i<stroke.length;i++) ctx.lineTo(stroke[i].x*SCALE+SCALE/2,stroke[i].y*SCALE+SCALE/2);
      ctx.stroke();
    }
  }

  // ── G-code streaming with pause/resume/stop ───────────────────────────────

  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;
    this._stopped = false;

    for (let i = 0; i < wps.length; i++) {
      // Check stop
      if (this._stopped || (isCancelled && isCancelled())) break;

      // Check pause — await until resume() is called
      if (this._paused) {
        await new Promise(resolve => { this._resumeResolve = resolve; });
      }
      if (this._stopped) break;

      const wp = wps[i];
      const isZOnly = i > 0 &&
        Math.abs(wps[i-1].x - wp.x) < 0.01 &&
        Math.abs(wps[i-1].y - wp.y) < 0.01;
      const f = isZOnly ? feedrateZ : feedrateXY;
      socket.send(`gcode-plot:G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${f}`);
      if (onProgress) onProgress(i+1, wps.length);
      if (i % 5 === 0) await new Promise(r => setTimeout(r, 0));
    }

    // Lift pen safely at end regardless of how we stopped
    const last = wps[wps.length-1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z${this._lastPenUpZ+5} F${feedrateXY}`);
  }
}
