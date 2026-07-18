/**
 * ImagePlotter — converts an image to marker-friendly scanline G-code paths.
 *
 * Pipeline:
 *  1. Load image → draw to hidden canvas at low resolution
 *  2. Extract RGBA pixel data
 *  3. Convert to luminance (perceptual grayscale)
 *  4. Apply CLAHE-style adaptive normalization to fight bright lighting bias
 *  5. Gaussian-blur to kill fine noise that would waste ink with a marker
 *  6. Binarize with per-row median threshold (lighting-robust)
 *  7. Trace horizontal scanlines → contiguous "pen-down" segments
 *  8. Render colour-coded preview to an output canvas
 *  9. Expose path as array of {x, y, z} waypoints in printer coordinates
 */
export class ImagePlotter {
  constructor() {
    this.waypoints = [];       // [{x, y, z}, ...]  printer coords
    this.processCanvas = document.createElement('canvas');
    this.previewCanvas = null; // set externally (document canvas)
    this.imageBitmap = null;
  }

  // ── 1. Image ingest ───────────────────────────────────────────────────────

  /** Load from a File/Blob object (file picker or drag-drop) */
  loadFromFile(file) {
    return new Promise((resolve, reject) => {
      const url = URL.createObjectURL(file);
      const img = new Image();
      img.onload = () => { this.imageBitmap = img; URL.revokeObjectURL(url); resolve(); };
      img.onerror = reject;
      img.src = url;
    });
  }

  /** Load from a video element (camera snapshot) */
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

  // ── 2. Core processing ────────────────────────────────────────────────────

  /**
   * Process the loaded image.
   * @param {number} numLines   – number of scanlines across the image (determines "resolution")
   * @param {number} threshold  – global threshold hint [0-255]; actual per-row threshold adapts around this
   * @param {object} workspace  – { centerX, centerY, workspaceWidth, workspaceHeight }
   * @param {number} penDownZ   – Z coord when pen touches paper
   * @param {number} penUpZ     – Z coord when pen is lifted
   * @returns {number}  total waypoint count
   */
  process(numLines, threshold, workspace, penDownZ, penUpZ) {
    if (!this.imageBitmap) throw new Error('No image loaded');

    // ── Step A: render to low-res processing canvas ────────────────────────
    //   We sample at (numLines * aspect) × numLines pixels.
    //   This coarse grid naturally matches a thick marker's line width.
    const aspect = this.imageBitmap.width / this.imageBitmap.height;
    const resH = numLines;
    const resW = Math.max(8, Math.round(numLines * aspect));

    const pc = this.processCanvas;
    pc.width  = resW;
    pc.height = resH;
    const ctx = pc.getContext('2d', { willReadFrequently: true });

    // Fill white so any transparent areas are treated as blank paper
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, resW, resH);
    ctx.drawImage(this.imageBitmap, 0, 0, resW, resH);

    const imgData = ctx.getImageData(0, 0, resW, resH);
    const px      = imgData.data; // RGBA flat array

    // ── Step B: convert to luminance ───────────────────────────────────────
    const lum = new Uint8Array(resW * resH);
    for (let i = 0; i < resW * resH; i++) {
      const r = px[i * 4];
      const g = px[i * 4 + 1];
      const b = px[i * 4 + 2];
      // Perceptual luminance (Rec 709)
      lum[i] = Math.round(0.2126 * r + 0.7152 * g + 0.0722 * b);
    }

    // ── Step C: CLAHE-lite — stretch contrast per-block so bright lighting
    //   doesn't wash out everything.  We use a global histogram stretch
    //   (percentile clipping) which is simple, fast, and very effective.
    const sorted  = Float32Array.from(lum).sort();
    const clipLow  = sorted[Math.floor(sorted.length * 0.02)] || 0;
    const clipHigh = sorted[Math.floor(sorted.length * 0.98)] || 255;
    const range    = Math.max(1, clipHigh - clipLow);

    const norm = new Uint8Array(resW * resH);
    for (let i = 0; i < lum.length; i++) {
      norm[i] = Math.round(Math.max(0, Math.min(255, ((lum[i] - clipLow) / range) * 255)));
    }

    // ── Step D: Gaussian blur (3×3) to kill single-pixel marker noise ──────
    const blurred = new Uint8Array(resW * resH);
    const kernel  = [1, 2, 1, 2, 4, 2, 1, 2, 1]; // unnormalised 3×3 Gaussian
    const kSum    = 16;
    for (let y = 0; y < resH; y++) {
      for (let x = 0; x < resW; x++) {
        let acc = 0;
        let ki  = 0;
        for (let ky = -1; ky <= 1; ky++) {
          for (let kx = -1; kx <= 1; kx++) {
            const sx = Math.max(0, Math.min(resW - 1, x + kx));
            const sy = Math.max(0, Math.min(resH - 1, y + ky));
            acc += norm[sy * resW + sx] * kernel[ki++];
          }
        }
        blurred[y * resW + x] = Math.round(acc / kSum);
      }
    }

    // ── Step E: binarize — per-row adaptive threshold ──────────────────────
    //   For each row we compute the median luminance and use a weighted
    //   combination of that median and the user-supplied global threshold.
    //   This means a row that is overall bright (e.g. lit from the side)
    //   will still correctly detect dark marks.
    const binary = new Uint8Array(resW * resH); // 1 = dark (pen-down)
    for (let y = 0; y < resH; y++) {
      const rowPixels = [];
      for (let x = 0; x < resW; x++) {
        rowPixels.push(blurred[y * resW + x]);
      }
      rowPixels.sort((a, b) => a - b);
      const rowMedian = rowPixels[Math.floor(rowPixels.length / 2)];
      // Blend: 60% local median, 40% global threshold
      const rowThreshold = Math.round(rowMedian * 0.6 + threshold * 0.4);

      for (let x = 0; x < resW; x++) {
        binary[y * resW + x] = blurred[y * resW + x] < rowThreshold ? 1 : 0;
      }
    }

    // ── Step F: build waypoints ────────────────────────────────────────────
    //   Each scanline row maps to one Y coordinate in printer space.
    //   Within a row, we trace contiguous dark-pixel runs as pen-down segments.
    //   Alternate rows sweep left→right and right→left (boustrophedon) to
    //   minimise rapid travel.
    const { centerX, centerY, workspaceWidth, workspaceHeight } = workspace;
    const minX = centerX - workspaceWidth  / 2.0;
    const maxX = centerX + workspaceWidth  / 2.0;
    const minY = centerY - workspaceHeight / 2.0;
    const maxY = centerY + workspaceHeight / 2.0;

    const waypoints = [];

    const toX = (col, reverse) => {
      const t = reverse ? (resW - 1 - col) / (resW - 1) : col / (resW - 1);
      return minX + t * workspaceWidth;
    };
    const toY = (row) => {
      // row 0 = top of image = higher Y in printer space (Y increases downward on the bed)
      return minY + (row / (resH - 1)) * workspaceHeight;
    };

    for (let row = 0; row < resH; row++) {
      const reverse = (row % 2) === 1; // boustrophedon sweep
      const y = toY(row);

      // Build ordered column sequence for this row
      const cols = [];
      for (let x = 0; x < resW; x++) cols.push(x);
      if (reverse) cols.reverse();

      let penDown = false;
      for (let ci = 0; ci < cols.length; ci++) {
        const col   = cols[ci];
        const isDark = binary[row * resW + col] === 1;
        const printerX = toX(col, reverse);

        if (isDark && !penDown) {
          // Lift before travel, arrive, then put pen down
          if (waypoints.length > 0) {
            // insert pen-up at current position before move
            const last = waypoints[waypoints.length - 1];
            waypoints.push({ x: last.x, y: last.y, z: penUpZ });
          }
          waypoints.push({ x: printerX, y, z: penUpZ }); // travel
          waypoints.push({ x: printerX, y, z: penDownZ }); // pen down
          penDown = true;
        } else if (!isDark && penDown) {
          waypoints.push({ x: printerX, y, z: penDownZ }); // end of segment
          waypoints.push({ x: printerX, y, z: penUpZ });   // lift
          penDown = false;
        } else if (isDark) {
          waypoints.push({ x: printerX, y, z: penDownZ });
        }
      }

      // End of row: always lift
      if (penDown && waypoints.length > 0) {
        const last = waypoints[waypoints.length - 1];
        waypoints.push({ x: last.x, y: last.y, z: penUpZ });
      }
    }

    this.waypoints = waypoints;

    // ── Step G: render preview canvas ─────────────────────────────────────
    if (this.previewCanvas) {
      this._renderPreview(binary, resW, resH, waypoints, minX, maxX, minY, maxY);
    }

    return waypoints.length;
  }

  // ── 3. Preview render ─────────────────────────────────────────────────────

  _renderPreview(binary, resW, resH, waypoints, minX, maxX, minY, maxY) {
    const cv  = this.previewCanvas;
    // Make preview square-ish at 300px wide
    const SCALE = 2;
    cv.width  = resW * SCALE;
    cv.height = resH * SCALE;
    const ctx = cv.getContext('2d');

    // Background
    ctx.fillStyle = '#0c0f16';
    ctx.fillRect(0, 0, cv.width, cv.height);

    // Draw binary map (dark pixels = ink)
    for (let y = 0; y < resH; y++) {
      for (let x = 0; x < resW; x++) {
        if (binary[y * resW + x] === 1) {
          ctx.fillStyle = 'rgba(0, 230, 255, 0.6)';
          ctx.fillRect(x * SCALE, y * SCALE, SCALE, SCALE);
        }
      }
    }

    // Draw planned pen paths on top
    const xRange = maxX - minX;
    const yRange = maxY - minY;

    ctx.lineWidth = 1;
    ctx.strokeStyle = '#39ff14';
    ctx.beginPath();
    let pathStarted = false;
    for (const wp of waypoints) {
      const px = ((wp.x - minX) / xRange) * cv.width;
      const py = ((wp.y - minY) / yRange) * cv.height;
      if (wp.z <= 5) { // pen down (approximate)
        if (!pathStarted) { ctx.moveTo(px, py); pathStarted = true; }
        else ctx.lineTo(px, py);
      } else {
        ctx.stroke();
        ctx.beginPath();
        pathStarted = false;
      }
    }
    ctx.stroke();
  }

  // ── 4. G-code streaming ───────────────────────────────────────────────────

  /**
   * Stream all waypoints to the printer via WebSocket.
   * Uses a step-by-step async loop so the UI stays responsive.
   * @param {WebSocket} socket
   * @param {Function} onProgress  (current, total) => void
   * @param {Function} isCancelled () => bool
   */
  async streamToPlotter(socket, feedrateXY, feedrateZ, onProgress, isCancelled) {
    const wps = this.waypoints;
    if (!wps.length) return;

    for (let i = 0; i < wps.length; i++) {
      if (isCancelled && isCancelled()) break;
      const wp = wps[i];
      // Use 'gcode-plot:' prefix so the server queues these in order
      // instead of overwriting them with the live hand-tracking G1 commands
      const gcode = `G1 X${wp.x.toFixed(1)} Y${wp.y.toFixed(1)} Z${wp.z.toFixed(1)} F${feedrateXY}`;
      socket.send(`gcode-plot:${gcode}`);
      if (onProgress) onProgress(i + 1, wps.length);
      // Small yield to keep browser painting and allow cancellation
      if (i % 5 === 0) await new Promise(r => setTimeout(r, 0));
    }

    // Lift pen at end
    const last = wps[wps.length - 1];
    socket.send(`gcode-plot:G1 X${last.x.toFixed(1)} Y${last.y.toFixed(1)} Z20 F${feedrateXY}`);
  }
}
