export class ScrollingChart {
  constructor(maxSamples = 150) {
    this.maxSamples = maxSamples;
    this.targetHistory = [];
    this.actualHistory = [];
  }

  addSample(target, actual) {
    this.targetHistory.push(target);
    this.actualHistory.push(actual);

    if (this.targetHistory.length > this.maxSamples) {
      this.targetHistory.shift();
      this.actualHistory.shift();
    }
  }

  clear() {
    this.targetHistory = [];
    this.actualHistory = [];
  }

  /**
   * Renders the scrolling chart onto an HTML5 canvas
   * @param {HTMLCanvasElement} canvas
   * @param {string} targetColor - Hex/CSS color for the target line
   * @param {string} actualColor - Hex/CSS color for the actual line
   */
  draw(canvas, targetColor = '#00f0ff', actualColor = '#ff007f') {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    
    // Sync internal canvas size with display size for sharp rendering
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    // Clear background
    ctx.fillStyle = '#0f111a';
    ctx.fillRect(0, 0, width, height);

    const len = this.targetHistory.length;
    if (len < 2) return;

    // Draw background grid lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    const gridLines = 4;
    for (let i = 1; i < gridLines; i++) {
      const y = (height / gridLines) * i;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    // Compute dynamic min/max scaling (with a minimum range to prevent hyper-scaling flat lines)
    let minVal = Math.min(...this.targetHistory, ...this.actualHistory);
    let maxVal = Math.max(...this.targetHistory, ...this.actualHistory);
    
    const minRange = 10.0; // minimum range of 10 degrees/units
    const center = (minVal + maxVal) / 2;
    if (maxVal - minVal < minRange) {
      minVal = center - minRange / 2;
      maxVal = center + minRange / 2;
    } else {
      // Add padding
      const pad = (maxVal - minVal) * 0.1;
      minVal -= pad;
      maxVal += pad;
    }

    const valueRange = maxVal - minVal;

    // Helper to map data points to canvas coordinates
    const getX = (index) => (index / (this.maxSamples - 1)) * width;
    const getY = (val) => height - ((val - minVal) / valueRange) * height;

    // Draw Target Line (Dashed neon)
    ctx.strokeStyle = targetColor;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(getX(0), getY(this.targetHistory[0]));
    for (let i = 1; i < len; i++) {
      ctx.lineTo(getX(i), getY(this.targetHistory[i]));
    }
    ctx.stroke();
    ctx.setLineDash([]); // Reset line dash

    // Draw Actual Line (Solid neon with glowing shadow)
    ctx.strokeStyle = actualColor;
    ctx.lineWidth = 2.0;
    
    // Canvas glow effect
    ctx.shadowBlur = 4;
    ctx.shadowColor = actualColor;
    
    ctx.beginPath();
    ctx.moveTo(getX(0), getY(this.actualHistory[0]));
    for (let i = 1; i < len; i++) {
      ctx.lineTo(getX(i), getY(this.actualHistory[i]));
    }
    ctx.stroke();
    
    // Reset shadow
    ctx.shadowBlur = 0;
  }
}
