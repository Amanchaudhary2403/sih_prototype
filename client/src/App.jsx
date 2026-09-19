import React, { useState, useEffect, useRef, useCallback } from 'react';
import { io } from 'socket.io-client';

const socket = io({
  transports: ['websocket', 'polling']
});

const SCENARIOS = [
  { id: 1, name: '[1] Stray Cattle - Static cow resting in the right ...' },
  { id: 2, name: '[2] Slow Rickshaw - Auto-rickshaw cruising at 3 m/s ...' },
  { id: 3, name: '[3] Pedestrian Crossing - Pedestrian crossing from left sh...' },
  { id: 4, name: '[4] Potholes & Bullock Cart - Two potholes + slow bullock cart...' },
  { id: 5, name: '[5] Multi-Hazard Gauntlet - Cow at 40 m, slow truck at 65 m ...' },
  { id: 6, name: '[6] Stray Dog + Parked Van - Dog wandering across road at 45 ...' },
];

function App() {
  const [activeScenario, setActiveScenario] = useState(5);
  const [frameData, setFrameData] = useState(null);
  const [paused, setPaused] = useState(false);
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const latestFrame = useRef(null);
  const rafId = useRef(null);

  // Connect to socket and store latest frame in a ref (no re-render per frame)
  useEffect(() => {
    socket.on('frame', (data) => {
      latestFrame.current = data;
    });
    socket.emit('command', { action: 'load', preset: 5 });
    return () => socket.off('frame');
  }, []);

  // Resize canvas to match its container pixel-for-pixel
  const resizeCanvas = useCallback(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;
    const rect = container.getBoundingClientRect();
    canvas.width = rect.width;
    canvas.height = rect.height;
  }, []);

  useEffect(() => {
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);
    return () => window.removeEventListener('resize', resizeCanvas);
  }, [resizeCanvas]);

  // Use requestAnimationFrame for smooth 60fps rendering
  useEffect(() => {
    function draw() {
      const canvas = canvasRef.current;
      if (!canvas) { rafId.current = requestAnimationFrame(draw); return; }
      const frame = latestFrame.current;
      if (frame) {
        setFrameData(frame); // update React state once per rAF for bottom panel
        renderCanvas(canvas, frame);
      }
      rafId.current = requestAnimationFrame(draw);
    }
    rafId.current = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(rafId.current);
  }, []);

  const renderCanvas = (canvas, f) => {
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    if (W === 0 || H === 0) return;

    // Clear
    ctx.fillStyle = '#0d121c';
    ctx.fillRect(0, 0, W, H);

    const PPM = W / 120;  // pixels-per-meter: scale so ~120m of road fills the screen
    const egoScrX = W * 0.18; // ego pinned 18% from left
    const egoWX = f.ego.x;

    const w2s = (wx, wy) => [
      egoScrX + (wx - egoWX) * PPM,
      H / 2 - wy * PPM
    ];

    // ── Grid lines ──
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    ctx.lineWidth = 1;
    for (let y = -6; y <= 6; y += 2) {
      const [, sy] = w2s(egoWX, y);
      ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
    }

    // ── Road ──
    const roadHalf = 3.5 * PPM;
    // Asphalt fill
    ctx.fillStyle = '#181f2a';
    ctx.fillRect(0, H/2 - roadHalf, W, roadHalf * 2);
    // White edge lines
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, H/2 - roadHalf); ctx.lineTo(W, H/2 - roadHalf);
    ctx.moveTo(0, H/2 + roadHalf); ctx.lineTo(W, H/2 + roadHalf);
    ctx.stroke();
    // Yellow dashed center line
    ctx.beginPath();
    ctx.setLineDash([PPM * 1.5, PPM * 1.5]);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 3;
    ctx.moveTo(0, H/2); ctx.lineTo(W, H/2);
    ctx.stroke();
    ctx.setLineDash([]);

    // ── Trail (blue solid) ──
    if (f.trail && f.trail.length > 1) {
      ctx.beginPath();
      let [sx, sy] = w2s(f.trail[0][0], f.trail[0][1]);
      ctx.moveTo(sx, sy);
      for (let i = 1; i < f.trail.length; i++) {
        [sx, sy] = w2s(f.trail[i][0], f.trail[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#0ea5e9';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // ── Sensor wedge ──
    if (f.sensorWedge) {
      ctx.beginPath();
      let [sx, sy] = w2s(f.sensorWedge[0][0], f.sensorWedge[0][1]);
      ctx.moveTo(sx, sy);
      for (let i = 1; i < f.sensorWedge.length; i++) {
        [sx, sy] = w2s(f.sensorWedge[i][0], f.sensorWedge[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.closePath();
      ctx.fillStyle = f.state === 'EVADE'
        ? 'rgba(239, 68, 68, 0.12)'
        : 'rgba(255, 255, 255, 0.04)';
      ctx.fill();
    }

    // ── Planned path (dashed green) ──
    if (f.path && f.path.length > 1) {
      ctx.beginPath();
      ctx.setLineDash([8, 4]);
      let [sx, sy] = w2s(f.path[0][0], f.path[0][1]);
      ctx.moveTo(sx, sy);
      for (let i = 1; i < f.path.length; i++) {
        [sx, sy] = w2s(f.path[i][0], f.path[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // ── Obstacles ──
    if (f.obstacles) {
      f.obstacles.forEach(obs => {
        const [sx, sy] = w2s(obs.x, obs.y);
        const ow = obs.w * PPM;
        const oh = obs.h * PPM;

        ctx.save();
        ctx.translate(sx, sy);
        ctx.fillStyle = obs.color || '#ef4444';
        ctx.fillRect(-ow/2, -oh/2, ow, oh);
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(-ow/2, -oh/2, ow, oh);
        ctx.restore();

        // Callout label
        const label = obs.speed > 0 ? `${obs.label} (${obs.speed}m/s)` : obs.label;
        ctx.font = 'bold 11px sans-serif';
        const tw = ctx.measureText(label).width;
        const above = obs.y > 0;
        const cy = above ? sy - 55 : sy + 55;

        // Vertical line
        ctx.beginPath();
        ctx.moveTo(sx, sy + (above ? -oh/2 : oh/2));
        ctx.lineTo(sx, cy);
        ctx.strokeStyle = obs.color || '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label box
        const bx = sx - tw/2 - 6;
        const by = above ? cy - 18 : cy;
        ctx.fillStyle = obs.color || '#ef4444';
        ctx.beginPath();
        ctx.roundRect(bx, by, tw + 12, 20, 4);
        ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.fillText(label, sx - tw/2, by + 14);
      });
    }

    // ── Ego vehicle ──
    {
      const [sx, sy] = w2s(f.ego.x, f.ego.y);
      const ew = f.ego.w * PPM;
      const eh = f.ego.h * PPM;

      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-f.ego.yaw);
      ctx.fillStyle = '#06b6d4';
      ctx.fillRect(-ew/2, -eh/2, ew, eh);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.strokeRect(-ew/2, -eh/2, ew, eh);
      // Heading arrow
      ctx.beginPath();
      ctx.moveTo(ew/2 + 8, 0);
      ctx.lineTo(ew/2, -6);
      ctx.lineTo(ew/2, 6);
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
      ctx.restore();
    }

    // ── Axis labels ──
    ctx.fillStyle = '#64748b';
    ctx.font = '12px sans-serif';
    ctx.fillText('Lateral Offset (m)', 15, H/2 + roadHalf + 18);
    ctx.fillText('Longitudinal Distance (m)', W/2 - 80, H - 8);
  };

  const handleScenarioChange = (id) => {
    setActiveScenario(id);
    setPaused(false);
    socket.emit('command', { action: 'load', preset: id });
  };

  const togglePause = () => {
    const next = !paused;
    setPaused(next);
    socket.emit('command', { action: next ? 'pause' : 'resume' });
  };

  const activeTitle = SCENARIOS.find(s => s.id === activeScenario)?.name || '';

  let perceptionStatus = 'LANE CLEAR';
  let perceptionColor = 'green';
  let autonomousState = 'CRUISE';
  let stateColor = 'green';
  if (frameData?.state === 'EVADE') {
    perceptionStatus = 'HAZARD DETECTED';
    perceptionColor = 'yellow';
    autonomousState = 'EMERGENCY REPLAN';
    stateColor = 'yellow';
  } else if (frameData?.state === 'BRAKING') {
    perceptionStatus = 'COLLISION IMMINENT';
    perceptionColor = 'red';
    autonomousState = 'HARD BRAKING';
    stateColor = 'red';
  }

  return (
    <div className="app-container">
      <div className="title-bar">
        Bird's-Eye View — {activeTitle} | t={frameData?.time || 0}s | Replans: {frameData?.telemetry?.replans || 0}
      </div>

      <div className="plot-container" ref={containerRef}>
        <div className="legend">
          <div className="legend-item">
            <div className="legend-line planned"></div>
            <span>Planned Trajectory</span>
          </div>
          <div className="legend-item">
            <div className="legend-line history"></div>
            <span>Ego Path History</span>
          </div>
        </div>
        <canvas ref={canvasRef} />
      </div>

      <div className="bottom-panel">
        <div className="scenario-list">
          {SCENARIOS.map(s => (
            <div
              key={s.id}
              className={`scenario-item ${activeScenario === s.id ? 'active' : ''}`}
              onClick={() => handleScenarioChange(s.id)}
            >
              {s.name}
            </div>
          ))}
          <div className="scenario-item" style={{ textAlign: 'center', color: '#60a5fa', fontWeight: 'bold' }}>
            Scenario: {activeTitle}
          </div>
        </div>

        <div className="status-box">
          <div className="status-label">PERCEPTION TARGET</div>
          <div className={`status-value ${perceptionColor}`}>{perceptionStatus}</div>
        </div>

        <div className="status-box">
          <div className="status-label">AUTONOMOUS STATE</div>
          <div className={`status-value ${stateColor}`}>{autonomousState}</div>
        </div>

        <div className="controls-box">
          <button className="btn" onClick={togglePause}>{paused ? 'RESUME' : 'PAUSE'}</button>
          <button className="btn" onClick={() => handleScenarioChange(activeScenario)}>RESTART</button>
        </div>
      </div>
    </div>
  );
}

export default App;
