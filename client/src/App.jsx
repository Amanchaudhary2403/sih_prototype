import React, { useState, useEffect, useRef } from 'react';
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

  useEffect(() => {
    socket.on('frame', (data) => setFrameData(data));
    socket.emit('command', { action: 'load', preset: 5 });
    return () => socket.off('frame');
  }, []);

  useEffect(() => {
    if (!canvasRef.current || !frameData) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // Clear and background
    ctx.fillStyle = '#0d121c';
    ctx.fillRect(0, 0, width, height);

    const pixelsPerMeter = 10;
    const egoWorldX = frameData.ego.x;
    
    // We want ego car near the right if possible, or left if scrolling.
    // The screenshot shows the car at x=110 on the right, meaning the camera is static or fixed.
    // We'll use a fixed trailing camera: Ego is at 80% of screen width.
    const egoScreenX = width * 0.8;

    function worldToScreen(wx, wy) {
      const sx = egoScreenX + (wx - egoWorldX) * pixelsPerMeter;
      const sy = height / 2 - wy * pixelsPerMeter; // Y is inverted in canvas vs plot
      return [sx, sy];
    }

    // Grid / Axes Background
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    // Draw horizontal grid lines (fake)
    for(let i = -6; i <= 6; i += 2) {
      const [_, sy] = worldToScreen(egoWorldX, i);
      ctx.beginPath();
      ctx.moveTo(0, sy);
      ctx.lineTo(width, sy);
      ctx.stroke();
    }

    // Road lines (Solid white edges, dashed yellow center)
    const roadHalfWidth = 3.5 * pixelsPerMeter;
    
    ctx.beginPath();
    ctx.moveTo(0, height / 2 - roadHalfWidth);
    ctx.lineTo(width, height / 2 - roadHalfWidth);
    ctx.moveTo(0, height / 2 + roadHalfWidth);
    ctx.lineTo(width, height / 2 + roadHalfWidth);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.beginPath();
    ctx.setLineDash([15, 15]);
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw Trail (Solid blue line)
    if (frameData.trail && frameData.trail.length > 0) {
      ctx.beginPath();
      const [sx0, sy0] = worldToScreen(frameData.trail[0][0], frameData.trail[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.trail.length; i++) {
        const [sx, sy] = worldToScreen(frameData.trail[i][0], frameData.trail[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#0284c7'; // Darker blue
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Draw Sensor Wedge (Faint dark grey)
    if (frameData.sensorWedge) {
      ctx.beginPath();
      const [sx0, sy0] = worldToScreen(frameData.sensorWedge[0][0], frameData.sensorWedge[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.sensorWedge.length; i++) {
        const [sx, sy] = worldToScreen(frameData.sensorWedge[i][0], frameData.sensorWedge[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.closePath();
      ctx.fillStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.fill();
    }

    // Draw Planned Path (Dashed green line)
    if (frameData.path && frameData.path.length > 0) {
      ctx.beginPath();
      ctx.setLineDash([10, 5]);
      const [sx0, sy0] = worldToScreen(frameData.path[0][0], frameData.path[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.path.length; i++) {
        const [sx, sy] = worldToScreen(frameData.path[i][0], frameData.path[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Draw Obstacles & Callouts
    if (frameData.obstacles) {
      frameData.obstacles.forEach(obs => {
        const [sx, sy] = worldToScreen(obs.x, obs.y);
        const w = obs.w * pixelsPerMeter;
        const h = obs.h * pixelsPerMeter;
        
        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(-obs.yaw || 0); // Inverted Y means inverted rotation
        ctx.fillStyle = obs.color || '#ef4444';
        
        // Draw obstacle body with white border
        ctx.fillRect(-w/2, -h/2, w, h);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(-w/2, -h/2, w, h);
        ctx.restore();

        // Draw Callout pointing to it (Matplotlib style)
        // Check if label contains speed, if not add it
        const labelText = obs.speed > 0 ? `${obs.label} (${obs.speed}m/s)` : obs.label;
        ctx.font = 'bold 11px sans-serif';
        const textWidth = ctx.measureText(labelText).width;
        
        // Line pointing up or down depending on Y position
        const isTop = obs.y > 0;
        const calloutY = isTop ? sy - 60 : sy + 60;
        
        ctx.beginPath();
        ctx.moveTo(sx, sy + (isTop ? -h/2 : h/2));
        ctx.lineTo(sx, calloutY);
        ctx.strokeStyle = obs.color || '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        
        // Draw arrow tip
        ctx.beginPath();
        ctx.moveTo(sx, sy + (isTop ? -h/2 : h/2));
        ctx.lineTo(sx - 4, sy + (isTop ? -h/2 - 6 : h/2 + 6));
        ctx.lineTo(sx + 4, sy + (isTop ? -h/2 - 6 : h/2 + 6));
        ctx.fillStyle = obs.color || '#ef4444';
        ctx.fill();

        // Draw text box
        const boxX = sx - textWidth / 2 - 6;
        const boxY = calloutY - (isTop ? 20 : 0);
        ctx.fillStyle = obs.color || '#ef4444';
        ctx.beginPath();
        ctx.roundRect(boxX, boxY, textWidth + 12, 20, 4);
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();
        
        ctx.fillStyle = '#ffffff';
        ctx.fillText(labelText, sx - textWidth / 2, boxY + 14);
      });
    }

    // Draw Ego
    const [egoSx, egoSy] = worldToScreen(frameData.ego.x, frameData.ego.y);
    const ew = frameData.ego.w * pixelsPerMeter;
    const eh = frameData.ego.h * pixelsPerMeter;
    
    ctx.save();
    ctx.translate(egoSx, egoSy);
    ctx.rotate(-frameData.ego.yaw); // Inverted
    
    // Draw body (cyan fill, white border)
    ctx.fillStyle = '#06b6d4';
    ctx.fillRect(-ew/2, -eh/2, ew, eh);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-ew/2, -eh/2, ew, eh);
    
    // Draw heading arrow (white triangle)
    ctx.beginPath();
    ctx.moveTo(ew/2 + 8, 0);
    ctx.lineTo(ew/2, -6);
    ctx.lineTo(ew/2, 6);
    ctx.closePath();
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.restore();

    // Axis Labels
    ctx.fillStyle = '#64748b';
    ctx.font = '10px sans-serif';
    ctx.fillText('Lateral Offset (m)', 10, height / 2 + 50);
    ctx.fillText('Longitudinal Distance (m)', width / 2, height - 10);

  }, [frameData]);

  const handleScenarioChange = (id) => {
    setActiveScenario(id);
    setPaused(false);
    socket.emit('command', { action: 'load', preset: id });
  };

  const togglePause = () => {
    const nextState = !paused;
    setPaused(nextState);
    socket.emit('command', { action: nextState ? 'pause' : 'resume' });
  };

  const activeTitle = SCENARIOS.find(s => s.id === activeScenario)?.name || 'Scenario';
  
  // Status logic
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
      {/* Title */}
      <div className="title-bar">
        Bird's-Eye View — {activeTitle} | t={frameData?.time || 0}s | Replans: {frameData?.telemetry?.replans || 0}
      </div>

      {/* Main Canvas Area */}
      <div className="plot-container">
        {/* Legend Overlay */}
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

        <canvas 
          ref={canvasRef} 
          width={1200} 
          height={500}
          style={{ width: '100%', height: '100%' }}
        />
      </div>

      {/* Bottom Panel */}
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
