import React, { useState, useEffect, useRef } from 'react';
import { io } from 'socket.io-client';

const socket = io('http://localhost:3001');

const SCENARIOS = [
  { id: 1, name: '01 UNMARKED VILLAGE ROAD' },
  { id: 2, name: '02 UNSIGNALISED URBAN INTERSECTION' },
  { id: 3, name: '03 HIGHWAY MERGE, SLOW VEHICLES' },
  { id: 4, name: '04 DENSE MARKET, MIXED TRAFFIC' },
  { id: 5, name: '05 SUDDEN CATTLE CROSSING' },
];

function App() {
  const [activeScenario, setActiveScenario] = useState(5);
  const [frameData, setFrameData] = useState(null);
  const [paused, setPaused] = useState(false);
  const [layers, setLayers] = useState({ lidar: true, occupancy: false, plan: true });
  
  const canvasRef = useRef(null);

  useEffect(() => {
    socket.on('frame', (data) => {
      setFrameData(data);
    });
    
    // Initial load
    socket.emit('command', { action: 'load', preset: 5 });

    return () => socket.off('frame');
  }, []);

  // Canvas drawing
  useEffect(() => {
    if (!canvasRef.current || !frameData) return;
    
    const ctx = canvasRef.current.getContext('2d');
    const width = canvasRef.current.width;
    const height = canvasRef.current.height;
    
    // Clear
    ctx.clearRect(0, 0, width, height);
    
    // Draw Road (Dark gray strip)
    ctx.fillStyle = 'rgba(255, 255, 255, 0.03)';
    ctx.fillRect(0, height / 2 - 40, width, 80);
    
    // Dashed center line
    ctx.beginPath();
    ctx.setLineDash([20, 20]);
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.stroke();
    ctx.setLineDash([]);
    
    // Camera transform: ego is always at x = width * 0.2
    const egoScreenX = width * 0.2;
    const pixelsPerMeter = 12;
    const egoWorldX = frameData.ego.x;
    
    function worldToScreen(wx, wy) {
      const sx = egoScreenX + (wx - egoWorldX) * pixelsPerMeter;
      const sy = height / 2 + wy * pixelsPerMeter;
      return [sx, sy];
    }

    // Draw Trail
    if (frameData.trail && frameData.trail.length > 0) {
      ctx.beginPath();
      const [sx0, sy0] = worldToScreen(frameData.trail[0][0], frameData.trail[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.trail.length; i++) {
        const [sx, sy] = worldToScreen(frameData.trail[i][0], frameData.trail[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#005577';
      ctx.lineWidth = 3;
      ctx.stroke();
    }
    
    // Draw Sensor Wedge
    if (layers.lidar && frameData.sensorWedge) {
      ctx.beginPath();
      const [sx0, sy0] = worldToScreen(frameData.sensorWedge[0][0], frameData.sensorWedge[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.sensorWedge.length; i++) {
        const [sx, sy] = worldToScreen(frameData.sensorWedge[i][0], frameData.sensorWedge[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.closePath();
      ctx.fillStyle = frameData.state === 'EVADE' ? 'rgba(239, 68, 68, 0.15)' : 'rgba(0, 212, 255, 0.05)';
      ctx.fill();
    }

    // Draw Path
    if (layers.plan && frameData.path && frameData.path.length > 0) {
      ctx.beginPath();
      const [sx0, sy0] = worldToScreen(frameData.path[0][0], frameData.path[0][1]);
      ctx.moveTo(sx0, sy0);
      for (let i = 1; i < frameData.path.length; i++) {
        const [sx, sy] = worldToScreen(frameData.path[i][0], frameData.path[i][1]);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 3;
      ctx.stroke();
      
      // Draw path nodes
      frameData.path.forEach(pt => {
        const [sx, sy] = worldToScreen(pt[0], pt[1]);
        ctx.beginPath();
        ctx.arc(sx, sy, 3, 0, Math.PI * 2);
        ctx.fillStyle = '#f59e0b';
        ctx.fill();
      });
    }

    // Draw Obstacles
    if (frameData.obstacles) {
      frameData.obstacles.forEach(obs => {
        const [sx, sy] = worldToScreen(obs.x, obs.y);
        const w = obs.w * pixelsPerMeter;
        const h = obs.h * pixelsPerMeter;
        
        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(obs.yaw || 0);
        
        ctx.fillStyle = obs.color || '#ef4444';
        ctx.fillRect(-w/2, -h/2, w, h);
        
        // Label
        ctx.fillStyle = '#fff';
        ctx.font = '10px monospace';
        ctx.fillText(`${obs.label} ${obs.speed}k`, -w/2, -h/2 - 5);
        
        ctx.restore();
      });
    }

    // Draw Ego
    const [egoSx, egoSy] = worldToScreen(frameData.ego.x, frameData.ego.y);
    const ew = frameData.ego.w * pixelsPerMeter;
    const eh = frameData.ego.h * pixelsPerMeter;
    
    ctx.save();
    ctx.translate(egoSx, egoSy);
    ctx.rotate(frameData.ego.yaw);
    ctx.fillStyle = '#00d4ff';
    ctx.fillRect(-ew/2, -eh/2, ew, eh);
    ctx.restore();

  }, [frameData, layers]);

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

  const handleReset = () => {
    setPaused(false);
    socket.emit('command', { action: 'reset' });
  };

  const toggleLayer = (layer) => {
    setLayers(prev => ({ ...prev, [layer]: !prev[layer] }));
  };

  // Determine scenario description
  let scenarioDesc = "A non-cooperative animal enters the path with no signalling and an unmodellable intent. Pure worst-case reaction test.";
  let scenarioResp = "Detection triggers EMERGENCY_REPLAN — the planner evaluates swerve feasibility first and falls back to a braking profile when lateral clearance is insufficient.";
  if (activeScenario === 1) {
    scenarioDesc = "Unmarked roads with wandering livestock. Tests long-range static hazard detection.";
    scenarioResp = "Lane deviation to bypass hazard while maintaining safe longitudinal velocity.";
  } else if (activeScenario === 2) {
    scenarioDesc = "Urban environments with unpredictable crossing pedestrians and fast rickshaws.";
    scenarioResp = "Predictive tracking and aggressive braking/swerving based on TTC.";
  } else if (activeScenario === 4) {
    scenarioDesc = "High density, unstructured traffic flow with minimal lateral spacing.";
    scenarioResp = "Continuous micro-adjustments in lateral offset to maintain minimum safety bounds.";
  }

  return (
    <div className="dashboard-container">
      {/* Top Bar */}
      <div className="top-bar">
        {SCENARIOS.map(s => (
          <div 
            key={s.id}
            className={`scenario-tab ${activeScenario === s.id ? 'active' : ''}`}
            onClick={() => handleScenarioChange(s.id)}
          >
            {s.name}
          </div>
        ))}
      </div>

      {/* Main Content */}
      <div className="main-content">
        {/* Left Column */}
        <div className="left-column">
          
          <div className="canvas-panel">
            <div className="canvas-header">
              <span>SCENARIO 0{activeScenario} · {SCENARIOS.find(s=>s.id===activeScenario)?.name.substring(3)}</span>
              <span>t+{frameData?.time || 0}s</span>
            </div>
            
            <div className="canvas-container">
              <canvas 
                ref={canvasRef} 
                width={800} 
                height={400}
                style={{ width: '100%', height: '100%' }}
              />
            </div>
            
            <div className="canvas-header" style={{ borderBottom: 'none', position: 'absolute', bottom: '45px', left: 0, right: 0 }}>
              <span style={{color: '#f59e0b'}}>STATE: {frameData?.state || 'CRUISE'}</span>
              <span>{frameData?.telemetry?.speed || 0} km/h</span>
            </div>

            <div className="canvas-controls">
              <div className="playback-controls">
                <button className="btn" onClick={togglePause}>
                  {paused ? '▶ RESUME' : '⏸ PAUSE'}
                </button>
                <button className="btn" onClick={handleReset}>
                  ↻ RESET
                </button>
              </div>
              <div className="layer-toggles">
                <button className={`btn ${layers.lidar ? 'active' : ''}`} onClick={() => toggleLayer('lidar')}>LIDAR RETURNS</button>
                <button className={`btn ${layers.occupancy ? 'active' : ''}`} onClick={() => toggleLayer('occupancy')}>PREDICTED OCCUPANCY</button>
                <button className={`btn ${layers.plan ? 'active' : ''}`} onClick={() => toggleLayer('plan')}>PLAN CORRIDOR</button>
              </div>
            </div>
          </div>

          <div className="info-panel">
            <div className="info-title">WHAT THIS SCENARIO STRESSES</div>
            <div className="info-text">{scenarioDesc}</div>
            <div className="info-text">
              <span className="response-label">RESPONSE</span>
              {scenarioResp}
            </div>
          </div>

        </div>

        {/* Right Column */}
        <div className="right-column">
          
          <div className="telemetry-panel">
            <div className="info-title" style={{marginBottom: '15px'}}>LIVE TELEMETRY</div>
            
            <div className="telemetry-row">
              <span className="telemetry-label">SPEED</span>
              <span className="telemetry-value">{frameData?.telemetry?.speed || '0.0'} km/h</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">REPLAN LATENCY</span>
              <span className="telemetry-value">{frameData?.telemetry?.replanLatency || 0} ms</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">REPLANS</span>
              <span className="telemetry-value">{frameData?.telemetry?.replans || 0}</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">TRACKED ACTORS</span>
              <span className="telemetry-value">{frameData?.telemetry?.trackedActors || 0}</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">MIN TTC</span>
              <span className="telemetry-value">{frameData?.telemetry?.minTTC || '— clear'}</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">DISTANCE</span>
              <span className="telemetry-value">{frameData?.telemetry?.distance || 0} m</span>
            </div>
            <div className="telemetry-row">
              <span className="telemetry-label">COLLISIONS</span>
              <span className="telemetry-value amber">{frameData?.telemetry?.collisions || 0}</span>
            </div>
          </div>

          <div className="stateflow-panel">
            <div className="info-title">STATEFLOW MODE</div>
            <div className="stateflow-state">{frameData?.state || 'CRUISE'}</div>
            <div className="stateflow-desc">
              {frameData?.stateDetail || 'Maintaining lane center and nominal speed.'}
            </div>
          </div>

          <div className="reference-panel">
            {/* Make sure cattle_ref.jpg exists in public folder, or use a placeholder */}
            <img src="cattle_ref.jpg" className="reference-img" alt="Reference" />
            <div className="reference-caption">
              REAL-WORLD REFERENCE · ZERO-WARNING INTRUSION · EMERGENCY REPLAN
            </div>
          </div>

        </div>
      </div>
    </div>
  );
}

export default App;
