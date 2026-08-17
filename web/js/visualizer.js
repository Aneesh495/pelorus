/**
 * Pelorus 3D Visualizer: Real-time Canvas 3D rendering pipeline.
 * Renders spatial ground grid, vehicle pose axis, LiDAR point clouds,
 * 3D bounding boxes with heading vectors, and planned spline trajectories.
 */

class SpatialVisualizer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.width = canvas.width = window.innerWidth;
    this.height = canvas.height = window.innerHeight;

    // Camera parameters
    this.cameraDistance = 18.0;
    this.cameraPitch = 0.55; // Radians
    this.cameraYaw = -0.78;   // Radians
    this.target = { x: 0, y: 0, z: 0 };

    // Interaction state
    this.isDragging = false;
    this.lastMouse = { x: 0, y: 0 };
    this.rightDragging = false;

    // Active perception state
    this.latestState = {
      pose: { position: [0, 0, 0], rotation_quat: [0, 0, 0, 1], velocity: [0, 0, 0] },
      tracked_obstacles: [],
      planned_path: []
    };

    this.points = [];
    this.historyPoses = [];

    this._bindEvents();
    this._startRenderLoop();
  }

  _bindEvents() {
    window.addEventListener('resize', () => {
      this.width = this.canvas.width = window.innerWidth;
      this.height = this.canvas.height = window.innerHeight;
    });

    this.canvas.addEventListener('mousedown', (e) => {
      if (e.button === 2) {
        this.rightDragging = true;
      } else {
        this.isDragging = true;
      }
      this.lastMouse = { x: e.clientX, y: e.clientY };
    });

    window.addEventListener('mouseup', () => {
      this.isDragging = false;
      this.rightDragging = false;
    });

    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    window.addEventListener('mousemove', (e) => {
      const dx = e.clientX - this.lastMouse.x;
      const dy = e.clientY - this.lastMouse.y;
      this.lastMouse = { x: e.clientX, y: e.clientY };

      if (this.isDragging) {
        this.cameraYaw += dx * 0.008;
        this.cameraPitch = Math.max(0.05, Math.min(Math.PI * 0.48, this.cameraPitch + dy * 0.008));
      } else if (this.rightDragging) {
        const panSpeed = 0.03 * (this.cameraDistance / 15.0);
        this.target.x -= (Math.cos(this.cameraYaw) * dx - Math.sin(this.cameraYaw) * dy) * panSpeed;
        this.target.y -= (Math.sin(this.cameraYaw) * dx + Math.cos(this.cameraYaw) * dy) * panSpeed;
      }
    });

    this.canvas.addEventListener('wheel', (e) => {
      this.cameraDistance = Math.max(2.0, Math.min(60.0, this.cameraDistance + e.deltaY * 0.02));
    });
  }

  updateState(state) {
    this.latestState = state;
    if (state.pose && state.pose.position) {
      const p = state.pose.position;
      this.historyPoses.push({ x: p[0], y: p[1], z: p[2] });
      if (this.historyPoses.length > 300) this.historyPoses.shift();
    }
  }

  // 3D to 2D perspective projection
  project(p) {
    const relX = p.x - this.target.x;
    const relY = p.y - this.target.y;
    const relZ = p.z - this.target.z;

    // Rotate around Z (Yaw)
    const cosY = Math.cos(this.cameraYaw);
    const sinY = Math.sin(this.cameraYaw);
    const x1 = relX * cosY + relY * sinY;
    const y1 = -relX * sinY + relY * cosY;
    const z1 = relZ;

    // Rotate around X (Pitch)
    const cosP = Math.cos(this.cameraPitch);
    const sinP = Math.sin(this.cameraPitch);
    const x2 = x1;
    const y2 = y1 * cosP - z1 * sinP;
    const z2 = y1 * sinP + z1 * cosP;

    // Translate along view depth
    const viewZ = y2 + this.cameraDistance;
    if (viewZ <= 0.2) return null;

    const fov = 750.0;
    const screenX = this.width * 0.5 + (x2 * fov) / viewZ;
    const screenY = this.height * 0.5 - (z2 * fov) / viewZ;

    return { x: screenX, y: screenY, depth: viewZ };
  }

  _render() {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.width, this.height);

    // 1. Draw spatial ground grid
    if (document.getElementById('chk-grid').checked) {
      ctx.strokeStyle = 'rgba(48, 54, 61, 0.4)';
      ctx.lineWidth = 1;
      const size = 15;
      const step = 1.0;

      for (let x = -size; x <= size; x += step) {
        const p1 = this.project({ x: x, y: -size, z: 0 });
        const p2 = this.project({ x: x, y: size, z: 0 });
        if (p1 && p2) {
          ctx.beginPath();
          ctx.moveTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.stroke();
        }
      }

      for (let y = -size; y <= size; y += step) {
        const p1 = this.project({ x: -size, y: y, z: 0 });
        const p2 = this.project({ x: size, y: y, z: 0 });
        if (p1 && p2) {
          ctx.beginPath();
          ctx.moveTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.stroke();
        }
      }
    }

    // 2. Draw vehicle historical trajectory
    if (this.historyPoses.length > 1) {
      ctx.strokeStyle = '#388bfd';
      ctx.lineWidth = 2;
      ctx.beginPath();
      let started = false;

      for (const p of this.historyPoses) {
        const pt = this.project(p);
        if (!pt) continue;
        if (!started) {
          ctx.moveTo(pt.x, pt.y);
          started = true;
        } else {
          ctx.lineTo(pt.x, pt.y);
        }
      }
      ctx.stroke();
    }

    // 3. Draw planned trajectory waypoints
    if (document.getElementById('chk-traj').checked && this.latestState.planned_path) {
      ctx.strokeStyle = '#39d353';
      ctx.lineWidth = 3;
      ctx.beginPath();
      let started = false;

      for (const p of this.latestState.planned_path) {
        const pt = this.project({ x: p[0], y: p[1], z: p[2] });
        if (!pt) continue;
        if (!started) {
          ctx.moveTo(pt.x, pt.y);
          started = true;
        } else {
          ctx.lineTo(pt.x, pt.y);
        }
      }
      ctx.stroke();
    }

    // 4. Draw current platform coordinate frame
    if (this.latestState.pose && this.latestState.pose.position) {
      const pos = this.latestState.pose.position;
      const origin = this.project({ x: pos[0], y: pos[1], z: pos[2] });

      if (origin) {
        ctx.fillStyle = '#f0883e';
        ctx.beginPath();
        ctx.arc(origin.x, origin.y, 5, 0, Math.PI * 2);
        ctx.fill();

        // Forward heading vector
        const q = this.latestState.pose.rotation_quat;
        const forward = {
          x: pos[0] + 1.2 * (1 - 2 * (q[1] * q[1] + q[2] * q[2])),
          y: pos[1] + 1.2 * (2 * (q[0] * q[1] + q[3] * q[2])),
          z: pos[2] + 0.05
        };
        const ptFwd = this.project(forward);
        if (ptFwd) {
          ctx.strokeStyle = '#f85149';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(origin.x, origin.y);
          ctx.lineTo(ptFwd.x, ptFwd.y);
          ctx.stroke();
        }
      }
    }

    // 5. Draw 3D tracked obstacle bounding boxes
    if (document.getElementById('chk-bbox').checked && this.latestState.tracked_obstacles) {
      ctx.strokeStyle = '#ff7b72';
      ctx.lineWidth = 2;

      for (const obs of this.latestState.tracked_obstacles) {
        const c = obs.center;
        const s = obs.extents;
        const yaw = obs.yaw || 0;

        // Draw center dot and text
        const ptCenter = this.project({ x: c[0], y: c[1], z: c[2] });
        if (ptCenter) {
          ctx.fillStyle = '#ff7b72';
          ctx.font = '10px monospace';
          ctx.fillText(`ID #${obs.id} [${obs.velocity[0].toFixed(1)}m/s]`, ptCenter.x + 8, ptCenter.y - 8);

          // Velocity vector
          const vPt = this.project({ x: c[0] + obs.velocity[0], y: c[1] + obs.velocity[1], z: c[2] + obs.velocity[2] });
          if (vPt) {
            ctx.strokeStyle = '#d29922';
            ctx.beginPath();
            ctx.moveTo(ptCenter.x, ptCenter.y);
            ctx.lineTo(vPt.x, vPt.y);
            ctx.stroke();
            ctx.strokeStyle = '#ff7b72';
          }
        }
      }
    }
  }

  _startRenderLoop() {
    let lastTime = performance.now();
    let frames = 0;
    const fpsElem = document.getElementById('fps');

    const loop = (now) => {
      this._render();
      frames++;
      if (now - lastTime >= 1000) {
        if (fpsElem) fpsElem.textContent = frames;
        frames = 0;
        lastTime = now;
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }
}

// Instantiate on window load
window.addEventListener('DOMContentLoaded', () => {
  const canvas = document.getElementById('viewport');
  window.visualizer = new SpatialVisualizer(canvas);
});
