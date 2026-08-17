/**
 * Pelorus Telemetry Client: Connects to native telemetry broadcaster
 * via HTTP Server-Sent Events (SSE) and updates DOM HUD and 3D canvas.
 */

class TelemetryClient {
  constructor() {
    this.statusDot = document.querySelector('.status-dot');
    this.statusText = document.getElementById('conn-status');

    this.posX = document.getElementById('pos-x');
    this.posY = document.getElementById('pos-y');
    this.posZ = document.getElementById('pos-z');
    this.speed = document.getElementById('speed');
    this.yaw = document.getElementById('yaw');

    this.frameSeq = document.getElementById('frame-seq');
    this.motCount = document.getElementById('mot-count');

    this._connect();
  }

  _connect() {
    try {
      const eventSource = new EventSource('/stream');

      eventSource.onopen = () => {
        if (this.statusDot) {
          this.statusDot.classList.remove('offline');
          this.statusDot.classList.add('online');
        }
        if (this.statusText) {
          this.statusText.textContent = 'STREAMING (SSE Active)';
        }
      };

      eventSource.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this._handleStateUpdate(data);
        } catch (e) {
          console.error('[Telemetry] JSON parse error:', e);
        }
      };

      eventSource.onerror = () => {
        if (this.statusDot) {
          this.statusDot.classList.remove('online');
          this.statusDot.classList.add('offline');
        }
        if (this.statusText) {
          this.statusText.textContent = 'RECONNECTING...';
        }
        eventSource.close();
        setTimeout(() => this._connect(), 2000);
      };
    } catch (e) {
      // Fallback: poll HTTP snapshot every 200ms
      this._pollFallback();
    }
  }

  _pollFallback() {
    setInterval(async () => {
      try {
        const resp = await fetch('/snapshot');
        if (resp.ok) {
          const data = await resp.json();
          this._handleStateUpdate(data);
        }
      } catch (e) {}
    }, 200);
  }

  _handleStateUpdate(state) {
    if (window.visualizer) {
      window.visualizer.updateState(state);
    }

    if (state.pose) {
      const p = state.pose.position;
      const v = state.pose.velocity;
      if (this.posX) this.posX.textContent = `${p[0].toFixed(3)} m`;
      if (this.posY) this.posY.textContent = `${p[1].toFixed(3)} m`;
      if (this.posZ) this.posZ.textContent = `${p[2].toFixed(3)} m`;

      if (v && this.speed) {
        const spd = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        this.speed.textContent = `${spd.toFixed(2)} m/s`;
      }
    }

    if (state.sequence_id !== undefined && this.frameSeq) {
      this.frameSeq.textContent = state.sequence_id;
    }

    if (state.tracked_obstacles && this.motCount) {
      this.motCount.textContent = state.tracked_obstacles.length;
    }
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.telemetryClient = new TelemetryClient();
});
