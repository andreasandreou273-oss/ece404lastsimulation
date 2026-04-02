from __future__ import annotations

import argparse
import json
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict

from pymavlink import mavutil


HOST = "127.0.0.1"
PORT = 8765

state_lock = threading.Lock()
drone_state: Dict[str, Dict[str, Any]] = {}
event_log_path = Path("mission_events.jsonl")
map_started_at = datetime.now()


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Three Drone Live Map</title>
  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
    crossorigin=""
  >
  <style>
    html, body {
      margin: 0;
      height: 100%;
      font-family: Arial, sans-serif;
      background: #101418;
      color: #f3f5f7;
    }
    #map {
      height: 100%;
      width: 100%;
    }
    .panel {
      position: absolute;
      top: 12px;
      left: 12px;
      z-index: 1000;
      background: rgba(16, 20, 24, 0.92);
      border: 1px solid rgba(255, 255, 255, 0.16);
      border-radius: 12px;
      padding: 12px 14px;
      min-width: 320px;
      max-width: 360px;
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.28);
    }
    .panel-right {
      left: auto;
      right: 12px;
      min-width: 280px;
      max-width: 320px;
    }
    .title {
      margin: 0 0 8px;
      font-size: 18px;
      font-weight: 700;
    }
    .subtitle {
      margin: 0 0 10px;
      font-size: 13px;
      color: #b7c0c8;
    }
    .drone {
      margin-top: 10px;
      padding-top: 10px;
      border-top: 1px solid rgba(255, 255, 255, 0.08);
      font-size: 13px;
      line-height: 1.5;
    }
    .name {
      font-weight: 700;
    }
    .dot {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 8px;
    }
    .connected { color: #77d970; }
    .error { color: #ff8a80; }
    .events {
      margin-top: 12px;
      padding-top: 10px;
      border-top: 1px solid rgba(255, 255, 255, 0.08);
    }
    .events-title {
      margin: 0 0 8px;
      font-size: 13px;
      font-weight: 700;
      color: #b7c0c8;
    }
    .events-box {
      max-height: 220px;
      overflow-y: auto;
      padding-right: 4px;
      font-size: 12px;
      line-height: 1.45;
    }
    .event-entry {
      margin-bottom: 8px;
      padding: 8px;
      border-radius: 8px;
      background: rgba(255, 255, 255, 0.05);
    }
    .event-time {
      color: #8eb6ff;
      font-size: 11px;
      margin-bottom: 2px;
    }
    .coordinate-entry {
      margin-bottom: 8px;
      padding: 8px;
      border-radius: 8px;
      background: rgba(255, 255, 255, 0.05);
      font-size: 12px;
      line-height: 1.45;
    }
    .coordinate-name {
      font-weight: 700;
      color: #ffd166;
      margin-bottom: 4px;
    }
  </style>
</head>
<body>
  <div class="panel">
    <p class="title">Three Drone Live Map</p>
    <p class="subtitle">Refreshes automatically from dedicated MAVLink map feeds</p>
    <div id="status"></div>
    <div class="events">
      <p class="events-title">Mission Events</p>
      <div id="events" class="events-box"></div>
    </div>
  </div>
  <div class="panel panel-right">
    <p class="title">Detected Coordinates</p>
    <p class="subtitle">Latest person detections from the scan mission</p>
    <div id="detections" class="events-box"></div>
  </div>
  <div id="map"></div>

  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>
  <script>
    const map = L.map('map').setView([35.147719, 33.412118], 17);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const markers = {};
    const detectionMarkers = {};
    let autoFitDone = false;

    function markerIcon(color) {
      return L.divIcon({
        className: '',
        html: `
          <div style="
            width: 34px;
            height: 34px;
            display: flex;
            align-items: center;
            justify-content: center;
            filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.35));
          ">
            <svg width="30" height="30" viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
              <line x1="20" y1="20" x2="44" y2="44" stroke="white" stroke-width="4" stroke-linecap="round"/>
              <line x1="44" y1="20" x2="20" y2="44" stroke="white" stroke-width="4" stroke-linecap="round"/>
              <circle cx="18" cy="18" r="9" fill="${color}" stroke="white" stroke-width="3"/>
              <circle cx="46" cy="18" r="9" fill="${color}" stroke="white" stroke-width="3"/>
              <circle cx="18" cy="46" r="9" fill="${color}" stroke="white" stroke-width="3"/>
              <circle cx="46" cy="46" r="9" fill="${color}" stroke="white" stroke-width="3"/>
              <rect x="22" y="22" width="20" height="20" rx="6" fill="${color}" stroke="white" stroke-width="3"/>
              <path d="M32 16 L27 24 H37 Z" fill="#101418" stroke="white" stroke-width="2" stroke-linejoin="round"/>
            </svg>
          </div>
        `,
        iconSize: [34, 34],
        iconAnchor: [17, 17],
        popupAnchor: [0, -16],
      });
    }

    function updateStatus(drones) {
      const status = document.getElementById('status');
      status.innerHTML = drones.map((drone) => {
        const connected = drone.connected && drone.lat !== null && drone.lon !== null;
        const details = connected
          ? `sysid=${drone.sysid} | lat=${drone.lat.toFixed(6)} | lon=${drone.lon.toFixed(6)} | alt=${drone.alt.toFixed(1)} m`
          : (drone.error || 'waiting for data...');
        return `
          <div class="drone">
            <div class="name">
              <span class="dot" style="background:${drone.color}"></span>${drone.name}
            </div>
            <div class="${connected ? 'connected' : 'error'}">${details}</div>
          </div>
        `;
      }).join('');
    }

    function updateMarkers(drones) {
      const points = [];
      drones.forEach((drone) => {
        if (drone.lat === null || drone.lon === null) {
          return;
        }

        points.push([drone.lat, drone.lon]);
        if (!markers[drone.name]) {
          markers[drone.name] = L.marker([drone.lat, drone.lon], {
            icon: markerIcon(drone.color)
          }).addTo(map);
          markers[drone.name].bindPopup(drone.name);
        } else {
          markers[drone.name].setLatLng([drone.lat, drone.lon]);
        }

        markers[drone.name].setPopupContent(
          `${drone.name}<br>sysid=${drone.sysid}<br>alt=${drone.alt.toFixed(1)} m`
        );
      });

      if (points.length >= 2) {
        const bounds = L.latLngBounds(points);
        if (!autoFitDone) {
          map.fitBounds(bounds.pad(0.5));
          autoFitDone = true;
        }
      }
    }

    function updateEvents(events) {
      const eventsBox = document.getElementById('events');
      eventsBox.innerHTML = events.map((event) => `
        <div class="event-entry">
          <div class="event-time">${event.timestamp}</div>
          <div>${event.message}</div>
        </div>
      `).join('');
      eventsBox.scrollTop = eventsBox.scrollHeight;
    }

    function updateDetections(events) {
      const detectionsBox = document.getElementById('detections');
      const detections = events.filter(
        (event) => event.event === 'person_detected' && event.lat !== undefined && event.lon !== undefined
      );

      if (detections.length === 0) {
        detectionsBox.innerHTML = '<div class="coordinate-entry">No person detected yet.</div>';
        return;
      }

      detectionsBox.innerHTML = detections.map((event) => `
        <div class="coordinate-entry">
          <div class="coordinate-name">${event.drone || 'drone'} | ${event.building || 'unknown building'} ${event.corner || ''}</div>
          <div>lat=${Number(event.lat).toFixed(6)}</div>
          <div>lon=${Number(event.lon).toFixed(6)}</div>
          <div>${event.timestamp}</div>
        </div>
      `).join('');
      detectionsBox.scrollTop = detectionsBox.scrollHeight;
    }

    function updateDetectionPins(events) {
      const detections = events.filter(
        (event) => event.event === 'person_detected' && event.lat !== undefined && event.lon !== undefined
      );

      detections.forEach((event, index) => {
        const key = `${event.timestamp}-${event.drone || 'drone'}-${index}`;
        if (detectionMarkers[key]) {
          return;
        }

        const marker = L.marker([Number(event.lat), Number(event.lon)]).addTo(map);
        marker.bindPopup(
          `Person detected<br>${event.drone || 'drone'}<br>${event.building || 'unknown building'} ${event.corner || ''}<br>` +
          `lat=${Number(event.lat).toFixed(6)}<br>lon=${Number(event.lon).toFixed(6)}`
        );
        detectionMarkers[key] = marker;
      });
    }

    async function refresh() {
      const response = await fetch('/positions');
      const data = await response.json();
      updateStatus(data.drones);
      updateMarkers(data.drones);
      updateEvents(data.events || []);
      updateDetections(data.detections || []);
      updateDetectionPins(data.detections || []);
    }

    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""


def update_drone_state(name: str, **kwargs: Any) -> None:
    with state_lock:
        drone_state[name].update(kwargs)


def track_drone(name: str, endpoint: str) -> None:
    while True:
        try:
            update_drone_state(name, connected=False, error=None)
            link = mavutil.mavlink_connection(endpoint, source_system=250)
            heartbeat = link.wait_heartbeat(timeout=30)
            update_drone_state(
                name,
                connected=True,
                sysid=heartbeat.get_srcSystem(),
                error=None,
            )

            while True:
                message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
                if message is None:
                    continue

                update_drone_state(
                    name,
                    connected=True,
                    lat=message.lat / 1e7,
                    lon=message.lon / 1e7,
                    alt=message.relative_alt / 1000.0,
                    error=None,
                )
        except Exception as exc:  # pragma: no cover - defensive loop for live usage
            update_drone_state(name, connected=False, error=str(exc))


class MapRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/positions":
            with state_lock:
                body = json.dumps(
                    {
                        "drones": list(drone_state.values()),
                        "events": load_recent_events(event_log_path),
                        "detections": load_detection_events(event_log_path),
                    }
                ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        return


def load_recent_events(path: Path, max_events: int | None = 12) -> list[dict[str, Any]]:
    if not path.exists():
        return []

    lines = path.read_text(encoding="utf-8").splitlines()
    events: list[dict[str, Any]] = []
    selected_lines = lines if max_events is None else lines[-max_events:]
    for line in selected_lines:
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        timestamp_text = event.get("timestamp")
        if isinstance(timestamp_text, str):
            try:
                if datetime.fromisoformat(timestamp_text) < map_started_at:
                    continue
            except ValueError:
                pass
        events.append(event)
    return events


def load_detection_events(path: Path) -> list[dict[str, Any]]:
    return [
        event
        for event in load_recent_events(path, max_events=None)
        if event.get("event") == "person_detected"
    ]


def main() -> None:
    global event_log_path

    parser = argparse.ArgumentParser(description="Show three drones on one live browser map")
    parser.add_argument("--drone1-port", type=int, default=14551, help="UDP port for drone1 map feed")
    parser.add_argument("--drone2-port", type=int, default=14561, help="UDP port for drone2 map feed")
    parser.add_argument("--drone3-port", type=int, default=14571, help="UDP port for drone3 map feed")
    parser.add_argument("--host", default=HOST, help="HTTP host for the map server")
    parser.add_argument("--port", type=int, default=PORT, help="HTTP port for the map server")
    parser.add_argument("--event-log", default=str(event_log_path), help="Mission event JSONL path")
    args = parser.parse_args()

    drones = {
        "drone1": {
            "endpoint": f"udp:127.0.0.1:{args.drone1_port}",
            "color": "red",
        },
        "drone2": {
            "endpoint": f"udp:127.0.0.1:{args.drone2_port}",
            "color": "blue",
        },
        "drone3": {
            "endpoint": f"udp:127.0.0.1:{args.drone3_port}",
            "color": "gold",
        },
    }

    with state_lock:
        drone_state.clear()
        for name, config in drones.items():
            drone_state[name] = {
                "name": name,
                "endpoint": config["endpoint"],
                "color": config["color"],
                "connected": False,
                "sysid": None,
                "lat": None,
                "lon": None,
                "alt": None,
                "error": None,
            }

    event_log_path = Path(args.event_log)

    for name, config in drones.items():
        thread = threading.Thread(
            target=track_drone,
            args=(name, config["endpoint"]),
            daemon=True,
        )
        thread.start()

    server = ThreadingHTTPServer((args.host, args.port), MapRequestHandler)
    print(f"Live map server running on http://{args.host}:{args.port}")
    print(f"drone1 map feed: udp:127.0.0.1:{args.drone1_port}")
    print(f"drone2 map feed: udp:127.0.0.1:{args.drone2_port}")
    print(f"drone3 map feed: udp:127.0.0.1:{args.drone3_port}")
    print("Open that address in your browser to see all drones on one map.")
    server.serve_forever()


if __name__ == "__main__":
    main()
