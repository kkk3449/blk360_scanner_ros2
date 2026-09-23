#!/usr/bin/env python3
"""KG data server for a split deployment (item 3).

Runs on the data/DT host (this PC: semantic pipeline, kg_to_usd, Isaac) and
serves the files a remote console host needs, plus accepts the console's
owner edits back so the knowledge graph has ONE authoritative copy here:

    GET  /manifest                 {"files": {name: {"mtime", "size"}}}
    GET  /files/<name>             file bytes (X-Mtime header)
    PUT  /files/<name>             replace file (console -> data host; KG json only)

Names are the basenames of the exported files (see FILES below). No auth,
lab LAN only.

    python3 scripts/kg_data_server.py [--port 8090]
"""
import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BLK = "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs"
TWIN = os.path.expanduser("~/ammr_twin")
FILES = {
    "testroom_epochs_kg.json": os.path.join(BLK, "testroom_epochs_kg.json"),
    "place_layer_T3_slic.json": os.path.join(BLK, "place_layer_T3_slic.json"),
    "place_ring_naming.json": os.path.join(BLK, "place_ring_naming.json"),
    "t3_place_scoped_relations.json": os.path.join(BLK, "t3_place_scoped_relations.json"),
    "map_vis_n2_1.yaml": os.path.join(TWIN, "map_vis_n2_1.yaml"),
    "map_vis_n2_1.pgm": os.path.join(TWIN, "map_vis_n2_1.pgm"),
    "robot_map_offset.json": os.path.join(TWIN, "robot_map_offset.json"),
    "t4_kg_scene.usda": os.path.join(BLK, "vis_sota_det4", "t4_kg_scene.usda"),
}
WRITABLE = {"testroom_epochs_kg.json", "robot_map_offset.json"}


class H(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print(f"[kg-data] {self.client_address[0]} {fmt % args}", flush=True)

    def _send(self, code, body, ctype="application/json", extra=None):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/manifest":
            out = {}
            for n, p in FILES.items():
                try:
                    st = os.stat(p)
                    out[n] = {"mtime": st.st_mtime, "size": st.st_size}
                except OSError:
                    pass
            return self._send(200, json.dumps({"files": out, "t": time.time()}))
        if self.path.startswith("/files/"):
            n = self.path[len("/files/"):]
            p = FILES.get(n)
            if not p or not os.path.exists(p):
                return self._send(404, "{}")
            with open(p, "rb") as f:
                data = f.read()
            return self._send(200, data, "application/octet-stream",
                              {"X-Mtime": str(os.path.getmtime(p))})
        self._send(404, "{}")

    def do_PUT(self):
        n = self.path[len("/files/"):] if self.path.startswith("/files/") else ""
        p = FILES.get(n)
        if not p or n not in WRITABLE:
            return self._send(403, json.dumps({"error": "not writable"}))
        length = int(self.headers.get("Content-Length", "0"))
        data = self.rfile.read(length)
        if n.endswith(".json"):
            try:
                json.loads(data.decode("utf-8"))
            except ValueError:
                return self._send(400, json.dumps({"error": "invalid json"}))
        tmp = p + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, p)
        print(f"[kg-data] {n} written by {self.client_address[0]} ({length} B)", flush=True)
        self._send(200, json.dumps({"ok": True, "mtime": os.path.getmtime(p)}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    a = ap.parse_args()
    print(f"[kg-data] serving {len(FILES)} files on :{a.port}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
