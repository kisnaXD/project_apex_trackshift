from __future__ import annotations

import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional, Tuple
from urllib.parse import quote


_SERVERS: Dict[Tuple[str, int], ThreadingHTTPServer] = {}


class _CorsStaticFileHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Cross-Origin-Resource-Policy", "cross-origin")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.end_headers()

    def log_message(self, format: str, *args) -> None:
        return


def start_model_server(
    model_path: str,
    host: str = "0.0.0.0",
    port: int = 8766,
    public_base_url: str = "http://localhost:8766",
) -> Optional[str]:
    model_abs_path = os.path.abspath(model_path)
    if not os.path.isfile(model_abs_path):
        return None

    model_dir = os.path.dirname(model_abs_path)
    model_name = os.path.basename(model_abs_path)
    key = (model_dir, int(port))

    if key not in _SERVERS:
        handler = partial(_CorsStaticFileHandler, directory=model_dir)
        try:
            server = ThreadingHTTPServer((host, int(port)), handler)
        except OSError:
            return None

        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        _SERVERS[key] = server

    return f"{public_base_url.rstrip('/')}/{quote(model_name)}"
