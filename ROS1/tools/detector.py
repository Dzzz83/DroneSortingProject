#!/usr/bin/env python3
"""Loopback RGB-only inference worker in the isolated detector environment."""

import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

def restore_image_boxes(detections, offset_x, offset_y, width, height):
    """Map padded-view detections back to the sensor's original pixel coordinates."""
    restored = []
    for detection in detections:
        left, top, right, bottom = detection["xyxy"]
        box = [max(0, left - offset_x), max(0, top - offset_y),
               min(width, right - offset_x), min(height, bottom - offset_y)]
        if box[0] < box[2] and box[1] < box[3]:
            restored.append({"xyxy": box, "confidence": detection["confidence"]})
    return restored


def main():
    import numpy as np
    from ultralytics import YOLO

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True, type=Path)
    parser.add_argument("--port", type=int, default=10001)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--context-scale", type=float, default=1,
                        help="Optional second padded RGB view (1–2); retains normal-view detections.")
    arguments = parser.parse_args()
    if not 0 < arguments.confidence <= 1 or arguments.image_size < 32 or not 0 < arguments.port < 65536:
        parser.error("Invalid confidence, image size, or port")
    if not math.isfinite(arguments.context_scale) or not 1 <= arguments.context_scale <= 2:
        parser.error("Context scale must be finite and between 1 and 2")
    if not arguments.weights.is_file():
        raise FileNotFoundError("Local trained weights required; no download.")
    model = YOLO(str(arguments.weights.resolve()))
    if model.names != {0: "package"}:
        raise ValueError("Expected a single package class.")
    device = int(arguments.device) if arguments.device.isdigit() else arguments.device
    model.predict(np.zeros((600, 800, 3), dtype=np.uint8), device=device, imgsz=arguments.image_size, verbose=False)

    def detect(image):
        result = model.predict(image, device=device, imgsz=arguments.image_size,
                               conf=arguments.confidence, verbose=False)[0]
        return [{"xyxy": box.xyxy[0].tolist(), "confidence": float(box.conf.item())} for box in result.boxes]

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                if self.path != "/detect":
                    raise ValueError("Unknown endpoint")
                width, height = int(self.headers["X-Width"]), int(self.headers["X-Height"])
                length = int(self.headers["Content-Length"])
                if not (0 < width <= 2048 and 0 < height <= 2048 and length == width * height * 3):
                    raise ValueError("Invalid packed RGB dimensions")
                self.connection.settimeout(5)
                body = self.rfile.read(length)
                if len(body) != length:
                    raise ValueError("Truncated RGB data")
                rgb = np.frombuffer(body, dtype=np.uint8).reshape(height, width, 3)
                # Ultralytics ndarray inputs are BGR; the sensor contract is RGB.
                bgr = rgb[:, :, ::-1].copy()
                detections = detect(bgr)
                if arguments.context_scale > 1:
                    # Close packages exceeded the training-view scale. Padding adds
                    # a smaller inference view without changing calibration or RGB data.
                    padded_width = int(width * arguments.context_scale)
                    padded_height = int(height * arguments.context_scale)
                    offset_x, offset_y = (padded_width - width) // 2, (padded_height - height) // 2
                    padded = np.full((padded_height, padded_width, 3), 114, dtype=np.uint8)
                    padded[offset_y:offset_y + height, offset_x:offset_x + width] = bgr
                    detections.extend(restore_image_boxes(detect(padded), offset_x, offset_y, width, height))
                    # Existing depth localization removes duplicate surface candidates.
                encoded = json.dumps({"sequence": self.headers["X-Sequence"], "detections": detections}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)
            except (ValueError, TypeError, KeyError) as error:
                self.send_error(400, str(error))

        def log_message(self, *_arguments):
            pass

    server = HTTPServer(("127.0.0.1", arguments.port), Handler)
    print("PACKAGE_DETECTOR_SERVICE_READY", arguments.port, flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
