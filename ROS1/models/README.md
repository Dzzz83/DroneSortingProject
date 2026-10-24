# Package detector

`package_detector.pt` is the inference checkpoint used by the warehouse camera
demo. It is tracked directly in Git (approximately 5.3 MB); no training or model
download is needed to run the demo. Verify it with
`sha256sum -c package_detector.sha256` from this directory.

- Architecture: Ultralytics YOLO11n, one class `{0: "package"}`.
- Source: `best.pt` from the local `package-training-trial` run on 2026-10-12.
- Base weights: `yolo11n.pt`, SHA-256
  `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`.
- Training: Ultralytics 8.3.195, PyTorch 2.5.1+cu124, 50 epochs, 640 px,
  batch 4, AdamW, seed 20261012, AMP disabled.
- Synthetic dataset: 200 warehouse views, 306 boxes, 40 empty views;
  train/validation/test split 140/30/30. Dataset manifest SHA-256:
  `665e233985ff82d52cf4458c7769ac3f07f8a5e74cc0690e7b90c85c778ef42e`.
- Runtime: confidence 0.25, 640 px, extra padded view with context scale 1.5.
  This is a simulation-specific detector, not a general package detector.

Training and dataset-export tools were removed from the demo checkout. Generated
environments and run logs remain excluded from Git.

Ultralytics publishes YOLO code and trained models under AGPL-3.0 unless covered
by an Enterprise license. See the [upstream license](https://www.ultralytics.com/license)
and [license text](https://www.ultralytics.com/legal/agpl-3-0-software-license).
This notice records the upstream terms; it does not assign a new license to the
project's Unity assets or other code. The ROS package still has a `TODO` license,
so project-wide redistribution licensing remains unresolved.
