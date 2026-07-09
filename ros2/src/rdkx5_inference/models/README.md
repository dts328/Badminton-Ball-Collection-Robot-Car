# Model Files

## Required Model
- yolov8n_pose_rdkx5.bin: YOLOv8-Pose model for RDK X5 BPU

## How to Obtain
1. Train YOLOv8-Pose model with your dataset
2. Export to ONNX format
3. Convert to RDK X5 bin format using horizon model conversion tool

## Model Specifications
- Input: 640x640 BGR image
- Output: 9 tensors (3 scales x 3 branches: bbox, class, keypoints)
- Classes: 2 (0=perfect, 1=damaged)
- Keypoints: 5 per detection
