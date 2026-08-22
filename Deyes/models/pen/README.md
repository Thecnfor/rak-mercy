# Pen detector artifact

Default real-robot candidate:

- file: `pen_student_01875_416_v1.onnx`
- architecture: distilled YOLOv5 student, width `0.1875`
- input: `416×416`
- classes: `1` (`pen`)
- SHA256: `8916cbf25949d6e8b03c01e6ca1c7871aeac0ad105c931f1dd9881cb5d5a4c4e`

Build the FP16 TensorRT engine on the target Jetson. TensorRT engine binaries
are deliberately not versioned because they depend on the target JetPack,
CUDA and TensorRT versions. `Deyes/tools/run_real_robot_dry_run.sh` verifies
this ONNX, builds the device-local engine, calculates its hash, and passes the
identity into the fail-closed one-shot launch.
