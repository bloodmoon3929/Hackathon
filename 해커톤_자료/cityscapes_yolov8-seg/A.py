from ultralytics import YOLO

model = YOLO('yolov8m-seg.pt')

results = model.train(
    data = 'data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='cityscapes_model'
)

print(f"경로 {results.save_dir}/weights/best.pt")
