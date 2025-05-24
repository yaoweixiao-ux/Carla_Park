import torch
from ultralytics import YOLO

model = YOLO('yolov8_carla_slot_obb.pt')  
model.export(format='onnx')
