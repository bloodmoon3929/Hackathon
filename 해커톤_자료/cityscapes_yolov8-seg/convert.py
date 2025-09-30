import os
import json
import sys

def polygon_to_normalized_coords(polygon, img_width, img_height):
    normalized_coords = []
    for point in polygon:
        x_norm = point[0] / img_width
        y_norm = point[1] / img_height
        normalized_coords.append((x_norm, y_norm))
    return normalized_coords

def convert_cityscapes_to_yolov8(json_path, output_dir, class_mapping):
    with open(json_path) as f:
        data = json.load(f)

    img_width = data['imgWidth']
    img_height = data['imgHeight']
    
    annotations = []
    for obj in data['objects']:
        label = obj['label']
        if label not in class_mapping or class_mapping[label] == 255:
            continue
        class_id = class_mapping[label]
        polygon = obj['polygon']
        norm_coords = polygon_to_normalized_coords(polygon, img_width, img_height)
        flattened_coords = [coord for point in norm_coords for coord in point]
        annotations.append((class_id, *flattened_coords))

    # Create output text file
    image_name = os.path.basename(json_path).replace('.json', '.txt')
    output_path = os.path.join(output_dir, image_name)
    
    with open(output_path, 'w') as out_file:
        for ann in annotations:
            out_file.write(' '.join(map(str, ann)) + '\n')

def process_folder(folder_path, class_mapping):
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".json"):
                json_path = os.path.join(root, file)
                convert_cityscapes_to_yolov8(json_path, root, class_mapping)

# Example class mapping based on trainId from Cityscapes labels
class_mapping = {
    'unlabeled': 4,
    'ego vehicle': 5,
    'rectification border': 6,
    'out of roi': 7,
    'static': 8,
    'dynamic': 9,
    'ground': 10,
    'road': 0,
    'sidewalk': 1,
    'parking': 11,
    'rail track': 12,
    'building': 13,
    'wall': 14,
    'fence': 15,
    'guard rail': 16,
    'bridge': 17,
    'tunnel': 18,
    'pole':19,
    'polegroup': 20,
    'traffic light': 21,
    'traffic sign': 22,
    'vegetation': 23,
    'terrain': 24,
    'sky': 25,
    'person': 2,
    'rider': 26,
    'car': 3,
    'truck': 27,
    'bus': 28,
    'caravan': 29,
    'trailer': 30,
    'train': 31,
    'motorcycle': 32,
    'bicycle': 33,
    'license plate': 34,
}

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python convert.py <annotation_folder_path>")
        sys.exit(1)

    folder_path = sys.argv[1]
    process_folder(folder_path, class_mapping)

    # You can use the rename command to make the names of the original image files and annotation files the same.
    # rename 's/gtFine_polygons/leftImg8bit/' *.txt
