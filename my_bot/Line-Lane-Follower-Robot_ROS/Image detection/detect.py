from imageai.Detection import ObjectDetection
import os
execution_path = os.getcwd()
detect = ObjectDetection()
detect.setModelTypeAsRetinaNet()
detect.setModelPath( os.path.join(execution_path , "resnet50_coco_best_v2.0.1.h5"))
detect.loadModel()
detections, extracted_images = detect.detectObjectsFromImage(input_image=os.path.join(execution_path , "image.jpg"), output_image_path=os.path.join(execution_path , "imagenew.jpg"), extract_detected_objects=True)
for eachObject in detections:
    print(eachObject["name"] , " : " , eachObject["percentage_probability"] )