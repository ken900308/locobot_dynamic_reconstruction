#!/usr/bin/env python3
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("Error: Could not import cv2 or numpy. Please install opencv-python and numpy. (Check if you are in the correct Docker or conda env)")
    sys.exit(1)

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Error: Could not import ROS 2 modules (rosbag2_py, rclpy, rosidl_runtime_py). Are you sure you sourced your ROS 2 environment? Or check if you are in the correct Docker/Conda environment!")
    sys.exit(1)

def convert_bag_to_mp4(bag_path, out_mp4='output.mp4', topic_name='/locobot/camera/camera/color/image_raw/compressed'):
    if not os.path.exists(bag_path):
        print(f"Bag path {bag_path} does not exist.")
        sys.exit(1)

    print(f"Opening bag: {bag_path}")
    reader = rosbag2_py.SequentialReader()
    
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
        
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f"[Warning] Topic {topic_name} not found in the bag.")
        print(f"Available topics: {list(type_map.keys())}")
        # We don't exit here immediately in case the topic name was a partial match or to let user see available topics clearly.
        sys.exit(1)
        
    msg_type = get_message(type_map[topic_name])
    
    print(f"Processing topic: {topic_name}")
    
    video_writer = None
    fps = 30.0 # Default fps, can be adjusted 
    
    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            
            cv_img = None
            if hasattr(msg, 'format') and hasattr(msg, 'data'):
                # CompressedImage
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif hasattr(msg, 'encoding') and hasattr(msg, 'data'):
                # Image
                # This is a fallback assumption
                if msg.encoding in ["rgb8", "bgr8"]:
                    channels = 3
                    cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
                    if msg.encoding == "rgb8":
                         cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else: 
                     print(f"Unsupported encoding: {msg.encoding}")
                     continue

            if cv_img is None:
                continue

            height, width = cv_img.shape[:2]
            
            if video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(out_mp4, fourcc, fps, (width, height))
            
            video_writer.write(cv_img)
            count += 1
            if count % 50 == 0:
                print(f"Processed {count} frames...")

    if video_writer is not None:
        video_writer.release()
        print(f"Successfully saved {count} frames to {out_mp4}")
    else:
        print("No images found to save.")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 8_bag2mp4.py <bag_dir_or_file> [output.mp4] [topic_name]")
        sys.exit(1)
        
    bag_path = sys.argv[1]
    # Set output name to bag name + .mp4 if not specified
    out_mp4 = sys.argv[2] if len(sys.argv) > 2 else bag_path.rstrip('/') + ".mp4"
    topic_name = sys.argv[3] if len(sys.argv) > 3 else "/locobot/camera/camera/color/image_raw/compressed"
    
    convert_bag_to_mp4(bag_path, out_mp4, topic_name)
