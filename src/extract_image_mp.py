#! /usr/bin/env python2
import argparse
import io
import multiprocessing
import os

import rosbag
from PIL import Image as PilImage
from tqdm import tqdm


def find_bag_files(bag_file_root):
    bag_file_dir = []
    bag_files = []
    for root, dirs, files in os.walk(bag_file_root):
        for file in files:
            if file.endswith('.bag'):
                bag_files.append(os.path.join(root, file))
                bag_file_dir.append(root)
    return bag_file_dir, bag_files

def extract_images(bag_file, bag_file_dir, args):
    sub_time_dir = bag_file_dir.split('/')[-1]
    if bag_file.split('/')[-1].split('.')[0][0] == '_':
        sub_time_dir += bag_file.split('/')[-1].split('.')[0]
    else:
        sub_time_dir += '_' + bag_file.split('/')[-1].split('.')[0]
    if not os.path.exists(os.path.join(args.output_dir, sub_time_dir)):
        os.makedirs(os.path.join(args.output_dir, sub_time_dir))
    
    output_time_dir = os.path.join(args.output_dir, sub_time_dir)
    
    bag = rosbag.Bag(bag_file, "r")

    count = {}
    total_count = 0
    bag_length = bag.get_message_count()
    
    for topic, msg, t in tqdm(bag.read_messages(), total=bag_length, leave=False):
        if msg._type == 'sensor_msgs/CompressedImage':
            sub_topic_dir = topic.replace('/', '_').strip('_')
            output_dir = os.path.join(output_time_dir, sub_topic_dir)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            if topic not in count:
                count[topic] = 0
            
            image = PilImage.open(io.BytesIO(msg.data))
            rotated_image = image.rotate(180)
            rotated_image.save(os.path.join(output_dir, "frame%06i_%06i.png" % (count[topic], total_count)), 'PNG', compress_level=6)
            count[topic] += 1
            total_count += 1

    bag.close()

def process_bag_file(args, bag_file, bag_file_dir):
    extract_images(bag_file, bag_file_dir, args)

def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file_root", nargs='?', default="./bags/2023_0829")
    parser.add_argument("output_dir", nargs='?', default="")

    args = parser.parse_args()

    if args.output_dir == "":
        args.output_dir = os.path.join(args.bag_file_root, "images_mp")
        if not os.path.exists(args.output_dir):
            os.makedirs(args.output_dir)
    
    bag_file_dir, bag_files = find_bag_files(args.bag_file_root)
    print("Output directory: {}".format(args.output_dir))
    print("")
    print("Bag file directory: {}".format(bag_file_dir))
    print("")
    print("Bag files: {}".format(bag_files))

    processes = []

    for bag_file, bag_file_dir in zip(bag_files, bag_file_dir):
        process = multiprocessing.Process(target=process_bag_file, args=(args, bag_file, bag_file_dir))
        processes.append(process)
        process.start()
    
    for process in processes:
        process.join()

if __name__ == '__main__':
    main()
