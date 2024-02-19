import argparse
import os

def create_data_structure(annotated_dir):
    # Copy labels and images to data/obj/
    os.system(f"cp {annotated_dir}/train/*.txt data/obj/")
    os.system(f"cp {annotated_dir}/valid/*.txt data/obj/")
    os.system(f"cp {annotated_dir}/train/*.jpg data/obj/")
    os.system(f"cp {annotated_dir}/valid/*.jpg data/obj/")

    # Write obj.data file
    with open('data/obj.data', 'w') as out:
        out.write('classes = 3\n')
        out.write('train = data/train.txt\n')
        out.write('valid = data/valid.txt\n')
        out.write('names = data/obj.names\n')
        out.write('backup = backup/')

    # Write train.txt file
    with open('data/train.txt', 'w') as out:
        for img in [f for f in os.listdir(f"{annotated_dir}/train") if f.endswith('jpg')]:
            out.write(f"data/obj/{img}\n")

    # Write valid.txt file
    with open('data/valid.txt', 'w') as out:
        for img in [f for f in os.listdir(f"{annotated_dir}/valid") if f.endswith('jpg')]:
            out.write(f"data/obj/{img}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create data structure for training object detection model.')
    parser.add_argument('annotated_dir', type=str, help='Path to annotated directory')
    args = parser.parse_args()

    create_data_structure(args.annotated_dir)
