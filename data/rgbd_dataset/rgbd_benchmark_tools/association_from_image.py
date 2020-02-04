import argparse
import glob
import os


def parse_args():
    parser = argparse.ArgumentParser(description='tracking demo')
    parser.add_argument('--data_path', type=str, help='data file path',
                        default='data_kitti_05')
    parser.add_argument('--rgb_folder', type=str, help='rgb folder name',
                        default='rgb_img')
    parser.add_argument('--depth_folder', type=str, help='depth folder name',
                        default='depth_img')
    parser.add_argument('--save_name', type=str, help='association file name',
                        default='assoc.txt')

    args = parser.parse_args()
    return args


def main():
    # read input
    args = parse_args()
    print('Associating ', args.data_path, '...')

    # get rgb and depth images files
    rgb_path = os.path.join(args.data_path, args.rgb_folder, '*')
    rgb_files = sorted(glob.glob(rgb_path))
    depth_path = os.path.join(args.data_path, args.depth_folder, '*')
    depth_files = sorted(glob.glob(depth_path))

    # save file into txt file
    save_path = os.path.join(args.data_path, args.save_name)
    file = open(save_path, "w")
    for rgb_img, depth_img in zip(rgb_files, depth_files):
        rgb_id = os.path.splitext(os.path.basename(rgb_img))[0]
        depth_id = os.path.splitext(os.path.basename(depth_img))[0]
        rgb = os.path.join(args.rgb_folder, os.path.basename(rgb_img))
        depth = os.path.join(args.depth_folder, os.path.basename(depth_img))
        line = [rgb_id, ' ', rgb, ' ', depth_id, ' ', depth, '\n']
        file.writelines(line)
    file.close()
    print('Finished! Association file saved at', save_path)


if __name__ == '__main__':
    main()
