folder="/home/cel/PERL/datasets/rgbd_dataset/"
cvo_version="cvo"
dataset="freiburg1_xyz/"
output_file="cvo_tracking_sift.txt"
semantic_folder="sift/"
mask_folder="test"

../build/$cvo_version $folder$dataset 1 $output_file $semantic_folder $mask_folder