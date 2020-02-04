path="/home/cel/PERL/datasets/rgbd_dataset/"
# #path="../"
# data_folder="freiburg3_nostructure_notexture_far"
data_folder="freiburg1_xyz"

# start evaluation
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_tracking_sift.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder


