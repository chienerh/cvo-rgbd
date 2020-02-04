path="/media/justin/LaCie/data/rgbd_dataset/"
#path="../"
data_folder="freiburg1_desk"

python3 evaluate_ate.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_tracking_sift.txt --verbose --plot $path$data_folder/figure/ate.png --title $data_folder
