path="/home/cel/PERL/datasets/rgbd_dataset/"
dataset="freiburg1_desk/"
file_name="acvo_poses_qt.txt"
gt_pth="/home/cel/PERL/datasets/rgbd_dataset/"$dataset"/groundtruth.txt"
traj_1_pth="/home/cel/PERL/datasets/rgbd_dataset/"$dataset$file_name
traj_2_pth="/home/cel/PERL/datasets/rgbd_dataset/"$dataset"cvo_tracking_sift.txt"
traj_3_pth="/home/cel/PERL/datasets/rgbd_dataset/"$dataset"cvo_poses_qt.txt"
# file_name="cvo_kf_tracking.txt"

evo_traj tum --ref $gt_pth --align_origin $traj_1_pth  $traj_2_pth -p
