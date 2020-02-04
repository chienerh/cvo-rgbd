path="/media/justin/LaCie/data/rgbd_dataset/"
# dataset="freiburg3_nostructure_notexture_far"
dataset="freiburg1_desk"
# dataset="freiburg3_structure_notexture_near"
traj_cvo="/iter_10000_3000_g1.txt"
traj_dvo="/DVO.txt"
gt="/groundtruth.txt"

evo_traj tum --ref $path$dataset$gt  --align_origin $path$dataset$traj_cvo $path$dataset$traj_dvo -p --plot_mode xy --save_plot $path$dataset/test2.pdf
# evo_traj tum --ref $path$dataset$gt  --align_origin $path$dataset$traj_cvo $path$dataset$traj_dvo -p --plot_mode xy