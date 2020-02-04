# echo "-------------------------------"
# echo "freiburg1_desk 1"
# ../build/testrkhs freiburg1_desk 1
# echo "-------------------------------"
# echo "freiburg1_desk2 1"
# ../build/testrkhs freiburg1_desk2 1
# echo "-------------------------------"
# echo "freiburg1_room 1"
# ../build/testrkhs freiburg1_room 1
# echo "-------------------------------"
# echo "freiburg1_360 1"
# ../build/testrkhs freiburg1_360 1
# echo "-------------------------------"
# echo "freiburg1_teddy 1"
# ../build/testrkhs freiburg1_teddy 1
# echo "-------------------------------"
# echo "freiburg1_xyz 1"
# ../build/testrkhs freiburg1_xyz 1
# echo "-------------------------------"
# echo "freiburg1_rpy 1"
# ../build/testrkhs freiburg1_rpy 1
# echo "-------------------------------"
# echo "freiburg1_plant 1"
# ../build/testrkhs freiburg1_plant 1
# echo "-------------------------------"
# echo "freiburg1_floor 1"
# ../build/testrkhs freiburg1_floor 1

# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_far 1"
# ../build/testrkhs freiburg3_nostructure_notexture_far 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_near 2"
# ../build/testrkhs freiburg3_nostructure_notexture_near 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_far 3"
# ../build/testrkhs freiburg3_nostructure_texture_far 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_near 4"
# ../build/testrkhs freiburg3_nostructure_texture_near 3
# echo "-------------------------------"
# echo "freiburg3_structure_notexture_far 5"
# ../build/testrkhs freiburg3_structure_notexture_far 3
# echo "-------------------------------"
# echo "freiburg3_structure_notexture_near 6"
# ../build/testrkhs freiburg3_structure_notexture_near 3
# echo "-------------------------------"
# echo "freiburg3_structure_texture_far 7"
# ../build/testrkhs freiburg3_structure_texture_far 3
# echo "-------------------------------"
# echo "freiburg3_structure_texture_near 8"
# ../build/testrkhs freiburg3_structure_texture_near 3

# echo "-------------------------------"
# echo "freiburg1_desk_v 1"
# ../build/testrkhs freiburg1_desk_v 1
# echo "-------------------------------"
# echo "freiburg1_desk2_v 1"
# ../build/testrkhs freiburg1_desk2_v 1
# echo "-------------------------------"
# echo "freiburg1_room_v 1"
# ../build/testrkhs freiburg1_room_v 1
# echo "-------------------------------"
# echo "freiburg1_360_v 1"
# ../build/testrkhs freiburg1_360_v 1
# echo "-------------------------------"
# echo "freiburg1_teddy_v 1 skiping..."
# # ../build/testrkhs freiburg1_teddy_v 1
# echo "-------------------------------"
# echo "freiburg1_xyz_v 1"
# ../build/testrkhs freiburg1_xyz_v 1
# echo "-------------------------------"
# echo "freiburg1_rpy_v 1"
# ../build/testrkhs freiburg1_rpy_v 1
# echo "-------------------------------"
# echo "freiburg1_plant_v 1"
# ../build/testrkhs freiburg1_plant_v 1
# echo "-------------------------------"
# echo "freiburg1_floor 1 skipping..."
# ../build/testrkhs freiburg1_floor_v 1

# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_far_v 1"
# ../build/testrkhs freiburg3_nostructure_notexture_far_v 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_near_v 2"
# ../build/testrkhs freiburg3_nostructure_notexture_near_v 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_far_v 3"
# ../build/testrkhs freiburg3_nostructure_texture_far_v 3
# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_near_v 4"
# ../build/testrkhs freiburg3_nostructure_texture_near_v 3
echo "-------------------------------"
echo "freiburg3_structure_notexture_far_v 5"
../build/testrkhs freiburg3_structure_notexture_far_v 3
echo "-------------------------------"
echo "freiburg3_structure_notexture_near_v 6"
../build/testrkhs freiburg3_structure_notexture_near_v 3
echo "-------------------------------"
echo "freiburg3_structure_texture_far_v 7"
../build/testrkhs freiburg3_structure_texture_far_v 3
echo "-------------------------------"
echo "freiburg3_structure_texture_near_v 8"
../build/testrkhs freiburg3_structure_texture_near_v 3




# start evaluation
# path="/media/justin/LaCie/data/rgbd_dataset/"

# data_folder="freiburg1_desk"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_desk2"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_room"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_360"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_teddy"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_floor"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_xyz"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_rpy"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg1_plant"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_nostructure_notexture_far"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_nostructure_texture_far"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_structure_notexture_far"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_nostructure_notexture_near"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_nostructure_texture_near"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

# data_folder="freiburg3_structure_notexture_near"
# python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder/cvo_poses_qt.txt --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder


