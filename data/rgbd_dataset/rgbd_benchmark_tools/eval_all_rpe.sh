# start evaluation
path="/media/justin/LaCie/data/rgbd_dataset/"
# file_name="/old_cvo_poses_qt.txt"
file_name="/dvo_traj.txt"

echo "-------------------------------"
echo "freiburg1_desk 1"
data_folder="freiburg1_desk"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_desk2 1"
data_folder="freiburg1_desk2"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_room 1"
data_folder="freiburg1_room"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_360 1"
data_folder="freiburg1_360"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_teddy 1"
data_folder="freiburg1_teddy"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_floor 1"
data_folder="freiburg1_floor"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_xyz 1"
data_folder="freiburg1_xyz"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_rpy 1"
data_folder="freiburg1_rpy"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg1_plant 1"
data_folder="freiburg1_plant"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder



echo "-------------------------------"
echo "freiburg3_nostructure_notexture_far 1"
data_folder="freiburg3_nostructure_notexture_far"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_nostructure_notexture_near 2"
data_folder="freiburg3_nostructure_notexture_near"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_nostructure_texture_far 3"
data_folder="freiburg3_nostructure_texture_far"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_nostructure_texture_near 4"
data_folder="freiburg3_nostructure_texture_near"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_structure_notexture_far 5"
data_folder="freiburg3_structure_notexture_far"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_structure_notexture_near 6"
data_folder="freiburg3_structure_notexture_near"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_structure_texture_far 7"
data_folder="freiburg3_structure_texture_far"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder

echo "-------------------------------"
echo "freiburg3_structure_texture_near 8"
data_folder="freiburg3_structure_texture_near"
python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $data_folder
