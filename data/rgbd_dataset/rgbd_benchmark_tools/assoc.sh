# path="/media/justin/LaCie/data/rgbd_dataset/"
# folder="freiburg1_xyz_v"

# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt


# start evaluation
path="/media/justin/LaCie/data/rgbd_dataset/"

for d in /media/justin/LaCie/data/rgbd_dataset/*/ ; do
    echo "$d"
    rm $d"assoc.txt"
    python3 associate.py $d"rgb.txt" $d"depth.txt" >> $d"assoc.txt"
done


# echo "-------------------------------"
# echo "freiburg1_desk 1"
# folder="freiburg1_desk"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_desk2 1"
# folder="freiburg1_desk2"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_room 1"
# folder="freiburg1_room"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_360 1"
# folder="freiburg1_360"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_teddy 1"
# folder="freiburg1_teddy"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_floor 1"
# folder="freiburg1_floor"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_xyz 1"
# folder="freiburg1_xyz"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_rpy 1"
# folder="freiburg1_rpy"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg1_plant 1"
# folder="freiburg1_plant"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt



# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_far 1"
# folder="freiburg3_nostructure_notexture_far"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_far 2"
# folder="freiburg3_nostructure_texture_far"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg3_structure_notexture_far 3"
# folder="freiburg3_structure_notexture_far"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg3_nostructure_notexture_near 4"
# folder="freiburg3_nostructure_notexture_near"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg3_nostructure_texture_near 5"
# folder="freiburg3_nostructure_texture_near"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt

# echo "-------------------------------"
# echo "freiburg3_structure_notexture_near 6"
# folder="freiburg3_structure_notexture_near"
# python3 associate.py $path$folder/rgb.txt $path$folder/depth.txt >> $path$folder/assoc.txt
