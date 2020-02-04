folder="/home/cel/PERL/datasets/rgbd_dataset/"
cvo_version="adaptive_cvo"
dataset="freiburg1_desk/"
output_file="setting_1_3000_g1.txt"
semantic_folder="setting_1/"
mask_folder="mask_top_5000/grid_4/"
# echo $semantic_folder
# ../build/$cvo_version $folder$dataset 1 $output_file $semantic_folder $mask_folder

for semantic in iter_10000 iter_20000
do
    for mask in 3000 
    do
        for grid in 1 
        do
            for sample in sample_1 sample_3 sample_5 sample_-1
            do
                echo "-------------------------------"
                echo "          ¯\_(ツ)_/¯"
                echo "-------------------------------"
                echo "Dataset: "$dataset
                echo "setting: "$semantic
                echo "# of points: "$mask
                echo "grid size: "$grid
                echo "sample: "$sample
                

                semantic_folder=$semantic"/"
                mask_folder="mask_top_"$mask"/grid_"$grid"/"$sample"/"
                output_file=$semantic"_"$mask"_g"$grid"_"$sample".txt"
                
                echo "writing result to: "$output_file
                echo "starting cvo... ¯\_(ツ)_/¯"
                ../build/$cvo_version $folder$dataset 1 $output_file $semantic_folder $mask_folder
            done
        done    
    done
done

# folder="/media/justin/LaCie/data/"
# cvo_version="adaptive_cvo"

# echo "-------------------------------"
# echo "kitti_05_all"
# dataset="kitti_05_all/"
# ../build/$cvo_version $folder$dataset 5
