# start evaluation
path="/media/justin/LaCie/data/rgbd_dataset/"
data_folder="freiburg1_desk"

for semantic in iter_10000 iter_20000
do
    for mask in 3000
    do
        for grid in 1
        do
            for sample in sample_1 sample_3 sample_5 sample_-1
                do
            
            file_name="/"$semantic"_"$mask"_g"$grid"_"$sample".txt"
            python3 evaluate_rpe.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --fixed_delta --verbose --plot $path$data_folder/figure/rpe.png --title $semantic"_"$mask"_grid"$grid
            # python3 evaluate_ate.py $path$data_folder/groundtruth.txt $path$data_folder$file_name --verbose --plot $path$data_folder/figure/ate.png --title $data_folder
            done
        done    
    done
done