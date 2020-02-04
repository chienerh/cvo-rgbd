folder="/home/cel/PERL/datasets/rgbd_dataset/"
cvo_version="cvo"
dataset="freiburg1_xyz/"
output_file="cvo_tracking_sift.txt"
semantic_folder="sift_flow/"
mask_folder="test"

for dataset in "freiburg1_xyz"
do
    for s_sigma in 1.0 1.2 1.4 1.6 1.8 2.0
    do
        for s_ell in 500 1000 1500 2000 2500 3000
        do
            savefile="/home/cel/PERL/Algorithms/rkhs_registration/dev/cpp/rkhs_se3_registration/result/sift_flow_"$dataset"_"$s_sigma"_"$s_ell".txt"
            logfile="/home/cel/PERL/Algorithms/rkhs_registration/dev/cpp/rkhs_se3_registration/result/sift_flow_"$dataset"_"$s_sigma"_"$s_ell"_tflog.txt"
            cd /home/cel/PERL/Algorithms/rkhs_registration/dev/cpp/rkhs_se3_registration/build
            ../build/$cvo_version $folder$dataset"/" 1 $output_file $semantic_folder $mask_folder $s_ell $s_sigma >> $logfile

            echo "-------------------------------"
            echo "Dataset: "$dataset", s_sigma: "$s_sigma", s_ell: "$s_ell
            cd /home/cel/PERL/Algorithms/rkhs_registration/data/rgbd_dataset/rgbd_benchmark_tools

            # start evaluation
            python3 evaluate_rpe.py $folder$dataset/groundtruth.txt $folder$dataset/cvo_tracking_sift.txt --fixed_delta --verbose --plot $folder$dataset/figure/rpe.png --title $dataset >> $savefile
            python3 evaluate_rpe.py $folder$dataset/groundtruth.txt $folder$dataset/cvo_tracking_sift.txt --fixed_delta --verbose --plot $folder$dataset/figure/rpe.png --title $dataset
            
        done
    done
done