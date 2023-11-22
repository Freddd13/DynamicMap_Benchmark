data_folder=/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt
benchmarks_path=/home/yxt/code/slam/dynamic_benchmark_ws/src/DynamicMap_Benchmark
# ====> export output for eval
min_dis=0.05 # since all voxel-based methods resolution is 0.1 / 2 = 0.05
for seq_num in 00 05 semindoor av2
do
    for method_name in removert erasor octomap octomapg octomapfg
    do
        data_path=${data_folder}/data/${seq_num}
        ${benchmarks_path}/scripts/build/export_eval_pcd ${data_path} ${method_name}_output.pcd ${min_dis}
        echo "exported ${method_name} seq ${seq_num} check ${method_name}_exportGT.pcd"
    done
done