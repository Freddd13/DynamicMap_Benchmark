data_folder=/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt
benchmarks_path=/home/yxt/code/slam/dynamic_benchmark_ws/src/DynamicMap_Benchmark/methods

# ====> export output for eval
voxelsize=0.2 # since all voxel-based methods resolution is 0.1 / 2 = 0.05
for seq_num in 00 05 semindoor av
do
    for method_name in removert erasor octomap octomapf octomapfg
    do
        data_path=${data_folder}/data/${seq_num}
        ${benchmarks_path}/benchmarks/build/export_eval_pcd ${data_path} ${method_name}_output.pcd ${voxelsize}
        echo "exported ${method_name} seq ${seq_num} prrr est and gt"
    done
done