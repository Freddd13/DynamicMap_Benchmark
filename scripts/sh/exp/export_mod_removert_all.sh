data_folder=/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt
benchmarks_path=/home/yxt/code/slam/dynamic_benchmark_ws/src/DynamicMap_Benchmark

# recompile scripts
cd ${benchmarks_path}/${method}/scripts
cmake -B build && cmake --build build 

# ====> export output for eval
voxelsize=0.2 # since all voxel-based methods resolution is 0.1 / 2 = 0.05
min_dis=0.05 # since all voxel-based methods resolution is 0.1 / 2 = 0.05
for seq_num in 00 05 semindoor
do
    data_path=${data_folder}/data/${seq_num}
    # ${benchmarks_path}/scripts/build/export_prrr_gt_pcd ${data_path} ${voxelsize}
    # echo "exported ${method_name} seq ${seq_num} prrr gt with voxelsize ${voxelsize}"
    for method_name in mod_removert
    do
        # for prrr
        ${benchmarks_path}/scripts/build/export_prrr_est_pcd ${data_path} ${method_name}_output.pcd ${voxelsize}
        echo "exported ${method_name} seq ${seq_num} prrr est and gt"
        # for benchmark
        ${benchmarks_path}/scripts/build/export_eval_pcd ${data_path} ${method_name}_output.pcd ${min_dis}
        echo "exported ${method_name} seq ${seq_num} check ${method_name}_exportGT.pcd"       
    done
done