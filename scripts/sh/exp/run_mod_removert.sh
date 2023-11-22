data_folder=/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt
benchmarks_path=/home/yxt/code/slam/dynamic_benchmark_ws/src/DynamicMap_Benchmark/methods

# ====> build methods, you can also build them manually only once
for method in mod_removert
do
    cd ${benchmarks_path}/${method}
    cmake -B build && cmake --build build
done

# ====> run methods
for seq_num in 00 05 semindoor
do
    data_path=${data_folder}/data/${seq_num}
    echo "Processing sequence ${seq_num} ... in mod_removert"
    if [ "${seq_num}" \> "av2" ]; then
        ${benchmarks_path}/mod_removert/build/mod_removert_run ${data_path} ${benchmarks_path}/mod_removert/config/params_av.yaml -1
    else
        ${benchmarks_path}/mod_removert/build/mod_removert_run ${data_path} ${benchmarks_path}/mod_removert/config/params_kitti.yaml -1
    fi
done