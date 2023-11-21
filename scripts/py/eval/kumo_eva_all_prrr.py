
'''
输入采样率相同的gt与est点云, 输出算法pr rr f1 结果
'''
import numpy as np
from tabulate import tabulate

from time import time
import sys, os, math
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)
from utils import cnt_staticAdynamic, check_file_exists, bc
import calc_prrr

# TODO: Change the parameters below to your own settings ====>>
Result_Folder = "/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt/"
algorithms = ["removert", "erasor", "octomap", "octomapg", "octomapfg"]
all_seqs = ["00", "05", "av2", "semindoor"]

voxel_size_ = 0.2
# TODO: Change the parameters below to your own settings <<===

if __name__ == "__main__":
    st_time = time()
    for seq in all_seqs:
        gt_pcd_path = f"{Result_Folder}/{seq}/prrr_export_gt.pcd"
        gt_pc_ = load_pcd(check_file_exists(gt_pcd_path))
        printed_data = []
        for algo in algorithms:
            est_pcd_path = f"{Result_Folder}/{seq}/prrr_eval/{algo}_export_est.pcd"
            est_pc_ = load_pcd(check_file_exists(est_pcd_path))

            assert et_pc_.np_data.shape[0] == gt_pc_.np_data.shape[0] , \
                "Error: The number of points in et_pc_ and gt_pc_ do not match.\
                \nThey must match for evaluation, if not Please run `export_eval_pcd`."
            
            # kumo: 由于export_eval_pcd是从gt生成的算法结果点云，所以两者大小是一样的，都是gt的点云大小
            evaluate(gt_pc_, est_pc_, voxelsize=voxel_size_)
            

    print(f"Time cost: {(time() - st_time):.2f}s")