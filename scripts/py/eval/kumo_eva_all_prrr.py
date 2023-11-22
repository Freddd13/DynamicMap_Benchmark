#! --*-- coding: utf-8 --*--
'''
输入采样率相同的gt与est点云, 输出算法pr rr f1 结果
'''
import numpy as np
from tabulate import tabulate

from time import time
import sys, os, math
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)
from utils.pcdpy3 import load_pcd
from utils import cnt_staticAdynamic, check_file_exists, bc
from calc_prrr import CalcPRRR

# TODO: Change the parameters below to your own settings ====>>
# Result_Folder = "/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt/data"
# algorithms = ["removert", "erasor", "octomap"]
# all_seqs = ["00", "05", "semindoor"]
# all_seqs = ["00", "05", "av2", "semindoor"]

Result_Folder = "/mnt/d/dataset/DUFOMap_provided_dynamic_pcd_w_gt/data"
algorithms = ["mod_removert"]
all_seqs = ["00", "05", "semindoor"]

voxel_size_ = 0.2
# TODO: Change the parameters below to your own settings <<===

if __name__ == "__main__":
    st_time = time()
    evaluator = CalcPRRR()
    full_res = []
    for seq in all_seqs:
        gt_pcd_path = f"{Result_Folder}/{seq}/prrr_export_gt.pcd"
        gt_pc_ = load_pcd(check_file_exists(gt_pcd_path))
        printed_data = []
        for algo in algorithms:
            est_pcd_path = f"{Result_Folder}/{seq}/prrr_eval/{algo}_output_export_est.pcd"
            est_pc_ = load_pcd(check_file_exists(est_pcd_path))

            # print(f"num / {seq} / {algo} / gt: {gt_pc_.np_data.shape[0]} / est: {est_pc_.np_data.shape[0]}")
            # assert est_pc_.np_data.shape[0] == gt_pc_.np_data.shape[0] , \
            #     "Error: The number of points in et_pc_ and gt_pc_ do not match.\
            #     \nThey must match for evaluation, if not Please run `export_eval_pcd`."
            
            # kumo: 由于export_eval_pcd是从gt生成的算法结果点云，所以两者大小是一样的，都是gt的点云大小
            evaluator.set_dynamic_classes([1])
            evaluator.set_dataset_name(seq)
            evaluator.set_algorithm_name(algo)
            eva_data = evaluator.evaluate(gt_pc_, est_pc_, voxelsize=voxel_size_)
            # evaluator.print_content()
            full_res.append(eva_data)
    
    print(tabulate(full_res, headers=['Dataset', 'Algorithm', 'ori_static pts', 'ori_dynamic pts', 'original_dy_raio%', 'remaining dy num', 'ghost rate', 'PR', 'RR', 'F1'], tablefmt='github'))

    print(f"Time cost: {(time() - st_time):.2f}s")