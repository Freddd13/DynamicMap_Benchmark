/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author: Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-12 18:14
 * @details: TODO
 * 
 * Input: Algorithm Result PCD file, and GT PC file
 * Output: raw map pcd file with label 0/1
 */

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include "timer.h"

/*
处理用于pr rr 评估的最终gt与est file
input: 原始给的GT cloud、算法输出的结果pcd、voxelsize
output: 输出到数据集目录/prrr_eval/下，同voxelsize下采样的gt与est pcd用于评测

1. 从原始GT为res点云下采样后找寻标签后保存至相应prrr_eval目录
*/

void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::ConstPtr src,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr dst,
                                pcl::PointCloud<pcl::PointXYZI>::ConstPtr ref,
                                double leaf_size) {
  /**< IMPORTANT
   * Because PCL voxlizaiton just does average the intensity of point cloud,
   * so this function is to conduct voxelization followed by nearest points
   * search to re-assign the label of each point */
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(
      new pcl::PointCloud<pcl::PointXYZI>);

  // 1. Voxelization
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(src);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);

  // 2. Find nearest point to update intensity (index and id)
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(ref);

  ptr_reassigned->points.reserve(ptr_voxelized->points.size());

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // Set dst <- output
  for (const auto& pt : ptr_voxelized->points) {
    if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      auto updated = pt;
      // Update meaned intensity to original intensity
      // if (((*src)[pointIdxNKNSearch[0]].intensity)!= 0) {
      //   std::cout << "intensity before: " <<
      //   ((*src)[pointIdxNKNSearch[0]].intensity) << std::endl;
      // }
      updated.intensity = (*ref)[pointIdxNKNSearch[0]].intensity;
      ptr_reassigned->points.emplace_back(updated);
    }
  }
  *dst = *ptr_reassigned;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 4) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [pcd_folder] [algorithm_output.pcd] [min_dis_cnt_as_same]";
        return 1;
    }

    std::string pcd_folder = argv[1];
    std::string run_pcd_name = argv[2];
    float voxelsize = std::stof(argv[3]);

    // check if the folder exists
    if (!std::filesystem::exists(pcd_folder)) {
        LOG(ERROR) << "File does not exist: " << pcd_folder;
        return -1;
    }
    std::string export_est_pcd_name =
        run_pcd_name.substr(0, run_pcd_name.size() - 4) + "_export_est.pcd";
    std::filesystem::path folder_path(pcd_folder);
    if (!std::filesystem::exists(folder_path / run_pcd_name)) {
        LOG(ERROR) << "File does not exist: " << run_pcd_name << " in " << pcd_folder;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr gt_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr et_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path / "gt_cloud.pcd", *gt_cloud) == -1) {
        LOG(ERROR) << "Couldn't read gt_cloud.pcd";
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path / run_pcd_name, *et_cloud) == -1) {
        LOG(ERROR) << "Couldn't read:" << run_pcd_name;
        return -1;
    }



    TIC;
    // 1. 对结果pcd下采样后，从gt cloud上给找回标签，存est
    pcl::PointCloud<pcl::PointXYZI>::Ptr et_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    voxelize_preserving_labels(et_cloud, et_cloud_downsampled, gt_cloud,
                               voxelsize);
    et_cloud_downsampled->width = et_cloud_downsampled->points.size();
    et_cloud_downsampled->height = 1;

    // 3. save est and gt
    if (!std::filesystem::exists(folder_path / "prrr_eval")) {
      std::filesystem::create_directory(folder_path / "prrr_eval");
    }
    pcl::io::savePCDFileBinary(folder_path / "prrr_eval" / export_est_pcd_name,
                               *et_cloud_downsampled);
    LOG(INFO) << "Exported est PCD file: " << export_est_pcd_name << " in "
              << pcd_folder;
    TOC("Export PRRR Estimate Label PCD file based on GT", true);
    return 0;
}
