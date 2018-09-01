#include <hdl_graph_slam/information_matrix_calculator.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace hdl_graph_slam {

InformationMatrixCalculator::InformationMatrixCalculator(ros::NodeHandle& nh) {
  use_const_inf_matrix = nh.param<double>("use_const_inf_matrix", false);
  const_stddev_x = nh.param<double>("const_stddev_x", 0.5);
  const_stddev_q = nh.param<double>("const_stddev_q", 0.1);

  var_gain_a = nh.param<double>("var_gain_a", 20.0);
  min_stddev_x = nh.param<double>("min_stddev_x", 0.1);
  max_stddev_x = nh.param<double>("max_stddev_x", 5.0);
  min_stddev_q = nh.param<double>("min_stddev_q", 0.05);
  max_stddev_q = nh.param<double>("max_stddev_q", 0.2);
  fitness_score_thresh = nh.param<double>("fitness_score_thresh", 0.5);
}

InformationMatrixCalculator::~InformationMatrixCalculator() {

}

/**
 * 计算信息矩阵
 * @param cloud1 闭环帧点云1
 * @param cloud2 闭环帧点云2
 * @param relpose  两帧点云的相对位置关系
 * @return
 */

Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const {

  //信息矩阵是否是常值
  if(use_const_inf_matrix) {
    //设置成单位矩阵
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    //上三角是位姿态，分别除以0.5之类的常数
    inf.topLeftCorner(3, 3).array() /= const_stddev_x;
    //下三角是角度，分别除以0.5之类的常数
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
    //返回信息矩阵
    return inf;
  }

  //根据fitness_score来动态改变信息矩阵
  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

  //这里设置两点之间的最大max_var_x（5），最小距离min_var_x（0.1）；最大 max_var_q（0.2），最小角度 min_var_q（0.05）
  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  /**
   * var_gain_a 表示增益,launch 里面设置是20； fitness_score_thresh 表示设置的最大阈值为2.0；
   */

  //设置x位姿的权重
  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  //设置q的权重
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  //除以动态权重大小
   inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}

/**
 * 该函数主要计算fitness_score
 * @param cloud1  闭环帧点云1
 * @param cloud2  闭环帧点云2
 * @param relpose 相对位姿态
 * @param max_range 设置最大量程
 * @return 返回的对应点之间的平均距离
 */

double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range) const {

  //建立kd-tree
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());

  //设置输入目标点云到KD-tree;
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;

  //通过点云1与点云2之间的位姿关系，将点云2变换到点云1附近
  pcl::transformPointCloud (*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;

  //对于点云2，每一个点寻找其点云1对应的临近点，并记录点云1对应的标号以对应的距离
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target

            //在点云1中寻找最近的目标点，记录点的距离以及标号
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)

    //如果计算的距离<max_range时

    if (nn_dists[0] <= max_range)
    {
      // Add to the fitness score
      //fitness score 这里记录所有点的距离总和
      fitness_score += nn_dists[0];
      nr++;
    }

  }

  if (nr > 0)
    //这实际返回的是满足要求的平均距离
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

}

