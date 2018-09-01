#include <hdl_graph_slam/registrations.hpp>

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

namespace hdl_graph_slam {

//这里主要调用的是pcl库，可以选择的库有ICP方法，GICP，NDT
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method(ros::NodeHandle& pnh) {
  using PointT = pcl::PointXYZI;

          // select a registration method (ICP, GICP, NDT, Ndt_omp, GICP_omp)
          // This package provides an OpenMP-boosted NDT algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded.
          // It can run up to 10 times faster than its original version in pcl.
  std::string registration_method = pnh.param<std::string>("registration_method", "NDT_OMP");
  if(registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    return icp;
  } else if(registration_method.find("GICP") != std::string::npos) {
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      return gicp;
    }
  } else {
    if(registration_method.find("NDT") == std::string::npos ) {
      std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
      std::cerr << "       : use NDT" << std::endl;
    }

    //这里设置ndt 算法的分辨率
    double ndt_resolution = pnh.param<double>("ndt_resolution", 0.5);
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: NDT " << ndt_resolution << std::endl;
      boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      //这里设置ndt算法的线程数目
      //默认的launch文件设置是0，意思没有调用，，对于比较好的机器我认为可以开多线程这样匹配的速度更快
      int num_threads = pnh.param<int>("ndt_num_threads", 0);
      std::string nn_search_method = pnh.param<std::string>("ndt_nn_search_method", "DIRECT7");
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
      boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if(num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      //ndt设置的迭代阈值
      ndt->setTransformationEpsilon(0.01);
      //设置最大迭代次数
      ndt->setMaximumIterations(64);
      //设置分辨率大小
      ndt->setResolution(ndt_resolution);
      //nn search 方法可以设置成KDTREE,这就跟传统的ndt方法没啥区别，速度比较 DIRECT1》DIRECT7， 但是DIRECT7更稳定
      if(nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if (nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
    }
  }

  return nullptr;
}

}
