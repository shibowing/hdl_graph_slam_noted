# hdlgraph slam 算法解析
#### 该算法主要分为 prefiltering; floor_detection; scan_matching; hdl_graph_slam
****
 ## prefilterfing.cpp      
   *  这部分主要是对点云实现预处理功能，主要包括点云网格滤波，点云离群点outlier 的去除 （这里可以选择statistic或者 raduis两种方式进行滤波），以及距离滤波（只要点云在指定距离范围内的点）
   ****
  
   ## floor_detection.cpp
  * 该cpp 主要接收 prefiltering .cpp  滤波后的点云，并完成地面检测
  * 主要是  boost::optional<Eigen::Vector4f> floor = detect(cloud); 函数功能
       * 可以设置tilt_matrix对Y轴方向的激光倾斜角进行补偿
       * 紧接着，调用 filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);函数，输入是滤波后的点云，输出是平面点云 ，vector 4f 对应的是想要的平面约束，plane clip 后续会根据平面约束滤出相关的点云
       * normal_filtering(filtered)；filtered表示平面点云， normal_filtering 主要通过非垂直方向对点云进行滤波
       *    pcl::RandomSampleConsensus<PointT> ransac(model_p);通过ransac 函数根据输入的准平面点云进行再一次滤波，主要滤出外点，同时保留内点，并计算平面内点所对应的平面方程系数
       
       
  ## scan_matching_odometry.cpp
 * 该函数主要计算相邻两帧之间的匹配，输入为prefilterfing.cpp滤波后得到的/filtered_points，紧接
 着进入 matching 函数
 ```
 matching（const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud）
 ```
 * step1: 首先判断是否是关键帧
 * step2: 这里主要调用PCL里面现成的库函数，主要有（ICP，GICP， NDT， GICP_OMP, NDT_OMP）,值得关注的是NDT_OMP，该算法通过采用多线程方式去完成NDT算法，因此效率是传统NDT算法的10倍，精度基本差不多； 这里如果选择KDTREE，那么和传统的NDT算法一致，剩下的速度比较： DIRECT1》DIRECT7， 但是DIRECT7更稳定同时速度比KDTREE 快
 * 选择完上述方法之后就开始完成匹配，这里主要是当前帧点云与Keyframe 进行匹配；
 * 有两个参数比较重要：**keyframe_delta_trans**
     **keyframe_delta_angle** 
                * The minimum tranlational distance and rotation angle between keyframes. If this value is zero, frames are always compared with the previous frame


   ## hdl_graph_slam.cpp
   * cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  该函数主要是odom信息与cloud信息的同步，同步之后检查关键帧是否更新
  * 关键帧判断 
     这里主要看关键帧设置的这两个阈值keyframe_delta_trans；keyframe_delta_angle；
 作者认为关键帧不能设置的太近，同时也不能太远；太近的关键帧会被抛弃，如果 keyframe_delta_trans=0 表示关键帧之间的距离为0，因此每一帧都是关键帧，scan_matching 函数就会变成相邻帧匹配； 变成关键帧的要求就是：

```
 
  // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = std::acos(Eigen::Quaterniond(delta.linear()).w());

    // too close to the previous frame
    if(dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }
    
 ```
 * optimization_timer_callback(const ros::TimerEvent& event) 将所有的位姿放在posegraph 中开始优化
  * loop detection 函数 ： 主要就是将当前帧和历史帧遍历，寻找loop
     *    寻找潜在闭环帧
     ```
     find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const
      ```
      * 由于每个关键帧都建立累计距离，当前帧对应的累计距离-边的累计距距离<distance_from_last_edge_thresh 那么就不能建立闭环，意思就是相邻两个闭环帧不能建立的过于密集
      * accum_distance_thresh 这里指的是当前帧与临近帧的距离阈值，假设距离阈值为5m, 那么当前帧距离5米范围内关键帧不再考虑
      * distance_thresh 小于该阈值范围内两个关键帧为潜在闭环帧，将所有满足条件的都存起来作为candidates

    
  * 潜在闭环完成匹配（matching 函数）
  
  ```
    Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe, hdl_graph_slam::GraphSLAM& graph_slam) {
    ```
    主要是当前帧与潜在闭环帧完成点云匹配，通过统计得分寻找最优匹配，并且只将最优匹配对应的transform作为最为闭环约束，这里对应的就是fitness_score_thresh
    
*     **全场亮点:计算不同loop的信息矩阵**
*     calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose)  这里relpose 表示闭环帧之间的位姿约束 
  * 首先判断是否使用常值信息矩阵，一般都是单位阵1/0.5=50  作为位姿， 1/0.1=10作为角度； 可以看出我们更相信位姿，而非角度；这里对应变量分别是：const_stddev_x const_stddev_q
  * 这里的fitness_socre 的计算非常朴素，主要是通过建立KD-TREE 在目标点云里面寻找source 点云每个点对应的最近点并记录两点间的距离
      fitness_score += nn_dists[i]; 
      return (fitness_score / n);
 上面这两个式子实际是记录所有点的距离，并将所有点的距离/n=平均距离； 通过求所有闭环帧KD-TREE查找对应点的平均距离来作为fitness_score    
 
*  gps对应的信息矩阵（这里体现g2o的强大☞处）作者主要看这几件事：
      * step1: GPS 坐标转换，主要是GPS地球坐标系->转换到大地坐标系（UTM)->UTM坐标系再转换成局部坐标
      * step2:这里只用gps的 X和Y信息，z 方向的信息不用，主要原因是不准，这就涉及设置信息矩阵，注意信息矩阵的类型：
      ```
      graph_slam->add_se3_prior_xy_edge((*seek)->node, xyz.head<2>(), information_matrix);
      ```
  
  ```
      //添加GPS约束，只给xy增加约束
g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
     //添加该顶点
    g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
    //添加gps对应的测量值（xy）还是UTM坐标
   edge->setMeasurement(xy);
   //添加对应的信息矩阵
  edge->setInformation(information_matrix);
   //添加该顶点
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}
```  

* 添加找对应的keyframe添加地面约束

     * step1：检测平面，这里主要涉及floor_detection.cpp 里面讲的平面检测；最后通过rassac 计算平面系数
     * step2：通过计算的平面系数来添加约束，主要对检测到平面的顶点，添加平面约束同时引进信息矩阵，信息矩阵是3*3的应该只对XYZ起到约束作用
     
  ```
      //添加平面约束又得设置是0.1，有的设置是100
      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      //添加平面约束
      graph_slam->add_se3_plane_edge(keyframe->node, graph_slam->floor_plane_node, coeffs, information);
```
```
//添加平面约束
g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
    //G2O本身自带平面这种边
    g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  //添加由ransssac 得到的平面系数
  edge->setMeasurement(plane_coeffs);
  //设置信息矩阵3*3
  edge->setInformation(information_matrix);
  //设置该顶点
  edge->vertices()[0] = v_se3;
  //以及该顶点对应的平面
  edge->vertices()[1] = v_plane;
  //添加平面
  graph->addEdge(edge);

  return edge;
}
```
# 调试参数含义
```
<?xml version="1.0"?>
<launch>
  <!-- arguments -->
  <param name="use_sim_time" value="true" />
  <arg name="nodelet_manager" default="velodyne_nodelet_manage" />
  <arg name="enable_floor_detection" default="true" />
  <arg name="enable_gps" default="true" />

  <!-- in case you use velodyne_driver, comment out the following line -->
  <node pkg="nodelet" type="nodelet" name="$(arg nodelet_manager)" args="manager" output="screen"/>

  <!-- prefiltering_nodelet -->
  <node pkg="nodelet" type="nodelet" name="prefiltering_nodelet" args="load hdl_graph_slam/PrefilteringNodelet $(arg nodelet_manager)">
    <param name="use_distance_filter" value="true" />   <!--使用距离滤波-->
    <param name="distance_near_thresh" value="1.0" />   <!--使用大于该距离范围的点云-->
    <param name="distance_far_thresh" value="50.0" />   <!--使用小于该距离范围的点云-->
    <!-- NONE, VOXELGRID, or APPROX_VOXELGRID -->
    <param name="downsample_method" value="VOXELGRID" />  <!--采取不同下采样的方法-->
    <param name="downsample_resolution" value="0.2" />   <!--下采用分辨率-->
    <!-- NONE, RADIUS, or STATISTICAL -->
    <param name="outlier_removal_method" value="RADIUS" />  <!--外点过滤-->
    <param name="statistical_mean_k" value="30" />         <!--统计的方式过滤外点，周围的neighbor数目最小30-->
    <param name="statistical_stddev" value="1.2" />         <!--统计的方式过滤外点，设置判断是否为离群点的阈值-->
    <param name="radius_radius" value="0.5" />              <!--以半径为0.5米的方式进行统计-->
    <param name="radius_min_neighbors" value="5" />           <!--统计的邻居的最小数目为5-->
  </node>

  <!-- scan_matching_odometry_nodelet -->
  <node pkg="nodelet" type="nodelet" name="scan_matching_odometry_nodelet" args="load hdl_graph_slam/ScanMatchingOdometryNodelet $(arg nodelet_manager)">
      <param name="keyframe_delta_trans" value="2.5" />       <!--两个关键帧进行匹配的最小距离间隔，如果设置的太小，就变成前后帧进行匹配-->
      <param name="keyframe_delta_angle" value="1.0" />         <!--两个关键帧进行匹配的最小角度间隔-->
      <param name="keyframe_delta_time" value="5.0" />            <!--两个关键帧进行匹配的时间间隔-->
      <param name="transform_thresholding" value="false" />
      <param name="max_acceptable_trans" value="2.0" />   <!--如果两个关键帧匹配后的相对位移太大，就丢弃当前帧，而采用上一帧的变换-->
      <param name="max_acceptable_angle" value="1.0" />    <!--如果两个关键帧匹配后的相对角度太大，就丢弃当前帧，而采用上一帧的变换-->
      <param name="downsample_method" value="NONE" />      <!--(VOXELGRID, APPROX_VOXELGRID, NONE)-->
      <param name="downsample_resolution" value="0.1" />
      <!-- ICP, GICP, NDT, GICP_OMP, or NDT_OMP(recommended) -->  <!--这里如果选择KDTREE，那么和传统的NDT算法一致，剩下的速度比较： DIRECT1》DIRECT7， 但是DIRECT7更稳定同时速度比KDTREE 快-->
      <param name="registration_method" value="NDT_OMP" />
      <param name="ndt_resolution" value="2.0" />   <!--比较重要的参数，设置ndt 分辨率-->
      <param name="ndt_num_threads" value="0" />     <!--比较重要的参数，设置ndt 线程数，可以多线程运行，提高速度-->
      <param name="ndt_nn_search_method" value="DIRECT7" />  <!--比kdtree 快-->
  </node>

  <!-- floor_detection_nodelet -->
  <node pkg="nodelet" type="nodelet" name="floor_detection_nodelet" args="load hdl_graph_slam/FloorDetectionNodelet $(arg nodelet_manager)"  if="$(arg enable_floor_detection)">
    <param name="sensor_height" value="2.0" />       <!--approximate sensor height [m]-->
    <param name="height_clip_range" value="0.5" />    <!--points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection-->
    <param name="floor_pts_thresh" value="1024" />    <!-- minimum number of support points of RANSAC to accept a detected floor plane-->
    <param name="floor_normal_thresh" value="5.0" />    <!--verticality check thresold for the detected floor plane [deg]-->
    <param name="use_normal_fitlering" value="true" />   <!--if true, points with "non-"vertical normals will be filtered before RANSAC-->
    <param name="normal_filter_thresh" value="10.0" />      <!--"non-"verticality check threshold [deg]-->
  </node>

  <!-- hdl_graph_slam_nodelet -->
  <node pkg="nodelet" type="nodelet" name="hdl_graph_slam_nodelet" args="load hdl_graph_slam/HdlGraphSlamNodelet $(arg nodelet_manager)">
    <!-- keyframe registration params -->
    <param name="enable_gps" value="$(arg enable_gps)" />
    <param name="max_keyframes_per_update" value="10" />   <!--设置每次循环从keyframe 加入到loop的数目-->
    <param name="keyframe_delta_trans" value="10.0" />     <!--设置前后两帧匹配的的距离，不能太近也不能太远，如果是0，就变成前后帧匹配-->
    <param name="keyframe_delta_angle" value="3.0" />      <!--设置前后两帧匹配的的角度，不能太近也不能太远，如果是0，就变成前后帧匹配-->
    <!-- loop closure params -->
    <param name="distance_thresh" value="30.0" />             <!--小于该阈值范围内两个关键帧为潜在闭环帧，将所有满足条件的都存起来作为candidate-->
    <param name="accum_distance_thresh" value="60.0" />      <!-- 这里指的是当前帧与临近帧的距离阈值，假设距离阈值为5m, 那么当前帧距离5米范围内关键帧不再考虑-->
    <param name="min_edge_interval" value="10.0" />          <!-- 这里指的是当前帧与临近帧的距离阈值，假设距离阈值为5m, 那么当前帧距离5米范围内关键帧不再考虑-->
    <param name="fitness_score_thresh" value="2.0" />         <!-- 闭环帧匹配得分-->
    <!-- scan matching params -->
    <param name="registration_method" value="NDT_OMP" />       <!-- 闭环帧匹配方法-->
    <param name="ndt_resolution" value="2.0" />                 <!-- 闭环帧匹配分辨率-->
    <param name="ndt_num_threads" value="0" />                   <!-- 闭环帧匹配线程数目-->
    <param name="ndt_nn_search_method" value="DIRECT7" />          <!-- 闭环帧寻找临近点搜索方法-->
    <!-- information matrix params -->
    <param name="floor_edge_stddev" value="100.0" />                <!-- 地面约束XYZ的信息矩阵 1/100-->
    <param name="gps_edge_stddev" value="1000.0" />                 <!-- GPS约束XY的信息矩阵 1/1000-->
    <param name="use_const_inf_matrix" value="false" />              <!-- 是否使用常值信息矩阵对于闭环帧匹配-->
    <param name="const_stddev_x" value="0.5" />                        <!--设置XYZ常值信息矩阵1/0.5-->
    <param name="const_stddev_q" value="0.1" />                        <!--设置角度常值信息矩阵1/0.5-->
    <param name="var_gain_a" value="20.0" />                           <!--设置动态信息矩阵增益-->
    <param name="min_stddev_x" value="0.1" />                           <!--设置动态信息矩阵xyz最小值-->
    <param name="max_stddev_x" value="5.0" />                           <!--设置动态信息矩阵xyz最大值-->
    <param name="min_stddev_q" value="0.05" />                          <!--设置动态信息矩阵角度最小值-->
    <param name="max_stddev_q" value="0.2" />                            <!--设置动态信息矩阵角度最大值-->
    <!-- update params -->
    <param name="graph_update_interval" value="3.0" />                 <!--graph_update_interval 的更新频率-->
    <param name="map_cloud_update_interval" value="10.0" />              <!--全局地图的更新频率-->
    <param name="map_cloud_resolution" value="0.05" />                    <!--全局地图的分辨率的更新频率-->
  </node>

  <node pkg="hdl_graph_slam" type="map2odom_publisher.py" name="map2odom_publisher" />
</launch>
```
    
    
 
   
    
   
          