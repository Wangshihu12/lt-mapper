#include "ltslam/LTslam.h"


gtsam::Pose3 LTslam::getPoseOfIsamUsingKey (const Key _key) {
    const Value& pose_value = isam->calculateEstimate(_key);
    auto p_pose_value = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&pose_value);
    gtsam::Pose3 pose = gtsam::Pose3{p_pose_value->value()};
    return pose;
}

void LTslam::writeAllSessionsTrajectories(std::string _postfix = "")
{
    // parse
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();
    for(const auto& key_value: isamCurrentEstimate) {

        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);
        int session_idx = parsed_digits.at(0);
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&key_value.value);
        if (!p) continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if( curr_node_idx == anchor_node_idx ) { 
            // anchor node 
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        } else { 
            // general nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }

    std::map<int, std::string> session_names;
    for(auto& _sess_pair: sessions_)
    {
        auto& _sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    // write
    for(auto& _session_info: parsed_poses) {
        int session_idx = _session_info.first;

    	std::string filename_local = save_directory_ + session_names[session_idx] + "_local_" + _postfix + ".txt";
    	std::string filename_central = save_directory_ + session_names[session_idx] + "_central_" + _postfix + ".txt";
        cout << filename_central << endl;

        std::fstream stream_local(filename_local.c_str(), std::fstream::out);
        std::fstream stream_central(filename_central.c_str(), std::fstream::out);

        gtsam::Pose3 anchor_transform = parsed_anchor_transforms[session_idx];
        for(auto& _pose: _session_info.second) {
            writePose3ToStream(stream_local, _pose);

            gtsam::Pose3 pose_central = anchor_transform * _pose; // se3 compose (oplus) 
            writePose3ToStream(stream_central, pose_central);
        }
    }

} // writeAllSessionsTrajectories


LTslam::LTslam()
: poseOrigin(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0))) 
{
} // ctor


LTslam::~LTslam() { }
// dtor

/**
 * [功能描述]：LT-SLAM的主要运行函数，执行多会话SLAM的完整流程
 * @return 无返回值
 */
void LTslam::run( void )
{
    // 初始化优化器和噪声常数
    initOptimizer();        // 初始化ISAM2优化器参数
    initNoiseConstants();   // 初始化各种噪声模型（先验、里程计、回环、大噪声等）

    // 加载所有会话数据并构建图结构
    loadAllSessions();      // 从指定目录加载所有会话的位姿数据
    addAllSessionsToGraph(); // 将所有会话的节点和边添加到因子图中

    // 第一次优化：仅使用现有边进行优化
    optimizeMultisesseionGraph(true); // 优化包含现有边的图结构
    writeAllSessionsTrajectories(std::string("bfr_intersession_loops")); // 保存优化前的轨迹

    // 检测并添加ScanContext回环
    detectInterSessionSCloops(); // 使用ScanContext检测会话间回环（内部同时检测RS回环）
    addSCloops();               // 将检测到的SC回环添加到因子图中
    optimizeMultisesseionGraph(true); // 优化包含现有边+SC回环边的图结构

    // 检测并添加RS回环，使用SC优化后的估计值进行粗略对齐
    bool toOpt = addRSloops(); // 使用优化后的估计值（通过SC粗略对齐）添加RS回环
    optimizeMultisesseionGraph(toOpt); // 优化包含现有边+SC回环边+RS回环边的完整图结构

    // 保存最终优化后的轨迹
    writeAllSessionsTrajectories(std::string("aft_intersession_loops")); // 保存优化后的轨迹
}

void LTslam::initNoiseConstants()
{
    // Variances Vector6 order means
    // : rad*rad, rad*rad, rad*rad, meter*meter, meter*meter, meter*meter
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
        odomNoise = noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        loopNoise = noiseModel::Diagonal::Variances(Vector6);
    }
    {
        gtsam::Vector Vector6(6);
        Vector6 << M_PI*M_PI, M_PI*M_PI, M_PI*M_PI, 1e8, 1e8, 1e8;
        // Vector6 << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
        largeNoise = noiseModel::Diagonal::Variances(Vector6);
    }

    float robustNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); 
    robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
    robustNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)
}


void LTslam::initOptimizer()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1; // TODO: study later
    isam = new ISAM2(parameters);
}


void LTslam::updateSessionsPoses()
{
    for(auto& _sess_pair: sessions_)
    {
        auto& _sess = _sess_pair.second;
        gtsam::Pose3 anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(_sess.index_));
        // cout << anchor_transform << endl;
        _sess.updateKeyPoses(isam, anchor_transform);
    }
} // updateSessionsPoses


void LTslam::optimizeMultisesseionGraph(bool _toOpt)
{
    if(!_toOpt)
        return;

    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isamCurrentEstimate = isam->calculateEstimate(); // must be followed by update 

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    updateSessionsPoses(); 

    if(is_display_debug_msgs_) {
        std::cout << "**********************************************" << std::endl;
        std::cout << "***** variable values after optimization *****" << std::endl;
        std::cout << std::endl;
        isamCurrentEstimate.print("Current estimate: ");
        std::cout << std::endl;
        // std::ofstream os("/home/user/Documents/catkin2021/catkin_ltmapper/catkin_ltmapper_dev/src/ltmapper/data/3d/kaist/PoseGraphExample.dot");
        // gtSAMgraph.saveGraph(os, isamCurrentEstimate);
    }
} // optimizeMultisesseionGraph


std::optional<gtsam::Pose3> LTslam::doICPVirtualRelative( // for SC loop
    Session& target_sess, Session& source_sess, 
    const int& loop_idx_target_session, const int& loop_idx_source_session)
{
    // 20201228: get relative virtual measurements using ICP (refer LIO-SAM's code)

    // parse pointclouds
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    int base_key = 0; // its okay. (using the origin for sc loops' co-base) 
    int historyKeyframeSearchNum = 25; // TODO move to yaml 

    source_sess.loopFindNearKeyframesLocalCoord(cureKeyframeCloud, loop_idx_source_session, 0);
    target_sess.loopFindNearKeyframesLocalCoord(targetKeyframeCloud, loop_idx_target_session, historyKeyframeSearchNum); 
    mtx.unlock(); // unlock after loopFindNearKeyframesWithRespectTo because many new in the loopFindNearKeyframesWithRespectTo

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    // giseop 
    // TODO icp align with initial 

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        mtx.lock();
        std::cout << "  [SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        mtx.unlock();
        return std::nullopt;
    } else {
        mtx.lock();
        std::cout << "  [SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
        mtx.unlock();
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative


std::optional<gtsam::Pose3> LTslam::doICPGlobalRelative( // For RS loop
    Session& target_sess, Session& source_sess, 
    const int& loop_idx_target_session, const int& loop_idx_source_session)
{
    // parse pointclouds
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());

    int base_key = 0; // its okay. (using the origin for sc loops' co-base) 
    int historyKeyframeSearchNum = 25; // TODO move to yaml 

    source_sess.loopFindNearKeyframesCentralCoord(cureKeyframeCloud, loop_idx_source_session, 0);
    target_sess.loopFindNearKeyframesCentralCoord(targetKeyframeCloud, loop_idx_target_session, historyKeyframeSearchNum); 
    mtx.unlock(); // unlock after loopFindNearKeyframesWithRespectTo because many new in the loopFindNearKeyframesWithRespectTo

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    // giseop 
    // TODO icp align with initial 

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        mtx.lock();
        std::cout << "  [RS loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this RS loop." << std::endl;
        mtx.unlock();
        return std::nullopt;
    } else {
        mtx.lock();
        std::cout << "  [RS loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this RS loop." << std::endl;
        mtx.unlock();
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    Eigen::Affine3f tWrong = pclPointToAffine3f(source_sess.cloudKeyPoses6D->points[loop_idx_source_session]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(target_sess.cloudKeyPoses6D->points[loop_idx_target_session]);

    return poseFrom.between(poseTo);
} // doICPGlobalRelative


/**
 * [功能描述]：使用 ScanContext 方法检测会话间的闭环（Inter-Session Loop Closure Detection）
 * 
 * 该函数通过比较源会话（source session）和目标会话（target session）中的 ScanContext 描述子，
 * 检测两个会话之间的相似位置，从而识别闭环候选对。检测结果分为两类：
 * - SCLoopIdxPairs_: 成功匹配的闭环索引对
 * - RSLoopIdxPairs_: 未找到匹配但需要后续通过最近邻姿态查找的索引对
 * 
 * @note 使用类成员变量：
 *       - target_sess_idx: 目标会话的索引
 *       - source_sess_idx: 源会话的索引
 *       - SCLoopIdxPairs_: 存储检测到的闭环索引对（目标会话索引，源会话索引）
 *       - RSLoopIdxPairs_: 存储需要后续处理的索引对（标记为 -1 的目标索引，源会话索引）
 * @return 无返回值（void），结果存储在成员变量 SCLoopIdxPairs_ 和 RSLoopIdxPairs_ 中
 */
void LTslam::detectInterSessionSCloops() // using ScanContext
{
    // 获取目标会话和源会话的引用
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    // 清空之前的闭环检测结果，准备存储新的闭环索引对
    SCLoopIdxPairs_.clear(); // 存储成功匹配的 ScanContext 闭环索引对
    RSLoopIdxPairs_.clear(); // 存储需要后续通过最近邻姿态查找的索引对
    
    // 获取目标会话和源会话的 ScanContext 管理器引用
    auto& target_scManager = target_sess.scManager;
    auto& source_scManager = source_sess.scManager;
    
    // 遍历源会话中的所有节点（关键帧），逐个与目标会话进行闭环检测
    for (int source_node_idx=0; source_node_idx < int(source_scManager.polarcontexts_.size()); source_node_idx++)
    {
        // 获取源会话当前节点的 ScanContext 描述符
        std::vector<float> source_node_key = source_scManager.polarcontext_invkeys_mat_.at(source_node_idx); // 极坐标上下文的紧凑特征向量（键值）
        Eigen::MatrixXd source_node_scd = source_scManager.polarcontexts_.at(source_node_idx); // 完整的极坐标上下文矩阵描述子

        // 在目标会话中检测与当前源节点最相似的节点
        // detectResult.first: 最近邻节点索引（-1 表示未找到匹配）
        // detectResult.second: 偏航角差异（yaw diff）
        auto detectResult = target_scManager.detectLoopClosureIDBetweenSession(source_node_key, source_node_scd);

        // 记录源会话和目标会话中的闭环节点索引
        int loop_idx_source_session = source_node_idx;
        int loop_idx_target_session = detectResult.first;

        // 如果在目标会话中未找到匹配的闭环节点
        if(loop_idx_target_session == -1) { // TODO: 建议使用 NO_LOOP_FOUND 常量替代 -1 
            // 将该索引对标记为需要后续处理（通过最近邻姿态查找）
            RSLoopIdxPairs_.emplace_back(std::pair(-1, loop_idx_source_session)); // -1 表示目标索引待定，将在后续通过最近邻姿态查找确定
            continue; // 跳过当前节点，继续处理下一个源节点
        }

        // 成功找到闭环匹配，将索引对添加到 ScanContext 闭环列表中
        // pair 的 first 是目标会话索引，second 是源会话索引
        SCLoopIdxPairs_.emplace_back(std::pair(loop_idx_target_session, loop_idx_source_session));
    }

    // 输出检测到的会话间闭环总数（绿色高亮显示）
    ROS_INFO_STREAM("\033[1;32m Total " << SCLoopIdxPairs_.size() << " inter-session loops are found. \033[0m");
} // detectInterSessionSCloops


void LTslam::detectInterSessionRSloops() // using ScanContext
{
    
} // detectInterSessionRSloops


void LTslam::addAllSessionsToGraph()
{
    for(auto& _sess_pair: sessions_)    
    {
        auto& _sess = _sess_pair.second;
        initTrajectoryByAnchoring(_sess);
        addSessionToCentralGraph(_sess);   
    }
} // addAllSessionsToGraph


std::vector<std::pair<int, int>> LTslam::equisampleElements(
    const std::vector<std::pair<int, int>>& _input_pair, float _gap, int _num_sampled)
{
    std::vector<std::pair<int, int>> sc_loop_idx_pairs_sampled;

    int equisampling_counter { 0 }; 

    std::vector<int> equisampled_idx;
    for (int i=0; i<_num_sampled; i++)
        equisampled_idx.emplace_back(std::round(float(i) * _gap));

    for (auto& _idx: equisampled_idx)
        sc_loop_idx_pairs_sampled.emplace_back(_input_pair.at(_idx));

    return sc_loop_idx_pairs_sampled;
}

/**
 * [功能描述]：将 ScanContext 检测到的闭环约束添加到 GTSAM 优化图中
 * 
 * 该函数处理之前通过 detectInterSessionSCloops() 检测到的会话间闭环。主要流程包括：
 * 1. 对检测到的闭环进行等间隔采样，避免闭环数量过多影响性能
 * 2. 对采样后的闭环对进行 ICP 配准，验证并计算精确的相对位姿
 * 3. 将配准成功的闭环约束添加到 GTSAM 因子图中进行全局优化
 * 
 * @note 使用类成员变量：
 *       - SCLoopIdxPairs_: 待处理的 ScanContext 闭环索引对
 *       - target_sess_idx: 目标会话索引
 *       - source_sess_idx: 源会话索引
 *       - gtSAMgraph: GTSAM 因子图
 *       - kNumSCLoopsUpperBound: 闭环数量上限
 *       - numberOfCores: 用于并行计算的 CPU 核心数
 *       - mtx: 互斥锁，用于保护共享资源
 * @return 无返回值（void），闭环约束直接添加到 gtSAMgraph 中
 */
void LTslam::addSCloops()
{
    // 如果没有检测到任何 ScanContext 闭环，直接返回
    if(SCLoopIdxPairs_.empty()) 
        return;

    // ========== 步骤 1: 对闭环进行等间隔采样 ==========
    // 获取检测到的所有闭环数量
    int num_scloops_all_found = int(SCLoopIdxPairs_.size());
    // 计算实际要添加的闭环数量（不超过设定的上限）
    int num_scloops_to_be_added = std::min( num_scloops_all_found, kNumSCLoopsUpperBound );
    // 计算等间隔采样的间隔大小
    int equisampling_gap = num_scloops_all_found / num_scloops_to_be_added;

    // 执行等间隔采样，从所有闭环中均匀选取指定数量的闭环
    auto sc_loop_idx_pairs_sampled = equisampleElements(SCLoopIdxPairs_, equisampling_gap, num_scloops_to_be_added);
    auto num_scloops_sampled = sc_loop_idx_pairs_sampled.size();

    // ========== 步骤 2: 准备添加选中的闭环 ==========
    // 获取目标会话和源会话的引用
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    // 用于记录已成功添加的闭环索引（调试用）
    std::vector<int> idx_added_loops; 
    idx_added_loops.reserve(num_scloops_sampled);
    
    // ========== 步骤 3: 并行处理每个采样的闭环对 ==========
    #pragma omp parallel for num_threads(numberOfCores) // 使用 OpenMP 并行加速处理
    for (int ith = 0; ith < num_scloops_sampled; ith++) 
    {
        // 获取当前闭环索引对
        auto& _loop_idx_pair = sc_loop_idx_pairs_sampled.at(ith);
        int loop_idx_target_session = _loop_idx_pair.first;  // 目标会话中的节点索引
        int loop_idx_source_session = _loop_idx_pair.second; // 源会话中的节点索引

        // 执行 ICP（Iterative Closest Point）配准，计算两个节点之间的精确相对位姿
        // 返回值是 optional 类型，配准失败时为空
        auto relative_pose_optional = doICPVirtualRelative(target_sess, source_sess, loop_idx_target_session, loop_idx_source_session); 

        // 如果 ICP 配准成功，将闭环约束添加到 GTSAM 图中
        if(relative_pose_optional) {
            mtx.lock(); // 加锁保护共享资源 gtSAMgraph
            gtsam::Pose3 relative_pose = relative_pose_optional.value(); // 提取相对位姿
            
            // 向 GTSAM 因子图添加带锚点的 Between 因子（闭环约束）
            // 该因子连接目标会话和源会话中的对应节点，并考虑各自的锚点节点
            gtSAMgraph.add( BetweenFactorWithAnchoring<gtsam::Pose3>(
                genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), // 目标节点的全局索引
                genGlobalNodeIdx(source_sess_idx, loop_idx_source_session), // 源节点的全局索引
                genAnchorNodeIdx(target_sess_idx), // 目标会话的锚点节点索引
                genAnchorNodeIdx(source_sess_idx), // 源会话的锚点节点索引
                relative_pose,  // 相对位姿约束
                robustNoise) ); // 鲁棒噪声模型，降低外点影响
            mtx.unlock(); // 解锁

            // ========== 调试信息输出（后续可能移除） ==========
            mtx.lock(); // 加锁保护共享资源 idx_added_loops 和 cout
            idx_added_loops.emplace_back(loop_idx_target_session); // 记录已添加的闭环
            // 输出闭环边的详细信息：连接的两个节点和对应的锚点节点
            cout << "SCdetector found an inter-session edge between " 
                << genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) << " and " << genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) 
                << " (anchor nodes are " << genAnchorNodeIdx(target_sess_idx) << " and " << genAnchorNodeIdx(source_sess_idx) << ")" << endl;
            mtx.unlock(); // 解锁
        }
    }
} // addSCloops


double LTslam::calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session)
{
    auto pose_s1 = isamCurrentEstimate.at<gtsam::Pose3>( genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) ); // node: s1 is the central 
    auto pose_s2 = isamCurrentEstimate.at<gtsam::Pose3>( genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) );
    auto pose_s1_anchor = isamCurrentEstimate.at<gtsam::Pose3>( genAnchorNodeIdx(target_sess_idx) );
    auto pose_s2_anchor = isamCurrentEstimate.at<gtsam::Pose3>( genAnchorNodeIdx(source_sess_idx) );

    gtsam::Pose3 hx1 = traits<gtsam::Pose3>::Compose(pose_s1_anchor, pose_s1); // for the updated jacobian, see line 60, 219, https://gtsam.org/doxygen/a00053_source.html
    gtsam::Pose3 hx2 = traits<gtsam::Pose3>::Compose(pose_s2_anchor, pose_s2); 
    gtsam::Pose3 estimated_relative_pose = traits<gtsam::Pose3>::Between(hx1, hx2); 

    gtsam::Matrix H_s1, H_s2, H_s1_anchor, H_s2_anchor;
    auto loop_factor = BetweenFactorWithAnchoring<gtsam::Pose3>(
        genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), genGlobalNodeIdx(source_sess_idx, loop_idx_source_session),
        genAnchorNodeIdx(target_sess_idx), genAnchorNodeIdx(source_sess_idx), 
        estimated_relative_pose, robustNoise);
    loop_factor.evaluateError(pose_s1, pose_s2, pose_s1_anchor, pose_s2_anchor, 
                                 H_s1,    H_s2,    H_s1_anchor,    H_s2_anchor);

    gtsam::Matrix pose_s1_cov = isam->marginalCovariance(genGlobalNodeIdx(target_sess_idx, loop_idx_target_session)); // note: typedef Eigen::MatrixXd  gtsam::Matrix
    gtsam::Matrix pose_s2_cov = isam->marginalCovariance(genGlobalNodeIdx(source_sess_idx, loop_idx_source_session));

    // calc S and information gain 
    gtsam::Matrix Sy = Eigen::MatrixXd::Identity(6, 6); // measurement noise, assume fixed
    gtsam::Matrix S = Sy + (H_s1*pose_s1_cov*H_s1.transpose() + H_s2*pose_s2_cov*H_s2.transpose());
    double Sdet = S.determinant(); 
    double information_gain = 0.5 * log( Sdet / Sy.determinant());

    return information_gain;
}

void LTslam::findNearestRSLoopsTargetNodeIdx() // based-on information gain 
{
    std::vector<std::pair<int, int>> validRSLoopIdxPairs;

    for(std::size_t i=0; i<RSLoopIdxPairs_.size(); i++)
    {
        // curr query pose 
        auto rsloop_idx_pair = RSLoopIdxPairs_.at(i);
        auto rsloop_idx_source_session = rsloop_idx_pair.second;
        auto rsloop_global_idx_source_session = genGlobalNodeIdx(source_sess_idx, rsloop_idx_source_session);

        auto source_node_idx = rsloop_idx_source_session;
        auto query_pose = isamCurrentEstimate.at<gtsam::Pose3>(rsloop_global_idx_source_session);
        gtsam::Pose3 query_sess_anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(source_sess_idx));
        auto query_pose_central_coord = query_sess_anchor_transform * query_pose;

        // find nn pose idx in the target sess 
        auto& target_sess = sessions_.at(target_sess_idx); 
        std::vector<int> target_node_idxes_within_ball;
        for (int target_node_idx=0; target_node_idx < int(target_sess.nodes_.size()); target_node_idx++) {
            auto target_pose = isamCurrentEstimate.at<gtsam::Pose3>(genGlobalNodeIdx(target_sess_idx, target_node_idx));
            if( poseDistance(query_pose_central_coord, target_pose) < 10.0 ) // 10 is a hard-coding for fast test
            {
                target_node_idxes_within_ball.push_back(target_node_idx);
                // cout << "(all) RS pair detected: " << target_node_idx << " <-> " << source_node_idx << endl;    
            }
        }

        // if no nearest one, skip 
        if(target_node_idxes_within_ball.empty())
            continue;

        // selected a single one having maximum information gain  
        int selected_near_target_node_idx; 
        double max_information_gain {0.0};     
        for (int i=0; i<target_node_idxes_within_ball.size(); i++) 
        {
            auto nn_target_node_idx = target_node_idxes_within_ball.at(i);
            double this_information_gain = calcInformationGainBtnTwoNodes(nn_target_node_idx, source_node_idx);
            if(this_information_gain > max_information_gain) {
                selected_near_target_node_idx = nn_target_node_idx;
                max_information_gain = this_information_gain;
            }
        }

        // cout << "RS pair detected: " << selected_near_target_node_idx << " <-> " << source_node_idx << endl;    
        // cout << "info gain: " << max_information_gain << endl;    
             
        validRSLoopIdxPairs.emplace_back(std::pair<int, int>{selected_near_target_node_idx, source_node_idx});
    }

    // update 
    RSLoopIdxPairs_.clear();
    RSLoopIdxPairs_.resize((int)(validRSLoopIdxPairs.size()));
    std::copy( validRSLoopIdxPairs.begin(), validRSLoopIdxPairs.end(), RSLoopIdxPairs_.begin() );
}


/**
 * [功能描述]：添加 Revisit/Random Sampling (RS) 闭环约束到 GTSAM 优化图中
 * 
 * RS 闭环是对 ScanContext 闭环的补充机制，用于处理那些未被 ScanContext 检测到的潜在闭环。
 * 该函数针对之前 ScanContext 检测失败（标记为 -1）的节点，通过最近邻搜索找到目标会话中
 * 距离最近的节点，然后进行 ICP 全局配准验证。主要流程包括：
 * 1. 检查是否启用 RS 闭环功能
 * 2. 为每个未匹配的源节点查找目标会话中最近的节点
 * 3. 对闭环候选进行等间隔采样
 * 4. 执行 ICP 全局配准并添加验证通过的闭环约束
 * 
 * @note 使用类成员变量：
 *       - RSLoopIdxPairs_: RS 闭环索引对（由 detectInterSessionSCloops 生成）
 *       - kNumRSLoopsUpperBound: RS 闭环数量上限，为 0 时禁用 RS 闭环
 *       - target_sess_idx: 目标会话索引
 *       - source_sess_idx: 源会话索引
 *       - gtSAMgraph: GTSAM 因子图
 *       - numberOfCores: 并行计算的 CPU 核心数
 *       - mtx: 互斥锁
 * @return 是否成功添加了 RS 闭环（true: 已添加，false: 未添加或未启用）
 */
bool LTslam::addRSloops()
{
    // ========== 步骤 1: 检查 RS 闭环功能是否启用 ==========
    // 如果 RS 闭环数量上限为 0，说明禁用了该功能
    if( kNumRSLoopsUpperBound == 0 )
        return false;

    // ========== 步骤 2: 查找最近的目标节点索引 ==========
    // 对于 RSLoopIdxPairs_ 中目标索引为 -1 的节点对，通过最近邻搜索
    // 在目标会话中找到距离最近的节点，更新目标索引
    findNearestRSLoopsTargetNodeIdx();

    // ========== 步骤 3: 解析和验证 RS 闭环候选 ==========
    // 获取所有找到的 RS 闭环候选数量
    int num_rsloops_all_found = int(RSLoopIdxPairs_.size());
    // 如果没有找到任何 RS 闭环候选，返回 false
    if( num_rsloops_all_found == 0 )
        return false;

    // ========== 步骤 4: 对闭环候选进行等间隔采样 ==========
    // 计算实际要添加的 RS 闭环数量（不超过设定的上限）
    int num_rsloops_to_be_added = std::min( num_rsloops_all_found, kNumRSLoopsUpperBound );
    // 计算等间隔采样的间隔大小
    int equisampling_gap = num_rsloops_all_found / num_rsloops_to_be_added;

    // 执行等间隔采样，从所有候选中均匀选取指定数量的闭环
    auto rs_loop_idx_pairs_sampled = equisampleElements(RSLoopIdxPairs_, equisampling_gap, num_rsloops_to_be_added);
    auto num_rsloops_sampled = rs_loop_idx_pairs_sampled.size();

    // 输出 RS 闭环候选的统计信息
    cout << "num of RS pair: " << num_rsloops_all_found << endl;         // 总共找到的 RS 闭环对数量
    cout << "num of sampled RS pair: " << num_rsloops_sampled << endl;   // 采样后要处理的 RS 闭环对数量

    // ========== 步骤 5: 准备添加选中的 RS 闭环 ==========
    // 获取目标会话和源会话的引用
    auto& target_sess = sessions_.at(target_sess_idx); 
    auto& source_sess = sessions_.at(source_sess_idx);

    // ========== 步骤 6: 并行处理每个采样的 RS 闭环对 ==========
    #pragma omp parallel for num_threads(numberOfCores) // 使用 OpenMP 并行加速处理
    for (int ith = 0; ith < num_rsloops_sampled; ith++) 
    {
        // 获取当前闭环索引对
        auto& _loop_idx_pair = rs_loop_idx_pairs_sampled.at(ith);
        int loop_idx_target_session = _loop_idx_pair.first;  // 目标会话中的节点索引（已通过最近邻更新）
        int loop_idx_source_session = _loop_idx_pair.second; // 源会话中的节点索引

        // 执行 ICP 全局配准，计算两个节点之间的精确相对位姿
        // 注意：这里使用 doICPGlobalRelative（全局配准），而非 doICPVirtualRelative（虚拟配准）
        // 返回值是 optional 类型，配准失败时为空
        auto relative_pose_optional = doICPGlobalRelative(target_sess, source_sess, loop_idx_target_session, loop_idx_source_session); 

        // 如果 ICP 配准成功，将 RS 闭环约束添加到 GTSAM 图中
        if(relative_pose_optional) {
            mtx.lock(); // 加锁保护共享资源 gtSAMgraph
            gtsam::Pose3 relative_pose = relative_pose_optional.value(); // 提取相对位姿
            
            // 向 GTSAM 因子图添加带锚点的 Between 因子（RS 闭环约束）
            // 该因子连接目标会话和源会话中的对应节点，并考虑各自的锚点节点
            gtSAMgraph.add( BetweenFactorWithAnchoring<gtsam::Pose3>(
                genGlobalNodeIdx(target_sess_idx, loop_idx_target_session), // 目标节点的全局索引
                genGlobalNodeIdx(source_sess_idx, loop_idx_source_session), // 源节点的全局索引
                genAnchorNodeIdx(target_sess_idx), // 目标会话的锚点节点索引
                genAnchorNodeIdx(source_sess_idx), // 源会话的锚点节点索引
                relative_pose,  // 相对位姿约束
                robustNoise) ); // 鲁棒噪声模型，降低外点影响
            mtx.unlock(); // 解锁

            // ========== 调试信息输出（后续可能移除） ==========
            mtx.lock(); // 加锁保护共享资源 cout
            // 输出 RS 闭环边的详细信息：连接的两个节点和对应的锚点节点
            cout << "RS loop detector found an inter-session edge between " 
                << genGlobalNodeIdx(target_sess_idx, loop_idx_target_session) << " and " << genGlobalNodeIdx(source_sess_idx, loop_idx_source_session) 
                << " (anchor nodes are " << genAnchorNodeIdx(target_sess_idx) << " and " << genAnchorNodeIdx(source_sess_idx) << ")" << endl;
            mtx.unlock(); // 解锁
        }
    }

    return true; // 成功处理 RS 闭环
} // addRSloops


void LTslam::initTrajectoryByAnchoring(const Session& _sess)
{
    int this_session_anchor_node_idx = genAnchorNodeIdx(_sess.index_);

    if(_sess.is_base_session_) {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, priorNoise));
    } else {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, largeNoise));
    }

    initialEstimate.insert(this_session_anchor_node_idx, poseOrigin);
} // initTrajectoryByAnchoring


void LTslam::addSessionToCentralGraph(const Session& _sess)
{
    // add nodes 
    for( auto& _node: _sess.nodes_)
    {        
        int node_idx = _node.second.idx;
        auto& curr_pose = _node.second.initial;

        int prev_node_global_idx = genGlobalNodeIdx(_sess.index_, node_idx - 1);
        int curr_node_global_idx = genGlobalNodeIdx(_sess.index_, node_idx);

        gtsam::Vector Vector6(6);
        if(node_idx == 0) { // TODO consider later if the initial node idx is not zero (but if using SC-LIO-SAM, don't care)
            // prior node 
            gtSAMgraph.add(PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise));
            initialEstimate.insert(curr_node_global_idx, curr_pose);
        } else { 
            // odom nodes 
            initialEstimate.insert(curr_node_global_idx, curr_pose);
        }
    }
    
    // add edges 
    for( auto& _edge: _sess.edges_)
    {
        int from_node_idx = _edge.second.from_idx;
        int to_node_idx = _edge.second.to_idx;
        
        int from_node_global_idx = genGlobalNodeIdx(_sess.index_, from_node_idx);
        int to_node_global_idx = genGlobalNodeIdx(_sess.index_, to_node_idx);

        gtsam::Pose3 relative_pose = _edge.second.relative;
        if( std::abs(to_node_idx - from_node_idx) == 1) {
            // odom edge (temporally consecutive)
            gtSAMgraph.add(BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, odomNoise));
            if(is_display_debug_msgs_) cout << "add an odom edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        } else {
            // loop edge
            gtSAMgraph.add(BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, robustNoise));
            if(is_display_debug_msgs_) cout << "add a loop edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        }
    }

}


/**
 * [功能描述]：加载所有会话数据，从指定目录读取会话位姿信息并构建会话对象
 * @return 无返回值
 */
void LTslam::loadAllSessions() 
{
    // 输出加载会话数据的提示信息
    ROS_INFO_STREAM("\033[1;32m Load sessions' pose dasa from: " << sessions_dir_ << "\033[0m");
    
    // 遍历会话目录中的所有子目录
    for(auto& _session_entry : fs::directory_iterator(sessions_dir_)) 
    {
        // 获取当前会话目录的名称
        std::string session_name = _session_entry.path().filename();        
        
        // 检查是否为指定的中心会话或查询会话（目前设计为双会话版本）
        if( !isTwoStringSame(session_name, central_sess_name_) & !isTwoStringSame(session_name, query_sess_name_) ) {
            continue; // 跳过非目标会话，目前设计为双会话版本（TODO: 扩展为N会话协同优化）
        }

        // 确定会话索引：中心会话或源会话
        int session_idx;
        if(isTwoStringSame(session_name, central_sess_name_))
            session_idx = target_sess_idx;  // 中心会话索引
        else
            session_idx = source_sess_idx;  // 源会话索引

        // 获取会话目录的完整路径
        std::string session_dir_path = _session_entry.path();

        // 创建会话对象并插入到会话映射中
        // 会话对象内部会读取图文件并加载节点和边信息
        sessions_.insert( std::make_pair(session_idx, 
                                         Session(session_idx, session_name, session_dir_path, isTwoStringSame(session_name, central_sess_name_))) );

        // 注释掉的代码：增加全局会话计数（TODO: 改为私有并提供incrSessionIdx方法）
        // LTslam::num_sessions++; 
    }

    // 设置布尔值输出格式
    std::cout << std::boolalpha;   
    
    // 输出加载的会话总数
    ROS_INFO_STREAM("\033[1;32m Total : " << sessions_.size() << " sessions are loaded.\033[0m");
    
    // 遍历并输出每个会话的详细信息
    std::for_each( sessions_.begin(), sessions_.end(), [](auto& _sess_pair) { 
                cout << " — " << _sess_pair.second.name_ << " (is central: " << _sess_pair.second.is_base_session_ << ")" << endl; 
                } );

} // loadSession

