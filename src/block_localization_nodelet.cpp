#include "utility.h"
#include "block_localization/pose_estimator.hpp"
#include "block_localization/queryMap.h"
#include "block_localization/cloud_info.h"
#include <hdl_global_localization/QueryGlobalLocalization.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)


namespace block_localization {

class BlockLocalizationNodelet : public ParamServer, public nodelet::Nodelet
{
public:
    BlockLocalizationNodelet() {
    }
    virtual ~BlockLocalizationNodelet() {
        if (trajectory_stream_.is_open()) trajectory_stream_.close();
        if (timing_log_stream_.is_open()) timing_log_stream_.close();
    }


    void onInit() override {
        nh = getNodeHandle();

        // imuConverter() has already rotated acceleration and angular velocity
        // into the LiDAR axes.  The GTSAM state therefore also uses LiDAR axes;
        // applying extRot again here would introduce an approximately 180 deg
        // flip on every optimization cycle.  Only the sensor-origin lever arm
        // remains between the two poses in this common-axis representation.
        imu2Lidar = gtsam::Pose3(gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
                                 gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
        lidar2Imu = gtsam::Pose3(gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
                                 gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
        // lidar2gt = gtsam::Pose3(gtsam::Rot3(L2GtRot), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z())); // T_lidar_gt
        lidar2gt = gtsam::Pose3(gtsam::Rot3(L2GtRot), gtsam::Point3(L2GtTrans.x(), L2GtTrans.y(), L2GtTrans.z()));

        imu_sub = nh.subscribe(imuTopic, 2000, &BlockLocalizationNodelet::imuHandler, this, ros::TransportHints().tcpNoDelay());
        points_sub = nh.subscribe("/cloud_info", 5, &BlockLocalizationNodelet::blocMainProcess, this, ros::TransportHints().tcpNoDelay());
        globalmap_sub = nh.subscribe("/globalmap", 1, &BlockLocalizationNodelet::globalmapCB, this, ros::TransportHints().tcpNoDelay());
        initialpose_sub = nh.subscribe("/initialpose", 8, &BlockLocalizationNodelet::initialposeCB, this, ros::TransportHints().tcpNoDelay());
        
        pose_pub = nh.advertise<nav_msgs::Odometry>("/global_odom", 2000);
        path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
        imuOdom_pub = nh.advertise<nav_msgs::Odometry>("/imu_incremental", 2000);

        // Use a normal (non-persistent) client.  Passing `this` here used to
        // convert the pointer to `true`, unintentionally creating a persistent
        // connection before the separately hosted map server was ready.
        mapQuery_client = nh.serviceClient<block_localization::queryMap>("/mapQuery");

        initializeParams();
    }

private:
    void initializeParams() {
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter = voxelgrid;

        pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
        pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());

        ndt->setTransformationEpsilon(ndt_epsilon);
        ndt->setResolution(ndt_resolution);
        if(ndt_neighbor_search_method == "DIRECT1") {
            NODELET_INFO("search_method DIRECT1 is selected");
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
            registration = ndt;
        } else if(ndt_neighbor_search_method == "DIRECT7") {
            NODELET_INFO("search_method DIRECT7 is selected");
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
            registration = ndt;
            
        } else if(ndt_neighbor_search_method == "GICP_OMP"){
            NODELET_INFO("search_method GICP_OMP is selected");
            registration = gicp;
        }
        else {
            if(ndt_neighbor_search_method == "KDTREE") {
                NODELET_INFO("search_method KDTREE is selected");
            } else {
                NODELET_WARN("invalid search method was given");
                NODELET_WARN("default method is selected (KDTREE)");
            }
            ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            registration = ndt;
        }


        // initialize pose estimator
        if(private_nh.param<bool>("/block_localization/specify_init_pose", true))
        {
            NODELET_INFO("Initialize pose estimator with specified parameters.");
            pose_estimator.reset(new block_localization::PoseEstimator(registration,
                ros::Time::now(),
                Eigen::Vector3d(private_nh.param<double>("/block_localization/init_pos_x", 0.0), private_nh.param<double>("/block_localization/init_pos_y", 0.0), private_nh.param<double>("/block_localization/init_pos_z", 0.0)),
                Eigen::Quaterniond(private_nh.param<double>("/block_localization/init_ori_w", 1.0), private_nh.param<double>("/block_localization/init_ori_x", 0.0),
                        private_nh.param<double>("/block_localization/init_ori_y", 0.0), private_nh.param<double>("/block_localization/init_ori_z", 0.0)),
                private_nh.param<double>("/block_localization/cool_time_duration", 0.5)
            ));
        }

        // GTSAM settings
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(gravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // assume zero initial bias
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e2); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1); // meter
        noiseModelBetweenBias = (gtsam::Vector(6) << 3.99395e-03, 3.99395e-03, 3.99395e-03, 1.56363e-03, 1.56363e-03, 1.56363e-03).finished();
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

        initializeOutputFiles();

        key_count = 0;
        time_count = 0;
        systemInitialized = false;
        doneFirstOpt = false;
  }

private:
    void imuHandler(const sensor_msgs::ImuConstPtr& imu_msg) {
        std::lock_guard<std::mutex> lock(imu_data_mutex);

        sensor_msgs::Imu thisImu = imuConverter(*imu_msg);
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (!doneFirstOpt)
            return;

        double imuTime = getROSTime(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / imuFrequency) : (imuTime - lastImuT_imu);
        if (dt < 1e-16) dt = 1e-16;

        lastImuT_imu = imuTime;

        // integrate this single imu message
        const auto& acc = thisImu.linear_acceleration;
        const auto& gyro = thisImu.angular_velocity;
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);
        
        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = thisImu.header.stamp;
        odom.header.frame_id = "globalmap_link";
        odom.child_frame_id = "odom_imu";

        // transform imu pose to lidar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 odomPose = imuPose.compose(imu2Lidar);

        odom.pose.pose.position.x = odomPose.translation().x();
        odom.pose.pose.position.y = odomPose.translation().y();
        odom.pose.pose.position.z = odomPose.translation().z();
        odom.pose.pose.orientation.x = odomPose.rotation().toQuaternion().x();
        odom.pose.pose.orientation.y = odomPose.rotation().toQuaternion().y();
        odom.pose.pose.orientation.z = odomPose.rotation().toQuaternion().z();
        odom.pose.pose.orientation.w = odomPose.rotation().toQuaternion().w();
        
        odom.twist.twist.linear.x = currentState.velocity().x();
        odom.twist.twist.linear.y = currentState.velocity().y();
        odom.twist.twist.linear.z = currentState.velocity().z();
        odom.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odom.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odom.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        imuOdom_pub.publish(odom);
    }


    void blocMainProcess(const block_localization::cloud_infoConstPtr& cloudinfo_msg) {
        sensor_msgs::PointCloud2 points_msg = cloudinfo_msg->cloud_deskewed;
        points_curr_time = getROSTime(&points_msg);

        if(firstkey) {
            // if (!first_query)
            // {
            //     return;
            // }
            // first_query = false;
            points_pre_time = getROSTime(&points_msg);
            mapqry_pre_time = getROSTime(&points_msg);

            // NODELET_INFO("Global localization query...");

            // if(queryGlobalLocalization(&points_msg)) {
            //     NODELET_INFO("Global localization succeed!");
            // }
            firstkey = false;
        }

        double time_interval = points_curr_time - points_pre_time;

        if(time_interval < key_interval) return; // select key frames according to time interval

        std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);

        if(!pose_estimator) {
            NODELET_ERROR("Waiting for initial pose input!");
            return;
        }

        if(!globalmap) {
            NODELET_ERROR("Globalmap has not been received!");
            return;
        }

        pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(points_msg, *pcl_cloud);

        if(pcl_cloud->empty()) {
            NODELET_ERROR("Cloud is empty!");
            return;
        }

        // transform pointcloud into odom_child_frame_id
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        if(!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, this->tf_listener)) {
            NODELET_ERROR("Point cloud cannot be transformed into target frame!");
            return;
        }
        auto filtered = downsample(cloud);

        // 0. initialize system
        if(!systemInitialized) {

            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty()) {
                if (getROSTime(&imuQueOpt.front()) < points_curr_time) {
                    lastImuT_opt = getROSTime(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            // Initialize the graph from the configured/interactive LiDAR pose.
            // prevPose_ is default-constructed as identity, so composing it here
            // discarded init_pos/init_ori and made the first optimization start
            // in the wrong place.
            const gtsam::Pose3 initialLidarPose(
                gtsam::Rot3::Quaternion(pose_estimator->quat().w(),
                                        pose_estimator->quat().x(),
                                        pose_estimator->quat().y(),
                                        pose_estimator->quat().z()),
                gtsam::Point3(pose_estimator->pos().x(),
                              pose_estimator->pos().y(),
                              pose_estimator->pos().z()));
            prevPose_ = initialLidarPose.compose(lidar2Imu);
            newgraph.add(PriorFactor<Pose3>(X(0), prevPose_, priorPoseNoise));
            // query initial BM
            queryMapSwitch(initialLidarPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            newgraph.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            newgraph.add(priorBias);
            // add values
            initialEstimate.insert(X(0), prevPose_);
            initialEstimate.insert(V(0), prevVel_);
            initialEstimate.insert(B(0), prevBias_);

            optimizer->update(newgraph, initialEstimate);

            newgraph.resize(0);
            initialEstimate.clear();
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            systemInitialized = true;
            key_count = 1;
            time_count = 1;
            return;
        }

        if (!prev_centroid.isApprox(curr_centroid, 1e-5)) {
            NODELET_INFO("Reset optimization.");
            // update registration target
            registration->setInputTarget(globalmap);
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer->marginalCovariance(X(key_count-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer->marginalCovariance(V(key_count-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer->marginalCovariance(B(key_count-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            newgraph.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            newgraph.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            newgraph.add(priorBias);
            // add values
            initialEstimate.insert(X(0), prevPose_);
            initialEstimate.insert(V(0), prevVel_);
            initialEstimate.insert(B(0), prevBias_);
            // optimize once
            optimizer->update(newgraph, initialEstimate);
            newgraph.resize(0);
            initialEstimate.clear();

            key_count = 1;
            prev_globalmap = globalmap;
            prev_centroid = curr_centroid;
        }

        // 1. ndt registration, imu integration and optimization
        const std::uint64_t active_map_revision = map_revision_.load();
        const bool first_local_map = !timing_has_matched_;
        const bool map_changed_for_timing =
            timing_has_matched_ && active_map_revision != last_match_map_revision_;
        const bool steady_timing_frame = !first_local_map && !map_changed_for_timing;
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::Quaternion(pose_estimator->quat().w(),pose_estimator->quat().x(),pose_estimator->quat().y(),pose_estimator->quat().z()),
                                            gtsam::Point3(pose_estimator->pos()(0),pose_estimator->pos()(1),pose_estimator->pos()(2)));
        // std::cout<<"prior value: "<<poseFrom.x()<<" "<<poseFrom.y()<<" "<<poseFrom.z()<<" "<<poseFrom.rotation().roll()<<" "
        //                   <<poseFrom.rotation().pitch()<<" "<<poseFrom.rotation().yaw()<<std::endl;
        auto aligned = pose_estimator->correct(filtered);
        recordMatcherTiming(first_local_map, map_changed_for_timing,
                            steady_timing_frame, active_map_revision);
        gtsam::Pose3 poseTo_beforeExt = gtsam::Pose3(gtsam::Rot3::Quaternion(pose_estimator->quat().w(),pose_estimator->quat().x(),pose_estimator->quat().y(),pose_estimator->quat().z()),
                                            gtsam::Point3(pose_estimator->pos()(0),pose_estimator->pos()(1),pose_estimator->pos()(2)));
        NODELET_INFO_STREAM_THROTTLE(1.0,
            "NDT pose xyz=[" << poseTo_beforeExt.x() << ", " << poseTo_beforeExt.y()
            << ", " << poseTo_beforeExt.z() << "] rpy=["
            << poseTo_beforeExt.rotation().roll() << ", "
            << poseTo_beforeExt.rotation().pitch() << ", "
            << poseTo_beforeExt.rotation().yaw() << "] converged="
            << pose_estimator->hasConverged() << " fitness="
            << pose_estimator->fitnessScore());
        // std::cout<<"update value:"<<poseTo.x()<<" "<<poseTo.y()<<" "<<poseTo.z()<<" "<<poseTo.rotation().roll()<<" "
        //                   <<poseTo.rotation().pitch()<<" "<<poseTo.rotation().yaw()<<std::endl;

        // imu integration
        imu_count = 0;
        sum_ang_vel = 0;
        std::lock_guard<std::mutex> lock(imu_data_mutex);
        while (!imuQueOpt.empty()) {
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = getROSTime(thisImu);
            if (imuTime < points_curr_time) {
                double dt = (lastImuT_opt < 0) ? (1 / imuFrequency) : (imuTime - lastImuT_opt);
                if (dt < 1e-16) dt = 1e-16;
                const auto& acc = thisImu->linear_acceleration;
                const auto& gyro = thisImu->angular_velocity;
                imuIntegratorOpt_->integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);
                
                lastImuT_opt = imuTime;
                imu_count++;
                sum_ang_vel += gyro.z;
                imuQueOpt.pop_front();
            } else
                break;
        }
        average_ang_vel = sum_ang_vel / imu_count;
        // NODELET_INFO_STREAM("Average angular velocity on z-axis between two lidar frames: " << average_ang_vel << "[rad/s]");
        
        gtsam::Pose3 poseTo = poseTo_beforeExt.compose(lidar2Imu);
        // add penalty for the effect of the large angular velocity on z-axis in ndt
        if(fabs(average_ang_vel) > penalty_thres)
            correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, penalty_weight * fabs(average_ang_vel / penalty_thres));
        else
            correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1); // meter
        // add pose factor
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_count), poseTo, correctionNoise);
        newgraph.add(pose_factor);

        // add imu factor
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key_count - 1), V(key_count - 1), X(key_count), V(key_count), B(key_count - 1), preint_imu);
        newgraph.add(imu_factor);
        // add imu bias
        newgraph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_count - 1), B(key_count), gtsam::imuBias::ConstantBias(),
                                                                    gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);

        //  incremental constraint
        if ((lidarPose.translation().z() - lidarPose_temp.translation().z())< -0.02)
        {
            gtsam::Quaternion quat(lidarPose.rotation().toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()-0.02));
        }
        else if ((lidarPose.translation().z() - lidarPose_temp.translation().z()) > 0.02)
        {
            gtsam::Quaternion quat(lidarPose.rotation().toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()+0.02));
        }

        if ((lidarPose.rotation().roll() - lidarPose_temp.rotation().roll())< -0.02)
        {

            gtsam::Rot3 newRotation = gtsam::Rot3::RzRyRx(lidarPose.rotation().roll() - 0.02, lidarPose.rotation().pitch(), lidarPose.rotation().yaw());
            gtsam::Quaternion quat(newRotation.toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()));
        }
        else if  ((lidarPose.rotation().roll() - lidarPose_temp.rotation().roll())> 0.02)
        {

            gtsam::Rot3 newRotation = gtsam::Rot3::RzRyRx(lidarPose.rotation().roll() + 0.02, lidarPose.rotation().pitch(), lidarPose.rotation().yaw());
            gtsam::Quaternion quat(newRotation.toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()));
        }
        if ((lidarPose.rotation().pitch() - lidarPose_temp.rotation().pitch())< -0.02)
        {

            gtsam::Rot3 newRotation = gtsam::Rot3::RzRyRx(lidarPose.rotation().roll() , lidarPose.rotation().pitch()- 0.02, lidarPose.rotation().yaw());
            gtsam::Quaternion quat(newRotation.toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()));
        }
        else if  ((lidarPose.rotation().pitch() - lidarPose_temp.rotation().pitch())> 0.02)
        {

            gtsam::Rot3 newRotation = gtsam::Rot3::RzRyRx(lidarPose.rotation().roll() , lidarPose.rotation().pitch()+ 0.02, lidarPose.rotation().yaw());
            gtsam::Quaternion quat(newRotation.toQuaternion());
            quat.normalized();
            lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(quat.w(),quat.x(),quat.y(),quat.z()),gtsam::Point3(lidarPose.translation().x(), lidarPose.translation().y(),lidarPose_temp.translation().z()));
        }
        // std::cout << "propState_.pose().z(): " << propState_.pose().z() << std::endl;

        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << odomLinearNoise, odomLinearNoise, odomLinearNoise, odomAngularNoise, odomAngularNoise, odomAngularNoise).finished());
        newgraph.add(BetweenFactor<Pose3>(X(key_count - 1), X(key_count), poseFrom.between(poseTo), odometryNoise));

        // insert predicted values
        initialEstimate.insert(X(key_count), poseTo);
        initialEstimate.insert(V(key_count), propState_.v());
        // Eigen::Vector3d z_zero(propState_.v()(0), propState_.v()(1), 0); // test in indoor,limit the z velocity to 0
        // initialEstimate.insert(V(key_count), z_zero); //test in indoor,limit the z velocity to 0
        initialEstimate.insert(B(key_count), prevBias_);

        // optimize
        optimizer->update(newgraph, initialEstimate);
        newgraph.resize(0);
        initialEstimate.clear();
        
        isamCurrentEstimate = optimizer->calculateEstimate();
        prevPose_ = isamCurrentEstimate.at<Pose3>(X(key_count));
        prevVel_ = isamCurrentEstimate.at<Vector3>(V(key_count));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = isamCurrentEstimate.at<imuBias::ConstantBias>(B(key_count));
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        pose_estimator->update(prevState_, prevBias_);

        // transform pose Twb * Tbl = Twl
        lidarPose = prevPose_.compose(imu2Lidar);
        NODELET_INFO_STREAM_THROTTLE(1.0,
            "Fused pose xyz=[" << lidarPose.x() << ", " << lidarPose.y() << ", "
            << lidarPose.z() << "] rpy=[" << lidarPose.rotation().roll() << ", "
            << lidarPose.rotation().pitch() << ", " << lidarPose.rotation().yaw() << "]");
        lidarPose_temp=gtsam::Pose3(lidarPose);

        // 2. after optimization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;
        double lastImuQT = -1;
        while (!imuQueImu.empty() && getROSTime(&imuQueImu.front()) < points_curr_time) {
            lastImuQT = getROSTime(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropagate
        if (!imuQueImu.empty()) {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i) {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = getROSTime(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / imuFrequency) : (imuTime - lastImuQT);
                if (dt < 1e-16) dt = 1e-16;
                const auto& acc = thisImu->linear_acceleration;
                const auto& gyro = thisImu->angular_velocity;
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(gyro.x, gyro.y, gyro.z), dt);
                lastImuQT = imuTime;
            }
        }

        // publish odometry and path
        publishGlobalOdomPath(points_msg.header.stamp, lidarPose);

        // save trajectory following TUM format

        // Twgt=Twl*Tlgt
        gt_pose=lidarPose.compose(lidar2gt);
        saveTUMTraj(points_curr_time, gt_pose);

        points_pre_time = points_curr_time;
        ++key_count;
        ++time_count;

        // the core step: check conditions for querying a new block map 
        queryMapSwitch(lidarPose);

        doneFirstOpt = true;
    }


    bool queryGlobalLocalization(const sensor_msgs::PointCloud2ConstPtr& cloud_in) {
        NODELET_INFO("Querying global location.");
        ros::ServiceClient query_client = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");
        hdl_global_localization::QueryGlobalLocalization qry;
        qry.request.cloud = *cloud_in;
        qry.request.max_num_candidates = 1;
        query_client.call(qry);
        NODELET_INFO("Globallocalization queried!");

        std::lock_guard<std::mutex> lock(pose_estimator_mutex);
        const auto &p = qry.response.poses[0].position;
        const auto &q = qry.response.poses[0].orientation;
        pose_estimator.reset(
            new block_localization::PoseEstimator(
                registration,
                ros::Time::now(),
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                private_nh.param<double>("/block_localization/cool_time_duration", 0.5))
        );
        if(!pose_estimator){
            NODELET_INFO("Fail to get initial pose from global localizer!");
            return false;
        }
            
        return true;
    }


    void globalmapCB(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        // NODELET_INFO("globalmap received!");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*points_msg, *cloud);
        // Matching and target replacement must not touch the registration
        // object concurrently when nodelets run in separate managers.
        std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
        if(!map_init) {
            globalmap = cloud;
            prev_globalmap = globalmap;
            registration->setInputTarget(globalmap);
            pcl::compute3DCentroid(*globalmap, curr_centroid);
            prev_centroid = curr_centroid;
            map_init = true;
            map_revision_.fetch_add(1);
        } else {
            gtsam::Vector4 next_centroid;
            pcl::compute3DCentroid(*cloud, next_centroid);
            const bool map_changed = !curr_centroid.isApprox(next_centroid, 1e-5);
            globalmap = cloud;
            curr_centroid = next_centroid;
            if (map_changed) map_revision_.fetch_add(1);
        }
    }


    void initialposeCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
        NODELET_INFO("Initial pose received!");
        std::lock_guard<std::mutex> lock(pose_estimator_mutex);
        const auto& p = pose_msg->pose.pose.position;
        const auto& q = pose_msg->pose.pose.orientation;
        pose_estimator.reset(
            new block_localization::PoseEstimator(
                registration,
                ros::Time::now(),
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                private_nh.param<double>("/block_localization/cool_time_duration", 0.5))
        );
    }


    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
        if(!downsample_filter) {
            return cloud;
        }
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        downsample_filter->setInputCloud(cloud);
        downsample_filter->filter(*filtered);
        filtered->header = cloud->header;
        return filtered;
    }


    void publishGlobalOdomPath(ros::Time timestamp, gtsam::Pose3 pose) {
        Eigen::Quaterniond quat(pose.rotation().toQuaternion().w(), pose.rotation().toQuaternion().x(), pose.rotation().toQuaternion().y(), pose.rotation().toQuaternion().z());
        quat.normalize();
        geometry_msgs::Quaternion odom_quat;
        odom_quat.w = quat.w();
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();

        // broadcast transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = timestamp;
        odom_trans.header.frame_id = "globalmap_link";
        odom_trans.child_frame_id = odom_child_frame_id;
        odom_trans.transform.translation.x = pose.x();
        odom_trans.transform.translation.y = pose.y();
        odom_trans.transform.translation.z = pose.z();
        odom_trans.transform.rotation = odom_quat;
        pose_broadcaster.sendTransform(odom_trans);

        // publish odometry
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timestamp;
        laserOdometryROS.header.frame_id = "globalmap_link";
        laserOdometryROS.child_frame_id = "odom_bloc";
        laserOdometryROS.pose.pose.position.x = pose.translation().x();
        laserOdometryROS.pose.pose.position.y = pose.translation().y();
        laserOdometryROS.pose.pose.position.z = pose.translation().z();
        laserOdometryROS.pose.pose.orientation = odom_quat;
        pose_pub.publish(laserOdometryROS);

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = laserOdometryROS.header;
        laserPose.pose = laserOdometryROS.pose.pose;
        odomPath.header.stamp = laserOdometryROS.header.stamp;
        odomPath.poses.push_back(laserPose);
        odomPath.header.frame_id = "globalmap_link";
        path_pub.publish(odomPath);
    }


    void saveTUMTraj(double timestamp, const gtsam::Pose3& pose) {
        if (!trajectory_stream_.is_open()) return;
        Eigen::Quaterniond quat(pose.rotation().toQuaternion().w(), pose.rotation().toQuaternion().x(), pose.rotation().toQuaternion().y(), pose.rotation().toQuaternion().z());
        quat.normalize();
        auto trans = pose.translation();
        trajectory_stream_ << std::fixed << std::setprecision(9)
                           << timestamp << " " << trans.x() << " " << trans.y() << " "
                           << trans.z() << " " << quat.x() << " " << quat.y() << " "
                           << quat.z() << " " << quat.w() << '\n';
        trajectory_stream_.flush();
    }


    void initializeOutputFiles() {
        const auto ensure_parent = [this](const std::string& filename) {
            if (filename.empty()) return false;
            const std::filesystem::path path(filename);
            const auto parent = path.parent_path();
            if (parent.empty()) return true;
            std::error_code error;
            std::filesystem::create_directories(parent, error);
            if (error) {
                NODELET_ERROR_STREAM("Cannot create output directory " << parent
                                     << ": " << error.message());
                return false;
            }
            return true;
        };

        if (ensure_parent(trajectory_output_path)) {
            trajectory_stream_.open(trajectory_output_path, std::ios::out | std::ios::trunc);
            if (trajectory_stream_) {
                NODELET_INFO_STREAM("TUM trajectory output: " << trajectory_output_path);
            } else {
                NODELET_ERROR_STREAM("Cannot open trajectory output: " << trajectory_output_path);
            }
        }

        timing_log_output_path_ =
            (std::filesystem::path(globalmap_dir) / "blockmap_timing.log").string();
        if (ensure_parent(timing_log_output_path_)) {
            timing_log_stream_.open(timing_log_output_path_,
                                    std::ios::out | std::ios::trunc);
            if (timing_log_stream_) {
                NODELET_INFO_STREAM("Matcher timing log: " << timing_log_output_path_);
            } else {
                NODELET_ERROR_STREAM("Cannot open matcher timing log: "
                                     << timing_log_output_path_);
            }
        }

    }


    void recordMatcherTiming(bool first_local_map, bool map_changed, bool steady,
                             std::uint64_t map_revision) {
        ++timing_frame_count_;
        const bool success = pose_estimator->hasConverged();
        const std::uint64_t matcher_total_us = pose_estimator->matcherTotalTimeUs();
        const std::uint64_t correspondence_us = pose_estimator->correspondenceTimeUs();

        if (!pose_estimator->hasCorrespondenceTiming() && !timing_unsupported_warned_) {
            NODELET_WARN("Correspondence timing is only available for NDT_OMP; "
                         "GICP_OMP records matcher_total_us only.");
            timing_unsupported_warned_ = true;
        }

        NODELET_INFO_STREAM("[BLOCKMAP_TIMING] frame=" << timing_frame_count_
            << ",first_local_map=" << static_cast<int>(first_local_map)
            << ",map_changed=" << static_cast<int>(map_changed)
            << ",steady=" << static_cast<int>(steady)
            << ",success=" << static_cast<int>(success)
            << ",correspondence_us=" << correspondence_us
            << ",matcher_total_us=" << matcher_total_us);

        if (timing_log_stream_) {
            timing_log_stream_ << "[BLOCKMAP_TIMING] frame=" << timing_frame_count_
                               << ",first_local_map=" << static_cast<int>(first_local_map)
                               << ",map_changed=" << static_cast<int>(map_changed)
                               << ",steady=" << static_cast<int>(steady)
                               << ",success=" << static_cast<int>(success)
                               << ",correspondence_us=" << correspondence_us
                               << ",matcher_total_us=" << matcher_total_us << '\n';
            timing_log_stream_.flush();
        }

        timing_has_matched_ = true;
        last_match_map_revision_ = map_revision;
    }


    void queryMapSwitch(gtsam::Pose3 pose) {
        double time_elapsed = points_curr_time - mapqry_pre_time;

        if((time_elapsed < mapqry_interval) && systemInitialized)
            return;
        
        mapqry_pre_time = points_curr_time;

        if (!ros::service::waitForService("/mapQuery", ros::Duration(2.0))) {
            NODELET_ERROR_THROTTLE(2.0, "Map query service is unavailable.");
            return;
        }
        block_localization::queryMap qry;
        qry.request.position.x = pose.x();
        qry.request.position.y = pose.y();
        qry.request.position.z = pose.z();
        if (!mapQuery_client.call(qry) || !qry.response.success) {
            NODELET_ERROR_THROTTLE(2.0, "Map query failed.");
        }
    }


    void resetOptimization() {
        // std::cout << "resetOptimization" << std::endl;
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = new gtsam::ISAM2(optParameters);
        gtsam::NonlinearFactorGraph newGraphFactors;
        newgraph = newGraphFactors;
        gtsam::Values NewGraphValues;
        initialEstimate = NewGraphValues;
    }


private:
    // imu
    std::mutex imu_data_mutex;
    std::deque<sensor_msgs::Imu> imuQueImu;
    std::deque<sensor_msgs::Imu> imuQueOpt;
    double lastImuT_opt = -1;
    double lastImuT_imu = -1;

    // publishers and subscribers
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    ros::Publisher imuOdom_pub;
    nav_msgs::Path odomPath;
    ros::Subscriber imu_sub;
    ros::Subscriber points_sub;
    ros::Subscriber globalmap_sub;
    ros::Subscriber initialpose_sub;

    // sensors extrinsics
    gtsam::Pose3 imu2Lidar;
    gtsam::Pose3 lidar2Imu;
    gtsam::Pose3 lidar2gt;

    // system settings
    double points_pre_time, points_curr_time, mapqry_pre_time;

    // tf and frame_ids
    tf::TransformBroadcaster pose_broadcaster;
    tf::TransformListener tf_listener;

    // globalmap and registration method
    bool map_init = false;
    ros::ServiceClient mapQuery_client;
    pcl::PointCloud<PointT>::Ptr prev_globalmap;
    pcl::PointCloud<PointT>::Ptr globalmap;
    gtsam::Vector4 prev_centroid;
    gtsam::Vector4 curr_centroid;
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Registration<PointT, PointT>::Ptr registration;
    std::atomic<std::uint64_t> map_revision_{0};

    // pose estimator
    std::mutex pose_estimator_mutex;
    std::unique_ptr<block_localization::PoseEstimator> pose_estimator;
    
    // add penalty factor for the large ang vel
    int imu_count;
    double sum_ang_vel;
    double average_ang_vel;

    // EVO trajectory and per-frame matcher timing outputs.
    std::ofstream trajectory_stream_;
    std::ofstream timing_log_stream_;
    std::string timing_log_output_path_;
    std::uint64_t timing_frame_count_ = 0;
    std::uint64_t last_match_map_revision_ = 0;
    bool timing_has_matched_ = false;
    bool timing_unsupported_warned_ = false;


public:
    bool first_query{true};
    bool firstkey{true};

    // GTSAM settings
    NonlinearFactorGraph newgraph;
    gtsam::ISAM2 *optimizer;
    gtsam::Values initialEstimate;
    gtsam::Values isamCurrentEstimate;
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    bool systemInitialized;
    bool doneFirstOpt;
    int key_count;
    int time_count;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;
    gtsam::Pose3 lidarPose;
    gtsam::Pose3 lidarPose_temp;
    gtsam::Pose3 gt_pose;
};

}


PLUGINLIB_EXPORT_CLASS(block_localization::BlockLocalizationNodelet, nodelet::Nodelet)
