#include "utility.h"
#include "block_localization/queryMap.h"

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <mutex>
#include <sstream>

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif


namespace block_localization {

class GlobalmapServerNodelet : public ParamServer, public nodelet::Nodelet
{
public:
    GlobalmapServerNodelet() {
    }
    virtual ~GlobalmapServerNodelet() {
    }

    void onInit() override {
        nh = getNodeHandle();

        loadAllMap();

        loadMapCentroids();

        // Load and publish the initial BM. The publisher is latched so the
        // localization nodelet receives a valid target immediately.
        globalmap = *loadMapFromIdx(0);
        active_map_indices_ = {0};
        globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
        publishGlobalmap();
        mapQueryServer = nh.advertiseService("/mapQuery", &GlobalmapServerNodelet::mapQueryCB, this);
    }

private:
    void publishGlobalmap() {
        sensor_msgs::PointCloud2 ros_cloud;
        {
            std::lock_guard<std::mutex> lock(globalmap_mutex);
            if (globalmap.empty()) return;
            pcl::toROSMsg(globalmap, ros_cloud);
        }
        // Concatenating block clouds into a default-constructed PCL cloud does
        // not preserve the source header.  Set a valid ROS header explicitly
        // so RViz can transform and refresh every switched local map.
        ros_cloud.header.frame_id = "globalmap_link";
        ros_cloud.header.stamp = ros::Time::now();
        globalmap_pub.publish(ros_cloud);
    }


    bool mapQueryCB(block_localization::queryMap::Request &req,
                    block_localization::queryMap::Response &res) {
        PointT searchPoint;
        searchPoint.x = req.position.x;
        searchPoint.y = req.position.y;
        // Block-map lookup is planar. A temporary vertical localization error
        // must not select a geographically unrelated block.
        searchPoint.z = 0.0f;
        // NODELET_INFO("K-nearest neighbor search at (%f, %f, %f).", searchPoint.x, searchPoint.y, searchPoint.z);
        
        // Keep active maps while the vehicle is still inside their XY extent.
        // The margin prevents centroid-order changes near a block boundary from
        // withdrawing a map which is still needed by the current scan.
        constexpr float retention_margin_m = 10.0f;
        constexpr std::size_t max_active_maps = 3;
        std::vector<int> selected_indices;
        for (const int map_idx : active_map_indices_) {
            if (map_idx < 0 || static_cast<std::size_t>(map_idx) >= map_bounds_.size()) continue;
            const auto& bounds = map_bounds_[map_idx];
            if (searchPoint.x >= bounds[0] - retention_margin_m &&
                searchPoint.x <= bounds[1] + retention_margin_m &&
                searchPoint.y >= bounds[2] - retention_margin_m &&
                searchPoint.y <= bounds[3] + retention_margin_m) {
                selected_indices.push_back(map_idx);
            }
        }

        // Fill the remaining slots with the nearest centroid candidates.
        pointIdxNKNSearch.clear();
        pointNKNSquareDistance.clear();
        int k_nearest = centroid_kdtree.nearestKSearch(
            searchPoint, 3, pointIdxNKNSearch, pointNKNSquareDistance);
        for (int i = 0; i < k_nearest && selected_indices.size() < max_active_maps; ++i) {
            const int candidate = pointIdxNKNSearch[i];
            if (std::find(selected_indices.begin(), selected_indices.end(), candidate) ==
                selected_indices.end()) {
                selected_indices.push_back(candidate);
            }
        }

        if (selected_indices.empty()) {
            res.success = false;
            return true;
        }

        pcl::PointCloud<PointT> queried_map;
        for (const int map_idx : selected_indices) {
            queried_map += *loadMapFromIdx(map_idx);
        }

        {
            std::lock_guard<std::mutex> lock(globalmap_mutex);
            globalmap.swap(queried_map);
            active_map_indices_ = selected_indices;
        }
        publishGlobalmap();
        std::ostringstream selected_stream;
        for (std::size_t i = 0; i < selected_indices.size(); ++i) {
            if (i != 0) selected_stream << ", ";
            selected_stream << selected_indices[i];
        }
        NODELET_INFO("Published retained block-map set [%s] for query (%.2f, %.2f).",
                     selected_stream.str().c_str(), req.position.x, req.position.y);

        res.success = true;
        return true;
    }


    void loadAllMap() {
        auto t1 = ros::WallTime::now();
        boost::format load_format("%03d.pcd");

        for (int map_id = 0; ; map_id++)
        {
            pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
            std::string map_name = globalmap_dir + (load_format % map_id).str();

            if (pcl::io::loadPCDFile(map_name, *tmp_cloud) == -1) {
                map_id -= 1;
                map_name = globalmap_dir + (load_format % map_id).str();
                NODELET_WARN("The last map is: %s", map_name.c_str());
                break;
            }

            tmp_cloud->header.frame_id = "globalmap_link";

            // downsampling
            boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
            voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            voxelgrid->setInputCloud(tmp_cloud);
            pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
            voxelgrid->filter(*filtered_cloud);
            tmp_cloud = filtered_cloud;

            PointT min_point;
            PointT max_point;
            pcl::getMinMax3D(*tmp_cloud, min_point, max_point);
            map_bounds_.push_back(
                {min_point.x, max_point.x, min_point.y, max_point.y});
            
            globalmap_vec.push_back(tmp_cloud);
        }
        auto t2 = ros::WallTime::now();
        NODELET_INFO("Globalmap server has already loaded %ld maps. Time cost: %f [msec]", globalmap_vec.size(), (t2 - t1).toSec() * 1000.0);
    }


    inline pcl::PointCloud<PointT>::Ptr loadMapFromIdx(int map_idx) {
        // NODELET_INFO("Globalmap NO.%03d is already loaded!", map_idx);
        return globalmap_vec[map_idx];
    }


    void loadMapCentroids() {
        std::string centroid_filename = globalmap_dir + "CentroidCloud.pcd";
        
        centroid_cloud.reset(new pcl::PointCloud<PointT>());
        if (pcl::io::loadPCDFile(centroid_filename, *centroid_cloud) == -1) {
            NODELET_ERROR("Fail to load the centroid cloud! Please check your source!");
            ros::shutdown();
        }

        NODELET_INFO("Already loaded %ld centroids of block maps.", centroid_cloud->points.size());

        if (centroid_cloud->empty()) {
            NODELET_ERROR("Fail to build a KD-Tree! Please check your centroid pointcloud!");
            ros::shutdown();
        }

        // Use XY only for the centroid KD-tree as well as for query points.
        for (auto& point : centroid_cloud->points) point.z = 0.0f;
        centroid_kdtree.setInputCloud(centroid_cloud);
    }


    void pubGridmap() {
        YAML::Node doc = YAML::LoadFile(yaml_path_);
        try
        {
            doc["resolution"] >> res_;
            doc["origin"][0] >> origin_[0];
            doc["origin"][1] >> origin_[1];
            doc["origin"][2] >> origin_[2];
            doc["negate"] >> negate_;
            doc["occupied_thresh"] >> occ_th_;
            doc["free_thresh"] >> free_th_;
        }
        catch(YAML::InvalidScalar)
        {
            ROS_ERROR("The .yaml does not contain tags required or they are invalid.");
            ros::shutdown();
        }
        mode_ = TRINARY;

        std::cout << "	       map name: " << map_name_
                    << "\n	     resolution: " << res_
                    << "\n	         origin: " << origin_[0] << ", " << origin_[1] << ", " << origin_[2]
                    << "\n	         negate: " << negate_
                    << "\n	occupied thresh: " << occ_th_
                    << "\n	    free thresh: " << free_th_ << std::endl;
        map_server::loadMapFromFile(&map_resp_, pgm_path_.c_str(), res_, negate_, occ_th_, free_th_, origin_, mode_);
        map_resp_.map.header.frame_id = "gridmap_link";
        map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> ("/gridmap", 1, true);
        map_publisher_.publish(map_resp_.map);
    }


private:
    // map settings
    pcl::PointCloud<PointT> globalmap;
    std::vector<pcl::PointCloud<PointT>::Ptr> globalmap_vec;
    std::vector<std::array<float, 4>> map_bounds_;
    std::vector<int> active_map_indices_;

    ros::ServiceServer mapQueryServer;
    ros::Publisher globalmap_pub;
    std::mutex globalmap_mutex;

    // block map centroids
    pcl::PointCloud<PointT>::Ptr centroid_cloud;

    // KD-Tree for block maps
    pcl::KdTreeFLANN<PointT> centroid_kdtree;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquareDistance;

    MapMode mode_;
    std::string map_name_;
    double res_;
    double origin_[3];
    int negate_;
    double occ_th_, free_th_;
    nav_msgs::GetMap::Response map_resp_;
    ros::Publisher map_publisher_;
};

}


PLUGINLIB_EXPORT_CLASS(block_localization::GlobalmapServerNodelet, nodelet::Nodelet)
