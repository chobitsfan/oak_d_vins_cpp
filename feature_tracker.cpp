#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Header.h"

#define CAM_W 640
#define CAM_H 400
#define PAIR_DIST_SQ 16

int main(int argc, char **argv) {
    double fx = 4.2007635451376063e+02;
    double cx = 3.0906091871893943e+02;
    double fy = 4.1970786993146550e+02;
    double cy = 2.0014588000163775e+02;
    double l_inv_k11 = 1.0 / fx;
    double l_inv_k13 = -cx / fx;
    double l_inv_k22 = 1.0 / fy;
    double l_inv_k23 = -cy / fy;
    fx = 4.1911737221609309e+02;
    cx = 3.1094101439493375e+02;
    fy = 4.1911302233054698e+02;
    cy = 1.9966812289233278e+02;
    double r_inv_k11 = 1.0 / fx;
    double r_inv_k13 = -cx / fx;
    double r_inv_k22 = 1.0 / fy;
    double r_inv_k23 = -cy / fy;

    ros::init(argc, argv, "feature_tracker", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Publisher pp_pub = nh.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 10);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();

    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xout_disp->setStreamName("disparity");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    featureTrackerLeft->initialConfig.setNumTargetFeatures(16*3);
    featureTrackerRight->initialConfig.setNumTargetFeatures(16*3);

    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(true);
    depth->setExtendedDisparity(false);
    depth->setSubpixel(false);
    depth->setDepthAlign(dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
    depth->setAlphaScaling(0);

    // Linking
    monoLeft->out.link(depth->left);
    depth->rectifiedLeft.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    monoRight->out.link(depth->right);
    depth->rectifiedRight.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    depth->disparity.link(xout_disp->input);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);

    int l_seq = -1, r_seq = -2, disp_seq = -3;
    std::vector<std::uint8_t> disp_frame;
    std::vector<dai::TrackedFeature> l_features, r_features;
    std::map<int, dai::Point2f> l_prv_features, r_prv_features;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> features_tp, prv_features_tp;

    // Clear queue events
    device.getQueueEvents();

    while(true) {
        auto q_name = device.getQueueEvent();

        if (q_name == "trackedFeaturesLeft") {
            auto data = outputFeaturesLeftQueue->get<dai::TrackedFeatures>();
            l_features = data->trackedFeatures;
            l_seq = data->getSequenceNum();
            features_tp = data->getTimestamp();

            //leftFeatureDrawer.trackFeaturePath(l_features);
        } else if (q_name == "trackedFeaturesRight") {
            auto data = outputFeaturesRightQueue->get<dai::TrackedFeatures>();
            r_features = data->trackedFeatures;
            r_seq = data->getSequenceNum();
        } else if (q_name == "disparity") {
            auto disp_data = disp_queue->get<dai::ImgFrame>();
            disp_seq = disp_data->getSequenceNum();
            disp_frame = disp_data->getData();
        }

        if (l_seq == r_seq && r_seq == disp_seq) {
            std::vector<dai::TrackedFeature> draw_l_features;
            //auto t1 = std::chrono::steady_clock::now();
            l_seq = -1;
            r_seq = -2;
            disp_seq = -3;
            std::map<int , dai::Point2f> features;
            sensor_msgs::PointCloud pp_msg;
            pp_msg.header.stamp = ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(features_tp.time_since_epoch()).count());
            pp_msg.channels = std::vector<sensor_msgs::ChannelFloat32>(6, sensor_msgs::ChannelFloat32());
            for (const auto &l_feature : l_features) {
                float x = l_feature.position.x;
                float y = l_feature.position.y;
                float cur_un_x = l_inv_k11 * x + l_inv_k13;
                float cur_un_y = l_inv_k22 * y + l_inv_k23;
                features[l_feature.id] = dai::Point2f(cur_un_x, cur_un_y);
                float row = roundf(y);
                float col = roundf(x);
                if (row > CAM_H - 1) row = CAM_H - 1;
                if (col > CAM_W - 1) col = CAM_W - 1;
                int disp = disp_frame[row * CAM_W + col];
                if (disp > 0) {
                    for (const auto &r_feature : r_features) {
                        float dy = y - r_feature.position.y;
                        float dx = x - disp - r_feature.position.x;
                        if (dy * dy + dx * dx <= PAIR_DIST_SQ) { //pair found
                            draw_l_features.push_back(l_feature);
                            float dt = std::chrono::duration<float>(features_tp - prv_features_tp).count();
                            float vx = 0, vy = 0;
                            auto prv_pos = l_prv_features.find(l_feature.id);
                            if (prv_pos != l_prv_features.end()) {
                                vx = (cur_un_x - prv_pos->second.x) / dt;
                                vy = (cur_un_y - prv_pos->second.y) / dt;
                            }
                            geometry_msgs::Point32 p;
                            p.x = cur_un_x;
                            p.y = cur_un_y;
                            p.z = 1;
                            pp_msg.points.push_back(p);
                            pp_msg.channels[0].values.push_back(l_feature.id);
                            pp_msg.channels[1].values.push_back(0);
                            pp_msg.channels[2].values.push_back(x);
                            pp_msg.channels[3].values.push_back(y);
                            pp_msg.channels[4].values.push_back(vx);
                            pp_msg.channels[5].values.push_back(vy);

                            x = r_feature.position.x;
                            y = r_feature.position.y;
                            vx = 0;
                            vy = 0;
                            cur_un_x = r_inv_k11 * x + r_inv_k13;
                            cur_un_y = r_inv_k22 * y + r_inv_k23;
                            prv_pos = r_prv_features.find(r_feature.id);
                            if (prv_pos != r_prv_features.end()) {
                                vx = (cur_un_x - prv_pos->second.x) / dt;
                                vy = (cur_un_y - prv_pos->second.y) / dt;
                            }
                            p.x = cur_un_x;
                            p.y = cur_un_y;
                            p.z = 1;
                            pp_msg.points.push_back(p);
                            pp_msg.channels[0].values.push_back(r_feature.id);
                            pp_msg.channels[1].values.push_back(0);
                            pp_msg.channels[2].values.push_back(x);
                            pp_msg.channels[3].values.push_back(y);
                            pp_msg.channels[4].values.push_back(vx);
                            pp_msg.channels[5].values.push_back(vy);

                            break;
                        }
                    }
                }
            }
            if (pp_msg.points.size() > 0) pp_pub.publish(pp_msg);
            l_prv_features = features;
            prv_features_tp = features_tp;
            r_prv_features.clear();
            for (const auto &r_feature : r_features) {
                r_prv_features[r_feature.id] = dai::Point2f(r_inv_k11 * r_feature.position.x + r_inv_k13, r_inv_k22 * r_feature.position.y + r_inv_k23);
            }
            //auto t2 = std::chrono::steady_clock::now();
            //std::cout << "cost " << std::chrono::duration<float, std::milli>(t2-t1).count() << " ms\n";
        }
    }
    return 0;
}
