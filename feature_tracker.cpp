#include <iostream>
#include <thread>
#include <chrono>

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
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"

#define CAM_W 640
#define CAM_H 400
#define PAIR_DIST_SQ 9

using namespace std::chrono_literals;

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

    bool imu_ok = false;
    int ccc=0;

    ros::init(argc, argv, "feature_tracker", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Publisher pp_pub = nh.advertise<sensor_msgs::PointCloud>("/feature_tracker/feature", 10);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu", 100);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();
    auto imu = pipeline.create<dai::node::IMU>();

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();

    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xout_disp->setStreamName("disparity");
    xout_imu->setStreamName("imu");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoLeft->setFps(20);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");
    monoRight->setFps(20);

    featureTrackerLeft->initialConfig.setNumTargetFeatures(16*5);
    featureTrackerRight->initialConfig.setNumTargetFeatures(16*5);

    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
    depth->setLeftRightCheck(true);
    depth->setExtendedDisparity(false);
    depth->setSubpixel(false);
    depth->setDepthAlign(dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
    depth->setAlphaScaling(0);

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 125);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 100);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

    // Linking
    monoLeft->out.link(depth->left);
    depth->rectifiedLeft.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    monoRight->out.link(depth->right);
    depth->rectifiedRight.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    depth->disparity.link(xout_disp->input);
    imu->out.link(xout_imu->input);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    std::cout << "Usb speed: " << device.getUsbSpeed() << "\n";
    std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << "\n";

    //device.setLogOutputLevel(dai::LogLevel::TRACE);
    //device.setLogLevel(dai::LogLevel::TRACE);

    // Output queues used to receive the results
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);
    auto imuQueue = device.getOutputQueue("imu", 50, false);

    int l_seq = -1, r_seq = -2, disp_seq = -3;
    std::vector<std::uint8_t> disp_frame;
    std::vector<dai::TrackedFeature> l_features, r_features;
    std::map<int, dai::Point2f> l_prv_features, r_prv_features, r_cur_features;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> features_tp, prv_features_tp;
    std::map<int, int> lr_id_mapping;

    // Clear queue events
    device.getQueueEvents();

    while(true) {
        std::this_thread::sleep_for(1ms);

        if (outputFeaturesLeftQueue->has()) {
            auto data = outputFeaturesLeftQueue->get<dai::TrackedFeatures>();
            l_features = data->trackedFeatures;
            l_seq = data->getSequenceNum();
            features_tp = data->getTimestamp();
            //std::cout << "ft latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - features_tp).count() << " ms\n";
        }
        if (outputFeaturesRightQueue->has()) {
            auto data = outputFeaturesRightQueue->get<dai::TrackedFeatures>();
            r_features = data->trackedFeatures;
            r_seq = data->getSequenceNum();
            r_cur_features.clear();
            for (const auto &feature : r_features) {
                r_cur_features[feature.id] = feature.position;
            }
        }
        if (disp_queue->has()) {
            auto disp_data = disp_queue->get<dai::ImgFrame>();
            disp_seq = disp_data->getSequenceNum();
            disp_frame = disp_data->getData();
            //std::cout << "stereo latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - disp_data->getTimestamp()).count() << " ms\n";
        }
        if (imuQueue->has()) {
            auto imuData = imuQueue->get<dai::IMUData>();
            auto imuPackets = imuData->packets;
            for(const auto& imuPacket : imuPackets) {
                auto& acc = imuPacket.acceleroMeter;
                auto& gyro = imuPacket.gyroscope;
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(acc.getTimestamp().time_since_epoch()).count());
                imu_msg.linear_acceleration.x = acc.z;
                imu_msg.linear_acceleration.y = acc.y;
                imu_msg.linear_acceleration.z = -acc.x;
                imu_msg.angular_velocity.x = gyro.z;
                imu_msg.angular_velocity.y = gyro.y;
                imu_msg.angular_velocity.z = -gyro.x;
                imu_pub.publish(imu_msg);
            }
            if (!imu_ok) {
                imu_ok = true;
                std::cout<< "imu ok\n";
            }
        }

        if (l_seq == r_seq && r_seq == disp_seq) {
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
                auto lr_id = lr_id_mapping.find(l_feature.id);
                if (lr_id != lr_id_mapping.end()) {
                    auto r_feature = r_cur_features.find(lr_id->second);
                    if (r_feature != r_cur_features.end()) {
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

                        x = r_feature->second.x;
                        y = r_feature->second.y;
                        vx = 0;
                        vy = 0;
                        cur_un_x = r_inv_k11 * x + r_inv_k13;
                        cur_un_y = r_inv_k22 * y + r_inv_k23;
                        prv_pos = r_prv_features.find(r_feature->first);
                        if (prv_pos != r_prv_features.end()) {
                            vx = (cur_un_x - prv_pos->second.x) / dt;
                            vy = (cur_un_y - prv_pos->second.y) / dt;
                        }
                        p.x = cur_un_x;
                        p.y = cur_un_y;
                        p.z = 1;
                        pp_msg.points.push_back(p);
                        pp_msg.channels[0].values.push_back(l_feature.id);
                        pp_msg.channels[1].values.push_back(1);
                        pp_msg.channels[2].values.push_back(x);
                        pp_msg.channels[3].values.push_back(y);
                        pp_msg.channels[4].values.push_back(vx);
                        pp_msg.channels[5].values.push_back(vy);

                        continue;
                    }
                }
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
                            lr_id_mapping[l_feature.id] = r_feature.id;
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
                            pp_msg.channels[0].values.push_back(l_feature.id);
                            pp_msg.channels[1].values.push_back(1);
                            pp_msg.channels[2].values.push_back(x);
                            pp_msg.channels[3].values.push_back(y);
                            pp_msg.channels[4].values.push_back(vx);
                            pp_msg.channels[5].values.push_back(vy);

                            break;
                        }
                    }
                }
            }
            ccc++;
            if (ccc > 60) {
                ccc = 0;
                std::cout << "total latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - features_tp).count() << " ms\n";
            }
            if (pp_msg.points.size() > 0) pp_pub.publish(pp_msg);
            l_prv_features = features;
            prv_features_tp = features_tp;
            r_prv_features.clear();
            for (const auto &r_feature : r_features) {
                r_prv_features[r_feature.id] = dai::Point2f(r_inv_k11 * r_feature.position.x + r_inv_k13, r_inv_k22 * r_feature.position.y + r_inv_k23);
            }
            //auto t2 = std::chrono::steady_clock::now();
            //std::cout << pp_msg.points.size() << " points, " << std::chrono::duration<float, std::milli>(t2-t1).count() << " ms\n";
        }
    }
    return 0;
}
