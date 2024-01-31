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

static const auto lineColor = cv::Scalar(200, 0, 200);
static const auto pointColor = cv::Scalar(0, 0, 255);

class FeatureTrackerDrawer {
   private:
    static const int circleRadius = 2;
    static const int maxTrackedFeaturesPathLength = 30;
    // for how many frames the feature is tracked
    static int trackedFeaturesPathLength;

    using featureIdType = decltype(dai::Point2f::x);

    std::unordered_set<featureIdType> trackedIDs;
    std::unordered_map<featureIdType, std::deque<dai::Point2f>> trackedFeaturesPath;

    std::string trackbarName;
    std::string windowName;

   public:
    void trackFeaturePath(std::vector<dai::TrackedFeature>& features) {
        std::unordered_set<featureIdType> newTrackedIDs;
        for(auto& currentFeature : features) {
            auto currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(!trackedFeaturesPath.count(currentID)) {
                trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
            }
            std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

            path.push_back(currentFeature.position);
            while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        std::unordered_set<featureIdType> featuresToRemove;
        for(auto& oldId : trackedIDs) {
            if(!newTrackedIDs.count(oldId)) {
                featuresToRemove.insert(oldId);
            }
        }

        for(auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
    }

    void drawFeatures(cv::Mat& img) {
        cv::setTrackbarPos(trackbarName.c_str(), windowName.c_str(), trackedFeaturesPathLength);

        for(auto& featurePath : trackedFeaturesPath) {
            std::deque<dai::Point2f>& path = featurePath.second;
            unsigned int j = 0;
            for(j = 0; j < path.size() - 1; j++) {
                auto src = cv::Point(path[j].x, path[j].y);
                auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
                cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
            }

            cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
        }
    }

    FeatureTrackerDrawer(std::string trackbarName, std::string windowName) : trackbarName(trackbarName), windowName(windowName) {
        cv::namedWindow(windowName.c_str());
        cv::createTrackbar(trackbarName.c_str(), windowName.c_str(), &trackedFeaturesPathLength, maxTrackedFeaturesPathLength, nullptr);
    }
};

int FeatureTrackerDrawer::trackedFeaturesPathLength = 10;

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

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto xoutPassthroughFrameLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutPassthroughFrameRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto xinTrackedFeaturesConfig = pipeline.create<dai::node::XLinkIn>();

    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();

    xoutPassthroughFrameLeft->setStreamName("passthroughFrameLeft");
    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutPassthroughFrameRight->setStreamName("passthroughFrameRight");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xinTrackedFeaturesConfig->setStreamName("trackedFeaturesConfig");
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
    featureTrackerLeft->passthroughInputImage.link(xoutPassthroughFrameLeft->input);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerLeft->inputConfig);

    monoRight->out.link(depth->right);
    depth->rectifiedRight.link(featureTrackerRight->inputImage);
    featureTrackerRight->passthroughInputImage.link(xoutPassthroughFrameRight->input);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerRight->inputConfig);

    depth->disparity.link(xout_disp->input);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    printf("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation! \n");

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, false);
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto passthroughImageRightQueue = device.getOutputQueue("passthroughFrameRight", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);

    auto inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig");

    const auto leftWindowName = "left";
    auto leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName);

    const auto rightWindowName = "right";
    auto rightFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", rightWindowName);

    int l_seq = -1, r_seq = -2, disp_seq = -3;
    std::vector<std::uint8_t> disp_frame;
    std::vector<dai::TrackedFeature> l_features, r_features;
    std::map<int, dai::Point2f> l_prv_features, r_prv_features;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> features_tp, prv_features_tp;

    // Clear queue events
    device.getQueueEvents();

    while(true) {
        auto q_name = device.getQueueEvent();

        if (q_name == "passthroughFrameLeft") {
            auto inPassthroughFrameLeft = passthroughImageLeftQueue->get<dai::ImgFrame>();
            cv::Mat passthroughFrameLeft = inPassthroughFrameLeft->getFrame();
            cv::Mat leftFrame;
            cv::cvtColor(passthroughFrameLeft, leftFrame, cv::COLOR_GRAY2BGR);
            leftFeatureDrawer.drawFeatures(leftFrame);
            cv::imshow(leftWindowName, leftFrame);
        } else if (q_name == "passthroughFrameRight") {
            auto inPassthroughFrameRight = passthroughImageRightQueue->get<dai::ImgFrame>();
            cv::Mat passthroughFrameRight = inPassthroughFrameRight->getFrame();
            cv::Mat rightFrame;
            cv::cvtColor(passthroughFrameRight, rightFrame, cv::COLOR_GRAY2BGR);
            rightFeatureDrawer.drawFeatures(rightFrame);
            cv::imshow(rightWindowName, rightFrame);
        } else if (q_name == "trackedFeaturesLeft") {
            auto data = outputFeaturesLeftQueue->get<dai::TrackedFeatures>();
            l_features = data->trackedFeatures;
            l_seq = data->getSequenceNum();
            features_tp = data->getTimestamp();

            //leftFeatureDrawer.trackFeaturePath(l_features);
        } else if (q_name == "trackedFeaturesRight") {
            auto data = outputFeaturesRightQueue->get<dai::TrackedFeatures>();
            r_features = data->trackedFeatures;
            r_seq = data->getSequenceNum();

            rightFeatureDrawer.trackFeaturePath(r_features);
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
            l_prv_features = features;
            prv_features_tp = features_tp;
            r_prv_features.clear();
            for (const auto &r_feature : r_features) {
                r_prv_features[r_feature.id] = dai::Point2f(r_inv_k11 * r_feature.position.x + r_inv_k13, r_inv_k22 * r_feature.position.y + r_inv_k23);
            }
            //auto t2 = std::chrono::steady_clock::now();
            //std::cout << "cost " << std::chrono::duration<float, std::milli>(t2-t1).count() << " ms\n";
            leftFeatureDrawer.trackFeaturePath(draw_l_features);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 's') {
            if(featureTrackerConfig.motionEstimator.type == dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW) {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
                printf("Switching to hardware accelerated motion estimation \n");
            } else {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
                printf("Switching to Lucas-Kanade optical flow \n");
            }
            auto cfg = dai::FeatureTrackerConfig();
            cfg.set(featureTrackerConfig);
            inputFeatureTrackerConfigQueue->send(cfg);
        }
    }
    return 0;
}
