#include <iostream>
#include <thread>
#include <chrono>

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/un.h>
#include <signal.h>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

#define CAM_W 640
#define CAM_H 400
#define PAIR_DIST_SQ 9

struct MyPoint2d {
    double x = 0;
    double y = 0;
    MyPoint2d() {}
    MyPoint2d(double px, double py) {
        x = px;
        y = py;
    }
};

double big_buf[12*1024/sizeof(double)];
bool gogogo = true;

void sig_func(int sig) {
    gogogo = false;
}

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
    enum DEV_TYPE {OAK_D, OAK_D_PRO} dev_type;

    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = sig_func;
    sigaction(SIGINT, &act, NULL);

    struct sockaddr_un ipc_local_addr, imu_addr, features_addr;
    memset(&ipc_local_addr, 0, sizeof(struct sockaddr_un));
    ipc_local_addr.sun_family = AF_UNIX;
    strcpy(ipc_local_addr.sun_path, "/tmp/chobits_2222");
    int ipc_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    unlink("/tmp/chobits_2222");
    bind(ipc_sock, (struct sockaddr*)&ipc_local_addr, sizeof(ipc_local_addr));
    memset(&imu_addr, 0, sizeof(struct sockaddr_un));
    imu_addr.sun_family = AF_UNIX;
    strcpy(imu_addr.sun_path, "/tmp/chobits_imu");
    memset(&features_addr, 0, sizeof(struct sockaddr_un));
    features_addr.sun_family = AF_UNIX;
    strcpy(features_addr.sun_path, "/tmp/chobits_features");

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
    /*dai::RawFeatureTrackerConfig config = featureTrackerLeft->initialConfig.get();
    config.cornerDetector.numMaxFeatures = 100;
    featureTrackerLeft->initialConfig.set(config);
    config = featureTrackerRight->initialConfig.get();
    config.cornerDetector.numMaxFeatures = 100;
    featureTrackerRight->initialConfig.set(config);*/
    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    featureTrackerLeft->setHardwareResources(2, 2);
    featureTrackerRight->setHardwareResources(2, 2);

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

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    std::cout << "Usb speed: " << device.getUsbSpeed() << "\n";
    std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << "\n";
    if (device.getDeviceName() == "OAK-D") dev_type = OAK_D; else dev_type = OAK_D_PRO;

    //device.setLogOutputLevel(dai::LogLevel::DEBUG);
    //device.setLogLevel(dai::LogLevel::DEBUG);

    // Output queues used to receive the results
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 1, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 1, false);
    auto disp_queue = device.getOutputQueue("disparity", 1, false);
    auto imuQueue = device.getOutputQueue("imu", 5, false);

    int l_seq = -1, r_seq = -2, disp_seq = -3;
    std::vector<std::uint8_t> disp_frame;
    std::vector<dai::TrackedFeature> l_features, r_features;
    std::map<int, MyPoint2d> l_prv_features, r_prv_features;
    std::map<int, dai::Point2f> r_cur_features;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> features_tp, prv_features_tp;
    std::map<int, int> lr_id_mapping;

    // Clear queue events
    //jakaskerl suggest remove this line
    //https://discuss.luxonis.com/d/3484-getqueueevent-takes-much-additional-time/7
    //device.getQueueEvents();

    while(gogogo) {
        auto q_name = device.getQueueEvent();

        if (q_name == "trackedFeaturesLeft") {
            auto data = outputFeaturesLeftQueue->get<dai::TrackedFeatures>();
            l_features = data->trackedFeatures;
            l_seq = data->getSequenceNum();
            features_tp = data->getTimestamp();
            //std::cout << "l ft " << l_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - features_tp).count() << " ms\n";
        } else if (q_name == "trackedFeaturesRight") {
            auto data = outputFeaturesRightQueue->get<dai::TrackedFeatures>();
            r_features = data->trackedFeatures;
            r_seq = data->getSequenceNum();
            //std::cout << "r ft " << r_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - data->getTimestamp()).count() << " ms\n";
            r_cur_features.clear();
            for (const auto &feature : r_features) {
                r_cur_features[feature.id] = feature.position;
            }
        } else if (q_name == "disparity") {
            auto disp_data = disp_queue->get<dai::ImgFrame>();
            disp_seq = disp_data->getSequenceNum();
            disp_frame = disp_data->getData();
            //std::cout << "stereo " << disp_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - disp_data->getTimestamp()).count() << " ms\n";
        } else if (q_name == "imu") {
            auto imuData = imuQueue->get<dai::IMUData>();
            auto imuPackets = imuData->packets;
            for(const auto& imuPacket : imuPackets) {
                auto& acc = imuPacket.acceleroMeter;
                auto& gyro = imuPacket.gyroscope;
                //std::cout << "imu latency, acc:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - acc.getTimestamp()).count() << " ms, gyro:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - gyro.getTimestamp()).count() << " ms\n";
                big_buf[0] = std::chrono::duration<double>(acc.getTimestamp().time_since_epoch()).count();
                // translate to ros frame, easier to understand in rviz
                if (dev_type == OAK_D) {
                    big_buf[1] = acc.z;
                    big_buf[2] = acc.y;
                    big_buf[3] = -acc.x;
                    big_buf[4] = gyro.z;
                    big_buf[5] = gyro.y;
                    big_buf[6] = -gyro.x;
                } else {
                    big_buf[1] = -acc.z;
                    big_buf[2] = -acc.y;
                    big_buf[3] = -acc.x;
                    big_buf[4] = -gyro.z;
                    big_buf[5] = -gyro.y;
                    big_buf[6] = -gyro.x;
                }
                sendto(ipc_sock, big_buf, 7*sizeof(double), 0, (struct sockaddr*)&imu_addr, sizeof(struct sockaddr_un));
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
            std::map<int , MyPoint2d> features;
            int c = 0;
            big_buf[1] = std::chrono::duration<double>(features_tp.time_since_epoch()).count();
            double* buf_ptr = big_buf + 2;
            for (const auto &l_feature : l_features) {
                float x = l_feature.position.x;
                float y = l_feature.position.y;
                double cur_un_x = l_inv_k11 * x + l_inv_k13;
                double cur_un_y = l_inv_k22 * y + l_inv_k23;
                features[l_feature.id] = MyPoint2d(cur_un_x, cur_un_y);
                auto lr_id = lr_id_mapping.find(l_feature.id);
                if (lr_id != lr_id_mapping.end()) {
                    auto r_feature = r_cur_features.find(lr_id->second);
                    if (r_feature != r_cur_features.end()) {
                        double dt = std::chrono::duration<double>(features_tp - prv_features_tp).count();
                        double vx = 0, vy = 0;
                        auto prv_pos = l_prv_features.find(l_feature.id);
                        if (prv_pos != l_prv_features.end()) {
                            vx = (cur_un_x - prv_pos->second.x) / dt;
                            vy = (cur_un_y - prv_pos->second.y) / dt;
                        }
                        buf_ptr[0] = l_feature.id;
                        buf_ptr[1] = cur_un_x;
                        buf_ptr[2] = cur_un_y;
                        buf_ptr[3] = x;
                        buf_ptr[4] = y;
                        buf_ptr[5] = vx;
                        buf_ptr[6] = vy;

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
                        buf_ptr[7] = cur_un_x;
                        buf_ptr[8] = cur_un_y;
                        buf_ptr[9] = x;
                        buf_ptr[10] = y;
                        buf_ptr[11] = vx;
                        buf_ptr[12] = vy;

                        if (c < 118) {
                            ++c;
                            buf_ptr += 13;
                        }

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
                            double dt = std::chrono::duration<double>(features_tp - prv_features_tp).count();
                            double vx = 0, vy = 0;
                            auto prv_pos = l_prv_features.find(l_feature.id);
                            if (prv_pos != l_prv_features.end()) {
                                vx = (cur_un_x - prv_pos->second.x) / dt;
                                vy = (cur_un_y - prv_pos->second.y) / dt;
                            }
                            buf_ptr[0] = l_feature.id;
                            buf_ptr[1] = cur_un_x;
                            buf_ptr[2] = cur_un_y;
                            buf_ptr[3] = x;
                            buf_ptr[4] = y;
                            buf_ptr[5] = vx;
                            buf_ptr[6] = vy;

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
                            buf_ptr[7] = cur_un_x;
                            buf_ptr[8] = cur_un_y;
                            buf_ptr[9] = x;
                            buf_ptr[10] = y;
                            buf_ptr[11] = vx;
                            buf_ptr[12] = vy;

                            if (c < 118) {
                                ++c;
                                buf_ptr += 13;
                            }

                            break;
                        }
                    }
                }
            }
            ccc++;
            if (ccc > 60) {
                ccc = 0;
                std::cout << "latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - features_tp).count() << " ms, " << c << " features\n";
            }
            if (c > 0) {
                big_buf[0] = c;
                sendto(ipc_sock, big_buf, 13*sizeof(double)*c+2*sizeof(double), 0, (struct sockaddr*)&features_addr, sizeof(struct sockaddr_un));
            }
            l_prv_features = features;
            prv_features_tp = features_tp;
            r_prv_features.clear();
            for (const auto &r_feature : r_features) {
                r_prv_features[r_feature.id] = MyPoint2d(r_inv_k11 * r_feature.position.x + r_inv_k13, r_inv_k22 * r_feature.position.y + r_inv_k23);
            }
            //auto t2 = std::chrono::steady_clock::now();
            //std::cout << pp_msg.points.size() << " points, " << std::chrono::duration<float, std::milli>(t2-t1).count() << " ms\n";
        }
    }

    close(ipc_sock);
    printf("bye\n");

    return 0;
}
