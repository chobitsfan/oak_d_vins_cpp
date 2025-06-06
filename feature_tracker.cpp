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
#include <sys/ipc.h>
#include <sys/shm.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

using namespace std::chrono_literals;

#define MAX_FEATURES_COUNT 60
//#define H264_STREAMING
#define VIDEO_FPS 20
#define VIDEO_BITRATE 1500
#define DEPTH_SUBPIXEL

struct MyPoint2d {
    double x = 0;
    double y = 0;
    MyPoint2d() {}
    MyPoint2d(double px, double py) {
        x = px;
        y = py;
    }
};

double big_buf[14*MAX_FEATURES_COUNT+2];

sensor_msgs::msg::Image mono_img;
sensor_msgs::msg::Image disp_img;
bool mono_img_avail = false;
bool disp_img_avail = false;
bool img_pub_go = true;

void img_pub_func(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mono_img_pub, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_img_pub) {
    while (img_pub_go) {
        if (mono_img_avail) {
            mono_img_pub->publish(mono_img);
            mono_img_avail = false;
        }
        if (disp_img_avail) {
            disp_img_pub->publish(disp_img);
            disp_img_avail = false;
        }
        std::this_thread::sleep_for(10ms);
    }
}

void calc_rect_cam_intri(dai::CalibrationHandler calibData, double* f, double* cx, double* cy, int cam_w, int cam_h) {
    //std::cout << "stereo baseline:" << calibData.getBaselineDistance(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false) << " cm\n";
    /*auto imu_ext = calibData.getCameraToImuExtrinsics(dai::CameraBoardSocket::CAM_B, true);
    for (auto& row : imu_ext) {
        for (float val: row) {
            printf("%f ", val);
        }
        printf("\n");
    }*/

    auto l_intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_B, cam_w, cam_h);
    float data[9];
    int i = -1;
    for (auto row : l_intrinsics) {
        for (auto val : row) {
            data[++i] = val;
        }
    }
    cv::Mat l_m = cv::Mat(3, 3, CV_32FC1, data);

    auto r_intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::CAM_C, cam_w, cam_h);
    i = -1;
    for (auto row : r_intrinsics) {
        for (auto val : row) {
            data[++i] = val;
        }
    }
    cv::Mat r_m = cv::Mat(3, 3, CV_32FC1, data);

    auto l_d = calibData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_B);
    auto r_d = calibData.getDistortionCoefficients(dai::CameraBoardSocket::CAM_C);
    auto extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);

    cv::Mat r = (cv::Mat_<double>(3,3) << extrinsics[0][0], extrinsics[0][1], extrinsics[0][2], extrinsics[1][0], extrinsics[1][1], extrinsics[1][2], extrinsics[2][0], extrinsics[2][1], extrinsics[2][2]);
    cv::Mat t = (cv::Mat_<double>(3,1) << extrinsics[0][3], extrinsics[1][3], extrinsics[2][3]);
    //std::cout << "stereo extrinsics\n" << r << "\n" << t << "\n";
    cv::Mat r1, r2, p1, p2, q;
    cv::stereoRectify(l_m, l_d, r_m, r_d, cv::Size(cam_w, cam_h), r, t, r1, r2, p1, p2, q, cv::CALIB_ZERO_DISPARITY, 0);

    std::cout << "P1\n" << p1 << "\nP2\n" << p2 << "\n";

    *f = p1.at<double>(0, 0);
    *cx = p1.at<double>(0, 2);
    *cy = p1.at<double>(1, 2);
}

int main(int argc, char **argv) {
    bool h264_ok = false;
    int cam_w, cam_h;
    bool imu_ok = false;
    int ccc=0;
    int long_ms=0;
    int short_ms=INT_MAX;
    unsigned int mono_pub_c = 0;
    unsigned int disp_pub_c = 0;
    unsigned char seq_num = 0;

    if (argc < 2) {
        printf("usage: %s imu_tk_cali.yml\n", argv[0]);
        return 0;
    }

    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("feature_tracker");
    auto mono_img_pub = ros_node->create_publisher<sensor_msgs::msg::Image>("mono_left", rclcpp::QoS(1).best_effort().durability_volatile());
    auto disp_img_pub = ros_node->create_publisher<sensor_msgs::msg::Image>("disparity", rclcpp::QoS(1).best_effort().durability_volatile());

#ifdef REC_IMU
    FILE* imu_file = fopen("oakd_imu.bin", "w");
    FILE* features_file = fopen("oakd_features.bin", "w");
#endif

#ifdef H264_STREAMING
    // IPC shared memory
    key_t key = ftok("shmfile", 65);
    // shmget returns an identifier in shmid
    int shmid = shmget(key, 500000, 0666 | IPC_CREAT);
    unsigned char* h264_pkt_data = (unsigned char*)shmat(shmid, (void*)0, 0);
#endif

    cv::FileStorage imu_yml;
    imu_yml.open(argv[1], cv::FileStorage::READ);
    cv::Mat acc_mis_align, acc_scale, acc_bias;
    imu_yml["acc_misalign"] >> acc_mis_align;
    imu_yml["acc_scale"] >> acc_scale;
    imu_yml["acc_bias"] >> acc_bias;
    cv::Mat acc_cor = acc_mis_align * acc_scale;
    //std::cout<<acc_mis_align<<"\n"<<acc_scale<<"\n"<<acc_bias<<"\n";
    cv::Mat gyro_mis_align, gyro_scale, gyro_bias;
    imu_yml["gyro_misalign"] >> gyro_mis_align;
    imu_yml["gyro_scale"] >> gyro_scale;
    imu_yml["gyro_bias"] >> gyro_bias;
    cv::Mat gyro_cor = gyro_mis_align * gyro_scale;
    imu_yml.release();
    //std::cout<<gyro_mis_align<<"\n"<<gyro_scale<<"\n"<<gyro_bias<<"\n";

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
#ifdef H264_STREAMING
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
#endif
    auto manip = pipeline.create<dai::node::ImageManip>();

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();
    auto xout_mono = pipeline.create<dai::node::XLinkOut>();
#ifdef H264_STREAMING
    auto xout_h264 = pipeline.create<dai::node::XLinkOut>();
#endif

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xout_disp->setStreamName("disparity");
    xout_imu->setStreamName("imu");
    xout_mono->setStreamName("mono");
#ifdef H264_STREAMING
    xout_h264->setStreamName("h264");
#endif

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoLeft->setCamera("left");
    monoLeft->setFps(20);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_480_P);
    monoRight->setCamera("right");
    monoRight->setFps(20);

    manip->initialConfig.setCropRect(0.2, 0.2, 0.8, 0.8);

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
    depth->initialConfig.setMedianFilter(dai::MedianFilter::MEDIAN_OFF);
    //depth->initialConfig.setConfidenceThreshold(0); // maximum confidence that it holds a valid value
    depth->setLeftRightCheck(true);
    depth->setExtendedDisparity(false);
#ifdef DEPTH_SUBPIXEL
    depth->setSubpixel(true);
    depth->setSubpixelFractionalBits(3);
#else
    depth->setSubpixel(false);
#endif
    depth->setDepthAlign(dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
    depth->setAlphaScaling(0);

    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

#ifdef H264_STREAMING
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(VIDEO_FPS);
    camRgb->setNumFramesPool(2, 2, 2, 2, 2);
    videoEnc->setDefaultProfilePreset(VIDEO_FPS, dai::VideoEncoderProperties::Profile::H264_MAIN);
    videoEnc->setKeyframeFrequency(VIDEO_FPS*2);
    videoEnc->setBitrateKbps(VIDEO_BITRATE);
    videoEnc->setNumFramesPool(2);
    videoEnc->input.setQueueSize(2);
    videoEnc->input.setBlocking(false);
#endif

    // Linking
    monoLeft->out.link(depth->left);
    depth->rectifiedLeft.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    monoRight->out.link(depth->right);
    depth->rectifiedRight.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    depth->disparity.link(xout_disp->input);
    imu->out.link(xout_imu->input);
    monoLeft->out.link(manip->inputImage);
    manip->out.link(xout_mono->input);
#ifdef H264_STREAMING
    //monoLeft->out.link(videoEnc->input);
    camRgb->video.link(videoEnc->input);
    videoEnc->bitstream.link(xout_h264->input);
#endif

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    std::cout << "Usb speed: " << device.getUsbSpeed() << "\n";
    std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << "\n";
    if (device.getDeviceName() != "OAK-D-LITE") printf("not OAK-D-LITE\n");

    cam_w = monoLeft->getResolutionWidth();
    cam_h = monoLeft->getResolutionHeight();
    printf("stereo res %dx%d\n", cam_w, cam_h);

    dai::CalibrationHandler calibData = device.readCalibration2();
    double f, cx, cy;
    float baseline = calibData.getBaselineDistance(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false) * 0.01f;
    calc_rect_cam_intri(calibData, &f, &cx, &cy, cam_w, cam_h);
    float hfov = 2 * atanf(cam_w / (2 * f));
    float vfov = 2 * atanf(cam_h / (2 * f));
    std::cout << "stereo baseline:" << baseline << " m, f:" << f << " px, cx:" << cx << ", cy:" << cy << " hfov:" << hfov * 180 / M_PI << " degrees, vfov:" << vfov * 180 / M_PI << " degrees\n";
    double l_inv_k11 = 1.0 / f;
    double l_inv_k13 = -cx / f;
    double l_inv_k22 = 1.0 / f;
    double l_inv_k23 = -cy / f;
    double r_inv_k11 = 1.0 / f;
    double r_inv_k13 = -cx / f;
    double r_inv_k22 = 1.0 / f;
    double r_inv_k23 = -cy / f;

    /*auto s_pairs = device.getAvailableStereoPairs();
    for (auto& s_pair : s_pairs) {
        std::cout << "stereo pair baseline:" << s_pair.baseline << " cm\n";
    }*/

    //device.setLogOutputLevel(dai::LogLevel::DEBUG);
    //device.setLogLevel(dai::LogLevel::DEBUG);

    // Output queues used to receive the results
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 1, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 1, false);
    auto disp_queue = device.getOutputQueue("disparity", 1, false);
    auto imuQueue = device.getOutputQueue("imu", 10, false);
    auto mono_queue = device.getOutputQueue("mono", 1, false);
#ifdef H264_STREAMING
    auto video = device.getOutputQueue("h264", 1, false);
#endif

    int l_seq = -1, r_seq = -2, disp_seq = -3;
#ifdef DEPTH_SUBPIXEL
    uint16_t* disp_data;
#else
    uint8_t* disp_data;
#endif
    std::vector<dai::TrackedFeature> l_features, r_features;
    std::map<int, MyPoint2d> l_prv_features, r_prv_features;
    double features_ts, prv_features_ts;
    double latest_exp_t = 0;
    //double last_acc_t = 0;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> l_ft_tp;

    // Clear queue events
    //jakaskerl suggest remove this line
    //https://discuss.luxonis.com/d/3484-getqueueevent-takes-much-additional-time/7
    //device.getQueueEvents();

    std::thread img_pub_worker(img_pub_func, mono_img_pub, disp_img_pub);

    while(rclcpp::ok()) {
        auto q_name = device.getQueueEvent();

        if (q_name == "trackedFeaturesLeft") {
            auto data = outputFeaturesLeftQueue->get<dai::TrackedFeatures>();
            l_features = data->trackedFeatures;
            l_seq = data->getSequenceNum();
            features_ts = std::chrono::duration<double>(data->getTimestampDevice().time_since_epoch()).count();
            l_ft_tp = data->getTimestamp();
            //std::cout << "l ft " << l_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - features_tp).count() << " ms\n";
        } else if (q_name == "trackedFeaturesRight") {
            auto data = outputFeaturesRightQueue->get<dai::TrackedFeatures>();
            r_features = data->trackedFeatures;
            r_seq = data->getSequenceNum();
            //std::cout << "r ft " << r_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - data->getTimestamp()).count() << " ms\n";
        } else if (q_name == "disparity") {
            auto disp_frame = disp_queue->get<dai::ImgFrame>();
            disp_seq = disp_frame->getSequenceNum();
            auto disp_frame_data = disp_frame->getData();
#ifdef DEPTH_SUBPIXEL
            disp_data = (uint16_t*)disp_frame_data.data();
#else
            disp_data = (uint8_t*)disp_frame_data.data();
#endif
            latest_exp_t = std::chrono::duration<double>(disp_frame->getExposureTime()).count();
            //std::cout << "stereo " << disp_seq << " latency:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - disp_data->getTimestamp()).count() << " ms\n";
            disp_pub_c++;
            if (disp_pub_c > 3) {
                disp_pub_c = 0;
                disp_img.header.stamp = ros_node->get_clock()->now();
                disp_img.height = disp_frame->getHeight();
                disp_img.width = disp_frame->getWidth();
                disp_img.is_bigendian = 0;
#ifdef DEPTH_SUBPIXEL
                disp_img.encoding = "mono16";
                disp_img.step = disp_img.width*2;
#else
                disp_img.encoding = "mono8";
                disp_img.step = disp_img.width;
#endif
                disp_img.data = disp_frame_data;
                disp_img_avail = true;
            }
        } else if (q_name == "imu") {
            auto imuData = imuQueue->get<dai::IMUData>();
            auto imuPackets = imuData->packets;
            for(const auto& imuPacket : imuPackets) {
                auto& acc = imuPacket.acceleroMeter;
                auto& gyro = imuPacket.gyroscope;
                //std::cout << "imu latency, acc:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - acc.getTimestamp()).count() << " ms, gyro:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - gyro.getTimestamp()).count() << " ms\n";
                big_buf[0] = std::chrono::duration<double>(acc.getTimestampDevice().time_since_epoch()).count();
                //if (big_buf[0] - last_acc_t > 0.007) printf("imu jitter %f\n", big_buf[0] - last_acc_t);
                //last_acc_t = big_buf[0];
                cv::Mat acc_raw = (cv::Mat_<double>(3,1) << acc.x, acc.y, acc.z);
                cv::Mat1d acc_cali = acc_cor * (acc_raw - acc_bias);
                cv::Mat gyro_raw = (cv::Mat_<double>(3,1) << gyro.x, gyro.y, gyro.z);
                cv::Mat1d gyro_cali = gyro_cor * (gyro_raw - gyro_bias);
                // translate to ros frame, easier to understand in rviz
                big_buf[1] = -acc_cali(2,0);
                big_buf[2] = -acc_cali(0,0);
                big_buf[3] = acc_cali(1,0);
                big_buf[4] = -gyro_cali(2,0);
                big_buf[5] = -gyro_cali(0,0);
                big_buf[6] = gyro_cali(1,0);
                sendto(ipc_sock, big_buf, 7*sizeof(double), 0, (struct sockaddr*)&imu_addr, sizeof(struct sockaddr_un));
            }
            if (!imu_ok) {
                imu_ok = true;
                std::cout<< "imu ok\n";
            }
        } else if (q_name == "h264") {
#ifdef H264_STREAMING
            if (!h264_ok) {
                h264_ok = true;
                std::cout<<"h264 ok\n";
            }
            auto h264Packet = video->get<dai::ImgFrame>();
            auto h264data = h264Packet->getData();
            //
            // IPC data structure: length+data+seq_num+flag
            // length - size of h264 bitstream (4 bytes)
            // data - h264 bitstream (n bytes)
            // seq_num - 1 byte
            // flag - 1 byte
            //
            int h264_pkt_len = h264data.size();
			//printf("h264enc len=%d\n", h264Packet->getData().size());
			h264_pkt_data[0] = (h264_pkt_len >> 24) & 0xff;
			h264_pkt_data[1] = (h264_pkt_len >> 16) & 0xff;
			h264_pkt_data[2] = (h264_pkt_len >> 8) & 0xff;
			h264_pkt_data[3] = h264_pkt_len & 0xff;
			memcpy(h264_pkt_data+4, h264data.data(), h264_pkt_len);
            h264_pkt_data[h264_pkt_len+4] = seq_num;
            seq_num++;
            if(seq_num == 0xff) seq_num = 0;
            h264_pkt_data[h264_pkt_len+5] = 1;
#endif
        } else if (q_name == "mono") {
            auto img_frame = mono_queue->get<dai::ImgFrame>();
            mono_pub_c++;
            if (mono_pub_c > 3) {
                mono_pub_c = 0;
                mono_img.header.stamp = ros_node->get_clock()->now();
                mono_img.height = img_frame->getHeight();
                mono_img.width = img_frame->getWidth();
                mono_img.is_bigendian = 0;
                mono_img.encoding = "mono8";
                mono_img.step = mono_img.width;
                mono_img.data = img_frame->getData();
                mono_img_avail = true;
            }
        }

        if (l_seq == r_seq && r_seq == disp_seq) {
            //auto t1 = std::chrono::steady_clock::now();
            l_seq = -1;
            r_seq = -2;
            disp_seq = -3;
            std::map<int , MyPoint2d> features;
            int c = 0;
            features_ts = features_ts - latest_exp_t * 0.5;
            big_buf[1] = features_ts;
            double* buf_ptr = big_buf + 2;
            for (const auto &l_feature : l_features) {
                float x = l_feature.position.x;
                float y = l_feature.position.y;
                double cur_un_x = l_inv_k11 * x + l_inv_k13;
                double cur_un_y = l_inv_k22 * y + l_inv_k23;
                features[l_feature.id] = MyPoint2d(cur_un_x, cur_un_y);
                int row = y;
                int col = x;
                int ceil_row = ceilf(y);
                int ceil_col = ceilf(x);
#ifdef DEPTH_SUBPIXEL
                float disps[4] = {0};
                float disp;
                disps[0] = disp_data[row * cam_w + col] / 8.0f;
                if (ceil_row != (int)y && ceil_row < cam_h) disps[1] = disp_data[ceil_row * cam_w + col] / 8.0f;
                if (ceil_col != (int)x && ceil_col < cam_w) disps[2] = disp_data[row * cam_w + ceil_col] / 8.0f;
                if (disps[1] && disps[2]) {
                    disps[3] = disp_data[ceil_row * cam_w + ceil_col] / 8.0f;
                }
#else
                int disps[4] = {0};
                int disp;
                disps[0] = disp_data[row * cam_w + col];
                if (ceil_row != (int)y && ceil_row < cam_h) disps[1] = disp_data[ceil_row * cam_w + col];
                if (ceil_col != (int)x && ceil_col < cam_w) disps[2] = disp_data[row * cam_w + ceil_col];
                if (disps[1] && disps[2]) {
                    disps[3] = disp_data[ceil_row * cam_w + ceil_col];
                }
#endif
                for (int i = 0; i < 4; i++) {
                    disp = disps[i];
                    if (disp > 0) {
                        bool pair_found = false;
                        for (const auto &r_feature : r_features) {
                            float dy = y - r_feature.position.y;
                            float dx = x - disp - r_feature.position.x;
                            if (fabsf(dy) <= 1 && fabsf(dx) <= 2) { //pair found
                                pair_found = true;
                                double dt = features_ts - prv_features_ts;
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
                                buf_ptr[13] = f * baseline / disp;

                                if (c < MAX_FEATURES_COUNT) {
                                    ++c;
                                    buf_ptr += 14;
                                }

                                break;
                            }
                        }
                        if (pair_found) break;
                    }
                }
            }
            int cost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - l_ft_tp).count();
            if (cost_ms > long_ms) long_ms = cost_ms;
            else if (cost_ms < short_ms) short_ms = cost_ms;
            ccc++;
            if (ccc > 60) {
                ccc = 0;
                std::cout << l_features.size() << " features " << c << " LR matched, latency(ms) max " << long_ms << ", min " << short_ms  << "\n";
                long_ms = 0;
                short_ms = INT_MAX;
                //latency ~ 40 ms
            }
            //if (c < 10) printf("too few features: %d\n", c);
            if (imu_ok && c > 0) {
                big_buf[0] = c;
                sendto(ipc_sock, big_buf, 14*sizeof(double)*c+2*sizeof(double), 0, (struct sockaddr*)&features_addr, sizeof(struct sockaddr_un));
            }
            l_prv_features = features;
            prv_features_ts = features_ts;
            r_prv_features.clear();
            for (const auto &r_feature : r_features) {
                r_prv_features[r_feature.id] = MyPoint2d(r_inv_k11 * r_feature.position.x + r_inv_k13, r_inv_k22 * r_feature.position.y + r_inv_k23);
            }
            //auto t2 = std::chrono::steady_clock::now();
            //std::cout << std::chrono::duration<float, std::milli>(t2-t1).count() << " ms\n";
        }
    }

    close(ipc_sock);

#ifdef REC_IMU
    fclose(imu_file);
    fclose(features_file);
#endif

#ifdef H264_STREAMING
    shmdt(h264_pkt_data);
    shmctl(shmid, IPC_RMID, NULL);
#endif

    img_pub_go = false;
    img_pub_worker.join();

    rclcpp::shutdown();
    printf("bye\n");

    return 0;
}
