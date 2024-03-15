#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

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
#include <errno.h>

#include <opencv2/barcode.hpp>
#include <opencv2/imgcodecs.hpp>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

#define CAM_W 640
#define CAM_H 400
#define PAIR_DIST_SQ 9
#define MJPG_PORT 8080
#define BUF_SZ 2048
#define HTML_HEADER "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"
#define MJPG_HEADER "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=jpgboundary\r\n\r\n"
#define PLAIN_HEADER "HTTP/1.0 200 OK\r\nContent-Type: text/plain\r\n\r\n"

struct MyPoint2d {
    double x = 0;
    double y = 0;
    MyPoint2d() {}
    MyPoint2d(double px, double py) {
        x = px;
        y = py;
    }
};

int mjpg_sockfd = -1;
cv::Ptr<cv::barcode::BarcodeDetector> bardet;
std::vector<cv::Point2f> bar_corners;
bool bar_found = false;
double big_buf[12*1024/sizeof(double)];
bool gogogo = true;
std::string barcode_txt = "barcode";
bool need_decode = false;

void wait_mjpg_conn() {
    char buf[BUF_SZ];

    /*sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_UNBLOCK, &signal_set, NULL);*/

    // Create a socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        perror("webserver (socket)");
        return;
    }
    printf("socket created successfully\n");

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));

    // Create the address to bind the socket to
    struct sockaddr_in host_addr;
    int host_addrlen = sizeof(host_addr);
    memset(&host_addr, 0, sizeof(host_addr));
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(MJPG_PORT);
    host_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind the socket to the address
    if (bind(sockfd, (struct sockaddr *)&host_addr, host_addrlen) != 0) {
        perror("webserver (bind)");
        return;
    }
    printf("socket successfully bound to address\n");

    // Listen for incoming connections
    if (listen(sockfd, SOMAXCONN) != 0) {
        perror("webserver (listen)");
        return;
    }
    printf("mjpg server listening for connections\n");

    while (true) {
        int new_sockfd = accept(sockfd, (struct sockaddr *)&host_addr, (socklen_t *)&host_addrlen);
        if (new_sockfd >= 0) {
            int val = read(new_sockfd, buf, BUF_SZ);
            if (val > 0) {
                //rcv_buf[val]=0;
                //printf("rcv:%s\n", rcv_buf);
                if (strncmp(buf, "GET / ", 6) == 0) {
                    int html_fd = open("index.html", O_RDONLY);
                    if (html_fd >= 0) {
                        printf("web page requested\n");
                        val = read(html_fd, buf, BUF_SZ);
                        if (val > 0) {
                            send(new_sockfd, HTML_HEADER, sizeof(HTML_HEADER)-1, MSG_NOSIGNAL);
                            send(new_sockfd, buf, val, MSG_NOSIGNAL);
                        }
                        shutdown(new_sockfd, SHUT_RDWR);
                        close(new_sockfd);
                        close(html_fd);
                    }
                } else if (strncmp(buf, "GET /cam ", 9) == 0) {
                    printf("cam img requested\n");
                    write(new_sockfd, MJPG_HEADER, sizeof(MJPG_HEADER)-1);
                    mjpg_sockfd = new_sockfd;
                } else if (strncmp(buf, "GET /bar ", 9) == 0) {
                    //printf("barcode requested\n");
                    send(new_sockfd, PLAIN_HEADER, sizeof(PLAIN_HEADER)-1, MSG_NOSIGNAL);
                    if (bar_found) {
                        /*std::stringstream ss;
                        for (int i = 0; i < bar_corners.size(); ++i) {
                            ss << bar_corners[i].x << "," << bar_corners[i].y << ",";
                        }
                        ss << barcode_txt;
                        send(new_sockfd, ss.str().c_str(), ss.str().size(), MSG_NOSIGNAL);*/
                        int sz = snprintf(buf, BUF_SZ, "%d,%d,%d,%d,%d,%d,%d,%d,%s", (int)bar_corners[0].x, (int)bar_corners[0].y, (int)bar_corners[1].x, (int)bar_corners[1].y, 
                            (int)bar_corners[2].x, (int)bar_corners[2].y, (int)bar_corners[3].x, (int)bar_corners[3].y, barcode_txt.c_str());
                        send(new_sockfd, buf, sz, MSG_NOSIGNAL);
                    } else {
                        send(new_sockfd, "no barcode", 10, MSG_NOSIGNAL);
                    }
                    shutdown(new_sockfd, SHUT_RDWR);
                    close(new_sockfd);
                }
            }
        } else {
            if (errno == EINTR) break;
        }
    }
    printf("wait_mjpg_conn finished\n");
    close(sockfd);
}

void new_mjpg_frame(std::shared_ptr<dai::ADatatype> msg) {
    if (mjpg_sockfd >= 0) {
        dai::ImgFrame* mjpeg = static_cast<dai::ImgFrame*>(msg.get());
        auto mjpeg_data = mjpeg->getData();
        char http_rsp[128];
        int sz = sprintf(http_rsp, "\r\n--jpgboundary\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", mjpeg_data.size());
        if (send(mjpg_sockfd, http_rsp, sz, MSG_NOSIGNAL) < 0) {
            perror("socket write");
            mjpg_sockfd = -1;
            return;
        }
        if (send(mjpg_sockfd, mjpeg_data.data(), mjpeg_data.size(), MSG_NOSIGNAL) < 0) {
            perror("socket write");
            mjpg_sockfd = -1;
            return;
        }
    }
}

void new_img(std::shared_ptr<dai::ADatatype> msg) {
    static int nn = 0;
    static int cc = 0;
    if (++cc > 10) {
        cc = 0;
        dai::ImgFrame* img = static_cast<dai::ImgFrame*>(msg.get());
        auto frame = img->getFrame();
        bar_found = bardet->detect(frame, bar_corners);
        if (bar_found) {
            need_decode = true;
            /*std::vector<std::string> barcodes;
            std::vector<cv::barcode::BarcodeType> bartypes;
            bardet->decode(frame, bar_corners, barcodes, bartypes);
            if (barcodes.empty() || barcodes[0].empty()) {
                barcode_txt = "barcode";
            } else {
                barcode_txt = barcodes[0];
            }*/
            //char f_buf[256];
            //sprintf(f_buf, "%d_%s.png", ++nn, barcode_txt.c_str());
            //cv::imwrite(f_buf, frame);
            /*for (const auto& barcode : barcodes) {
                if (!barcode.empty()) {
                    std::cout << "barcode:" << barcode << "\n";
                    barcode_txt = barcode;
                }
            }*/
        }
    }
}

void new_big_img(std::shared_ptr<dai::ADatatype> msg) {
    static int nn = 0;
    if (need_decode) {
        need_decode = false;
        dai::ImgFrame* img = static_cast<dai::ImgFrame*>(msg.get());
        auto frame = img->getFrame();
        //auto croped = frame(cv::Rect(bar_corners[1].x*3, bar_corners[1].y*3, (bar_corners[2].x - bar_corners[1].x)*3, (bar_corners[0].y - bar_corners[1].y)*3));
        //cv::imwrite("croped.png", croped);
        //cv::imwrite("full.png", frame);
        std::vector<cv::Point2f> bar_corners_b;
        std::vector<std::string> barcodes;
        std::vector<cv::barcode::BarcodeType> bartypes;
        for (const auto& bar_corner : bar_corners) {
            bar_corners_b.emplace_back(bar_corner.x*3, bar_corner.y*3);
        }
        bardet->decode(frame, bar_corners_b, barcodes, bartypes);
        if (barcodes.empty() || barcodes[0].empty()) {
            barcode_txt = "barcode";
            char f_buf[256];
            sprintf(f_buf, "%d_barcode.png", ++nn);
            cv::imwrite(f_buf, frame);
        } else {
            barcode_txt = barcodes[0];
        }
    }
}

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

    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = sig_func;
    sigaction(SIGINT, &act, NULL);
    /*sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, NULL);*/

    std::thread conn_t(wait_mjpg_conn);

    bardet = cv::makePtr<cv::barcode::BarcodeDetector>();

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
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videnc = pipeline.create<dai::node::VideoEncoder>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto manip_b = pipeline.create<dai::node::ImageManip>();

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();
    auto xout_imu = pipeline.create<dai::node::XLinkOut>();
    auto xout_mjpg = pipeline.create<dai::node::XLinkOut>();
    //auto xout_img = pipeline.create<dai::node::XLinkOut>();
    auto xout_manip = pipeline.create<dai::node::XLinkOut>();
    auto xout_manip_b = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xout_disp->setStreamName("disparity");
    xout_imu->setStreamName("imu");
    xout_mjpg->setStreamName("mjpeg");
    //xout_img->setStreamName("img");
    xout_manip->setStreamName("manip");
    xout_manip_b->setStreamName("manip_b");

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

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(10);
    camRgb->setIsp3aFps(5);
    //camRgb->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF);
    //camRgb->initialControl.setManualFocus(125);
    //camRgb->setPreviewSize(640,360);
    //camRgb->setVideoSize(640,360);
    //camRgb->setIspScale(1, 2);
    manip->initialConfig.setFrameType(dai::RawImgFrame::Type::RAW8);
    manip->initialConfig.setResize(640,360);
    manip->setMaxOutputFrameSize(640*360);
    manip_b->initialConfig.setFrameType(dai::RawImgFrame::Type::RAW8);
    manip_b->setMaxOutputFrameSize(1920*1080);

    videnc->setDefaultProfilePreset(10, dai::VideoEncoderProperties::Profile::MJPEG);
    videnc->setQuality(20);

    // Linking
    monoLeft->out.link(depth->left);
    depth->rectifiedLeft.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    monoRight->out.link(depth->right);
    depth->rectifiedRight.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    depth->disparity.link(xout_disp->input);
    imu->out.link(xout_imu->input);

    //camRgb->video.link(videnc->input);
    videnc->bitstream.link(xout_mjpg->input);

    //monoLeft->out.link(xout_img->input);

    camRgb->isp.link(manip->inputImage);
    camRgb->isp.link(manip_b->inputImage);
    manip->out.link(videnc->input);
    manip->out.link(xout_manip->input);
    manip_b->out.link(xout_manip_b->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    std::cout << "Usb speed: " << device.getUsbSpeed() << "\n";
    std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << "\n";

    //device.setLogOutputLevel(dai::LogLevel::DEBUG);
    //device.setLogLevel(dai::LogLevel::DEBUG);

    // Output queues used to receive the results
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 1, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 1, false);
    auto disp_queue = device.getOutputQueue("disparity", 1, false);
    auto imuQueue = device.getOutputQueue("imu", 5, false);
    device.getOutputQueue("mjpeg", 1, false)->addCallback(new_mjpg_frame);
    //device.getOutputQueue("img", 1, false)->addCallback(new_img);
    device.getOutputQueue("manip", 1, false)->addCallback(new_img);
    device.getOutputQueue("manip_b", 1, false)->addCallback(new_big_img);

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
                big_buf[1] = acc.z;
                big_buf[2] = acc.y;
                big_buf[3] = -acc.x;
                big_buf[4] = gyro.z;
                big_buf[5] = gyro.y;
                big_buf[6] = -gyro.x;
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
    pthread_kill(conn_t.native_handle(), SIGINT);
    printf("join thread\n");
    conn_t.join();
    printf("bye\n");

    return 0;
}
