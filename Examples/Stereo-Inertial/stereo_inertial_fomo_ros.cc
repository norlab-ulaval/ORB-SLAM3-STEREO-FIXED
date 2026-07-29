/**
 * This file is part of ORB-SLAM3
 *
 * Modified for ROS 2 rosbag reading.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <vector>
#include <thread>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

// ROS 2 dependencies
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std;

struct TrajectoryPoint {
    double t;
    Eigen::Vector3f twb;
    Eigen::Quaternionf q;
};

// Subtract `offset` from the first (whitespace-separated) token on every non-empty line
// of `path`.  `scale` converts the offset from seconds to whatever unit the file uses
// (1.0 for TUM/seconds, 1e9 for EuRoC/nanoseconds).
void CorrectTimestamps(const string &path, double offset, double scale = 1.0)
{
    ifstream fin(path);
    if(!fin.is_open()) { cerr << "CorrectTimestamps: cannot open " << path << endl; return; }
    ostringstream buf;
    string line;
    while(getline(fin, line))
    {
        if(line.empty()) { buf << "\n"; continue; }
        istringstream iss(line);
        double ts;
        if(!(iss >> ts)) { buf << line << "\n"; continue; } // pass comment/header lines through
        string rest;
        getline(iss, rest);
        buf << fixed << setprecision(6) << (ts - offset * scale) << rest << "\n";
    }
    fin.close();
    ofstream fout(path, ios::trunc);
    fout << buf.str();
}

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_inertial_fomo_ros path_to_vocabulary path_to_settings path_to_rosbag output_trajectory_path [time_offset_seconds] [--realtime] [--no-imu]" << endl;
        return 1;
    }

    // Scan argv for optional flags (position-independent).
    bool bRealTime = false;
    bool bUseImu = true;
    std::string sTimeOffset = "";
    for(int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if(arg == "--realtime")      bRealTime = true;
        else if(arg == "--no-imu")   bUseImu = false;
        else if(i >= 5 && arg.substr(0, 2) != "--") sTimeOffset = arg;
    }

    if (bRealTime) {
        cout << "Running realtime\n";
    }

    string pathBag(argv[3]);
    string strOutName(argv[4]);

    cout << "Opening rosbag: " << pathBag << endl;
    
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = pathBag;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        cerr << "Failed to open rosbag: " << e.what() << endl;
        return 1;
    }

    rosbag2_storage::StorageFilter filter;
    if (bUseImu) {
        filter.topics.push_back("/vectornav/data_raw");
    }
    filter.topics.push_back("/zedx/left/image_rect");
    filter.topics.push_back("/zedx/right/image_rect");
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;

    // Create SLAM system first so we can read the atlas timestamps before deciding the offset.
    ORB_SLAM3::System::eSensor sensorType = bUseImu ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO;
    ORB_SLAM3::System SLAM(argv[1],argv[2],sensorType, false); // last param is visualization

    const double kAtlasMarginSec = 5.0; // seconds before the first atlas KF
    double time_offset = 0.0;
    bool time_offset_computed = false;
    double first_cam_ts = -1.0;

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    cv::Mat cvTbc;
    fSettings["Tbc"] >> cvTbc;
    if(cvTbc.type() != CV_32F)
        cvTbc.convertTo(cvTbc, CV_32F);
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    std::ofstream odomFile(strOutName + "_bak");
    odomFile << fixed;

    cout << endl << "-------" << endl;
    cout << "Start processing rosbag sequence ..." << endl;

    if(bRealTime) {
        std::cout << "[RealTime] Frame-drop mode ENABLED. "
                  << "Will start in offline mode until first localization.\n"
                  << "Budget: 100.0ms/frame @ 10.0 Hz\n";
    }

    cv::Mat imLeft, imRight;
    double tLeft = -1.0, tRight = -1.0;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    bool bHasLocalized = false;
    bool bImuExcited = false;
    double tLost = -1.0;

    std::vector<std::vector<TrajectoryPoint>> trajectory_segments;
    std::vector<TrajectoryPoint> current_segment;

    struct TimingLog {
        int    frame;
        double slam_ms;  // TrackStereo wall time
        double full_ms;  // total frame wall time
    };
    std::vector<TimingLog> timingLogs;

    std::chrono::steady_clock::time_point rt_wall_start;
    double rt_cam_start_ts = 0.0;
    int  frames_dropped   = 0;
    int  ni               = 0;

    while (reader.has_next()) {
        auto msg = reader.read_next();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        if (msg->topic_name == "/vectornav/data_raw") {
            sensor_msgs::msg::Imu imu_msg;
            imu_serialization.deserialize_message(&serialized_msg, &imu_msg);
            
            double t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
            if (time_offset_computed) t += time_offset;
            
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                t
            ));
        } else if (msg->topic_name == "/zedx/left/image_rect" || msg->topic_name == "/zedx/right/image_rect") {
            sensor_msgs::msg::Image img_msg;
            image_serialization.deserialize_message(&serialized_msg, &img_msg);
            
            double t = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;

            if (!time_offset_computed) {
                first_cam_ts = t;
                
                ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
                double atlas_min_ts = std::numeric_limits<double>::max();
                if(pAtlas) {
                    for(ORB_SLAM3::Map* pMap : pAtlas->GetAllMaps())
                        for(ORB_SLAM3::KeyFrame* pKF : pMap->GetAllKeyFrames())
                            if(pKF && !pKF->isBad())
                                atlas_min_ts = std::min(atlas_min_ts, pKF->mTimeStamp);
                }

                if(atlas_min_ts < std::numeric_limits<double>::max()) {
                    double auto_offset = atlas_min_ts - kAtlasMarginSec - first_cam_ts;
                    if(!sTimeOffset.empty()) {
                        double cli_offset = stod(sTimeOffset);
                        double diff = std::abs(cli_offset - auto_offset);
                        if(diff > 10.0)
                            cerr << "WARNING: CLI time_offset (" << cli_offset
                                 << " s) differs from auto-computed offset (" << auto_offset
                                 << " s) by " << diff << " s. Using CLI override." << endl;
                        time_offset = cli_offset;
                    } else {
                        time_offset = auto_offset;
                    }
                    cout << "Atlas earliest KF timestamp: " << fixed << setprecision(3) << atlas_min_ts << " s" << endl;
                    cout << "Dataset first cam timestamp: " << first_cam_ts << " s" << endl;
                    cout << "Applying time offset:        " << time_offset << " s" << endl;
                } else if(!sTimeOffset.empty()) {
                    time_offset = stod(sTimeOffset);
                    cout << "No atlas loaded. Applying CLI time offset: " << time_offset << " s" << endl;
                }
                
                time_offset_computed = true;
                
                for (auto& imu : vImuMeas) {
                    imu.t += time_offset;
                }
            }

            t += time_offset;

            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img_msg, ""); // Copy with original encoding
            } catch (cv_bridge::Exception& e) {
                cerr << "cv_bridge exception: " << e.what() << endl;
                continue;
            }

            if (msg->topic_name == "/zedx/left/image_rect") {
                imLeft = cv_ptr->image.clone();
                tLeft = t;
            } else {
                imRight = cv_ptr->image.clone();
                tRight = t;
            }

            // Check if we have a synchronized pair
            if (!imLeft.empty() && !imRight.empty()) {
                if (std::abs(tLeft - tRight) < 0.005) {
                    double tframe = tLeft;
                    auto t_frame_start = std::chrono::steady_clock::now();

                    if(bRealTime && bHasLocalized) {
                        double t_cam_elapsed  = tframe - rt_cam_start_ts;
                        double t_wall_elapsed = std::chrono::duration<double>(t_frame_start - rt_wall_start).count();
                        double frame_period   = 0.1; // 10 Hz from fomo.yaml
                        double behind_ms = (t_wall_elapsed - t_cam_elapsed) * 1000.0;

                        if(behind_ms > 0.5 * frame_period * 1000.0) {
                            frames_dropped++;
                            std::cout << std::fixed << std::setprecision(1)
                                      << "Frame " << ni << ": SKIPPED   ("
                                      << behind_ms << "ms behind schedule)\n";
                            imLeft.release();
                            imRight.release();
                            ni++;
                            continue;
                        }
                    }

                    if(bUseImu && !bImuExcited) {
                        for(size_t i=0; i<vImuMeas.size(); i++) {
                            if (vImuMeas[i].w.norm() > 0.1 || std::abs(vImuMeas[i].a.norm() - 9.81) > 0.5) {
                                bImuExcited = true;
                                cout << "IMU Excitation detected at frame " << ni << ". Starting processing." << endl;
                                break;
                            }
                        }
                    }

                    auto t_slam_start = std::chrono::steady_clock::now();
                    size_t num_imu = vImuMeas.size();
                    Sophus::SE3f Tcw;
                    if (bUseImu) {
                        Tcw = SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);
                    } else {
                        Tcw = SLAM.TrackStereo(imLeft, imRight, tframe);
                    }
                    auto t_slam_end = std::chrono::steady_clock::now();
                    double slam_ms = std::chrono::duration<double,std::milli>(t_slam_end - t_slam_start).count();

                    // Clear IMU measurements since they were passed to TrackStereo
                    vImuMeas.clear();

                    int trackingState = SLAM.GetTrackingState();
                    if(trackingState == 2 || trackingState == 5) // OK=2, OK_KLT=5
                    {
                        if (tLost >= 0 && !current_segment.empty()) {
                            trajectory_segments.push_back(current_segment);
                            current_segment.clear();
                        }
                        if (!bHasLocalized) {
                            bHasLocalized = true;
                            if (bRealTime) {
                                std::cout << "First localization event triggered at frame " << ni << ". Switching to realtime mode.\n";
                                rt_cam_start_ts = tframe;
                                rt_wall_start = std::chrono::steady_clock::now();
                            }
                        }
                        tLost = -1.0; 
                        Sophus::SE3f Twb = (Tbc * Tcw).inverse();
                        TrajectoryPoint pt;
                        pt.t = tframe;
                        pt.twb = Twb.translation();
                        pt.q = Twb.unit_quaternion();
                        current_segment.push_back(pt);
                    }
                    else if(trackingState == 4 && bHasLocalized) // LOST
                    {
                        if (tLost < 0) {
                            tLost = tframe;
                            if (!current_segment.empty()) {
                                trajectory_segments.push_back(current_segment);
                                current_segment.clear();
                            }
                            cerr << "Tracking lost (State: LOST) at frame " << ni << ". Will retry for 10 seconds..." << endl;
                        } else if (tframe - tLost > 10.0) {
                            cerr << "Tracking lost for over 10 seconds. Stopping sequence to save trajectory..." << endl;
                            break;
                        }
                    }

                    double full_ms = std::chrono::duration<double,std::milli>(
                        std::chrono::steady_clock::now() - t_frame_start).count();

                    timingLogs.push_back({ni, slam_ms, full_ms});

                    // Simulated sleep for non-realtime playback to visualize at 10Hz if processing is faster
                    if(!bRealTime) {
                        double ttrack = slam_ms / 1000.0;
                        if(ttrack < 0.1) {
                            usleep((0.1 - ttrack) * 1e6);
                        }
                    }

                    const bool is_realtime = (slam_ms < 100.0);
                    std::cout << std::fixed << std::setprecision(1)
                              << "Frame " << ni << ": "
                              << (is_realtime ? "REAL-TIME" : "DELAYED  ")
                              << "  SLAM: " << std::setw(6) << slam_ms << "ms"
                              << "  Full: " << std::setw(6) << full_ms << "ms"
                              << "  IMU: "  << num_imu << "\n";

                    imLeft.release();
                    imRight.release();
                    ni++;
                } else {
                    // Frame drop/skip if desynchronized
                    if (tLeft < tRight) imLeft.release();
                    else imRight.release();
                }
            }
        }
    }

    if (!current_segment.empty()) {
        trajectory_segments.push_back(current_segment);
    }
    
    size_t max_len = 0;
    int max_idx = -1;
    for (size_t i = 0; i < trajectory_segments.size(); ++i) {
        if (trajectory_segments[i].size() > max_len) {
            max_len = trajectory_segments[i].size();
            max_idx = i;
        }
    }

    if (max_idx >= 0) {
        for (const auto& pt : trajectory_segments[max_idx]) {
            odomFile << setprecision(6) << pt.t << " "
                     << setprecision(9) << pt.twb(0) << " " << pt.twb(1) << " " << pt.twb(2) << " "
                     << pt.q.x() << " " << pt.q.y() << " " << pt.q.z() << " " << pt.q.w() << "\n";
        }
    }

    odomFile.close();

    std::ofstream firstOdomFile(strOutName + "_first_bak");
    firstOdomFile << fixed;
    if (!trajectory_segments.empty()) {
        for (const auto& pt : trajectory_segments[0]) {
            firstOdomFile << setprecision(6) << pt.t << " "
                          << setprecision(9) << pt.twb(0) << " " << pt.twb(1) << " " << pt.twb(2) << " "
                          << pt.q.x() << " " << pt.q.y() << " " << pt.q.z() << " " << pt.q.w() << "\n";
        }
    }
    firstOdomFile.close();

    // --- Atlas integrity check & pointcloud export ---
    {
        ORB_SLAM3::Atlas* atlas = SLAM.GetAtlas();
        if(atlas)
        {
            auto allMaps = atlas->GetAllMaps();
            size_t nKFs  = (atlas->GetCurrentMap()) ? atlas->GetCurrentMap()->KeyFramesInMap() : 0;
            auto   allMP = atlas->GetAllMapPoints();
            size_t nGoodMP = 0;
            for(auto* pMP : allMP)
                if(pMP && !pMP->isBad()) nGoodMP++;

            cout << "\n=== Atlas Integrity Check ===" << endl;
            cout << "  Maps in atlas:       " << allMaps.size() << "  (expected: 1 for pure localization)" << endl;
            cout << "  KFs in current map:  " << nKFs           << "  (expected: same as loaded atlas)"    << endl;
            cout << "  Good map points:     " << nGoodMP        << endl;
            if(allMaps.size() == 1)
                cout << "  [PASS] Single map - localised against loaded atlas, no new map created." << endl;
            else
                cout << "  [FAIL] Multiple maps detected - new maps were created during the run." << endl;
            cout << "=============================" << endl;

            std::string filename = "pointcloud.csv";
            std::ofstream file(filename);
            for(ORB_SLAM3::MapPoint* pMP : allMP)
            {
                if(pMP && !pMP->isBad())
                {
                    Eigen::Vector3f pos = pMP->GetWorldPos();
                    file << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
                }
            }
            file.close();
            cout << "Point cloud written to " << filename << " (" << nGoodMP << " points)" << endl;
        }
    }

    SLAM.Shutdown();

    SLAM.SaveTrajectoryEuRoC(strOutName);
    SLAM.SaveKeyFrameTrajectoryEuRoC(strOutName + "_kf");

    if(time_offset != 0.0)
    {
        cout << "Correcting timestamps in output files (offset = " << time_offset << " s)..." << endl;
        CorrectTimestamps(strOutName + "_bak", time_offset, 1.0);   
        CorrectTimestamps(strOutName + "_first_bak", time_offset, 1.0);   
        CorrectTimestamps(strOutName,           time_offset, 1e9);  
        CorrectTimestamps(strOutName + "_kf",   time_offset, 1e9);  
        cout << "Timestamp correction done." << endl;
    }

    return 0;
}
