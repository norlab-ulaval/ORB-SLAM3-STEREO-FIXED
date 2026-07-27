/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>
#include <vector>
#include <dirent.h>

#include <opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

struct TrajectoryPoint {
    double t;
    Eigen::Vector3f twb;
    Eigen::Quaternionf q;
};

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

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
        cerr << endl << "Usage: ./stereo_inertial_fomo path_to_vocabulary path_to_settings path_to_sequence_folder output_trajectory_path [time_offset_seconds]" << endl;
        return 1;
    }

    // Load sequence
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;

    string pathSeq(argv[3]);
    string pathCam0 = pathSeq + "/zedx_left";
    string pathCam1 = pathSeq + "/zedx_right";
    string pathImu = pathSeq + "/vectornav.csv";

    cout << "Loading images..." << endl;
    LoadImages(pathCam0, pathCam1, vstrImageLeft, vstrImageRight, vTimestampsCam);
    cout << "LOADED!" << endl;

    cout << "Loading IMU..." << endl;
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    nImages = vstrImageLeft.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    if(vTimestampsImu.size()<=0)
    {
        cerr << "ERROR: Failed to load IMU" << endl;
        return 1;
    }

    // Create SLAM system first so we can read the atlas timestamps before deciding the offset.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true);

    // --- Auto-compute time offset from the atlas ---
    // The atlas keyframes carry the absolute timestamps from the mapping session.
    // The query dataset may come from a different date.  We align the query timestamps
    // so that the first camera frame lands just before the earliest keyframe in the atlas.
    const double kAtlasMarginSec = 5.0; // seconds before the first atlas KF
    double time_offset = 0.0;
    {
        ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
        double atlas_min_ts = std::numeric_limits<double>::max();
        if(pAtlas)
        {
            for(ORB_SLAM3::Map* pMap : pAtlas->GetAllMaps())
                for(ORB_SLAM3::KeyFrame* pKF : pMap->GetAllKeyFrames())
                    if(pKF && !pKF->isBad())
                        atlas_min_ts = std::min(atlas_min_ts, pKF->mTimeStamp);
        }

        if(atlas_min_ts < std::numeric_limits<double>::max())
        {
            // offset such that: vTimestampsCam[0] + offset = atlas_min_ts - kAtlasMarginSec
            double auto_offset = atlas_min_ts - kAtlasMarginSec - vTimestampsCam[0];

            if(argc >= 6)
            {
                // CLI argument is treated as an explicit override; warn if it differs significantly.
                double cli_offset = stod(argv[5]);
                double diff = std::abs(cli_offset - auto_offset);
                if(diff > 10.0)
                    cerr << "WARNING: CLI time_offset (" << cli_offset
                         << " s) differs from auto-computed offset (" << auto_offset
                         << " s) by " << diff << " s. Using CLI value as override." << endl;
                time_offset = cli_offset;
            }
            else
            {
                time_offset = auto_offset;
            }

            cout << "Atlas earliest KF timestamp: " << fixed << setprecision(3) << atlas_min_ts << " s" << endl;
            cout << "Dataset first cam timestamp: " << vTimestampsCam[0] << " s" << endl;
            cout << "Applying time offset:        " << time_offset << " s" << endl;
        }
        else
        {
            // No atlas loaded (pure mapping mode) – fall back to CLI argument if provided.
            if(argc >= 6)
            {
                time_offset = stod(argv[5]);
                cout << "No atlas loaded. Applying CLI time offset: " << time_offset << " s" << endl;
            }
        }
    }

    if(time_offset != 0.0)
    {
        for(size_t i=0; i<vTimestampsCam.size(); i++) vTimestampsCam[i] += time_offset;
        for(size_t i=0; i<vTimestampsImu.size();  i++) vTimestampsImu[i]  += time_offset;
    }

    // Find first IMU measurement to consider (must be re-done after offset is applied).
    int first_imu = 0;
    while(first_imu<(int)vTimestampsImu.size() && vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    if(first_imu>0)
        first_imu--; // first imu measurement to be considered

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    cv::Mat cvTbc;
    fSettings["Tbc"] >> cvTbc;
    if(cvTbc.type() != CV_32F)
        cvTbc.convertTo(cvTbc, CV_32F);
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    string strOutName(argv[4]);
    std::ofstream odomFile(strOutName + "_bak");
    odomFile << fixed;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    cv::Mat imLeft, imRight;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_track = 0.f;

    bool bHasLocalized = false;
    bool bImuExcited = false;
    double tLost = -1.0;

    std::vector<std::vector<TrajectoryPoint>> trajectory_segments;
    std::vector<TrajectoryPoint> current_segment;

    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if(ni>0) {
            // std::cout << std::fixed << "IMU Diff this frame last frame: " << vTimestampsImu[ni] - vTimestampsImu[ni-1] << std::endl;
            // std::cout << std::fixed << "Cam Diff this frame last frame: " << vTimestampsCam[ni] - vTimestampsCam[ni-1] << std::endl;
            while(first_imu<vTimestampsImu.size() && vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                         vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                         vTimestampsImu[first_imu]));
                first_imu++;
            }
            if(first_imu >= vTimestampsImu.size())
            {
                 double lastImuTime = vTimestampsImu.empty() ? 0.0 : vTimestampsImu.back();
                 if (lastImuTime < vTimestampsCam[ni]) {
                      cerr << "WARNING: IMU data exhausted at frame " << ni
                           << ". Last IMU timestamp: " << fixed << setprecision(6) << lastImuTime
                           << ", Current Cam timestamp: " << vTimestampsCam[ni] << endl;
                 }
            }
        }
        if(vImuMeas.empty() && ni > 1000) {
            cerr << "ERROR: No IMU measurements for frame " << ni << ", skipping." << endl;
            break;
        }

        if(!bImuExcited) {
            for(size_t i=0; i<vImuMeas.size(); i++) {
                if (vImuMeas[i].w.norm() > 0.1 || std::abs(vImuMeas[i].a.norm() - 9.81) > 0.5) {
                    bImuExcited = true;
                    cout << "IMU Excitation detected at frame " << ni << ". Starting processing." << endl;
                    break;
                }
            }
            // Note: we do NOT skip the frame here any more.  Feeding every frame to
            // TrackStereo keeps mLastFrame up-to-date so that when the first excited
            // frame arrives the tracker does not see a multi-second timestamp jump
            // that would otherwise destroy the loaded atlas map.
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the images to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);
        
        int trackingState = SLAM.GetTrackingState();
        if(trackingState == 2 || trackingState == 5) // OK=2, OK_KLT=5
        {
            if (tLost >= 0 && !current_segment.empty()) {
                trajectory_segments.push_back(current_segment);
                current_segment.clear();
            }
            bHasLocalized = true;
            tLost = -1.0; // Reset lost timer upon localization/relocalization
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

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);


        // Print real-time status
        std::cout << "Frame " << ni << ": "
                  << (ttrack < T ? "REAL-TIME" : "DELAYED")
                  << " (processing: " << ttrack << "s, target: " << T << "s)\n"
                  << "IMU measurements between frames: " << vImuMeas.size() << std::endl;
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

    // --- Atlas integrity check & pointcloud export (must run BEFORE Shutdown) ---
    // After Shutdown() the internal map data is freed; accessing it causes a segfault.
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
                cout << "  [PASS] Single map – localised against loaded atlas, no new map created." << endl;
            else
                cout << "  [FAIL] Multiple maps detected – new maps were created during the run." << endl;
            cout << "=============================" << endl;

            // Export map points to pointcloud.csv
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

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC(strOutName);
    SLAM.SaveKeyFrameTrajectoryEuRoC(strOutName + "_kf");

    // Correct timestamps: the time_offset was added before feeding frames to SLAM,
    // so all saved timestamps are `time_offset` seconds too large.
    if(time_offset != 0.0)
    {
        cout << "Correcting timestamps in output files (offset = " << time_offset << " s)..." << endl;
        CorrectTimestamps(strOutName + "_bak", time_offset, 1.0);   // seconds
        CorrectTimestamps(strOutName,           time_offset, 1e9);  // nanoseconds
        CorrectTimestamps(strOutName + "_kf",   time_offset, 1e9);  // nanoseconds
        cout << "Timestamp correction done." << endl;
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    DIR *dir;
    struct dirent *ent;
    vector<string> filenames;

    if ((dir = opendir(strPathLeft.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            string filename = ent->d_name;
            if (filename.length() > 0 && filename[0] == '.') continue;
            if (filename.length() > 4 && filename.substr(filename.length() - 4) == ".png") {
                filenames.push_back(filename);
            }
        }
        closedir(dir);
    } else {
        fprintf(stderr, "Could not open directory %s %s: %s\n",
                strPathLeft.c_str(), strPathRight.c_str(), strerror(errno));
        return;
    }

    sort(filenames.begin(), filenames.end());

    vstrImageLeft.reserve(filenames.size());
    vstrImageRight.reserve(filenames.size());
    vTimeStamps.reserve(filenames.size());

    for (const auto& filename : filenames) {
        vstrImageLeft.push_back(strPathLeft + "/" + filename);
        vstrImageRight.push_back(strPathRight + "/" + filename);

        // Extract timestamp from filename (remove .png extension)
        string timestamp_str = filename.substr(0, filename.length() - 4);
        double t = stod(timestamp_str);
        vTimeStamps.push_back(t / 1e6); // timestamp in microseconds
    }
    std::cout << std::fixed << "Last camera timestamp: " << vTimeStamps.back() << std::endl;
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(30000);
    vAcc.reserve(30000);
    vGyro.reserve(30000);

    string s;
    getline(fImu, s); // Skip header: t,ax,ay,az,lx,ly,lz

    while(!fImu.eof())
    {
        getline(fImu,s);
        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s; // Last item
            data[6] = stod(item);

            // t in column 0
            vTimeStamps.push_back(data[0]/1e6); // Timestamp seems to be in microseconds

            // Custom mapping based on user input:
            // ax, ay, az (col 1, 2, 3) -> Angular Velocity (Gyro)
            // lx, ly, lz (col 4, 5, 6) -> Linear Acceleration (Accel)

            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
        }
    }
    std::cout << std::fixed << "Last imu timestamp: " << vTimeStamps.back() << std::endl;
}
