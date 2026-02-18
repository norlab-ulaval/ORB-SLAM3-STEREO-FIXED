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

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./stereo_inertial_fomo path_to_vocabulary path_to_settings path_to_sequence_folder" << endl;
        return 1;
    }

    // Load sequence
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    int first_imu = 0;

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

    // Find first imu to be considered, supposing imu measurements start first
    while(first_imu<vTimestampsImu.size() && vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    if(first_imu>0)
        first_imu--; // first imu measurement to be considered

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat imLeft, imRight;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_track = 0.f;

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

        if(ni>0)
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

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

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
                  << " (processing: " << ttrack << "s, target: " << T << "s)"
                  << std::endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("trajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    ORB_SLAM3::Atlas* atlas = nullptr;
    atlas = SLAM.GetAtlas();

    std::string filename = "pointcloud.csv";
    std::ofstream file(filename);
    for(ORB_SLAM3::MapPoint* pMP : atlas->GetAllMapPoints())
    {
        if(pMP && !pMP->isBad())
        {
            Eigen::Vector3f pos = pMP->GetWorldPos();
            file << pos.x() << "," << pos.y() << "," << pos.z() << std::endl;
        }
    }
    file.close();

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
        vTimeStamps.push_back(t / 1e6); // Assuming timestamp in microseconds based on example filename 1738249093881602 (16 digits -> microseconds for epoch time)
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

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

    // Check for timestamp oscillation
    // If detected, assume 200Hz freq for all samples
    int inconsistencies = 0;
    for(size_t i=1; i<vTimeStamps.size(); i++)
    {
        if(vTimeStamps[i] <= vTimeStamps[i-1])
        {
            vTimeStamps[i] = vTimeStamps[i-1] + 0.005;
            inconsistencies++;
        }
    }

    if(inconsistencies > 0)
        cerr << "Corrected " << inconsistencies << " IMU timestamp inconsistencies (assumed 200Hz)." << endl;
}
