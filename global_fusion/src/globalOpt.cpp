/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "globalOpt.h"
#include "Factors.h"

GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
    N_MAX_GPS_WINDOWS = 6;
    TH_MIN_EIGEN_RATIO = 50;
    decayFactor = -0.2*log(2.0);
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;


    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	GPSPositionMap[t] = tmp;
    EstimateGPSHeading(t);
    newGPS = true;

}


void GlobalOptimization::EstimateGPSHeading(double t)
{
    mPoseMap.lock();
    // no VI data
    if(globalPoseMap.size() < 1)
    {
        mPoseMap.unlock();
        return;
    }

    // if 1 or 2 gps point :
    if(GPSPositionMap.size() < 3)
    {
        // new gps(reset)
        if(GPSPositionMap.size() == 1)
        {
            GPSHeadingMap.clear();
        }
        mPoseMap.unlock();
        return;
    }
   
    // calc cov, stdv
    vector<double> vX, vY, vSquaredDiffX, vSquaredDiffY, vCrossDiffXY;
    vX.reserve(N_MAX_GPS_WINDOWS);
    vY.reserve(N_MAX_GPS_WINDOWS);
    vSquaredDiffX.reserve(N_MAX_GPS_WINDOWS);
    vSquaredDiffY.reserve(N_MAX_GPS_WINDOWS);
    vCrossDiffXY.reserve(N_MAX_GPS_WINDOWS);
    // latest element
    auto iterGPS = GPSPositionMap.find(t);
    // copy max 5 before GPS point (total 6 windows elements)
    for(int nIter = 0; (nIter<N_MAX_GPS_WINDOWS) && (iterGPS != GPSPositionMap.begin()); nIter++, iterGPS--)
    {
        // new -> old order save
        vX.emplace_back(iterGPS->second[0]);
        vY.emplace_back(iterGPS->second[1]);
    }

    double meanX = accumulate(vX.begin(), vX.end(), 0.0)/vX.size();
    double meanY = accumulate(vY.begin(), vY.end(), 0.0)/vY.size();
    // calc squared diff
    for(int i = 0; i<vX.size(); i++)
    {
        vSquaredDiffX.emplace_back(vX[i]-meanX);
        vSquaredDiffY.emplace_back(vY[i]-meanY);
        vCrossDiffXY.emplace_back(vSquaredDiffX[i]*vSquaredDiffY[i]);
        vSquaredDiffX[i] = pow(vSquaredDiffX[i], 2);
        vSquaredDiffY[i] = pow(vSquaredDiffY[i], 2);
    }

    // calc varince for axis
    double sxx = accumulate(vSquaredDiffX.begin(), vSquaredDiffX.end(), 0.0)/vSquaredDiffX.size();
    double syy = accumulate(vSquaredDiffY.begin(), vSquaredDiffY.end(), 0.0)/vSquaredDiffY.size();
    double sxy = accumulate(vCrossDiffXY.begin(), vCrossDiffXY.end(), 0.0)/vCrossDiffXY.size();

    // if no moving
    double stdv = sqrt(sxx+syy);
    if(stdv<1.5)
    {
        mPoseMap.unlock();
        return;
    }

    // calc cov
    Eigen::Matrix<double, 2, 2> cov;
    cov << sxx, sxy, sxy, syy;
    Eigen::EigenSolver<Eigen::Matrix<double, 2,2> > es(cov);
    auto eigenvalues = es.eigenvalues();
    auto eigenvectors = es.eigenvectors();
    uint32_t majorIdx = 0;
    // set major idx by major eigenvalue
    if (eigenvalues[1].real() > eigenvalues[0].real())
        majorIdx = 1;
    // PCA -> if major axies have low eigenvalue scale
    if(eigenvalues[majorIdx].real() <= 0.0001)
    {
        mPoseMap.unlock();
        return;
    }
    double eigenratio = eigenvalues[majorIdx].real()/eigenvalues[!majorIdx].real();
    // if nonlinear
    if(eigenratio<TH_MIN_EIGEN_RATIO)
    {
        mPoseMap.unlock();
        return;
    }
    //normalized major eigenvetor components
    double normalMajorEVecX = eigenvectors(0, majorIdx).real();
    double normalMajorEVecY = eigenvectors(1, majorIdx).real();
    // eigenvector normalization
    double len = sqrt(pow(normalMajorEVecX, 2)+pow(normalMajorEVecY, 2));
    normalMajorEVecX /= len;
    normalMajorEVecY /= len;

    // calc direction
    double normalDirX = 0.0;
    double normalDirY = 0.0;
    for(int i = 0; i < vX.size()-1; i++)
    {
        //new -> old order search
        double partialDx = double(vX[i]-vX[i+1]);
        double partialDy = double(vY[i]-vY[i+1]);
        double partialLen = sqrt(pow(partialDx, 2)+pow(partialDy, 2));
        if(partialLen == 0)
            continue;
        normalDirX += partialDx/partialLen;
        normalDirY += partialDy/partialLen;
    }
    len = sqrt(pow(normalDirX, 2)+pow(normalDirY, 2));
    if(len<0.00001)
    {
        mPoseMap.unlock();
        return;
    }
    normalDirX /= len;
    normalDirY /= len;
    double dot = normalMajorEVecX*normalDirX+normalMajorEVecY*normalDirY;
    // if has opposite direction -> correct
    if(dot<0)
    {
        normalMajorEVecX = -normalMajorEVecX;
        normalMajorEVecY = -normalMajorEVecY;
    }

    double yaw = atan2(normalMajorEVecY, normalMajorEVecX);
    //yaw stdv func
    double yawStdv = 15.0/(0.04*eigenratio-1)*M_PI/180.0;
    auto q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yawStdv, Eigen::Vector3d::UnitZ());
    double qStdv = pow(q.z(), 2);
    
    iterGPS = GPSPositionMap.find(t);
    auto iterPose = globalPoseMap.find(t);
    for(int i = 0; i<N_MAX_GPS_WINDOWS && iterGPS!=GPSPositionMap.begin() && iterPose!=globalPoseMap.begin(); i++, iterGPS--, iterPose--)
    {
        auto iterT = iterGPS->first;
        // already has gps yaw
        if(GPSHeadingMap.find(iterT) != GPSHeadingMap.end())
            continue;
        
        // calc has roll, pitch close to 0
        auto originVerticalAxis = Eigen::Vector3d(0.0, 0.0, 1.0);
        auto poseQuat = Eigen::Quaterniond(iterPose->second[3], iterPose->second[4], iterPose->second[5], iterPose->second[6]);
        auto imuVerticalAxis = poseQuat.toRotationMatrix()*Eigen::Vector3d(1.0, 0.0, 0.0);
        double dot = imuVerticalAxis.dot(originVerticalAxis);

        // if nGPS -> inf, assistant -> 0
        double assistant = exp(decayFactor*GPSHeadingMap.size());
        double accuracy = std::max(dot, assistant);
        
        GPSHeadingMap[iterT] = pair<double, double>(yaw, qStdv/accuracy);
    }
    mPoseMap.unlock();
}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */

                }
                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }

                auto iterGPSHeading = GPSHeadingMap.find(t);
                if(iterGPSHeading != GPSHeadingMap.end())
                {
                    ceres::CostFunction* GPSHeadingCost = QError::Create(iterGPSHeading->second.first, iterGPSHeading->second.second);
                    problem.AddResidualBlock(GPSHeadingCost, NULL, q_array[i]);
                }
            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}
