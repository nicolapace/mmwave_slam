#include "mmWaveDataHdl.hpp"
#include "DataHandlerClass.h"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

mmWaveDataHdl::mmWaveDataHdl() {}

void mmWaveDataHdl::onInit()
{
    ros::NodeHandle private_nh("~");

    std::string mySerialPort;
    std::string myFrameID;
    int mypunti_desiderati;
    bool myuseAgglo;
    bool myuseLior;
    bool mytakeSideInfo;
    double mydist_outlier_th;
    double mycdist_th;
    int myBaudRate;
    int myMaxAllowedElevationAngleDeg;
    int myMaxAllowedAzimuthAngleDeg;
   
    private_nh.getParam("data_port", mySerialPort);
   
    private_nh.getParam("data_rate", myBaudRate);
    
    private_nh.getParam("frame_id", myFrameID);

    if (!(private_nh.getParam("max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg))) {
        myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
    }

    if (!(private_nh.getParam("max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg))) {
        myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
    }

    //Set desire number of points for the agglomerative clustering algorithm

    private_nh.getParam("pdesiderati", mypunti_desiderati);

    // set use agglomerative cluster default to false
    if (!(private_nh.getParam("use_agglo", myuseAgglo))) {
        myuseAgglo = false;  // Use max angle if none specified
    }

    // private_nh.getParam("use_agglo", myuseAgglo);

    // set use LIOR filter default to false 
    if (!(private_nh.getParam("use_lior", myuseLior))) {
        myuseLior = false;  // Use max angle if none specified
    }

    // set use LIOR filter default to false 
    if (!(private_nh.getParam("distInlierTh", myuseLior))) {
        mydist_outlier_th = 0.2;  // Use dist inlier 0.2m by default
    }

    if (!(private_nh.getParam("take_sideinfo", mytakeSideInfo))) {
        mytakeSideInfo = false;  // don't take sideInfo by default
    }

    private_nh.getParam("cdist", mycdist_th);

    

    ROS_INFO("mmWaveDataHdl: data_port = %s", mySerialPort.c_str());
    ROS_INFO("mmWaveDataHdl: data_rate = %d", myBaudRate);
    ROS_INFO("mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
    ROS_INFO("mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);
    ROS_INFO("mmWaveDataHdl: punti_desiderati = %d", mypunti_desiderati);
    ROS_INFO("mmWaveDataHdl: use_agglo = %d", myuseAgglo);
    ROS_INFO("mmWaveDataHdl: use_lior = %d", myuseLior);
    ROS_INFO("mmWaveDataHdl: take_sideinfo = %d", mytakeSideInfo);
    ROS_INFO("mmWaveDataHdl: distInlierTh = %f", mydist_outlier_th);
    ROS_INFO("mmWaveDataHdl: cdist = %f", mycdist_th);

   
    DataUARTHandler DataHandler(&private_nh);
    DataHandler.setFrameID( (char*) myFrameID.c_str() );
    DataHandler.setUARTPort( (char*) mySerialPort.c_str() );
    DataHandler.setBaudRate( myBaudRate );
    DataHandler.setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
    DataHandler.setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
    DataHandler.setPuntiDesiderati( mypunti_desiderati );
    DataHandler.setUseAgglo( myuseAgglo );
    DataHandler.setUseLior( myuseLior );
    DataHandler.settakeSideInfo( mytakeSideInfo );
    DataHandler.setdistInlierTh( mydist_outlier_th );
    DataHandler.setcdist_th( mycdist_th);
    DataHandler.start();
   
    NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
}

}