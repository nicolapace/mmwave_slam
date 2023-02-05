#include <DataHandlerClass.h>
#include <string>
#include <vector>
#include "fastcluster.h"
#include <chrono>

using namespace std;
using namespace std::chrono;

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity;
            float velocity;
        };
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (mmWaveCloudType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, velocity, velocity))


// point euclidean distance 3D
double distance(const mmWaveCloudType& p1, const mmWaveCloudType& p2) {
  double sprod = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) + (p2.z - p1.z)*(p2.z - p1.z);
  double d = sqrt(sprod);
  
  return d;
}

DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {
    DataUARTHandler_pub = nh->advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_scan_pcl", 100);
    // radar_scan_pub = nh->advertise<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100);
    // marker_pub = nh->advertise<visualization_msgs::Marker>("/ti_mmwave/radar_scan_markers", 100);
    maxAllowedElevationAngleDeg = 90;   // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90;     // Use max angle if none specified
    // USE_AGGLOMERATIVE = false;          // Not use Agglomerative Clustering if none specified
    // USE_LIOR_FILTER = false;            // Not use Lior Filter if none specified
    // dist_outlier_th = 0.2;              // Use 0.2m by default for the Th inlier points in LIOR filter
    // punti_desiderati = 200;             // Use 200 points by default in the agglomerative clustering

    // Wait for parameters
    while(!nh->hasParam("/ti_mmwave/doppler_vel_resolution")){}

    nh->getParam("/ti_mmwave/numAdcSamples", nr);
    nh->getParam("/ti_mmwave/numLoops", nd);
    nh->getParam("/ti_mmwave/num_TX", ntx);
    nh->getParam("/ti_mmwave/f_s", fs);
    nh->getParam("/ti_mmwave/f_c", fc);
    nh->getParam("/ti_mmwave/BW", BW);
    nh->getParam("/ti_mmwave/PRI", PRI);
    nh->getParam("/ti_mmwave/t_fr", tfr);
    nh->getParam("/ti_mmwave/max_range", max_range);
    nh->getParam("/ti_mmwave/range_resolution", vrange);
    nh->getParam("/ti_mmwave/max_doppler_vel", max_vel);
    nh->getParam("/ti_mmwave/doppler_vel_resolution", vvel);

    ROS_INFO("\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
        nr, nd, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);
}

void DataUARTHandler::setFrameID(char* myFrameID)
{
    frameID = myFrameID;
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of punti desiderati*/
void DataUARTHandler::setPuntiDesiderati(int mypunti_desiderati)
{
    punti_desiderati = mypunti_desiderati;
}

/*Implementation of setUseAgglo*/
void DataUARTHandler::setUseAgglo(bool myuseAgglo)
{
    USE_AGGLOMERATIVE = myuseAgglo;
}

/*Implementation of setUseLior*/
void DataUARTHandler::setUseLior(bool myuseLior)
{
    USE_LIOR_FILTER = myuseLior;
}

/*Implementation of takeSideInfo*/
void DataUARTHandler::settakeSideInfo(bool mytakeSideInfo)
{
    TAKE_SIDEINFO = mytakeSideInfo;
}

/*Implementation of setUseLior*/
void DataUARTHandler::setdistInlierTh(double mydist_outlier_th)
{
    dist_outlier_th = mydist_outlier_th;
}

void DataUARTHandler::setcdist_th(double mycdist_th)
{
    cdist_th = mycdist_th;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{
    
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};
    
    /*Open UART Port and error checking*/
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    
    /*Quick magicWord check to synchronize program with data Stream*/
    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        ROS_ERROR("Non agganciato");
    }
    
    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);
    
    while(ros::ok())
    {
        /*Start reading UART data and writing to buffer while also checking for magicWord*/
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        
        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        
            /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            /*increment countSync*/
            countSync++;
            
            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }
            
            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
            
            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);
            
            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));
              
        }
      
    }
    
    
    mySerialObject.close();
    
    pthread_exit(NULL);
}


int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataUARTHandler::syncedBufferSwap(void)
{   

    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}

void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    int j = 0;
    int k = 0;
    int punti_presenti;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;

    float x_ros = 0;
    float y_ros = 0;
    float z_ros = 0;
    
    //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan(new pcl::PointCloud<mmWaveCloudType>);
    // ti_mmwave_rospkg::RadarScan radarscan;

    // init variabili
    int temporally ;

    double* distmat;
    int* merge;
    double* height;
    int* labels;
    int* old_labels;

    //init delle robe che servono per estrarre i centroidi 
    int step , npoints;
    int opt_method = HCLUST_METHOD_SINGLE;
    double eps = 0.001;
    double cdist = cdist_th + eps;
    int centri=0;
    

    // int punti_desiderati = 200;          //E' un parametro settabile da shell
    // condidero gli indici dei valori saltati per poter estrapolare le corrispettive intensity 
    // in READ_SIDE_INFO
    vector<int> vector_saltare;      // riabilitare se si desidera estrapolare le SideInfo
    int indice_da_saltare;

    float intencity_th = 0.2;           // soglia di intensità
    double dist_outlier;                // distanza dai punti ad alta intensità
    //double dist_outlier_th = 0.2;     // soglia distanza dai punti ad alta intensità -- E' una varabile inseribile da shell
    int n_outlier;                      // alta intensità vicini
    int n_outlier_th = 3;               // soglia del filtro LIOR punti ad alta intensità vicini
    

    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);   
    pthread_mutex_lock(&currentBufp_mutex);

    //Calculate ratios for max desired elevation and azimuth angles
    if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90)) {
        maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
        maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
    } else maxElevationAngleRatioSquared = -1;
    if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90)) maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
    else maxAzimuthAngleRatio = -1;
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:
            
            //init variables -- questa equivale alla npoints del programma demo3D.cpp
            mmwData.numObjOut = 0;

            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
            if(currentBufp->size() < 12)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get version (4 bytes)
            memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
            currentDatap += ( sizeof(mmwData.header.version) );
            
            //get totalPacketLen (4 bytes)
            memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
            currentDatap += ( sizeof(mmwData.header.totalPacketLen) );
            
            //get platform (4 bytes)
            memcpy( &mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
            currentDatap += ( sizeof(mmwData.header.platform) );      
            
            //if packet doesn't have correct header size (which is based on platform), throw it away
            //  (does not include magicWord since it was already removed)
            
            headerSize = 8 * 4;  // header size 8 elements uint32_t (4 byte)
            
            if(currentBufp->size() < headerSize) {
                sorterState = SWAP_BUFFERS;
                break;
            }
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
            currentDatap += ( sizeof(mmwData.header.frameNumber) );
            
            //get timeCpuCycles (4 bytes)
            memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
            currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );
            
            //get numDetectedObj (4 bytes)
            memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
            currentDatap += ( sizeof(mmwData.header.numDetectedObj) );
            
            //get numTLVs (4 bytes)
            memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
            currentDatap += ( sizeof(mmwData.header.numTLVs) );
            
            memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
            currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
            
            //if packet lengths do not match, throw it away
            if(mmwData.header.totalPacketLen == currentBufp->size() )
            {
               sorterState = CHECK_TLV_TYPE;
            }
            else sorterState = SWAP_BUFFERS;

            break;
            
        case READ_OBJ_STRUCT:
            
            // CHECK_TLV_TYPE code has already read tlvType and tlvLen
            i = 0;
            offset = 0;
            
            // SDK version is at least 3.x
            mmwData.numObjOut = mmwData.header.numDetectedObj;
            vector_saltare.reserve(mmwData.numObjOut);
            npoints = mmwData.numObjOut;
            //cout << "num_obj: " << mmwData.header.numDetectedObj << "\n";

            pcl_conversions::toPCL(ros::Time::now(), RScan->header.stamp);
            RScan->header.frame_id = frameID;
            RScan->height = 1;
            
            // la size dell' oggetto Rscan non deve essere numObjOut ma deve essere punti_desiderati 
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);

            // Populate pointcloud
            punti_presenti = 0;

            while( i < mmwData.numObjOut ) {
                
                // SDK version is 3.x+
                //get object x-coordinate (meters)
                memcpy( &mmwData.newObjOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.x));
                currentDatap += ( sizeof(mmwData.newObjOut.x) );
            
                //get object y-coordinate (meters)
                memcpy( &mmwData.newObjOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.y));
                currentDatap += ( sizeof(mmwData.newObjOut.y) );
            
                //get object z-coordinate (meters)
                memcpy( &mmwData.newObjOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.z));
                currentDatap += ( sizeof(mmwData.newObjOut.z) );
            
                //get object velocity (m/s)
                memcpy( &mmwData.newObjOut.velocity, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.velocity));
                currentDatap += ( sizeof(mmwData.newObjOut.velocity) );

                // coordinate per sensore AWR1843AOP. Il punto (255;255;255) che è identificativo di "Nessun Punto Rilevato" viene di seguito convertito in (255;-255;-255)
                x_ros = mmwData.newObjOut.y;
                y_ros = -mmwData.newObjOut.z;
                z_ros = -mmwData.newObjOut.x;


                // controllo sulle posizioni dei punti da pubblicare -- taglio fuori tutti i punti con coordinata z più alta di quello che mi interessa e per sicurezza taglio
                // fuori anche i punti distanti < 5cm anche se da cfg dovrebbe già tagliarli
                if ((((maxElevationAngleRatioSquared == -1) ||
                             (((z_ros * z_ros) / (x_ros * x_ros +   y_ros * y_ros)) < maxElevationAngleRatioSquared)) &&
                            ((maxAzimuthAngleRatio == -1) || (fabs(y_ros / x_ros) < maxAzimuthAngleRatio))  && (z_ros < 0.4)
                    ) || (( x_ros == 255 )&&( y_ros == -255 )&&( z_ros == -255)) )
                {   
                    // Se i punti rientrano nel cono di visione dichiarato allora allocali in Rscan
                    // oppure se è il punto indicativo di nessun punto rilevato viene allocato 
                    // Se tutto funziona bene ci sarà sempre almeno 1 punto allocato

                    //siccome è un vector Rscan->points si può fare una push_back 
                    RScan->points[punti_presenti].x = x_ros;   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[punti_presenti].y = y_ros;   // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(Z-axis)
                    RScan->points[punti_presenti].z = z_ros;   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor -(X-axis)

		            RScan->points[punti_presenti].velocity = mmwData.newObjOut.velocity;
                    
                    // 
                    if(!TAKE_SIDEINFO) {RScan->points[punti_presenti].intensity = 255;}  //fake intensity if don't use SideInfo from cfg file                    
                    else{vector_saltare[i] = 0 ;}       // 0 significa che il punto rispetta le condizioni sulle coordinate
                    
                    punti_presenti ++;                  // tiene conto dei punti effettivamente allocati dopo la verifica dovuta all'if
                }
                else{
                    if(TAKE_SIDEINFO) {vector_saltare[i] = 1;}    //1 significa che il punto non rispetta le condizioni sulle coordinate 
                }
                
                i++;

            }
            
            RScan->width = punti_presenti;
            RScan->points.resize(RScan->width * RScan->height);

            sorterState = CHECK_TLV_TYPE;
            
            break;

        case READ_SIDE_INFO:

            // avendo posto 2 in GUIMonitor le SideInfo lato firmware non vengono proprio inviate quindi posso tagliare le memcpy
            // indice_da_saltare = 0;
          
            // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
            if (punti_presenti > 0)
            {   
                
                // Rilevato almeno 1 punto utile

                // setto un flag TAKE_SIDEINFO che permette di scegliere se usare o meno le SideInfo in accordo a quanto settato nel file CFG settabile da file launch
                // flag non serve più poichè se entro nel tlv entro solo se volgio le SideInfo
                // Take SideInfo

                j = 0;
                for (i = 0; i < mmwData.numObjOut; i++)
                {   
                    // considera la intensity solo dei punti che soddisfano le condizioni sulle coordinate
                    if( vector_saltare[i] == 0 )
                    {
                        //get snr (unit is 0.1 steps of dB)
                        memcpy( &mmwData.sideInfo.snr, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.snr));
                        currentDatap += ( sizeof(mmwData.sideInfo.snr) );

                        RScan->points[j].intensity = (float) mmwData.sideInfo.snr / 10.0;   // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
                        j++;
                    }
                }
                // svuota vector_saltare
                vector_saltare.clear();
            }
            else{ 

                //se punti_presenti <= 0
                ROS_ERROR("mmwData.numObjOut < 1 -- Something is wrong");
                i = 0;
            
                currentDatap += tlvLen;
            }

            sorterState = CHECK_TLV_TYPE;
            
            break;
                    

        case READ_LOG_MAG_RANGE:
            {


              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_NOISE:
            {
        
              i = 0;
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
           
            break;
            
        case READ_AZIMUTH:
            {
        
              i = 0;
            
            //   while (i++ < tlvLen - 1)
            //   {
            //          //ROS_INFO("DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
            //   }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_DOPPLER:
            {
        
              i = 0;
            
            //   while (i++ < tlvLen - 1)
            //   {
            //          //ROS_INFO("DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
            //   }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_STATS:
            {
        
              i = 0;
            
            //   while (i++ < tlvLen - 1)
            //   {
            //          //ROS_INFO("DataUARTHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
            //   }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
        
        case CHECK_TLV_TYPE:

            if(tlvCount++ >= mmwData.header.numTLVs)  // Done parsing all received TLV sections
            {
                // Publish detected object pointcloud -- Pubblica lo stesso anche se non faccio agglomerative e se non ci sono punti rilevati
                if ((mmwData.numObjOut > 0))
                {   

                    /*Rileva se Nessun elemento è stato visto -- il sensore continua a funionare correttamente*/
                    if((punti_presenti == 1) && (RScan->points[0].x == 255) && (RScan->points[0].y == -255) && (RScan->points[0].z == -255)){

                        ROS_INFO("Nessun Punto Rilevato");

                    }else{
                        
                        // Punti Utili
                        //ROS_INFO("pp: %d" , punti_presenti);

                        if((punti_presenti > 1) && USE_AGGLOMERATIVE){

                            RScan->width = punti_desiderati;
                            RScan->points.resize(RScan->width * RScan->height);

                            npoints = punti_presenti;

                            // qualche punto verrà scartato perchè non rispetta la condizione dell'if ma cmq Rscan è composto da npoints
                            while(npoints < punti_desiderati){

                                temporally = npoints;
                                // una volta raccolti tutti i punti in Rscan si possono calcolare le distanze tra tutti i punti 
                                distmat = new double[(npoints*(npoints-1))/2];
                                k = 0;
                                for (i=0; i<npoints; i++) {
                                    for (j=i+1; j<npoints; j++) {
                                        distmat[k] = distance(RScan->points[i], RScan->points[j]);
                                        k++;
                                    }
                                }
                                // clustering call -- costruzione della matrice di dissimilarità
                                merge = new int[2*(npoints-1)];
                                height = new double[npoints-1];
                                labels = new int[npoints];
                                old_labels = new int[npoints];
                                hclust_fast(npoints, distmat, opt_method, merge, height);

                                // popola old labels con labels tutte diverse 
                                for (i=0; i<npoints; i++){  old_labels[i] = i;}

                                //condizione sulla distanza 
                                for(k=0;k<(npoints-1);k++){
                                    if (height[k] >= cdist) {
                                        break;
                                    } 
                                }
                                step = npoints - k;

                                // condizione per uscire dal while riga 590 nel caso in cui tutti i punti rilevati violano la condizione di distanza
                                if(temporally <= step){
                                    break;
                                }

                                // oltre alla condizione sullo step bisogna aggiungere la condizione sul raggingimento dei punti necessari che saranno mmwData.numObjOut + n_centroidi
                                j = 0;          // azzero il contatore dei centroidi prodotti

                                while(  (temporally > 1)  &&  ((npoints + j) <  punti_desiderati ) ){
                                    // se sono violo la condizione sulla distanza non iterare più
                                    if(temporally <= step){
                                        break;
                                    }else{

                                        // finchè non ho 2 cluster posso cercare di ottenerne 1
                                        // la variabile temporally funge da n_cluster 
                                        temporally = temporally - 1;

                                        cutree_k(npoints, merge, temporally , labels);

                                        // scorro su tutti i punti per beccare la prima labels diversa 
                                        for(i=0; i< npoints;i++){
                                            
                                            // condizione sulla label su cui creare il cluster 
                                            if(old_labels[i] != labels[i]) {
                                                
                                                // 1 soluzione possibile 
                                                for( k=0;k<npoints;k++){
                                                    if(labels[k]==labels[i]){
                                                        x_ros = x_ros + RScan->points[k].x;
                                                        y_ros = y_ros + RScan->points[k].y;
                                                        z_ros = z_ros + RScan->points[k].z;
                                                        centri ++;
                                                    }
                                                }
                                                // prelevo il punto medio tra i punti più vicini tra i due cluster 
                                                x_ros /=centri;
                                                y_ros /=centri;
                                                z_ros /=centri;
                                                // pubblica centroide 
                                                // radar_scan_pub.publish(radarscan);               non mi interessano le letture su radarscan
                                                // j conta quanti centroidi vengono aggiunti
                                                RScan->points[npoints + j].x = x_ros;
                                                RScan->points[npoints + j].y = y_ros;
                                                RScan->points[npoints + j].z = z_ros;
                                                RScan->points[npoints + j].intensity = 255;        // setta la massima intensità ai punti agglomerati
                                                RScan->points[npoints + j].velocity = 0;          // setta la massima intensità ai punti agglomerati
                                                j++;

                                                x_ros = 0;
                                                y_ros = 0;
                                                z_ros = 0;
                                                centri = 0;
                                                
                                                // condizione di uscia dal for poichè basta la prima labels dissimile da old_labels
                                                break;
                                            }
                                        }

                                        for(i=0; i<npoints; i++){old_labels[i]=labels[i];}
                                        
                                    }
                                    
                                }

                                npoints = j + npoints;

                                // clean up
                                delete[] distmat;
                                delete[] merge;
                                delete[] height;
                                delete[] labels;
                                delete[] old_labels;
                            }

                        }

                    }




                    // Publish PointCloud
                    DataUARTHandler_pub.publish(RScan);
                    
                }else{
                    /*Non c'3 nemmeno il punto speciale*/
                    ROS_ERROR("punti_presenti <= 0 -- Punto Speciale non rilevato");
                }

                //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            
            else  // More TLV sections to parse
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));
                
                //ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                switch(tlvType)
                {
                case MMWDEMO_OUTPUT_MSG_NULL:
                
                    break;
                
                case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Object TLV");
                    sorterState = READ_OBJ_STRUCT;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Range TLV");
                    sorterState = READ_LOG_MAG_RANGE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                    //ROS_INFO("DataUARTHandler Sort Thread : Noise TLV");
                    sorterState = READ_NOISE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
                    sorterState = READ_AZIMUTH;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                    //ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
                    sorterState = READ_DOPPLER;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_STATS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
                    sorterState = READ_STATS;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                    //ROS_INFO("DataUARTHandler Sort Thread : Side info TLV");
                    sorterState = READ_SIDE_INFO;
                    break;

                case MMWDEMO_OUTPUT_MSG_MAX:
                    //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                    sorterState = READ_HEADER;
                    break;
                
                default: break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;
    
    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}

// void DataUARTHandler::visualize(const ti_mmwave_rospkg::RadarScan &msg){
//     visualization_msgs::Marker marker;

//     marker.header.frame_id = frameID;
//     marker.header.stamp = ros::Time::now();
//     marker.id = msg.point_id;
//     marker.type = visualization_msgs::Marker::SPHERE;
//     marker.lifetime = ros::Duration(tfr);
//     marker.action = marker.ADD;

//     marker.pose.position.x = msg.x;
//     marker.pose.position.y = msg.y;
//     marker.pose.position.z = 0;

//     marker.pose.orientation.x = 0;
//     marker.pose.orientation.y = 0;
//     marker.pose.orientation.z = 0;
//     marker.pose.orientation.w = 0;

//     marker.scale.x = .03;
//     marker.scale.y = .03;
//     marker.scale.z = .03;
    
//     marker.color.a = 1;
//     marker.color.r = (int) 255 * msg.intensity;
//     marker.color.g = (int) 255 * msg.intensity;
//     marker.color.b = 1;

//     marker_pub.publish(marker);
// }
