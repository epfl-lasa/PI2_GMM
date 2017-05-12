#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
//#include "MathLib/Matrix.h" // apparently conflicting with Matrix3.h
#include "MathLib/Matrix3.h"
//#include "MathLib/Vector.h"
#include "MathLib/Vector3.h"

#define TOPIC_NAME_POSE "/KUKA/Pose"



using namespace MathLib;

class GravityCompensator
{
    Matrix recordedRotAxis;
    Vector recordedRotAngle;
    Matrix recordedWrench;
    Matrix recordedObsMatrix;
   // Matrix recordedR3 [];


    Vector estimatedParameters;

    Vector3 newRotAxis;
    double newRotAngle;
    Vector newWrenchData;
    Matrix newObsMatrix;
    Matrix3 newR3;
    Matrix3 toolR3;

    Vector compensatedWrenches;
    Vector3 compensatedForces;
    Vector3 compensatedTorques;
    Vector compensatedWrenchesBase;
    Vector3 compensatedForcesBase;
    Vector3 compensatedTorquesBase;

    bool poseReady;
    bool wrenchReady;

    int sampleSize;
    int recLimit;
    int recCountPos;
    int recCountWrench;

    bool modelReady;
    bool compensate;

    ros::NodeHandle nodeHandle;
    ros::Publisher pubBase;
    ros::Publisher pubEE;
    ros::Publisher pubPose;
    ros::Subscriber poseSub;
    ros::Subscriber wrenchSub;
    ros::Subscriber ctrlSub;
    geometry_msgs::WrenchStamped pubMessageBase;
    geometry_msgs::WrenchStamped pubMessageEE;
    geometry_msgs::PoseStamped latestPoseMsg;
    geometry_msgs::WrenchStamped latestWrenchMsg;

// Observation matrix stuff
    Vector3 phi;
    Vector3 g;
    Vector3 rotAxis;
    Matrix3 rotMat;
    Matrix hf;
    Matrix ht;
    Matrix3 crossMat;
    Vector wrenchVect;

    public:

    GravityCompensator(int _sampleSize);

    void ctrlSubCallback(const std_msgs::Int32);

    void posSubCallback(const geometry_msgs::PoseStamped);
    void wrenchSubCallback(const geometry_msgs::WrenchStamped);

    void computeObservationMatrix(bool record);
    void computeEstimatedParams(void);
    void computeCompensatedWrenches(void);
    int saveModel(void);
    int loadModel(void);
    void tareOffset();
};
