#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <project1/IntegrationMethodConfig.h>

#include <robotics_hw1/MotorSpeed.h>

#include "project1/CustomOdometry.h"

#include "project1/ResetPose.h"
#include "project1/SetPose.h"

// Logging ------------------------------------------------------------------

#define LOG_PARAMS
//#define LOG_WMOTOR
//#define LOG_VMOTOR
//#define LOG_TIME
//#define LOG_TWIST
//#define LOG_POSE

// Queues -------------------------------------------------------------------

#define MOTOR_QUEUE     20
#define SYNC_QUEUE      20
#define TARGET_QUEUE    20

// Synchronizations ---------------------------------------------------------

typedef message_filters::sync_policies
        ::ApproximateTime<robotics_hw1::MotorSpeed,
                          robotics_hw1::MotorSpeed,
                          robotics_hw1::MotorSpeed,
                          robotics_hw1::MotorSpeed> MotorsSyncPolicy;

// Names --------------------------------------------------------------------

#define NODE_NAME       "scout"

#define PARENT_FRAME    "odom"
#define CHILD_FRAME     "base_link"

// Calculus -----------------------------------------------------------------

#define RPM_TO_RADS(rpm)        (rpm * M_PI / 30.0)
#define W_TO_V(w, r)            (w * r)

#define DDRIVE_V(r, l)          ((r + l) / 2.0)
#define DDRIVE_W(r, l, b)       ((r - l) / b)

#define INT_EUL(dt, x, y, theta, v, w) {    \
    x += v * dt * cos(theta);               \
    y += v * dt * sin(theta);               \
    theta += w * dt;                        \
}
#define INT_RK(dt, x, y, theta, v, w) {     \
    double alfa = (w * dt) / 2.0;           \
    x += v * dt * cos(theta + alfa);        \
    y += v * dt * sin(theta + alfa);        \
    theta += w * dt;                        \
}

//===========================================================================

class Robot
{
    // ROS communications ---------------------------------------------------

    private:

    ros::NodeHandle handle;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> subFR, subFL, subRR, subRL;
    message_filters::Synchronizer<MotorsSyncPolicy> sync;

    ros::Publisher pubTws;
    ros::Publisher pubOdm;
    ros::Publisher pubCtm;

    tf::TransformBroadcaster brdcst;

    dynamic_reconfigure::Server<project1::IntegrationMethodConfig> srvDyn;

    ros::ServiceServer srvRst, srvSet;
    

    // Robot parameters -----------------------------------------------------

    double wheel;
    double baseline;
    double reduction;

    // Computations state ---------------------------------------------------

    bool started;

    // Time -----------------------------------------------------------------

    ros::Time t0;
    double tD;

    // Odometry computations ------------------------------------------------

    bool intRK;

    double wFR, wFL, wRR, wRL;
    double vFR, vFL, vRR, vRL;
    double vL, vR;
    double vX;
    double wZ;

    double pX, pY;
    double aT;
    tf::Quaternion tfQ;
    tf::Transform tfT;

    // Ctors ----------------------------------------------------------------

    public:

    Robot() : subFR(handle, "/motor_speed_fr", MOTOR_QUEUE),                        // motors subs
              subFL(handle, "/motor_speed_fl", MOTOR_QUEUE),
              subRR(handle, "/motor_speed_rr", MOTOR_QUEUE),
              subRL(handle, "/motor_speed_rl", MOTOR_QUEUE),
              sync(MotorsSyncPolicy(SYNC_QUEUE), subFR, subFL, subRR, subRL)        // motors sync
    {
        // wheel param
        if (!handle.getParam("/scout/wheel", wheel))
            ROS_ERROR("wheel parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) wheel radius: %.4f", wheel);
        #endif

        // gearbox reduction param
        if (!handle.getParam("/scout/reduction", reduction))
            ROS_ERROR("reduction parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) gearbox reduction: %.4f", reduction);
        #endif

        // baseline param
        if (!handle.getParam("/scout/baseline", baseline))
            ROS_ERROR("baseline parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) baseline length: %.4f", baseline);
        #endif

        // node state
        started = false;

        // odometry vars
        pX = 0.0;
        pY = 0.0;
        aT = 0.0;

        updateTransform();
        notifyTransform();

        // synchro subs
        sync.registerCallback(boost::bind(&Robot::onMotors, this, _1, _2, _3, _4));

        // target pubs
        pubTws = handle.advertise<geometry_msgs::TwistStamped>("/scout/twist", TARGET_QUEUE);
        pubOdm = handle.advertise<nav_msgs::Odometry>("/scout/odom/basic", TARGET_QUEUE);
        pubCtm = handle.advertise<project1::CustomOdometry>("/scout/odom/custom", TARGET_QUEUE);

        // dyn params bind
        srvDyn.setCallback(boost::bind(&Robot::onIntMethod, this, _1, _2));

        // odom services
        srvRst = handle.advertiseService("/scout/reset", &Robot::resetPose, this);
        srvSet = handle.advertiseService("/scout/set", &Robot::setPose, this);
    }

    // Event handlers -------------------------------------------------------

    void onMotors(const robotics_hw1::MotorSpeed::ConstPtr& fr, 
                  const robotics_hw1::MotorSpeed::ConstPtr& fl,
                  const robotics_hw1::MotorSpeed::ConstPtr& rr,
                  const robotics_hw1::MotorSpeed::ConstPtr& rl)
    {
        // handle time
        bool doComp = updateTime(fr->header.stamp,
                                 fl->header.stamp,
                                 rr->header.stamp,
                                 rl->header.stamp);

        // odometry computations
        if (doComp) {
            updateWheels(fr->rpm,
                         fl->rpm,
                         rr->rpm,
                         rl->rpm);
            updateTwist();
            updatePose();
            updateTransform();

            notifyTwist();
            notifyOdometry();
            notifyCustomOdometry();
            notifyTransform();
        }
    }

    // Event notifiers -----------------------------------------------------

    void notifyTwist()
    {
        geometry_msgs::TwistStamped msg;
        msg.header.stamp.sec = t0.sec;
        msg.header.stamp.nsec = t0.nsec;
        msg.twist.linear.x = vX;
        msg.twist.angular.z = wZ;
        pubTws.publish(msg);
    }
    void notifyOdometry()
    {
        nav_msgs::Odometry msg;
        msg.header.stamp.sec = t0.sec;
        msg.header.stamp.nsec = t0.nsec;
        msg.header.frame_id = PARENT_FRAME;
        msg.child_frame_id = CHILD_FRAME;
        msg.twist.twist.linear.x = vX;
        msg.twist.twist.angular.z = wZ;
        msg.pose.pose.position.x = pX;
        msg.pose.pose.position.y = pY;
        msg.pose.pose.orientation.z = tfQ.getZ();
        msg.pose.pose.orientation.w = tfQ.getW();
        pubOdm.publish(msg);
    }
    void notifyCustomOdometry()
    {
        project1::CustomOdometry msg;
        msg.odom.header.stamp.sec = t0.sec;
        msg.odom.header.stamp.nsec = t0.nsec;
        msg.odom.header.frame_id = PARENT_FRAME;
        msg.odom.child_frame_id = CHILD_FRAME;
        msg.odom.twist.twist.linear.x = vX;
        msg.odom.twist.twist.angular.z = wZ;
        msg.odom.pose.pose.position.x = pX;
        msg.odom.pose.pose.position.y = pY;
        msg.odom.pose.pose.orientation.z = tfQ.getZ();
        msg.odom.pose.pose.orientation.w = tfQ.getW();
        msg.method.data = intRK ? "rk" : "euler";
        pubCtm.publish(msg);
    }
    void notifyTransform()
    {
        brdcst.sendTransform(tf::StampedTransform(tfT, t0, PARENT_FRAME, CHILD_FRAME));
    }

    // Robot updates -------------------------------------------------------

    bool updateTime(ros::Time tFR, ros::Time tFL, ros::Time tRR, ros::Time tRL)
    {
        // time = avg(approx sync events times)
        double t = (tFR.toSec() + tFL.toSec() + tRR.toSec() + tRL.toSec()) / 4.0;
        ros::Time t1(t);

        // first update
        if (!started) {

            // init time
            t0.sec = t1.sec;
            t0.nsec = t1.nsec;

            // start
            started = true;
            // skip computations
            return false;
        }

        // delta time
        tD = t1.toSec() - t0.toSec();

        t0.sec = t1.sec;
        t0.nsec = t1.nsec;

        #ifdef LOG_TIME
            ROS_INFO("(TIM) tD: %.2f", tD);
        #endif

        // do computations
        return true;
    }
    void updateWheels(double rpmFR, double rpmFL, double rpmRR, double rpmRL) 
    {
        // ang vel (rad/s)
        wFR = RPM_TO_RADS(rpmFR);
        wFL = RPM_TO_RADS(-rpmFL);
        wRR = RPM_TO_RADS(rpmRR);
        wRL = RPM_TO_RADS(-rpmRL);

        #ifdef LOG_WMOTOR
            ROS_INFO("(MTR) wFR: %.2f, wFL: %.2f, wRR: %.2f, wRL %.2f", wFR, wFL, wRR, wRL);
        #endif

        // lin vel (m/s)
        vFR = W_TO_V(wFR, wheel);
        vFL = W_TO_V(wFL, wheel);
        vRR = W_TO_V(wRR, wheel);
        vRL = W_TO_V(wRL, wheel);

        #ifdef LOG_VMOTOR
            ROS_INFO("(MTR) vFR: %.2f, vFL: %.2f, vRR: %.2f, vRL %.2f", vFR, vFL, vRR, vRL);
        #endif

        // skid steering to equivalent diff drive
        // side speeds = avg(same side) / gb reduction
        vR = (vFR + vRR) / 2.0;
        vL = (vFL + vRL) / 2.0;

        vR = vR / reduction;
        vL = vL / reduction;

        #ifdef LOG_VMOTOR
            ROS_INFO("(MTR) vR: %.2f, vL: %.2f", vR, vL);
        #endif
    }
    void updateTwist()
    {
        vX = DDRIVE_V(vR, vL);
        wZ = DDRIVE_W(vR, vL, baseline);

        #ifdef LOG_TWIST
            ROS_INFO("(TWS) vX: %.2f, wZ: %.2f", vX, wZ);
        #endif
    }
    void updatePose()
    {
        if (intRK)        { INT_RK(tD, pX, pY, aT, vX, wZ); }
        else              { INT_EUL(tD, pX, pY, aT, vX, wZ); }

        #ifdef LOG_POSE
            ROS_INFO("(POS) pX: %.2f, pY: %.2f, aT: %.2f", pX, pY, aT);
        #endif
    }
    void updateTransform()
    {
        tfT.setOrigin(tf::Vector3(pX, pY, 0.0));
        tfQ.setRPY(0.0, 0.0, aT);
        tfT.setRotation(tfQ);
    }

    // Dynamic reconfigure handlers ----------------------------------------

    void onIntMethod(project1::IntegrationMethodConfig &config, uint32_t level) {
        intRK = config.intMethod;

        #ifdef LOG_PARAMS
            if (intRK)
                ROS_INFO("(PRM) integration method: Runge-Kutta");
            else
                ROS_INFO("(PRM) integration method: Euler");
        #endif
    }

    // Service handlers ----------------------------------------------------

    bool resetPose(project1::ResetPose::Request &req, project1::ResetPose::Response &res)
    {
        pX = 0.0;
        pY = 0.0;
        aT = 0.0;

        updateTransform();

        notifyOdometry();
        notifyCustomOdometry();
        notifyTransform();

        return true;
    }
    bool setPose(project1::SetPose::Request &req, project1::SetPose::Response &res)
    {
        pX = req.x;
        pY = req.y;
        aT = req.theta;

        updateTransform();

        notifyOdometry();
        notifyCustomOdometry();
        notifyTransform();

        return true;
    }
};

//==========================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    Robot node;
    ros::spin();
    return 0;
}