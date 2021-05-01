#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "project1/DiffDriveSpeeds.h"

// Logging ------------------------------------------------------------------

#define LOG_PARAMS
//#define LOG_TIME
//#define LOG_CALIB
#define LOG_RESULT

// Queues -------------------------------------------------------------------

#define WHEELS_QUEUE   20
#define ODOM_QUEUE     20
#define SYNC_QUEUE     20
#define TARGET_QUEUE   20

// Synchronizations ---------------------------------------------------------

typedef message_filters::sync_policies
        ::ApproximateTime<project1::DiffDriveSpeeds,
                          nav_msgs::Odometry> CalibSyncPolicy;

// Names --------------------------------------------------------------------

#define NODE_NAME               "calibration"

// Calculus -----------------------------------------------------------------

#define V_MAG(x, y)                 (sqrt((pow(x, 2) + pow(y, 2))))

#define DDRIVE_V(r, l)              ((r + l) / 2.0)
#define DDRIVE_GBR(r, l, v)         (DDRIVE_V(r, l) / v)
#define DDRIVE_BLN(r, l, gbr, w)    ((r - l) / gbr / w)

#define SAFE_DENOM                  0.01                        // min den for calib var to be updated

//===========================================================================

class Calibration
{
    // ROS communications ---------------------------------------------------

    private:

    ros::NodeHandle handle;

    message_filters::Subscriber<project1::DiffDriveSpeeds> subWhl;
    message_filters::Subscriber<nav_msgs::Odometry> subOdm;
    message_filters::Synchronizer<CalibSyncPolicy> sync;

    ros::Publisher pubGbr;
    ros::Publisher pubBln;

    // Calibration parameters -----------------------------------------------

    double duration;

    // Computations state ---------------------------------------------------

    bool started, running;

    // Time -----------------------------------------------------------------

    double t0, tE;

    // Calibration computations ---------------------------------------------

    double cumGbr, cumBln;
    unsigned int cntGbr, cntBln;
    double avgGbr, avgBln;

    bool dirtyGbr, dirtyBln;

    // Ctors ----------------------------------------------------------------

    public:

    Calibration() : subWhl(handle, "/scout/wheels", WHEELS_QUEUE),
                    subOdm(handle, "/scout_odom", ODOM_QUEUE),
                    sync(CalibSyncPolicy(SYNC_QUEUE), subWhl, subOdm)
    {
        // calib duration param
        if (!handle.getParam("/calib/duration", duration))
            ROS_ERROR("duration parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) calibration duration: %.4f", duration);
        #endif

        // node state
        started = false;
        running = true;

        // elapsed time
        tE = 0.0;

        // calib vars
        cumGbr = 0.0;
        cumBln = 0.0;
        cntGbr = 0;
        cntBln = 0;
        avgGbr = 0.0;
        avgBln = 0.0;

        dirtyGbr = false;
        dirtyBln = false;

        // synchro subs
        sync.registerCallback(boost::bind(&Calibration::onCalibration, this, _1, _2));

        // target pubs
        pubGbr = handle.advertise<std_msgs::Float64>("/calib/reduction", TARGET_QUEUE);
        pubBln = handle.advertise<std_msgs::Float64>("/calib/baseline", TARGET_QUEUE);
    }

    // Event handlers -------------------------------------------------------

    void onCalibration(const project1::DiffDriveSpeeds::ConstPtr& whl, 
                       const nav_msgs::Odometry::ConstPtr& odm)
    {
        // handle time
        bool doComp = updateTime(whl->header.stamp,
                                 odm->header.stamp);
        
        // errors computations
        if (doComp) {
            update(whl->vR,                         // wheels
                   whl->vL,

                   odm->twist.twist.linear.x,       // twist
                   odm->twist.twist.linear.y,
                   odm->twist.twist.angular.z);

            notify();
        }
    }

    // Event notifiers -----------------------------------------------------

    void notify()
    {
        // send msgs
        std_msgs::Float64 msgGbr, msgBln;

        if (dirtyGbr) {
            msgGbr.data = avgGbr;
            pubGbr.publish(msgGbr);
        }
        if (dirtyBln) {
            msgBln.data = avgBln;
            pubBln.publish(msgBln);
        }

        // clean vars
        dirtyGbr = false;
        dirtyBln = false;
    }

    // Calibration updates -------------------------------------------------

    bool updateTime(ros::Time tPrj, ros::Time tMnf)
    {
        // skip if stopped
        if (!running) return false;

        // time = avg(approx sync events times)
        double t1 = (tPrj.toSec() + tMnf.toSec()) / 2.0;

        // first update
        if (!started) {

            // init time
            t0 = t1;

            // start
            started = true;
            // skip computations
            return false;
        }

        // elapsed time
        tE = t1 - t0;

        #ifdef LOG_TIME
            ROS_INFO("(TIM) tE: %.2f", tE);
        #endif

        // do computations
        return true;
    }
    void update(double vR, double vL, double vX, double vY, double wZ)
    {
        // cumulate, count vars for avg
        // on safe div condition && not neg estimation
        double v = V_MAG(vX, vY);
        if (v > SAFE_DENOM) {
            double newGbr = DDRIVE_GBR(vR, vL, v);
            if (newGbr > 0) { 
                cumGbr += newGbr;
                cntGbr += 1;
                avgGbr = cumGbr / cntGbr;

                dirtyGbr = true;
            }
        }

        if (avgGbr * wZ > SAFE_DENOM) {
            double newBln = DDRIVE_BLN(vR, vL, avgGbr, wZ);
            if (newBln > 0) {
                cumBln += newBln;
                cntBln += 1;
                avgBln = cumBln / cntBln;

                dirtyBln = true;
            }
        }

        #ifdef LOG_CALIB
            if (dirtyGbr && dirtyBln)
                ROS_INFO("(CLB) gbr: %.2f, bln: %.2f", avgGbr, avgBln);
            else if (dirtyGbr)
                ROS_INFO("(CLB) gbr: %.2f", avgGbr);
            else if (dirtyBln)
                ROS_INFO("(CLB) bln: %.2f", avgBln);
        #endif

        // stop calib
        if (tE >= duration) {
            #ifdef LOG_RESULT
                ROS_INFO("(RES) gbr: %.4f, bln: %.4f", avgGbr, avgBln);
            #endif

            running = false;
        }
    }
};

//==========================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    Calibration node;
    ros::spin();
    return 0;
}