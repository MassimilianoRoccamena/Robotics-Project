#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "project1/Correction.h"

// Logging ------------------------------------------------------------------

#define LOG_PARAMS
//#define LOG_TIME
//#define LOG_CALIB
#define LOG_RESULT

// Queues -------------------------------------------------------------------

#define PRJ_QUEUE      20
#define MNF_QUEUE      20
#define SYNC_QUEUE     20
#define TARGET_QUEUE   20

// Synchronizations ---------------------------------------------------------

typedef message_filters::sync_policies
        ::ApproximateTime<nav_msgs::Odometry,
                          nav_msgs::Odometry> OdomsSyncPolicy;

// Names --------------------------------------------------------------------

#define NODE_NAME                   "calibration"

// Calculus -----------------------------------------------------------------

#define SAFE_DENOM                  0.01        // min den for calib var to be updated

#define V_MAG(x, y)                 (sqrt((pow(x, 2) + pow(y, 2))))

#define AVG(cum, cnt)               (cum / cnt)
#define EMA(past, now, f)           ((1-f) * past + f * now)

//===========================================================================

class Calibration
{
    // ROS communications ---------------------------------------------------

    private:

    ros::NodeHandle handle;

    message_filters::Subscriber<nav_msgs::Odometry> subPrj, subMnf;
    message_filters::Synchronizer<OdomsSyncPolicy> sync;

    ros::Publisher pubGbr, pubBln;

    // Calibration parameters -----------------------------------------------

    double duration;
    double forget;

    // Computations state ---------------------------------------------------

    bool started, running;

    // Time -----------------------------------------------------------------

    ros::Time t0;
    double tE;

    // Corrections computations ---------------------------------------------

    double cumGbr, cumBln;
    unsigned int cntGbr, cntBln;
    double avgGbr, avgBln;
    double emaGbr, emaBln;

    bool dirty;

    // Ctors ----------------------------------------------------------------

    public:

    Calibration() : subPrj(handle, "/scout/odom/basic", PRJ_QUEUE),
                    subMnf(handle, "/scout_odom", MNF_QUEUE),
                    sync(OdomsSyncPolicy(SYNC_QUEUE), subPrj, subMnf)
    {
        // duration param
        if (!handle.getParam("/calib/duration", duration))
            ROS_ERROR("duration parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) calibration duration: %.4f", duration);
        #endif

        // forget param
        if (!handle.getParam("/calib/forget", forget))
            ROS_ERROR("forget parameter not found");

        #ifdef LOG_PARAMS
            ROS_INFO("(PRM) ema coefficient: %.4f", forget);
        #endif

        // node state
        started = false;
        running = true;

        // elapsed time
        tE = 0.0;

        // corrections vars
        cumGbr = 0.0;
        cumBln = 0.0;
        cntGbr = 0;
        cntBln = 0;
        avgGbr = 0.0;
        avgBln = 0.0;
        emaGbr = 0.0;
        emaBln = 0.0;

        dirty = false;

        // synchro subs
        sync.registerCallback(boost::bind(&Calibration::onOdometries, this, _1, _2));

        // target pubs
        pubGbr = handle.advertise<project1::Correction>("/calib/reduction", TARGET_QUEUE);
        pubBln = handle.advertise<project1::Correction>("/calib/baseline", TARGET_QUEUE);
    }

    // Event handlers -------------------------------------------------------

    void onOdometries(const nav_msgs::Odometry::ConstPtr& prj, 
                      const nav_msgs::Odometry::ConstPtr& mnf)
    {
        // handle time
        bool doComp = updateTime(prj->header.stamp,
                                 mnf->header.stamp);
        
        // corrections computations
        if (doComp) {
            update(prj->twist.twist.linear.x,
                   prj->twist.twist.linear.y,
                   prj->twist.twist.angular.z,

                   mnf->twist.twist.linear.x,
                   mnf->twist.twist.linear.y,
                   mnf->twist.twist.angular.z);

            if (dirty) {
                notifyReduction();
                notifyBaseline();
            }

            dirty = false;
        }
    }

    // Event notifiers -----------------------------------------------------

    void notifyReduction()
    {
        project1::Correction msg;
        msg.header.stamp = ros::Time::now();
        msg.avg = avgGbr;
        msg.ema = emaGbr;
        pubGbr.publish(msg);
    }
    void notifyBaseline()
    {
        project1::Correction msg;
        msg.header.stamp = ros::Time::now();
        msg.avg = avgBln;
        msg.ema = emaBln;
        pubBln.publish(msg);
    }

    // Calibration updates -------------------------------------------------

    bool updateTime(ros::Time tPrj, ros::Time tMnf)
    {
        // skip if stopped
        if (!running) return false;

        // time = avg(approx sync events times)
        double t = (tPrj.toSec() + tMnf.toSec()) / 2.0;
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

        // elapsed time
        tE = t1.toSec() - t0.toSec();

        #ifdef LOG_TIME
            ROS_INFO("(TIM) tE: %.2f", tE);
        #endif

        // do computations
        return true;
    }
    void update(double xPRJ, double yPRJ, double zPRJ, double xMNF, double yMNF, double zMNF)
    {
        double vMNF = V_MAG(xMNF, yMNF);
        if (vMNF > SAFE_DENOM && zMNF > SAFE_DENOM) {

            // gearbox reduction
            double vPRJ = V_MAG(xPRJ, yPRJ);
            double newGbr = vPRJ / vMNF;

            cumGbr += newGbr;
            cntGbr += 1;
            avgGbr = AVG(cumGbr, cntGbr);
            emaGbr = EMA(avgGbr, newGbr, forget);
            
            // baseline
            double newBln = zPRJ / zMNF;
            newBln /= newGbr;

            cumBln += newBln;
            cntBln += 1;
            avgBln = AVG(cumBln, cntBln);
            emaBln = EMA(avgBln, newBln, forget);

            dirty = true;
        }

        #ifdef LOG_CALIB
            if (dirty) {
                ROS_INFO("(AVG) gbr: %.2f, bln: %.2f", avgGbr, avgBln);
                ROS_INFO("(EMA) gbr: %.2f, bln: %.2f", emaGbr, emaBln);
            }
        #endif

        // stop calib
        if (tE >= duration) {
            #ifdef LOG_RESULT
                ROS_INFO("(AVG) gbr: %.4f, bln: %.4f", avgGbr, avgBln);
                ROS_INFO("(EMA) gbr: %.4f, bln: %.4f", emaGbr, emaBln);
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