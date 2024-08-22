#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <eigen3/Eigen/Dense>
#include "include/panda_kinematics.h"
#include <vector>
#include <iostream>

using namespace std;





class PandaArmController {
public:
    PandaArmController(ros::NodeHandle& nh, const std::string& jointTopic, const std::vector<std::string>& jointNames, const std::vector<std::string>& fkJoints)
        : node(nh), FK_Joints(fkJoints) {

        jointPublisher = node.advertise<sensor_msgs::JointState>(jointTopic, 1000);
        jointMsg.name = jointNames;
        jointMsg.velocity = {};
        jointMsg.effort = {};
        
        baseTargetToRightHandTf = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
        tfBroadcaster.sendTransform(tf::StampedTransform(baseTargetToRightHandTf, ros::Time::now(), "/panda_1_link0", "/baseLinkToRightHand"));
        
        targetIKvec.resize(7);
        targetIKvec2.resize(6);
        
        PandaKinematics::setup(panda);
    }

    void run(double frequency) {
        ros::Rate rate(frequency);
        while (node.ok()) {
            if (getBaseTargetToRightHandTf() == 0) {
                computeIK();
                publishJointStates();
            }
            rate.sleep();
        }
    }

private:
    ros::NodeHandle& node;
     ros::Publisher jointPublisher;




    sensor_msgs::JointState jointMsg;
    std::vector<std::string> FK_Joints;
    tf::TransformListener tfListener;
    tf::Transform baseTargetToRightHandTf;
    tf::TransformBroadcaster tfBroadcaster;





    Kinematics panda;
    std::vector<double> targetIKvec;
    std::vector<double> targetIKvec2;

    int getBaseTargetToRightHandTf() {
        tf::Transform baseSourceToRightHandTf;
        std::vector<tf::Transform> transformVector;

        int pass = 0;
        for (size_t i = 0; i < FK_Joints.size() - 1; ++i) {
            pass += addToTransVector(FK_Joints[i], FK_Joints[i + 1], transformVector);
            if (pass != 0) {
                ROS_ERROR("tf not found found from base  to LeftHand");
                return -1;
            }
        }

        baseSourceToRightHandTf = transformVector[0];
        for (size_t i = 1; i < transformVector.size(); ++i) {
            baseSourceToRightHandTf *= transformVector[i];
        }

        transformVector.clear();
        pass += addToTransVector("/panda_1_link0", "/world", transformVector);
        pass += addToTransVector("/world", "/base_link", transformVector);

        if (pass != 0) {
            ROS_ERROR(" tf not found from base to RightHand.");
            return -1;
        }

        baseTargetToRightHandTf = transformVector[0].inverseTimes(transformVector[1] * baseSourceToRightHandTf);

        // baseTargetToRightHandTf = transformVector[0] * transformVector[1] * baseSourceToRightHandTf;
        tfBroadcaster.sendTransform(tf::StampedTransform(baseTargetToRightHandTf, ros::Time::now(), "/panda_1_link0", "/baseLinkToRightHand"));
        return 0;
    }

    int addToTransVector(const std::string& sourceFrame, const std::string& targetFrame, std::vector<tf::Transform>& transVector) {
        tf::StampedTransform transform;
        try {
            tfListener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return -1;
        }
        transVector.push_back(tf::Transform(transform.getRotation(), transform.getOrigin()));
        return 0;
    }

    void computeIK() {
        targetIKvec2[0] = baseTargetToRightHandTf.getOrigin().getX();
        targetIKvec2[1] = baseTargetToRightHandTf.getOrigin().getY();
        targetIKvec2[2] = baseTargetToRightHandTf.getOrigin().getZ();

        Eigen::Quaterniond Q(
            baseTargetToRightHandTf.getRotation().getW(),
            baseTargetToRightHandTf.getRotation().getX(),
            baseTargetToRightHandTf.getRotation().getY(),
            baseTargetToRightHandTf.getRotation().getZ());

        Eigen::AngleAxisd aa(Q);
        Eigen::Vector3d norm = aa.axis() * aa.angle();

        for (int i = 0; i < 3; ++i) {
            targetIKvec2[3 + i] = norm[i];
        }

        Eigen::Map<Eigen::VectorXd> x(targetIKvec2.data(), targetIKvec2.size());
        Eigen::VectorXd q = panda.xToQ(x, 0);

        for (int i = 0; i < 7; ++i) {
            targetIKvec[i] = q[i];
        }
    }

    void publishJointStates() {
        jointMsg.header.stamp = ros::Time::now();
        jointMsg.position = targetIKvec;
        jointPublisher.publish(jointMsg);
        ROS_INFO_STREAM("Published Position: \n"
    << jointMsg.position[0] << "\n"
    << jointMsg.position[1] << "\n"
    << jointMsg.position[2] << "\n"
    << jointMsg.position[3] << "\n"
    << jointMsg.position[4] << "\n"
    << jointMsg.position[5] << "\n"
    << jointMsg.position[6]);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_right_hand");

    ros::NodeHandle node;


      std::vector<std::string> jointNames = {"panda_1_joint1", "panda_1_joint2", "panda_1_joint3", "panda_1_joint4", "panda_1_joint5", "panda_1_joint6", "panda_1_joint7"};
     std::vector<std::string> fkJoints = {"base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "ee_link"};

    PandaArmController pandaArm(node, "/panda_joint_states_right", jointNames, fkJoints);

    ros::Duration(2).sleep();
        printf("\nEnter any key to start moving: ");
    char response;
    cin >> response;
   //     pass the rate  
    pandaArm.run(60.0);

    ros::shutdown();
    return 0;
}

