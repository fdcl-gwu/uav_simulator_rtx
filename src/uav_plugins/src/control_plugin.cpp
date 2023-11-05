#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include "std_msgs/String.h"
#include <ros/ros.h>

#include "fdcl/common_types.hpp"
#include "fdcl/matrix_utils.hpp"
#include "fdcl/ros_utils.hpp"

namespace igm = ignition::math;

namespace gazebo {

    class UavControlPlugin : public ModelPlugin {

    public:
        /*
         * This function is called when the plugin is loaded into the simulator.
         * The input is the model and sdf.
         * The output is void.
         */
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // print out process id
            std::cout << ">>> process id: " << getpid() << std::endl;

            this->world = _model->GetWorld();
            this->model = _model;

            this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
            std::cout << ">>> bodyName: " << this->link_name << std::endl;
            this->link = _model->GetLink(this->link_name);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->update_connection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&UavControlPlugin::update, this));

            // Start the ROS subscriber.
            this->topic_name = _sdf->GetElement("topicName")->Get<std::string>();
            std::cout << ">>> topic name: " << this->topic_name << std::endl;
            this->sub_fm = this->n.subscribe(
                this->topic_name,               // topic name
                1,                              // queue size
                UavControlPlugin::update_fm     // callback function
            );
        }

        /*
         * This function is called by the world update start event.
         * It is called at every simulation iteration.
         * It is used to update the force and moment applied to the UAV.
         */
        void update(void)
        {
            no_msg_counter++;
            if (no_msg_counter > 100) {
                this->reset_uav();

                if (!print_reset_message) {
                    std::cout << ros::Time::now()
                              << ": no new force messages, resetting UAV .."
                              << std::endl;
                    print_reset_message = true;
                }
                return;
            }

            // Both must be in world frame.
            this->calculate_force();
            this->link->SetForce(this->force);
            this->link->SetTorque(this->M_out);
        }

        /*
         * This function calculates the force and moment applied to the UAV.
         * The force and moment are calculated in the body frame.
         * The force and moment are then transformed to the world frame.
         * The force and moment are then converted to the ignition format.
         * The force and moment are then stored in the class variables. 
         */
        void calculate_force(void)
        {
            this->update_uav_rotation();

            fdcl::Vector3 force_body;
            this->ignition_to_eigen(this->f, force_body);

            fdcl::Vector3 force_world = this->R * force_body;
            this->eigen_to_ignition(force_world, this->force);

            fdcl::Vector3 M_body;
            this->ignition_to_eigen(this->M, M_body);

            fdcl::Vector3 M_world = this->R * M_body;
            this->eigen_to_ignition(M_world, this->M_out);
        }

        /*
         * This function updates the rotation matrix of the UAV.
         * The rotation matrix is stored in the class variable R.
         */
        void update_uav_rotation(void)
        {
            ignition::math::Pose3d pose = this->link->WorldPose();
            ignition::math::Quaterniond q = pose.Rot();

            fdcl::Vector3 q13(q.X(), q.Y(), q.Z());
            double q4 = q.W();

            fdcl::Matrix3 hat_q = fdcl::hat(q13);
            this->R = eye3 + 2 * q4 * hat_q + 2 * hat_q * hat_q;
        }

        /*
         * This function is called when a new force message is received.
         * The force and moment are stored in the class variables.
        */
        static void update_fm(const geometry_msgs::Wrench::ConstPtr &msg)
        {

            f[0] = msg->force.x;
            f[1] = msg->force.y;
            f[2] = msg->force.z;

            M[0] = msg->torque.x;
            M[1] = msg->torque.y;
            M[2] = msg->torque.z;

            no_msg_counter = 0;
            print_reset_message = false;
        }

        /*
         * This function resets the force and moment applied to the UAV.
        */
        void reset_uav(void)
        {
            this->link->SetForce(zero_fM);
            this->link->SetTorque(zero_fM);
        }


        /*
         * This function converts the ignition format to the eigen format.
         * The input is the ignition format.
         * The output is the eigen format.
         */
        void ignition_to_eigen(
            const ignition::math::Vector3d input, fdcl::Vector3 &output)
        {
            output(0) = input[0];
            output(1) = input[1];
            output(2) = input[2];
        }

        void eigen_to_ignition(
            const fdcl::Vector3 input, ignition::math::Vector3d &output)
        {
            output[0] = input(0);
            output[1] = input(1);
            output[2] = input(2);
        }

    private:
        ros::Time t0 = ros::Time::now();

        physics::ModelPtr model;                // Pointer to the model
        physics::WorldPtr world;                // Pointer to the world
        physics::LinkPtr link;                  // Pointer to the link

        std::string link_name;                  // Name of the link
        std::string topic_name;                 // Name of the topic

        event::ConnectionPtr update_connection; // Pointer to the update event connection
        ros::Subscriber sub_fm;                 // ROS subscriber
        ros::NodeHandle n;                      // ROS node handle

        static igm::Vector3d M;                 // Moment (rotational forces) in body frame
        igm::Vector3d M_out;                    // Moment in world frame
        static igm::Vector3d f;                 // Force (translational forces) in body frame

        fdcl::Matrix3 R = fdcl::Matrix3::Identity();
        igm::Vector3d force = igm::Vector3d::Zero;

        static int no_msg_counter;
        static bool print_reset_message;

        const igm::Vector3d zero_fM = igm::Vector3d::Zero;
        const fdcl::Matrix3 eye3 = fdcl::Matrix3::Identity();
    };

    // Register this plugin with the simulator.
    GZ_REGISTER_MODEL_PLUGIN(UavControlPlugin)

} // End of namespace gazebo.

igm::Vector3d gazebo::UavControlPlugin::f = igm::Vector3d::Zero;
igm::Vector3d gazebo::UavControlPlugin::M = igm::Vector3d::Zero;
int gazebo::UavControlPlugin::no_msg_counter = 0;
bool gazebo::UavControlPlugin::print_reset_message = false;
