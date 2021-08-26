
#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class QuadrotorDynamicsPlugin : public ModelPlugin
  {
    public: 
        QuadrotorDynamicsPlugin(){}

        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            std::cerr << "\nQuadrotor dynamics plugin attached to " << model->GetName() << std::endl;

            // Save model pointer
            this->model_ = model;

            // Save the main body link
            this->link_ = model->GetLink("link");

            // Create the transport node
            this->node_ = transport::NodePtr(new transport::Node());
            this->node_->Init(this->model_->GetWorld()->Name());

            // Create a topic for simulation
            std::string topic_name = "~/" + this->model_->GetName() + "/motors_out";

            // Subscribe to this topic
            this->sub_ = this->node_->Subscribe(topic_name, &QuadrotorDynamicsPlugin::OnMsg, this);

            // Publish sensor data to topics
            this->imu_pub_ = this->node_->Advertise<gazebo::msgs::Vector3d>("~/" + this->model_->GetName() + "/imu");

            // Connect to world updates
            this->update_conn_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&QuadrotorDynamicsPlugin::OnUpdate, this, _1));
        }

        // TODO: make a custom message for motor throttle
        void OnMsg(ConstVector2dPtr &msg)
        {
            // TODO: Update motor speed setpoints based on message from flight controller
        }

        void OnUpdate(const common::UpdateInfo& info)
        {
            // TODO: remove this
            m = 1.32;   // kg
            g = 9.81;
            // b = 0.0000033;
            b = 1;
            l = 0.1;    // 10cm quadrotor arms?
            d = 0.05;

            motor_max_thrust = 1 * g;

            // Bang bang controller with memory
            ignition::math::Pose3d pose = this->model_->RelativePose();
            double z = pose.Pos().Z();
            double motor_throttle = 0.0;
            if (z >= 2 && z <= 3.5)
            {
                motor_throttle = 0.315;
                state = 2;
            }
            else if ((z > 1.0 && z < 2.0 && state != 2) || z > 3.5)
            {
                motor_throttle = 0.21;
                state = 1;
            }
            else if (z > 1.0 && z < 2.0 && state == 2)
            {
                motor_throttle = 0.6;
            }
            else
            {
                motor_throttle = 0.45;
                state = 0;
            }
            

            std::cout << "Z: " << z << std::endl;

            // simple motor throttle - thrust linear model
            rot1_thrust = motor_throttle * motor_max_thrust;
            rot2_thrust = motor_throttle * motor_max_thrust;
            rot3_thrust = motor_throttle * motor_max_thrust;
            rot4_thrust = motor_throttle * motor_max_thrust;

            // Update the vehicle dynamics
            UpdateDynamics();

            // Publish data to the flight controller
            PublishIMU();
        }

        // Compute the forces and moments to apply to the main body link of the quadrotor
        void UpdateDynamics()
        {
            // Get wind force
            double f_wx = 0.0;
            double f_wy = 0.0;
            double f_wz = 0.0;

            // Get wind torque
            double t_wx = 0.0;
            double t_wy = 0.0;
            double t_wz = 0.0;

            // Get model RPY
            ignition::math::Pose3d pose = this->link_->RelativePose();
            double roll = pose.Roll();
            double pitch = pose.Pitch();
            double yaw = pose.Yaw();

            // Compute the total thrust
            double thrust = b * (rot1_thrust + rot2_thrust + rot3_thrust + rot4_thrust);

            // Compute the angular torques on the body
            double torque_x = 0;//b * l * (rot3_thrust - rot1_thrust);
            double torque_y = 0;//b * l * (rot4_thrust - rot2_thrust);
            double torque_z = 0;//d * (rot2_thrust + rot4_thrust - rot1_thrust - rot3_thrust);

            // Force transformations from world frame to body frame
            double tf_x = sin(pitch);
            double tf_y = cos(pitch) * sin(roll);
            double tf_z = cos(pitch) * cos(roll);

            // Forces in the body frame
            double f_x = 0.0;//m*g*tf_x + f_wx;
            double f_y = 0.0;//-m*g*tf_y + f_wy;
            double f_z = thrust;// - (m*g*tf_z + f_wz);

            // Moments in the body frame
            double m_x = torque_x + t_wx;
            double m_y = torque_y + t_wy;
            double m_z = torque_z + t_wz;

            // Collect forces and moments into two vectors
            const ignition::math::Vector3d forces =
                ignition::math::Vector3d (f_x, f_y, f_z);
            const ignition::math::Vector3d moments =
                ignition::math::Vector3d (m_x, m_y, m_z);

            // Apply forces and torques to model link
            link_->AddRelativeForce(forces);
            link_->AddRelativeTorque(moments);

            std::cout << f_z << std::endl;
        }

        void PublishIMU()
        {
            // Make a message for roll, pitch, yaw
            gazebo::msgs::Vector3d imu_msg;

            // Get model RPY
            ignition::math::Pose3d robot_pose = this->model_->RelativePose();
            
            // Roll
            imu_msg.set_x(robot_pose.Roll());

            // Pitch
            imu_msg.set_y(robot_pose.Pitch());

            // Yaw 
            imu_msg.set_z(robot_pose.Yaw());

            // Publish the message
            imu_pub_->Publish(imu_msg);

        }


    private:
        /// \brief Pointer to the model.
        physics::ModelPtr model_;

        /// \brief Pointer to the model's base link.
        physics::LinkPtr link_;

        /// \brief Transport node and subscriber
        transport::NodePtr node_;
        transport::SubscriberPtr sub_;

        // Publisher and update connection
        event::ConnectionPtr update_conn_;
        transport::PublisherPtr imu_pub_;

        // Motor thrust state
        double motor_max_thrust;
        double rot1_thrust;
        double rot2_thrust;
        double rot3_thrust;
        double rot4_thrust;

        // Quadrotor dynamical constants
        double b;       // thrust factor
        double d;       // drag factor
        double l;       // quadrotor arm length (m)

        double g;       // gravity
        double m;       // mass
        double I_xx;    // I_x
        double I_yy;    // I_y
        double I_zz;    // I_z

        // Motor constants
        // Linear model of motor speed based on throttle input
        double K_motor; // Slope for linear model of motor speed
        double b_motor; // y-intercept for linear model of motor speed

        // example controller
        int state;

  };
  GZ_REGISTER_MODEL_PLUGIN(QuadrotorDynamicsPlugin)
}