#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
double follower_x, follower_y, follower_t;
double master_x, master_y, master_t;

class Follow{
	private:
        //master
		ros::Subscriber masterGetPose_; //leitura odom
        geometry_msgs::Pose2D masterPose_;

        //follower
        ros::Subscriber followerGetPose_; //leitura odom
        geometry_msgs::Pose2D followerPose_;

        //control
        ros::Publisher FollowerControl_;
 
	public:
		Follow(ros::NodeHandle nh){
			masterGetPose_ = nh.subscribe("tb3_0/odom", 100, &Follow::getMasterPose, this);
			followerGetPose_ = nh.subscribe("tb3_1/odom", 100, &Follow::getFollowerPose, this);
            FollowerControl_ = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel", 100);
   		}
           
		void getMasterPose(const nav_msgs::Odometry::ConstPtr msg){
			geometry_msgs::Pose2D pose2d;
			pose2d.x = msg->pose.pose.position.x;
			pose2d.y = msg->pose.pose.position.y;

			tf::Quaternion q(
					msg->pose.pose.orientation.x,
					msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z,
					msg->pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			pose2d.theta = yaw;
            master_t = pose2d.theta;

            master_x = pose2d.x;
            master_y = pose2d.y;

			ROS_INFO("[Master] x: %.2lf, y: %.2lf, theta: %.2lf", pose2d.x, pose2d.y, pose2d.theta);
		}

        void getFollowerPose(const nav_msgs::Odometry::ConstPtr msg){
			geometry_msgs::Pose2D pose2d;
			pose2d.x = msg->pose.pose.position.x;
			pose2d.y = msg->pose.pose.position.y;

			tf::Quaternion q(
					msg->pose.pose.orientation.x,
					msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z,
					msg->pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			pose2d.theta = yaw;
            follower_t = pose2d.theta;

            follower_x = pose2d.x;
            follower_y = pose2d.y;

			ROS_INFO("[Follower] x: %.2lf, y: %.2lf, theta: %.2lf", pose2d.x, pose2d.y, pose2d.theta);
		}

        void moveToTarget()
        {
            //control
            geometry_msgs::Twist command;

            double vk_p, v_t, k_pa, k_pl;
            double maxLinearVel, maxAngVel;
            double omega = 0, theta_r, error, ang;

            k_pl = 0.3; 
            k_pa = 6;

            //p n haver escorregamento das rodas, padrões definidos pelo robô
            maxLinearVel  = 0.22;
            maxAngVel     = 2.84;

            error = sqrt (
                            pow(master_x - follower_x, 2) +   
                            pow(master_y - follower_y, 2) 
                         );
            v_t = k_pl * error;  

            //linear
            if(v_t > maxLinearVel)
                v_t = maxLinearVel;

            //n bater quando ficar próximo de 25 cm
            if(error < 0.25)
                command.linear.x = 0.0;               
            else 
                command.linear.x = v_t;

            //angular
            if((master_x - follower_x) != 0)
                ang = (master_y - follower_y) / (master_x - follower_x);
            else        
                ang = 0;

            theta_r = atan(ang);
            omega = k_pa * (theta_r - follower_t);

            if(abs(omega) > maxAngVel)
            {
                if (omega < 0)
                    omega = -maxAngVel;
                else 
                    omega = maxAngVel;
            }

            command.angular.z = omega;     

            FollowerControl_.publish(command);

            ROS_INFO("[Follower] Linear Control %.2f, Angular Control %.2f", v_t, omega);
        }
};

int main(int argc, char** argv){
	ros::init(argc,argv,"tb3_follow_node");
	ros::NodeHandle nh;
	Follow follow(nh);
    ros::Rate loop_rate(1);

	while(ros::ok()){
		ros::spinOnce();
        follow.moveToTarget();
        loop_rate.sleep();
	}
	return 0;
}
