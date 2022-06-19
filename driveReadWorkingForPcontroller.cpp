#include<ros/ros.h>
 #include<sensor_msgs/LaserScan.h>
 #include<geometry_msgs/Twist.h>
#include<ctime>
 #include<tf2_ros/transform_listener.h>
 #include<geometry_msgs/TransformStamped.h>
 #include<geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>


    ros::Publisher _pub;
    ros::Subscriber _sub;
    geometry_msgs::Twist _vel;
    sensor_msgs::LaserScan _lsval;
	tf2_ros::Buffer tfBuffer;

	
	float minF = 100;
	float minL = 100;
	float minR = 100;
	float dThresh = .4;
	float speedF = 0.25;
	float speedT = 0.5;
	time_t curtime;
	
	float angleGive = 1.5;
	float xgoal = -1;
	float ygoal = -1;
	float anglediff = 0;
	float xdiff = 0;
	float ydiff = 0;
	float distdiff = 0;
    float anglesign = 0;
    
    float forSpeed;
    float rotSpeed;
    //Below are motion functions, called by their direction to simplify code in the main navigation
    void Forward(){
		
		ros::Rate rate(20);
		
		//Set the linear speed in the x to the speed value assigned above
		_vel.linear.x=speedF;
		
		//Set the angular speed to 0 so it stops trying to turn while driving forward
		_vel.angular.z = 0;
		
		//Publish the velocity commands to the robot
		_pub.publish(_vel);
		
		// Print how the robot is moving for validation 
		ROS_INFO("FORWARD");
	}
	
	void MoveIt(float speedForward, float rotSpeed){
				ros::Rate rate(20);
		
		//Set the linear speed in the x to the speed value assigned above
		_vel.linear.x=speedForward;
		
		//Set the angular speed to 0 so it stops trying to turn while driving forward
		_vel.angular.z = rotSpeed;
		
		//Publish the velocity commands to the robot
		_pub.publish(_vel);
		
		// Print how the robot is moving for validation 
		ROS_INFO("FORWARD");
	}
	
	void Back(){
		
		ros::Rate rate(20);
		_vel.linear.x=-speedF;
		_vel.angular.z = 0;
		_pub.publish(_vel);
		ROS_INFO("BACK");
	}
	void Left(){
		ros::Rate rate(20);
		_vel.angular.z = speedT;
		//_vel.linear.x = 0;
		_pub.publish(_vel);
		ROS_INFO("LEFT");
	}
	void Right(){
		ros::Rate rate(20);
		_vel.angular.z = -speedT;
		//_vel.linear.x = 0;
		_pub.publish(_vel);
		ROS_INFO("RIGHT");
	}
    
    
    
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
		   geometry_msgs::TransformStamped transformStamped;
		   geometry_msgs::Transform transform;
		   double x, y, z, w, t3, t4;
		   geometry_msgs::Pose2D robot_pose;
		   try
		   {
			   //tfBuffer.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));

			   transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
			   transform = transformStamped.transform;
			   x = transform.rotation.x;
			   y = transform.rotation.y;
			   z = transform.rotation.z;
			   w = transform.rotation.w;

			   t3 = 2.0*(w*z + x*y);
			   t4 = 1.0 - 2.0 * (y*y + z*z);
			   robot_pose.theta = atan2(t3, t4); 	 
			   robot_pose.x = transform.translation.x;
			   robot_pose.y = transform.translation.y;
			   _pub.publish(robot_pose);

			   ROS_INFO("Robot Pose (%f, %f, %f)",  robot_pose.x, robot_pose.y, robot_pose.theta);


		   } catch (tf2::TransformException &ex) {
			   ROS_ERROR("%s",ex.what());
			   return;
		   }
			
		
		
		
		
		
		
		
		//pulling current time at start of loop, not used but here for future implementation
		curtime = time(NULL);
		
		//minimum values reset to number it will never reach (forgot name of this)
		minF = 100;
		minL = 100;
		minR = 100;

		//Collecting range data and finding the minimum distance
		
		for (int i=80;i<100;i++){
			if(!isinf(msg->ranges[i]) and msg->ranges[i] < minF){
					minF = msg->ranges[i];
			}
		}
		for (int i=50;i<80;i++){
			if(!isinf(msg->ranges[i]) and msg->ranges[i] < minR){
					minR = msg->ranges[i];
			}
		}
		for (int i=100;i<130;i++){
			if(!isinf(msg->ranges[i]) and msg->ranges[i] < minL){
					minL = msg->ranges[i];
			}
		}
		
		//Output minimum values for testing
		
		//ROS_INFO("L[%f]: F[%f]: R%f]", minL, minF, minR);

		//Wall Following Logic
		xdiff = xgoal - robot_pose.x;
		ydiff = ygoal - robot_pose.y;
		
		angleGive = atan2(ydiff,xdiff);
		ROS_INFO("Given[%f] [%f] [%f]",angleGive, xdiff, ydiff);

		/*
		if (angleGive < 0){
			angleGive = angleGive + 6.283;
		}*/
		anglediff = robot_pose.theta - angleGive;
		
		anglesign = abs(anglediff)/anglediff;
		ROS_INFO("sign[%f]",anglesign);
		
		/*
		if(anglediff > 3.14){
			anglediff = anglediff - 3.14;
		}
		
		if(anglediff < -3.14){
			anglediff = anglediff + 3.14;
		}*/
		ROS_INFO("Diff[%f]",anglediff);

		if (abs(anglediff) < 0.05){
			//rotSpeed = -25*pow(anglediff,2)*anglesign;
			//rotSpeed = -3.162*pow(abs(anglediff),1.5)*anglesign;
			rotSpeed = -10*anglediff;
			forSpeed = 0.2;
			
			
			

		}else{
			rotSpeed = -1*0.5*anglesign;
			forSpeed = 0;
		}
		distdiff = pow(2.0,3.0);
		ROS_INFO("A[%f]",rotSpeed);
		/*
		distdiff = sqrt(pow((xgoal - robot_pose.x),2)-pow((ygoal-robot_pose.y),2));
		
		if (distdiff < 0.2){
			forSpeed = distdiff;
		}
		else{
			forSpeed = 0.25;
		}
		*/
		
		MoveIt(forSpeed,rotSpeed);
		
		
		/*
		if(minF < dThresh){
			if(minR < dThresh){
			Left();
			}
			else if(minL < dThresh){
			Right();
			}
			else {
				Right();
			}
		}
		else{
			
			if (minR < dThresh){
				Left();
			}
			else if( minL < dThresh){
				Right();
			}
			else{
				Forward();
			}
		}*/
}

int main(int argc, char** argv)
{
		ros::init(argc, argv,"rplidar_node_client");
        ros::NodeHandle _n;
		ros::Rate rate(10.0);


		_pub=_n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

		tf2_ros::TransformListener tfListener(tfBuffer);
		_sub=_n.subscribe("/scan",100,laser_callback);
    
    
    
	ros::spin();
	
return 0;
}
