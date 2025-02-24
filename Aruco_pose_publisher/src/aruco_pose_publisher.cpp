// Libraries
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <thread>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h> // For broadcasting transforms
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2 conversions
#include <cmath> // For std::sqrt

using namespace std;

#define UDP_PORT_MULTIRROTOR_ARUCO 28000

// Structure definition
typedef struct
{
	char header[3];		// "RPP" (Robot Pose Packet) character sequence
    uint8_t updated;
	float pos[3];		// Position of the Aruco marker
	float rot[9];		// Rotation angles of Aruco marker
} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;


// Global variables
int endThreadSignal = 0;
int validArucoPose = 0;
tf2::Vector3  arucoMarkerPosition;

tf2::Transform world_to_aruco;
tf2::Transform aruco_to_camera;
tf2::Transform camera_to_drone;

geometry_msgs::PoseStamped current_pose;

ros::Publisher pose_pub;  // Declare publisher globally

// Thread function declaration
void receiveArucoMRPoseThread();
void keyboardCommandThread();

tf2::Quaternion quaternionFromRotationMatrix(const tf2::Matrix3x3& m) {
    double tr = m[0][0] + m[1][1] + m[2][2]; // Trace of the matrix
    double S = 0.0;

    tf2::Quaternion q;

    if (tr > 0) {
        S = std::sqrt(tr + 1.0) * 2; // S=4w
        q.setW((tr + 1.0) / S);
        q.setX((m[2][1] - m[1][2]) / S);
        q.setY((m[0][2] - m[2][0]) / S);
        q.setZ((m[1][0] - m[0][1]) / S);
    } else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
        S = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4x
        q.setW((m[2][1] - m[1][2]) / S);
        q.setX((m[0][0] - m[1][1] - m[2][2] + 1.0) / S);
        q.setY((m[0][1] + m[1][0]) / S);
        q.setZ((m[0][2] + m[2][0]) / S);
    } else if (m[1][1] > m[2][2]) {
        S = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4y
        q.setW((m[0][2] - m[2][0]) / S);
        q.setX((m[0][1] + m[1][0]) / S);
        q.setY((m[1][1] - m[0][0] - m[2][2] + 1.0) / S);
        q.setZ((m[1][2] + m[2][1]) / S);
    } else {
        S = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4z
        q.setW((m[1][0] - m[0][1]) / S);
        q.setX((m[0][2] + m[2][0]) / S);
        q.setY((m[1][2] + m[2][1]) / S);
        q.setZ((m[2][2] - m[0][0] - m[1][1] + 1.0) / S);
    }

    q.normalize(); // Important: Normalize the quaternion!
    return q;
}

void publishTransforms()
{

}

void computeCurrentPose()
{
    tf2::Transform world_to_drone = world_to_aruco * aruco_to_camera * camera_to_drone;

    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = world_to_drone.getOrigin().x();
    current_pose.pose.position.y = world_to_drone.getOrigin().y();
    current_pose.pose.position.z = world_to_drone.getOrigin().z();
    tf2::Quaternion q = world_to_drone.getRotation();
    current_pose.pose.orientation.x = q.x();
    current_pose.pose.orientation.y = q.y();
    current_pose.pose.orientation.z = q.z();
    current_pose.pose.orientation.w = q.w();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher_node");
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;

    thread arucoMarkerThread;
    thread keyboardThread;

    // Publisher to send the current position
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    
    // Initialize camera to drone transform
    camera_to_drone.setOrigin(tf2::Vector3(0.0, 0.1, -0.3));
    tf2::Quaternion q;
    q.setRPY(0.0, -M_PI/2, M_PI/2); 
    camera_to_drone.setRotation(q);
    
	world_to_aruco.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); // Set fixed position (adjust to your setup)
	tf2::Quaternion q_fixed;
	q_fixed.setRPY(M_PI/2, 0, -M_PI/2);
	world_to_aruco.setRotation(q_fixed);

    // Initialize the global position in case is not received
    current_pose.header.stamp = ros::Time::now(); // Initialize the timestamp
    current_pose.header.frame_id = "map";         // Initialize the frame ID
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = 1;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = 0;
    current_pose.pose.orientation.w = 1;
    current_pose.header.stamp = ros::Time::now();    

	// Create the thread for receiving Aruco marker pose through UDP socket
	arucoMarkerThread = thread(&receiveArucoMRPoseThread);
    arucoMarkerThread.detach();
    keyboardThread = thread(&keyboardCommandThread);
    keyboardThread.detach();

    ros::Rate loop_rate(50);
    
    // *** THE KEY CHANGE: Wait for initial pose data ***
    while (ros::ok() && !validArucoPose && endThreadSignal == 0) {
        ros::spinOnce();  // Process incoming messages (including Aruco data)
        usleep(10000); // Small delay to avoid busy-waiting
    }

    if (endThreadSignal == 1) return 0; // Check if the user wants to exit
    
    while (ros::ok() && endThreadSignal == 0)
    {   
		geometry_msgs::TransformStamped transformStamped_world_to_aruco;
		geometry_msgs::TransformStamped transformStamped_aruco_to_camera;
		geometry_msgs::TransformStamped transformStamped_camera_to_drone;
		ros::Time now = ros::Time::now();

		// Publish world -> aruco
		transformStamped_world_to_aruco.header.stamp = now;
		transformStamped_world_to_aruco.header.frame_id = "map";
		transformStamped_world_to_aruco.child_frame_id = "aruco";
		transformStamped_world_to_aruco.transform.translation.x = world_to_aruco.getOrigin().x();
		transformStamped_world_to_aruco.transform.translation.y = world_to_aruco.getOrigin().y();
		transformStamped_world_to_aruco.transform.translation.z = world_to_aruco.getOrigin().z();
		tf2::Quaternion q = world_to_aruco.getRotation();
		transformStamped_world_to_aruco.transform.rotation.x = q.x();
		transformStamped_world_to_aruco.transform.rotation.y = q.y();
		transformStamped_world_to_aruco.transform.rotation.z = q.z();
		transformStamped_world_to_aruco.transform.rotation.w = q.w();
		br.sendTransform(transformStamped_world_to_aruco);

		// Publish aruco -> camera
		transformStamped_aruco_to_camera.header.stamp = now;
		transformStamped_aruco_to_camera.header.frame_id = "aruco";
		transformStamped_aruco_to_camera.child_frame_id = "camera";
		transformStamped_aruco_to_camera.transform.translation.x = aruco_to_camera.getOrigin().x();
		transformStamped_aruco_to_camera.transform.translation.y = aruco_to_camera.getOrigin().y();
		transformStamped_aruco_to_camera.transform.translation.z = aruco_to_camera.getOrigin().z();
		q = aruco_to_camera.getRotation();
		transformStamped_aruco_to_camera.transform.rotation.x = q.x();
		transformStamped_aruco_to_camera.transform.rotation.y = q.y();
		transformStamped_aruco_to_camera.transform.rotation.z = q.z();
		transformStamped_aruco_to_camera.transform.rotation.w = q.w();
		br.sendTransform(transformStamped_aruco_to_camera);

		// Publish camera -> drone
		transformStamped_camera_to_drone.header.stamp = now;
		transformStamped_camera_to_drone.header.frame_id = "camera";
		transformStamped_camera_to_drone.child_frame_id = "drone";
		transformStamped_camera_to_drone.transform.translation.x = camera_to_drone.getOrigin().x();
		transformStamped_camera_to_drone.transform.translation.y = camera_to_drone.getOrigin().y();
		transformStamped_camera_to_drone.transform.translation.z = camera_to_drone.getOrigin().z();
		q = camera_to_drone.getRotation();
		transformStamped_camera_to_drone.transform.rotation.x = q.x();
		transformStamped_camera_to_drone.transform.rotation.y = q.y();
		transformStamped_camera_to_drone.transform.rotation.z = q.z();
		transformStamped_camera_to_drone.transform.rotation.w = q.w();
		br.sendTransform(transformStamped_camera_to_drone);
        
        computeCurrentPose();
        pose_pub.publish(current_pose);
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    endThreadSignal = 1;
    usleep(100000);
    cout << "Terminated ctrl+c" << endl;

    return 0;
}

/*
 * Thread for receiving Aruco Data
 */
void receiveArucoMRPoseThread()
{
	DATA_PACKET_ROBOT_POSE * dataPacketRobotPose;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	int k = 0;
    double theta = 0.0;
	int errorCode = 0;


	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		errorCode = 1;
		cout << endl << "ERROR: could not open socket." << endl;
	}

	// Set listenning address and port for server
	bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
	addrReceiver.sin_family = AF_INET;
	addrReceiver.sin_addr.s_addr = INADDR_ANY;
	addrReceiver.sin_port = htons(UDP_PORT_MULTIRROTOR_ARUCO);

	// Associates the address to the socket
	if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
	{
		errorCode = 1; 
		cout << endl << "ERROR: could not associate address to socket." << endl;
	}
	else
	{
		// Set the socket as non blocking
		fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
	}


	/******************************** THREAD LOOP START ********************************/
	while(errorCode == 0 && endThreadSignal == 0)
	{	
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if (dataReceived > 0)
		{
			// Check if message is correct
			if(buffer[0] == 'R' && buffer[1] == 'P' && buffer[2] == 'P')
			{	
				// Get the code sent by the GCS
				dataPacketRobotPose = (DATA_PACKET_ROBOT_POSE*)buffer;
				
                if(dataPacketRobotPose->updated == 1){
                    // Extract the fields
					tf2::Vector3 t_cam(-dataPacketRobotPose->pos[0], 
										-dataPacketRobotPose->pos[1], 
										-dataPacketRobotPose->pos[2]);

					tf2::Matrix3x3 R_cam;
					for (int i = 0; i < 3; i++) {
						for (int j = 0; j < 3; j++) {
							R_cam[i][j] = dataPacketRobotPose->rot[3 * i + j];
						}
					}
					// Compute yaw from the rotation matrix
					double roll, pitch, yaw;
					R_cam.getRPY(roll, pitch, yaw); // Extract Euler angles

					// Construct a new rotation matrix with only yaw
					tf2::Matrix3x3 R_yaw;
					R_yaw.setRPY(0.0, -pitch+M_PI, M_PI); // Only keep yaw

					// Transform the translation using only yaw
					tf2::Vector3 t_aruco = R_yaw * t_cam;

					// Convert the rotation matrix to quaternion
					tf2::Quaternion q_aruco;
					R_yaw.getRotation(q_aruco);

					// Update the transformation
					aruco_to_camera.setOrigin(t_aruco);
					aruco_to_camera.setRotation(q_aruco);

                    cout << "Posición recibida de Aruco:   " << t_cam[0] << "   " << t_cam[1] << "    " << t_cam[2] << endl;
		    cout << "roll:  " << roll << "  Pitch:  " << pitch << "  Yaw:  " << yaw << endl;
                    cout << "Orientación recibida de Aruco:   " << R_cam[0][0] << "   " << R_cam[0][1] << "    " << R_cam[0][2] << endl;
                    cout << "Orientación recibida de Aruco:   " << R_cam[1][0] << "   " << R_cam[1][1] << "    " << R_cam[1][2] << endl;
                    cout << "Orientación recibida de Aruco:   " << R_cam[2][0] << "   " << R_cam[2][1] << "    " << R_cam[2][2] << endl;
                    
                    validArucoPose = 1; // Signal that a valid pose has been received
                }
            }
		}
		else
		{
			// Wait 2 ms
			usleep(10000);
		}
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close the socket
	close(socketReceiver);	
}

void keyboardCommandThread()
{
    string cmd;

    do
    {
        cin >> cmd;
        if(cmd == "exit")
            endThreadSignal = 1;
    } while (endThreadSignal == 0);
    
    usleep(100000);
    cout << "Terminated" << endl;
}
