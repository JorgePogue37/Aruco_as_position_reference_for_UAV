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
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define UDP_PORT_MULTIRROTOR_ARUCO 28000

// Structure definition
typedef struct
{
	char header[3];		// "RPP" (Robot Pose Packet) character sequence
    uint8_t updated;
	float pos[3];		// Position of the Aruco marker
	float rot[3];		// Rotation angles of Aruco marker
} __attribute__((packed)) DATA_PACKET_ROBOT_POSE;


// Global variables
int endThreadSignal = 0;
int validArucoPose = 0;
tf2::Vector3  arucoMarkerPosition;
float arucoMarkerRotation[3] = {0.0, 0.0, 0.0};

geometry_msgs::PoseStamped current_pose;

ros::Publisher pose_pub;  // Declare publisher globally

// Thread function declaration
void receiveArucoMRPoseThread();
void keyboardCommandThread();


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher_node");
    ros::NodeHandle n;

    thread arucoMarkerThread;
    thread keyboardThread;

    // Publisher to send the current position
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

    // Initialize the global position in case is not received
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = 1;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = 0;
    current_pose.pose.orientation.w = 1;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "map";        

	// Create the thread for receiving Aruco marker pose through UDP socket
	arucoMarkerThread = thread(&receiveArucoMRPoseThread);
    arucoMarkerThread.detach();
    keyboardThread = thread(&keyboardCommandThread);
    keyboardThread.detach();

    ros::Rate loop_rate(60);

    while (ros::ok() && endThreadSignal == 0)
    {   
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
                    arucoMarkerPosition[0] = cos(theta)*dataPacketRobotPose->pos[0] + sin(theta)*dataPacketRobotPose->pos[2];
                    arucoMarkerPosition[1] = dataPacketRobotPose->pos[1];
                    arucoMarkerPosition[2] = -sin(theta)*dataPacketRobotPose->pos[0] + cos(theta)*dataPacketRobotPose->pos[2];
                    for(k = 0; k < 3; k++)
                        arucoMarkerRotation[k] = M_PI*dataPacketRobotPose->rot[k]/180;
                    
                    arucoMarkerRotation[0] = arucoMarkerRotation[0] + M_PI;
                    if (arucoMarkerRotation[0] > 2*M_PI) arucoMarkerRotation[0] = arucoMarkerRotation[0] - 2*M_PI;
                  	
  		            // Convert Euler angles to quaternion
		            tf2::Quaternion q;
		            q.setRPY(arucoMarkerRotation[2], arucoMarkerRotation[0], -arucoMarkerRotation[1]); // Assuming ZYX (yaw, pitch, roll) and radians
	
                    tf2::Matrix3x3 rotation_matrix(q); // Create rotation matrix from Quaternion	

                    tf2::Vector3 transformedArucoMarkerPosition = -(rotation_matrix.transpose() * arucoMarkerPosition); // Inverted translation (camera to world)

                    cout << "Posición recibida de Aruco:   " << arucoMarkerPosition[0] << "   " << arucoMarkerPosition[1] << "    " << arucoMarkerPosition[2] << endl;
                    cout << "Orientación recibida de Aruco:   " << arucoMarkerRotation[0] << "   " << arucoMarkerRotation[1] << "    " << arucoMarkerRotation[2] << endl;

                    // Create and populate the current pose
                    current_pose.header.stamp = ros::Time::now();
                    current_pose.header.frame_id = "map"; // or "local_origin" as required
                    current_pose.pose.position.x = transformedArucoMarkerPosition[0];
                    current_pose.pose.position.y = transformedArucoMarkerPosition[1];
                    current_pose.pose.position.z = transformedArucoMarkerPosition[2];

		            // Set the orientation in the pose message
		            current_pose.pose.orientation.x = q.getX();
		            current_pose.pose.orientation.y = q.getY();
		            current_pose.pose.orientation.z = q.getZ();
		            current_pose.pose.orientation.w = q.getW();
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
