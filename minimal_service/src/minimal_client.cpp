#include <ros/ros.h>
#include <math.h>
#include <minimal_service/minimal_server_msg.h>
#include <iostream>
#include <string>

using namespace std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<minimal_service::minimal_server_msg>("specify_value");
    
    minimal_service::minimal_server_msg srv;
 	
    double amplitude = 0.0;
    double frequency = 0.0;
    string answer;
    while (ros::ok()) {
    	cout << endl;
        cout << "Do you want to change the amplitue and frequency? (Y/N)";
        cin >> answer;
        cout << endl;
        if (answer.compare("Y") == 0 || answer.compare("y") == 0) {
        	cout << "Please enter the value of amplitude and frequency: ";
        	cin >> amplitude >> frequency;
       
        	srv.request.amplitude = amplitude;
        	srv.request.frequency = frequency;

        	bool success = client.call(srv);
            // This method does the actual work of locating the server node, transmitting the request
            // data, waiting for a response, and storing the response data the Response we provided.
        	if (success) {
            	cout << "The amplitude you specified is " << srv.response.amplitude 
                     << " and the frequency you specified is " << srv.response.frequency << endl;
        	} 
        	else {
            	ROS_ERROR("Failed to call service specify_value");
            	return 1;
      		}
      	}
        else {
        	ROS_INFO("Quit");
        	return 1;
        }
        
    }
    return 0;
}
