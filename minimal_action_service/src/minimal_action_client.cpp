// This is a class based version of minimal_action_client.cpp
// and action client is incoporated in this source file, a substitude
// client.
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>

// this #include refers to the new "action" message defined for this package
// the action message can be found in: .../minimal_action_service/action/minimal_action_msg.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (minimal_action_msg) and appended name (Action)
// If you write a new client of the server in this package, you will need to include example_action_server in your package.xml,
// and include the header file below
#include <minimal_action_service/minimal_action_msgAction.h>
using namespace std;

class MinimalActionClientClass
{
public:
    MinimalActionClientClass(); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    ~MinimalActionClientClass();
    // void executeActionCB(const actionlib::SimpleActionServer<minimal_action_service/minimal_action_msgAction>::GoalConstPtr& goal);
    // prototype for action service callback function, the agrument is a pointer to a goal message
    int actonClientMem();
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    // use the name of our server, which is: minimal_action (named in minimal_commander_class.cpp)
    actionlib::SimpleActionClient<minimal_action_service::minimal_action_msgAction> action_client_;

    // some private member variable
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    minimal_action_service::minimal_action_msgGoal goal_;
    string answer_;
    bool server_exists_;
    bool finished_before_timeout_;
    double cycles_get_back_;
    void doneActionCB(const actionlib::SimpleClientGoalState& state, const minimal_action_service::minimal_action_msgResultConstPtr& result);

};

// use the name of our server, which is: example_action (named in example_action_server.cpp)
// the "true" argument says that we want our new client to run as a separate thread (a good idea)
MinimalActionClientClass::MinimalActionClientClass() : action_client_("minimal_action", true)
{ // constructor
    ROS_INFO("in class constructor of MinimalActionClientClass");
}

MinimalActionClientClass::~MinimalActionClientClass()
{ // deconstructor
	ROS_INFO("in class deconstructor of MinimalActionClientClass");
}

int MinimalActionClientClass::actonClientMem()
{
	// attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    server_exists_ = action_client_.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // this causes the new action client to attempt to connect to the named server, allows fow waiting only up 
    // some time limit
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    // bool server_exists = action_client_.waitForServer(); //wait forever

    if (!server_exists_) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

	cout << endl;
    cout << "Do you want to change the amplitue and frequency? (Y/N)";
    cin >> answer_;
    cout << endl;
    if (answer_.compare("Y") == 0 || answer_.compare("y") == 0){
    	cout << "Please enter the value of amplitude, frequency, and cycles: ";
		cin >> goal_.amplitude_input >> goal_.frequency_input >> goal_.cycles_input;
    	while(true) {
            if (abs(goal_.cycles_input - cycles_get_back_) <= 0.01) {
                goal_.amplitude_input = 0.0;
                goal_.frequency_input = 0.0;
                goal_.cycles_input = 0.0;
            }
			// action_client_.sendGoal(goal); // simple example--send goal, but do not specify callbacks
			action_client_.sendGoal(goal_,boost::bind(&MinimalActionClientClass::doneActionCB,this,_1,_2)); 
			// we could also name additional callback functions here, if desired
			// action_client_.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

			finished_before_timeout_ = action_client_.waitForResult(ros::Duration(5.0));
			// this causes the client to suspend, waiting for on the server, but with a specified time-out limit.
			// if the server returns within the specified time limit, the goalCB functio with be triggered.
			// this function receives the "result" message provided by the server.
			// bool finished_before_timeout = action_client_.waitForResult(); // wait forever...
			
			if (!finished_before_timeout_) {
			    ROS_WARN("giving up waiting on result for goal number");
			    return 0;
			}
			else {
			  //if here, then server returned a result to us
			}
		}
	}
    else {
        ROS_INFO("Quit");
        return 1;
    }
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void MinimalActionClientClass::doneActionCB(const actionlib::SimpleClientGoalState& state,
        const minimal_action_service::minimal_action_msgResultConstPtr& result) 
{
    ROS_INFO("doneActionCB: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("amplitude = %f; frequency = %f; cycles = %f, outcome = [%s]", 
    		result->amplitude_output, result->frequency_output, result->cycles_output, result->outcome.c_str());
    cycles_get_back_ = result->cycles_output;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "minimal_action_client_node"); // name this node 
       
    MinimalActionClientClass minimalActionClientClass;
    minimalActionClientClass.actonClientMem();

    return 0;
}