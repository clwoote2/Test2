#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "ardrone_command/serialized_ardrone_command.h"
#include "ardrone_command/commandInterface.h"
#include "ardrone_command/command.hpp"

#include "ardrone_command/altitude_control_state.h"
#include "ardrone_command/qr_code_state_info.h"
#include "ardrone_command/qr_go_to_point_control_info.h"
#include "ardrone_command/qr_orientation_control_info.h"
#include "ardrone_command/command_status_info.h"


/*
This function takes altitude control messages and prints out the associated values.
@param inputMessage: The altitude control message to print out.
*/
void handleAltitudeControlStateMessage(const ardrone_command::altitude_control_state::ConstPtr &inputMessage);

/*
This function takes QR code state messages and prints out the associated values.
@param inputMessage: The QR code state message to print out.
*/
void handleQRCodeStateMessage(const ardrone_command::qr_code_state_info::ConstPtr &inputMessage);

/*
This function takes qr go to point messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleQRGoToPointMessage(const ardrone_command::qr_go_to_point_control_info::ConstPtr &inputMessage);

/*
This function takes QR orientation messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleQROrientationMessage(const ardrone_command::qr_orientation_control_info::ConstPtr &inputMessage);

/*
This function takes command status messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleCommandStatusInfoMessage(const ardrone_command::command_status_info::ConstPtr &inputMessage);

int main(int argc, char** argv)
{


ros::init(argc, argv, "ardrone_command_test");
ros::NodeHandle nodeHandle;


//Subscribe to get information about different topics published from the ardrone_command node
//ros::Subscriber altitudeInfo = nodeHandle.subscribe("/ardrone_command/altitude_control", 1000, &handleAltitudeControlStateMessage);

//ros::Subscriber QRStateInfo = nodeHandle.subscribe("/ardrone_command/qr_code_state_estimates", 1000, &handleQRCodeStateMessage);

//ros::Subscriber QRGoToPointInfo = nodeHandle.subscribe("/ardrone_command/go_to_point_control", 1000, &handleQRGoToPointMessage);

//ros::Subscriber QROrientationInfo = nodeHandle.subscribe("/ardrone_command/orientation_control", 1000, &handleQROrientationMessage);

ros::Subscriber commandInfo = nodeHandle.subscribe("/ardrone_command/command_processing", 1000, &handleCommandStatusInfoMessage);








std::vector<command> commands;

command commandWait;
commandWait.setWaitCommand(2.0);
commands.push_back(commandWait);


//Make commands to send
command commandTakeoff;
commandTakeoff.setTakeoffCommand();
commands.push_back(commandTakeoff);


command commandSetTargetAltitude;
commandSetTargetAltitude.setTargetAltitudeCommand(500.0);
commands.push_back(commandSetTargetAltitude);

command commandWaitUntilTargetAltitudeReached;
commandWaitUntilTargetAltitudeReached.setWaitUntilAltitudeReached(10.0);
commands.push_back(commandWaitUntilTargetAltitudeReached);

command commandWaitForQRCode;
commandWaitForQRCode.setWaitUntilSpecificQRCodeIsSpottedCommand("BigQRCode", 3.0);
commands.push_back(commandWaitForQRCode);

command commandLookAtQRCodePoint;
commandLookAtQRCodePoint.setMaintainOrientationTowardSpecificQRCode("BigQRCode");
commands.push_back(commandLookAtQRCodePoint);

command commandGoToQRCodePoint;
commandGoToQRCodePoint.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0+.5);
commands.push_back(commandGoToQRCodePoint);



command QRCodePointWait1; QRCodePointWait1.setWaitUntilPositionAtSpecificQRCodePointReachedCommand(100.0);
commands.push_back(QRCodePointWait1);

command commandGoToQRCodePoint2;
commandGoToQRCodePoint2.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0-.5);
commands.push_back(commandGoToQRCodePoint2);

commands.push_back(QRCodePointWait1);


command commandGoToQRCodePoint3;
commandGoToQRCodePoint3.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", -.5, 0.0, 3.0-.5);
commands.push_back(commandGoToQRCodePoint3);

commands.push_back(QRCodePointWait1);


command commandGoToQRCodePoint4;
commandGoToQRCodePoint4.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", -.5, 0.0, 3.0+.5);
commands.push_back(commandGoToQRCodePoint4);

commands.push_back(QRCodePointWait1);


command commandGoToQRCodePoint5;
commandGoToQRCodePoint5.setMaintainPositionAtSpecificQRCodePoint("BigQRCode", .5, 0.0, 3.0+.5);
commands.push_back(commandGoToQRCodePoint5);

commands.push_back(QRCodePointWait1);


command commandWait2;
commandWait2.setWaitCommand(2.0);
commands.push_back(commandWait2);

command commandCancelMaintainPositionAtQRCodePoint;
commandCancelMaintainPositionAtQRCodePoint.setCancelMaintainPositionAtSpecificQRCodePoint();
commands.push_back(commandCancelMaintainPositionAtQRCodePoint);


command commandCancelLookAtQRCodePoint;
commandCancelLookAtQRCodePoint.setCancelMaintainOrientationTowardSpecificQRCode();
commands.push_back(commandCancelLookAtQRCodePoint);




//Serialize and send commands
ardrone_command::commandInterface interface;
for(int i=0; i<commands.size(); i++)
{
auto commandMessage = commands[i].serialize();
ardrone_command::commandInterface::Request request;
request.command = commandMessage;

ardrone_command::commandInterface::Response response;

ros::service::call("/ardrone_command/commandInterface", request, response);
//sleep(1);
}


while(true) //Wait forever, displaying information received from the command system
{
ros::spin();
}



ros::shutdown();

return true;
}




/*
This function takes altitude control messages and prints out the associated values.
@param inputMessage: The altitude control message to print out.
*/
void handleAltitudeControlStateMessage(const ardrone_command::altitude_control_state::ConstPtr &inputMessage)
{
printf("Altitude Message:\n %lf seconds\ntarget altitude: %lf\nCurrent P term: %lf\nCurrent I term: %lf\n", inputMessage->time_stamp.toSec(), inputMessage->target_altitude, inputMessage->current_p_term, inputMessage->current_i_term);
}

/*
This function takes QR code state messages and prints out the associated values.
@param inputMessage: The QR code state message to print out.
*/
void handleQRCodeStateMessage(const ardrone_command::qr_code_state_info::ConstPtr &inputMessage)
{
printf("QR Code State Message:\nTime: %lf seconds\nQR code ID: %s\nQR Code Size: %lf\nPose Matrix:\n", inputMessage->time_stamp.toSec(), inputMessage->qr_code_identifier.c_str(), inputMessage->qr_code_size);

for(int i=0; i<4; i++)
{
for(int ii=0; ii<4; ii++)
{
printf("%lf ", inputMessage->transform[i*4+ii]); 
}
printf("\n");
}
}

/*
This function takes qr go to point messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleQRGoToPointMessage(const ardrone_command::qr_go_to_point_control_info::ConstPtr &inputMessage)
{
printf("QR Go To Point Message:\nTime: %lf seconds\nTarget Point (camera): %lf %lf %lf\nTarget Point (local): %lf %lf %lf\nDistance to target: %lf\nMode: %d\nI Term for X Axis: %lf\nI Term for Y Axis: %lf\nQR XYZ Throttle: %lf %lf %lf\n", inputMessage->time_stamp.toSec(), inputMessage->target_point_camera_xyz[0], inputMessage->target_point_camera_xyz[1],inputMessage->target_point_camera_xyz[2], inputMessage->target_point_local_xyz[0], inputMessage->target_point_local_xyz[1], inputMessage->target_point_local_xyz[2], inputMessage->estimated_distance_to_target, inputMessage->mode, inputMessage->qr_x_axis_I_term, inputMessage->qr_y_axis_I_term, inputMessage->qr_xyz_throttle[0], inputMessage->qr_xyz_throttle[1], inputMessage->qr_xyz_throttle[2]);
}

/*
This function takes QR orientation messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleQROrientationMessage(const ardrone_command::qr_orientation_control_info::ConstPtr &inputMessage)
{
printf("Orientation Control Message:\nTime: %lf seconds\nRotation Throttle: %lf\n", inputMessage->time_stamp.toSec(), inputMessage->z_rotation_throttle);
}

/*
This function takes command status messages and prints out the associated values.
@param inputMessage: The message to print out.
*/
void handleCommandStatusInfoMessage(const ardrone_command::command_status_info::ConstPtr &inputMessage)
{
printf("Command status info:\nTime: %lf seconds\nCommand Number: %u\n", inputMessage->time_stamp.toSec(), inputMessage->commandNumber);

std::vector<command> receivedCommands = deserialize_commands(inputMessage->command);

for(int i=0; i<receivedCommands.size(); i++)
{
printf("Command Type: %s\n", commandTypeToString(receivedCommands[i].type).c_str());
}

}


