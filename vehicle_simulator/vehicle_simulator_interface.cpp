#include <astro/astro.h>
#include "vehicle_simulator_interface.h"

void
astro_vehicle_simulator_define_pose_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_FMT);
	astro_test_ipc_exit(err, "Could not define", ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME);
}

void
astro_vehicle_simulator_publish_pose_message(astro_vehicle_simulator_pose_message *message, double timestamp)
{
	IPC_RETURN_TYPE err;

	message->timestamp = timestamp;
	message->host = astro_get_host();

	err = IPC_publishData(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, message);
	astro_test_ipc_exit(err, "Could not publish", ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME);
}


void
astro_vehicle_simulator_subscribe_pose_message(astro_vehicle_simulator_pose_message *message,
		astro_handler_t handler, astro_subscribe_t subscribe_how)
{
	astro_subscribe_message(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_FMT,
			message, sizeof(astro_vehicle_simulator_pose_message), handler, subscribe_how);
}

void
astro_vehicle_simulator_unsubscribe_pose_message(astro_handler_t handler)
{
	astro_unsubscribe_message(ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME, handler);
}