#ifndef ASTRO_VEHICLE_SIMULATOR_INTERFACE_H
#define ASTRO_VEHICLE_SIMULATOR_INTERFACE_H

#include <astro/astro.h>
#include "vehicle_simulator_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void
astro_vehicle_simulator_define_pose_message(void);

void
astro_vehicle_simulator_publish_pose_message(astro_vehicle_simulator_pose_message *message, double timestamp);

void
astro_vehicle_simulator_subscribe_pose_message(astro_vehicle_simulator_pose_message *message,
		astro_handler_t handler, astro_subscribe_t subscribe_how);

void
astro_vehicle_simulator_unsubscribe_pose_message(astro_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif