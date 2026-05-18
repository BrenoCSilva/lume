#include <astro/astro.h>
#include "publish_example_messages.h"

// astro rebot eckerman que eu fiz

void
example_define_messages_eckerman()
{

	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH,
			ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
	astro_test_ipc_exit(err, "Could not define message", ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);

}

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        astro_ipc_disconnect();
        printf("Publish_example: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publish                                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
astro_publish_command_message_eckerman(	astro_robot_ackerman_motion_command_message *message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData((char *) ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, message);
	astro_test_ipc_exit(err, "Could not publish", (char *) ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
}


void
astro_rebot_ackerman_message()
{
	static double velocidade = 0;
	astro_robot_ackerman_motion_command_message astro_message;
	astro_robot_and_trailers_motion_command_t mensagem;

	printf("Mensagem enviada através do módulo publish_example_ackerman");

	astro_message.num_motion_commands = 1;
	astro_message.host =  astro_get_host();
	astro_message.timestamp = astro_get_time();
	velocidade++;
	mensagem.v = velocidade;
	astro_message.motion_command = &mensagem;
	astro_publish_command_message_eckerman(&astro_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc , char **argv)
{
	astro_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	example_define_messages_eckerman();

	astro_rebot_ackerman_message();


	return 0;
}
