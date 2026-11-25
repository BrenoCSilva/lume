#include <astro/astro.h>
#include <astro/vehicle_simulator_interface.h>

static double max_velocity = 2.5;	// limite de velocidade (m/s), vem do .ini

static void
pose_message_handler(astro_vehicle_simulator_pose_message *message)
{
	printf("vehicle_simulator_monitor: x=%.2f y=%.2f theta=%.2f v=%.2f (de %s, t=%.3f)\n",
			message->x, message->y, message->theta, message->v, message->host, message->timestamp);

	if (message->v > max_velocity)
		printf("vehicle_simulator_monitor: ALERTA velocidade %.2f m/s acima do limite %.2f m/s\n",
				message->v, max_velocity);
				fflush(stdout);
}


static void
read_parameters(int argc, char **argv)
{
	astro_param_t param_list[] =
	{
		{ "vehicle_simulator_monitor", "max_velocity", ASTRO_PARAM_DOUBLE, &max_velocity, 0, NULL }
	};

	astro_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		astro_ipc_disconnect();
		printf("vehicle_simulator_monitor: desconectado.\n");
		exit(0);
	}
}

int
main(int argc, char **argv)
{
	astro_ipc_initialize(argc, argv);
		astro_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);
		read_parameters(argc, argv);


	astro_vehicle_simulator_subscribe_pose_message(NULL, (astro_handler_t) pose_message_handler, ASTRO_SUBSCRIBE_LATEST);

	printf("vehicle_simulator_monitor: aguardando mensagens.\n");
	fflush(stdout);

	astro_ipc_dispatch();

	return 0;
}