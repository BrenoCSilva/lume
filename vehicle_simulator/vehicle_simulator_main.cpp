#include <math.h>
#include <astro/astro.h>
#include <astro/vehicle_simulator_interface.h>

static double dt = 1.0;			// tempo de cada passo (s)
static double mean_velocity = 2.0;	// velocidade media (m/s)
static double yaw_rate = 0.1;		// taxa de giro (rad/s)

static double x = 0.0;
static double y = 0.0;
static double theta = 0.0;
static double v = 2.0;
static double sim_time = 0.0;

static void
publish_pose(void)
{
	astro_vehicle_simulator_pose_message message;

	message.x = x;
	message.y = y;
	message.theta = theta;
	message.v = v;

	astro_vehicle_simulator_publish_pose_message(&message, astro_get_time());

	printf("vehicle_simulator: publicou x=%.2f y=%.2f theta=%.2f v=%.2f\n", x, y, theta, v);
	fflush(stdout);
}

static void
simulation_step(void)
{
	sim_time += dt;
	v = mean_velocity + 1.0 * sin(sim_time);
	theta += yaw_rate * dt;
	x += v * cos(theta) * dt;
	y += v * sin(theta) * dt;

	publish_pose();
}

static void
read_parameters(int argc, char **argv)
{
		astro_param_t param_list[] =
	{
		{ "vehicle_simulator", "dt", ASTRO_PARAM_DOUBLE, &dt, 0, NULL },
		{ "vehicle_simulator", "mean_velocity", ASTRO_PARAM_DOUBLE, &mean_velocity, 0, NULL },
		{ "vehicle_simulator", "yaw_rate", ASTRO_PARAM_DOUBLE, &yaw_rate, 0, NULL }
	};

	astro_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		astro_ipc_disconnect();
		printf("vehicle_simulator: desconectado.\n");
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

	astro_vehicle_simulator_define_pose_message();

	astro_ipc_addPeriodicTimer(dt, (TIMER_HANDLER_TYPE) simulation_step, NULL);

	astro_ipc_dispatch();

	return 0;
}