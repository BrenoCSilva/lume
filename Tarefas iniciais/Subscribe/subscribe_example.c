#include <astro/astro.h>
#include "../Publish/publish_example_messages.h"
#include "/home/lume/astro/src/base_ackerman/base_ackerman_interface.h"
#include "/home/lume/astro/src/param_daemon/param_interface.h"



bool var1_name = FALSE;
bool var2_name = FALSE;


// Coloque esta função antes do read_parameters
static void meu_handler_de_parametro(char *module, char *variable, char *value) {
    printf("\n>>> Atenção: O parâmetro mudou! var2_name agora vale %d\n\n", var2_name);
    printf("modulo: %s / variavel> %s / valor: %s\n", module, variable, value);
}

static void 
read_parameters(int argc, char *argv[])
{
    // Required parameters
    /*astro_param_t required_params[] = {
        {"breno", "parametro", ASTRO_PARAM_ONOFF, &var1_name, 1, NULL},
    };
    astro_param_allow_unfound_variables(0); // Use 0 for required params
    astro_param_install_params(argc, argv, required_params, sizeof(required_params)/sizeof(required_params[0]));*/

    // Optional parameters
    astro_param_t optional_params[] = 
	{
        {"breno", "parametro", ASTRO_PARAM_ONOFF, &var2_name, 1, (astro_param_change_handler_t) meu_handler_de_parametro},
        //{"breno", "parametro", ASTRO_PARAM_ONOFF, &var2_name, 1, NULL},
    };
    astro_param_allow_unfound_variables(1); // Use 1 for optional params
    astro_param_install_params(argc, argv, optional_params, sizeof(optional_params)/sizeof(optional_params[0]));
}
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
astro_simple_handler(astro_string_example_message *current_message)
{
	printf("Mensagem recebida: %s\n",current_message->content_message);

}


extern void function_gnuplot_txt(astro_robot_ackerman_motion_command_message *msg);
extern void function_gnuplot_array(astro_robot_ackerman_motion_command_message *msg);
extern void gnuplot_txt(astro_robot_ackerman_motion_command_message *msg);

extern void example_base();

   
   
static void
robot_ackerman_motion_command_message_handler(astro_robot_ackerman_motion_command_message *msg)
{

	//printf("Mensagem recebida: %s\n", msg->host);
	if(var2_name){
		astro_obstacle_avoider_publish_base_ackerman_motion_command(msg->motion_command, msg->num_motion_commands, msg->timestamp);
		printf("forçando movimento\n");
	}

    //printf("%lf %lf %lf\n", msg->motion_command->x, msg->motion_command->y, msg->timestamp);

    if (1)
    {
        //function_gnuplot_txt(msg);
        example_base();

    }
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        astro_ipc_disconnect();
        printf("Subscribe_example: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
astro_simple_subscribe_motion_command(astro_string_example_message *message,
				     astro_handler_t handler, astro_subscribe_t subscribe_how)
{
	astro_subscribe_message(ASTRO_STRING_EXAMPLE_MESSAGE_NAME, ASTRO_STRING_EXAMPLE_MESSAGE_FMT,
                	   message, sizeof(astro_string_example_message), handler, subscribe_how);
}

///////////////////////////////////////////////////////////////////////////////////////////////

static void subscribe_relevant_messages(astro_robot_ackerman_motion_command_message *mensagem, astro_handler_t handler, astro_subscribe_t subscribe_how){

    astro_subscribe_message(ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, ASTRO_ROBOT_ACKERMAN_MOTION_COMMAND_FMT, mensagem, sizeof(astro_robot_ackerman_motion_command_message), handler, subscribe_how);

}

int
main(int argc , char **argv)
{
	astro_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	//read_parameters(argc, argv);

    example_base();
	printf("Aguardando mensagem\n");

	astro_simple_subscribe_motion_command(NULL, (astro_handler_t) astro_simple_handler, ASTRO_SUBSCRIBE_LATEST);
 	subscribe_relevant_messages(NULL, (astro_handler_t) robot_ackerman_motion_command_message_handler, ASTRO_SUBSCRIBE_LATEST);
	astro_ipc_dispatch();


	return 0;
}
