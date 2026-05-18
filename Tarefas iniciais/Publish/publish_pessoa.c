#include <astro/astro.h>
#include "publish_example_pessoa.h"

void
define_messages_pessoa()
{

	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(ASTRO_PESSOA_NAME, IPC_VARIABLE_LENGTH,
			ASTRO_PESSOA_FMT);
	astro_test_ipc_exit(err, "Não foi possível definir a mensagem.", ASTRO_PESSOA_NAME);

}



void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        astro_ipc_disconnect();
        printf("Publicacao: nao consegui conectar ipc.\n");
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publish                                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
astro_publish_pessoa_command_message(Pessoa *pessoa)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData((char *) ASTRO_PESSOA_NAME, pessoa);
	astro_test_ipc_exit(err, "Nao consegui publicar.", (char *) ASTRO_PESSOA_FMT);
}

//  Precisamos passar todas as informações necessárias para cada publish, principalmente contendo:
// 1. a mensagem em si / 2. o host /  3. o timestamp / 4. chave da mensagem (sempre analisar a tipagem da struct que irá utilizar).
void
astro_pessoa()
{
    Pessoa pessoa;

    printf("Mensagem enviada atraves do astro_pesssoa!\n");
    pessoa.name = "Breno Costa Silva";
    pessoa.idade = 22;
    pessoa.host = astro_get_host();
    pessoa.timestamp = astro_get_time();


    astro_publish_pessoa_command_message(&pessoa);
}




int main(int argc, char** argv){
    
    astro_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);

    define_messages_pessoa();

    astro_pessoa();
}