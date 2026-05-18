#include <astro/astro.h>
#include "../Publish/publish_example_pessoa.h"


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// função responsável por realizar alguma ação após o recebimento da mensagem publicada
void astro_handler_pessoa(Pessoa* pessoa){

    printf("Nome:%s\nIdade:%d\n", pessoa->name, pessoa->idade);

}

void astro_subscribe_pessoa(Pessoa *pessoa, astro_handler_t handler, astro_subscribe_t subscribe_how){

    astro_subscribe_message(ASTRO_PESSOA_NAME, ASTRO_PESSOA_FMT, pessoa, sizeof(Pessoa), handler, subscribe_how);

}

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        astro_ipc_disconnect();
        printf("Incricao: Não consegui conectar.\n");
        exit(0);
    }
}

int main(int argc, char** argv){

    astro_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);
    printf("Aguardando uma nova mensagem\n");

    astro_subscribe_pessoa(NULL, (astro_handler_t)astro_handler_pessoa,ASTRO_SUBSCRIBE_LATEST );
    astro_ipc_dispatch();
}