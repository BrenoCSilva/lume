#include <astro/astro.h>
#ifndef ASTRO_PESSOA_H
#define ASTRO_PESSOA_H


typedef struct{

    char* name;
    int idade;
    double timestamp;
	char *host; 
    
} Pessoa;


#define ASTRO_PESSOA_NAME "nome_da_pessoa"
#define ASTRO_PESSOA_FMT "{string,int,double,string}" 



#endif