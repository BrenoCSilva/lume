#include <astro/astro.h>
#include "../Publish/publish_example_messages.h"
#include "/home/lume/astro/src/base_ackerman/base_ackerman_interface.h"
#include "/home/lume/astro/src/param_daemon/param_interface.h"

#define MAX_PONTOS 500

void function_gnuplot_txt(astro_robot_ackerman_motion_command_message *msg){
    
    if (1)
    {
        static FILE *gnuplot_pipeMP;
        static double timestamp_inicial = 0;
        static int first = 1;
        if (first)
        {
            first = 0;
            gnuplot_pipeMP = popen("gnuplot", "w");

            FILE *temp = fopen("vd_debug.txt", "w");
            fclose(temp);


            // SETUP CONFIG INICIAL 

            timestamp_inicial = msg->timestamp;
             // Envia comandos como se estivesse no terminal do Gnuplot
            fprintf(gnuplot_pipeMP, "set title 'Gráfico de velocidade x tempo' \n");
            fprintf(gnuplot_pipeMP, "set xlabel 'Tempo (s)'\n");
            fprintf(gnuplot_pipeMP, "set ylabel 'Velocidade (m/s)'\n");
            fprintf(gnuplot_pipeMP, "set grid \n");
        
        }

        static double previous_timestamp = 0.0;
        static double previous_v = 0.0;
        

        if (previous_timestamp > 0.0)
        {
            FILE *file = fopen("vd_debug.txt", "a");
            fprintf(file, "%lf %lf\n",  msg->timestamp - timestamp_inicial, msg->motion_command->v);
            fclose(file);
            
           
        }

        fprintf(gnuplot_pipeMP, "plot 'vd_debug.txt' with lines title 'v(m/s)'\n");

        fflush(gnuplot_pipeMP);


        previous_v = msg->motion_command->v;
        previous_timestamp = msg->timestamp;
    
        //pclose(gnuplot_pipeMP);
   
    }
}


void function_gnuplot_array(astro_robot_ackerman_motion_command_message *msg){
   // printf("chegui eaquizn\n");
    if (1)
    {   
        static int indice = 0;
        static FILE *gnuplot_pipeMP;
        static double timestamp_inicial = 0;
        static int first = 1;
        if (first)
        {
            first = 0;
            gnuplot_pipeMP = popen("gnuplot", "w");

            // SETUP CONFIG INICIAL 

            timestamp_inicial = msg->timestamp;
             // Envia comandos como se estivesse no terminal do Gnuplot
            fprintf(gnuplot_pipeMP, "set title 'Gráfico de velocidade x tempo' \n");
            fprintf(gnuplot_pipeMP, "set xlabel 'Tempo (s)'\n");
            fprintf(gnuplot_pipeMP, "set ylabel 'Velocidade (m/s)'\n");
            fprintf(gnuplot_pipeMP, "set grid \n");
    
        }

        static double previous_timestamp = 0.0;
        static double array_velocidade[MAX_PONTOS],  array_tempo[MAX_PONTOS];
        

        if (previous_timestamp > 0.0)
        {   
            // grava os 500 primeiros pontos no array
            if(indice < MAX_PONTOS){
                array_velocidade[indice] = msg->motion_command[10].v;
                array_tempo[indice] = msg->timestamp - timestamp_inicial;
                printf("qtd_pontos: %d\n", indice);
                indice++;
            }

            else{
                // atualiza os valores de todo mundo em uma posicao, de tal maneira a ultima posicao ficará livre para o novo valor lido
                for (int i = 0; i < MAX_PONTOS - 1; i++) {
                    array_velocidade[i] = array_velocidade[i+1];
                    array_tempo[i] = array_tempo[i+1];
                 }
                array_velocidade[MAX_PONTOS - 1] = msg->motion_command->v;
                array_tempo[MAX_PONTOS - 1] = msg->timestamp - timestamp_inicial;
            }
        }
       
        // 1. Sinaliza ao Gnuplot para esperar dados da RAM
        //fprintf(gnuplot_pipeMP, "plot '-'");
       // fprintf(gnuplot_pipeMP, " using 2:1  \n");
        fprintf(gnuplot_pipeMP, "plot '-' \n");

        // 2. Envia os dados 500 pontos deslocados + o atual para serem projetados
        for(int i=0 ; i < indice; i++){
           fprintf(gnuplot_pipeMP, "%lf %lf\n", array_tempo[i], array_velocidade[i]);
        }

        // 3. Sinalizacao de que os dados ja foram lidos e podem ser projetados
        fprintf(gnuplot_pipeMP, "e\n");
        fflush(gnuplot_pipeMP);

        previous_timestamp = msg->timestamp;
    }
}

   

void example_base(){

    // 1. Criar e salvar dados no arquivo
    FILE *arquivo = fopen("dados.txt", "w");
    fprintf(arquivo, "1 10\n"); // Ponto 1 (X=1, Y=10)
    fprintf(arquivo, "2 20\n"); // Ponto 2 (X=2, Y=20)
    fclose(arquivo);

    // 2. Abrir o Gnuplot
    FILE *pipe = popen("gnuplot ", "w");

    // 3. Configurar e Plotar-persist
    fprintf(pipe, "set title 'Exemplo Simples'\n");
    fprintf(pipe, "set grid\n");
    fprintf(pipe, "plot 'dados.txt' with lines title 'segmento'\n");

    // 4. Fechar o cano (pipe)
    pclose(pipe);
}


void function_gnuplot(astro_robot_ackerman_motion_command_message *msg){
   // printf("chegui eaquizn\n");
    if (1)
    {   
        static int indice = 0;
        static FILE *gnuplot_pipeMP;
        static double timestamp_inicial = 0;
        static int first = 1, j=0;
        double soma_time = 0;
        if (first)
        {
            first = 0;
            gnuplot_pipeMP = popen("gnuplot -persist", "w");

            // SETUP CONFIG INICIAL 

            //timestamp_inicial = msg->timestamp;
             // Envia comandos como se estivesse no terminal do Gnuplot
            fprintf(gnuplot_pipeMP, "set title 'Gráfico de velocidade x tempo' \n");
            fprintf(gnuplot_pipeMP, "set xlabel 'Tempo (s)'\n");
            fprintf(gnuplot_pipeMP, "set ylabel 'Velocidade (m/s)'\n");
            fprintf(gnuplot_pipeMP, "set grid \n");

            /*printf("Primeiro: num_comand %d\n", msg->num_motion_commands);

            for(int i =0; i < msg->num_motion_commands; i++){

                printf("time: %lf\n", msg->motion_command[i].time);
                printf("velocidade: %lf\n", msg->motion_command[i].v);

            }*/

    
        }


        static double previous_timestamp = 0.0;
        static double previous_v = 0.0;
        static double array_velocidade[MAX_PONTOS],  array_tempo[MAX_PONTOS];

            printf(" num_comand %d\n", msg->num_motion_commands);
        
        if (previous_v > 0.0)
        {   
            // grava os 500 primeiros pontos no array
            if(1){
                 
                for(int i =0; i < msg->num_motion_commands; i++){
                 array_velocidade[i] = msg->motion_command[i].v;
                 soma_time +=msg->motion_command[i].time;
                 array_tempo[i] = soma_time ;
                }
                printf("qtd_pontos: %d\n", msg->num_motion_commands);
                j++;
              
                //indice++;
            }

        }
       
        // 1. Sinaliza ao Gnuplot para esperar dados da RAM
        //fprintf(gnuplot_pipeMP, "plot '-'");
       // fprintf(gnuplot_pipeMP, " using 2:1  \n");
        fprintf(gnuplot_pipeMP, "plot '-' w l\n");

        // 2. Envia os dados 500 pontos deslocados + o atual para serem projetados
        for(int i=0 ; i < msg->num_motion_commands; i++){
           fprintf(gnuplot_pipeMP, "%lf %lf\n", array_tempo[i], array_velocidade[i]);
           printf("time: %lf\n", array_tempo[i]);
        }


        // 3. Sinalizacao de que os dados ja foram lidos e podem ser projetado
        fprintf(gnuplot_pipeMP, "e\n");
        fflush(gnuplot_pipeMP);
        
       previous_timestamp = msg->timestamp;
        previous_v = msg->motion_command->v;
    }
}

   
   
