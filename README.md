#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

char timestamp_inicial[32], timestamp_final[32];

typedef struct {
    int signal;               
    double timestamp_1;   
    double timestamp_2;   
    double time;
} Traffic_signal;


int read_term(char* linha) {
        
    char *delimitadores = " \n\t"; 
    char* copia_linha = strdup(linha);
    char *termo = strtok(copia_linha, delimitadores);  // retorna o ponteiro para o primeiro termo[ate o delimitador]
    
    if(termo == NULL) {
        free(copia_linha);
        return 0;
    }
    
    if(strcmp(termo, "TRAFFIC_MANAGER_SIGNAL:") == 0){
        free(copia_linha);
        return 1;
    }

    if(strcmp(termo, "VELODYNE_VARIABLE_SCAN_IN_FILE0") == 0){
        int i=0;
        static int flag = 1;
        while(termo != NULL && i < 7){
            termo = strtok(NULL, delimitadores);
            i++;
        }
        if(flag){
            strcpy(timestamp_inicial, termo);
            strcpy(timestamp_final, termo);
            //printf("inicio: %s\n", timestamp_inicial);
            //printf("fim:%s\n", timestamp_final);
            flag = 0;
        }else{
            strcpy(timestamp_final, termo);
        }
    }
    free(copia_linha);
    return 0;
}


std::vector<Traffic_signal> read_file(char* caminho_arquivo){

    FILE* arquivo_p; 

    arquivo_p = fopen(caminho_arquivo, "r");

    if(!arquivo_p){
        printf("Error ao abrir o arquivo, finalizando programa\n");
        /*EXEC_COOLDOWN(astro_global_publish_astro_error_message_custom_string(
				ASTRO_ERROR_CODE_1401,
				"Error ao abrir o arquivo."
				), 5);*/
        exit(1);
    }

    std::vector<Traffic_signal> signals_file;
    char* linha = NULL;
    size_t tamanho = 0;
    int return_linha;
    while(getline(&linha, &tamanho, arquivo_p) != -1){ // getline retorna o tamanho da linha, se -1, linha vazia.

        return_linha = read_term(linha); // le os termos da linha
        if(return_linha) {
            Traffic_signal s;
            sscanf(linha, "TRAFFIC_MANAGER_SIGNAL: %d %*lf %*s %*lf", &s.signal, &s.timestamp_1, &s.timestamp_2);
            s.time = atof(timestamp_final);
            signals_file.push_back(s);
        }

    }

    free(linha);
    fclose(arquivo_p);
    return signals_file;
}

int inc = 0;
void gerar_arquivos_entrada(char* file_path){

    FILE* arquivo_p; 

    arquivo_p = fopen(file_path, "w");

    if(!arquivo_p){
        printf("Error ao abrir o arquivo, finalizando programa\n");
        exit(1);
    }

    char signal[32] = "13";
    double timestamp1 = 20.000;
    double timestamp2 = 20.000;
    int j;
    for(int i=0 ; i<10; i++){
        j = i + inc;
        fprintf(arquivo_p, "TRAFFIC_MANAGER_SIGNAL: %s %lf traffic_manager %lf\n", signal, timestamp1 + j, timestamp2 + j);
    }
    inc++;
    fclose(arquivo_p);

}


void gerar_arquivos_saida(char* file_path, std::vector<Traffic_signal> signals){

    FILE* arquivo_p; 
    arquivo_p = fopen(file_path, "w");

    if(!arquivo_p){
        printf("Error ao abrir o arquivo, finalizando programa\n");
        exit(1);
    }

    for(auto& signal : signals){
        fprintf(arquivo_p, "%d %lf \n", signal.signal, signal.timestamp_1);
    }
    fclose(arquivo_p);
}

void plotar_sinais_digitais(const std::vector<Traffic_signal>& msg = {}, bool fim = false) {
    
    static std::vector<std::vector<Traffic_signal>> container_acumulado;
    
    if (!msg.empty()) {
        container_acumulado.push_back(msg);
    }
    if (fim && !container_acumulado.empty()) {
        static FILE *gnuplot_pipe = popen("gnuplot -persist", "w");

        fprintf(gnuplot_pipe, "set title 'Análise de Troca de Sinais'\n");
        fprintf(gnuplot_pipe, "set xlabel 'Timestamp (s)'\n");
        fprintf(gnuplot_pipe, "set grid\n");

        //fprintf(gnuplot_pipe, "set yrange [8.5 : 13.5]\n");
        
        //fprintf(gnuplot_pipe, "set ytics ('Arq2: VM (12)' 9, 'Arq2: VD (13)' 10, 'Arq1: VM (12)' 12, 'Arq1: VD (13)' 13)\n"); 
        
        // Se tiver 1 arquivo: vai de 10.5 a 13.5. Se tiver 3: vai de 4.5 a 13.5.
        double y_min = 13.5 - (3.0 * container_acumulado.size());
        fprintf(gnuplot_pipe, "set yrange [%lf : 13.5]\n", y_min);
        
        // 2. CONSTRUÇÃO DINÂMICA DOS YTICS (Legendas do Eixo Y)
        // Vamos usar um loop para montar a string com os nomes de todas as pistas
        fprintf(gnuplot_pipe, "set ytics (");
        for (size_t c = 0; c < container_acumulado.size(); c++) {
            int offset = (int)c * -3;
            int v_vermelho = 12 + offset;
            int v_verde = 13 + offset;

            // Escreve as duas legendas da pista atual
            fprintf(gnuplot_pipe, "'Arq%lu: VM (12)' %d, 'Arq%lu: VD (13)' %d", 
                    c + 1, v_vermelho, c + 1, v_verde);

            // Se não for o último arquivo, adiciona uma vírgula para separar o próximo do loop
            if (c < container_acumulado.size() - 1) {
                fprintf(gnuplot_pipe, ", ");
            }
        }
        fprintf(gnuplot_pipe, ")\n");

        fprintf(gnuplot_pipe, "set xrange [%s: %s]\n", timestamp_inicial, timestamp_final);

        fprintf(gnuplot_pipe, "plot ");
        for (size_t i = 0; i < container_acumulado.size(); i++) {
            fprintf(gnuplot_pipe, "'-' using 1:2 with steps linewidth 2 title 'Arquivo %lu'", i + 1);
            if (i < container_acumulado.size() - 1) fprintf(gnuplot_pipe, ", ");
        }
        fprintf(gnuplot_pipe, "\n");
        
        for (int c=0; c < container_acumulado.size();c++ ) {
            const auto& curva = container_acumulado[c];
            int offset = (int)c * -3;
            for (size_t i = 1; i < curva.size(); i++) {
                int sinal_com_offset = curva[i].signal + offset;
                fprintf(gnuplot_pipe, "%lf %d\n", curva[i].time, sinal_com_offset);
            }
            fprintf(gnuplot_pipe, "e\n");
        }
        //fprintf(gnuplot_pipe, "e\n");
   

        fflush(gnuplot_pipe);
    }
}


void plotar_sinais_tri_estatal(const std::vector<Traffic_signal>& msg = {}, bool fim = false) {
    static std::vector<std::vector<Traffic_signal>> container_acumulado;
    
    if (!msg.empty()) {
        container_acumulado.push_back(msg);
    }
    
    if (fim && !container_acumulado.empty()) {
        static FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
        size_t qtd_arquivos = container_acumulado.size();

        fprintf(gnuplot_pipe, "set title 'Análise Tri-estatal de Sinais (Verde / OFF / Vermelho)'\n");
        fprintf(gnuplot_pipe, "set xlabel 'Timestamp Unix (s)'\n");
        fprintf(gnuplot_pipe, "set grid\n");

        // 1. CÁLCULO DINÂMICO DO RANGE VERTICAL (Y)
        // Se tiver 1 arquivo, vai de 10.5 a 13.5. Cada arquivo tira 3 unidades para baixo.
        double y_min = 13.5 - (3.0 * qtd_arquivos);
        fprintf(gnuplot_pipe, "set yrange [%lf : 13.5]\n", y_min);
        
        // 2. CONSTRUÇÃO DINÂMICA DAS LEGENDAS (YTICS)
        fprintf(gnuplot_pipe, "set ytics (");
        for (size_t c = 0; c < qtd_arquivos; c++) {
            int centro_pista = 12 + ((int)c * -3);
            
            int v_vermelho = centro_pista - 1;
            int v_off      = centro_pista;
            int v_verde    = centro_pista + 1;

            // Adiciona as 3 marcações (VM, OFF, VD) para esta pista no eixo Y
            fprintf(gnuplot_pipe, "'Arq%lu: VM (-1)' %d, 'Arq%lu: OFF (0)' %d, 'Arq%lu: VD (1)' %d", 
                    c + 1, v_vermelho, c + 1, v_off, c + 1, v_verde);

            if (c < qtd_arquivos - 1) {
                fprintf(gnuplot_pipe, ", ");
            }
        }
        fprintf(gnuplot_pipe, ")\n"); 

        fprintf(gnuplot_pipe, "set xrange [%s: %s]\n", timestamp_inicial, timestamp_final);

        // 3. COMANDO PLOT
        fprintf(gnuplot_pipe, "plot ");
        for (size_t i = 0; i < qtd_arquivos; i++) {
            fprintf(gnuplot_pipe, "'-' using 1:2 with steps linewidth 2 title 'Arquivo %lu'", i + 1);
            if (i < qtd_arquivos - 1) fprintf(gnuplot_pipe, ", ");
        }
        fprintf(gnuplot_pipe, "\n");

        // 4. ENVIO DOS DADOS MAPEADOS
        for (size_t c = 0; c < qtd_arquivos; c++) {
            const auto& curva = container_acumulado[c];
            int centro_pista = 12 + ((int)c * -3);

            for (size_t i = 1; i < curva.size(); i++) {
                // Mapeia o sinal original do arquivo para o padrão [-1, 0, 1]
                int sinal_mapeado = 0; 
                
                if (curva[i].signal == 13) {         // Exemplo: Verde original
                    sinal_mapeado = 1;
                } else if (curva[i].signal == 12) {  // Exemplo: Vermelho original
                    sinal_mapeado = -1;
                } else {                             // Qualquer outra coisa (ou ID do OFF) vira 0
                    sinal_mapeado = 0;
                }

                // Aplica a posição da pista baseado no centro dela
                int valor_final_y = centro_pista + sinal_mapeado;
                
                fprintf(gnuplot_pipe, "%lf %d\n", curva[i].time, valor_final_y);
            }
            fprintf(gnuplot_pipe, "e\n"); 
        }
   
        fflush(gnuplot_pipe);
        container_acumulado.clear(); 
    }
}



int main(int argc, char* argv[]){


    printf("iniciando programa\n");

    std::vector<std::vector<Traffic_signal>> list_files;
    int i;
    char caminho_saida[32];
    for( i=1; i < argc; i++){

        std::vector<Traffic_signal> signals_file = read_file(argv[i]);
        snprintf(caminho_saida, sizeof(caminho_saida), "saida%d.txt", i);
        gerar_arquivos_saida(caminho_saida, signals_file);
        list_files.push_back(signals_file);

        //gerar_arquivos_entrada(argv[i]);         
    }
    
    printf("\nLeitura concluída! Total de arquivos lidos: %lu\n", list_files.size());


    if (list_files.size() >= 2) {
        plotar_sinais_tri_estatal(list_files[0], false);
        for( i=1; i < list_files.size() ; i++){
            auto& arq1 = list_files[0];
            auto& arq2 = list_files[i];
            
            // Pega o menor tamanho entre os dois para evitar ler além do limite
            size_t min_linhas;
            printf("arq1 com %d sinais e arq2 com %d sinais\n", list_files[0].size(), list_files[1].size());
            if(arq1.size() < arq2.size()){
                min_linhas = arq1.size();
            }else{
                 min_linhas = arq2.size();
            }
            printf("\nComparando os primeiros %lu sinais comuns:\n", min_linhas);
            for (size_t j = 0; j < min_linhas; j++) {
                printf("Sinal [%lu] -> Arq1 : %lf | Arq%d : %lf | Dif: %lf\n",
                       j, i+1, arq1[j].time, arq2[j].time, 
                       arq2[j].time - arq1[j].time);
            }
            plotar_sinais_tri_estatal(list_files[i], false);
            //plotar_sinais_digitais(list_files[0], list_files[1]);
        }

        plotar_sinais_tri_estatal({}, true);
    }   

    return 0;
}


