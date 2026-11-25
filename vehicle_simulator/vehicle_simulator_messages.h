#ifndef ASTRO_VEHICLE_SIMULATOR_MESSAGES_H
#define ASTRO_VEHICLE_SIMULATOR_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	double x;		/**< Posicao x no referencial global (m) */
	double y;		/**< Posicao y no referencial global (m) */
	double theta;		/**< Orientacao do veiculo (rad) */
	double v;		/**< Velocidade linear (m/s) */
	double timestamp;	/**< Obrigatorio: instante de criacao da mensagem */
	char *host;		/**< Obrigatorio: host de origem da mensagem */
} astro_vehicle_simulator_pose_message;

#define ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_NAME	"astro_vehicle_simulator_pose"
#define ASTRO_VEHICLE_SIMULATOR_POSE_MESSAGE_FMT	"{double,double,double,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif