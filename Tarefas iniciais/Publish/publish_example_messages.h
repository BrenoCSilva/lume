#include <astro/astro.h>
#ifndef ASTRO_MESSAGE_EXAMPLE_H
#define ASTRO_MESSAGE_EXAMPLE_H

#ifdef __cplusplus
extern "C"
{
#endif


#define MAXSIZE 1024

typedef struct
{
	int num_message;
	char *content_message; /** Could be anything and any amount **/
	double timestamp;
	char *host;                 /**< The host from which this message was sent **/
} astro_string_example_message;

#define      ASTRO_STRING_EXAMPLE_MESSAGE_NAME         "astro_string_example_message"
#define      ASTRO_STRING_EXAMPLE_MESSAGE_FMT          "{int,string,double,string}"

#ifdef __cplusplus
}
#endif

#endif
