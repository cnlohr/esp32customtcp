#include <cnip_http.h>

CNIP_IRAM int HandleCommand( uint8_t * data, int len, uint8_t * ds )
{
	uint8_t * dsstart = ds;
	data[len] = 0;
//	printf( "COMMAND: %s\n", data );

	ds += sprintf( (char*)ds, "HELLO\n" );


	return ds - dsstart;	 
}

