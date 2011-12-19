/*------------------------------------------------------------------*/

#include <sstream>
#include <cstdio>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ach.h>
#include <ctime>
#include <amino.hpp>
#include <amino/time.h>


int main(int argc, char** argv)
{
	ach_channel_t ach_channel;
	memset( &ach_channel, 0, sizeof(ach_channel) );

	int r = ach_open(&ach_channel, "obj-centroid", NULL);
	if( !(ACH_OK == r) )
	{
		fprintf(stderr, "Could not open the channel\n");
		return -1;
	}

	double data[3] = { 0 };
	size_t counter = 0;
	while( 1 )
	{
		size_t frame_size = 0;
		int r = ach_get_last( &ach_channel, data, sizeof(data), &frame_size );
		if( ACH_OK != r )
		{
			if( ACH_STALE_FRAMES == r ) usleep( 15000 );
			if( !( ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r ) ) break;
		}
		else
		{
			printf( "%4zu: %lf, %lf, %lf\n", counter, data[0], data[1], data[2] );
		}
	}

	ach_close( &ach_channel );

	return 0;
}

