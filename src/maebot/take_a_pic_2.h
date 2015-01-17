#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include "common/getopt.h"

#include "../imagesource/image_u8x3.h"
#include "../imagesource/image_source.h"
#include "../imagesource/image_convert.h"

#ifndef TAKE_A_PIC_H
#define TAKE_A_PIC_H

typedef struct state state_t;
struct state {
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;
    pthread_mutex_t mutex;

};

void take_a_pic (int num )
{
    //instantiate a state variable
	state_t *state = calloc(1, sizeof(*state));


	//get camera addresses
	zarray_t *urls = image_source_enumerate();

    	printf("Cameras:\n");
	    for (int i = 0; i < zarray_size(urls); i++) {
		char *url;
	        zarray_get(urls, i, &url);
	      	printf("  %3d: %s\n", i, url);
    	}

    	if (zarray_size(urls) == 0) {
        	printf("No cameras found.\n");
        	exit(0);
    	}
    	zarray_get(urls, 0, &state->url);
    


    	pthread_mutex_init(&state->mutex, NULL);

    //////////////////////////////////////////////////////////
    	state->isrc = image_source_open(state->url);
    	if (state->isrc == NULL) {
        	printf("Unable to open device %s\n", state->url);
        	exit(-1);
    	}

    	image_source_t *isrc = state->isrc;

    	if (isrc->start(isrc))
        	exit(-1);

    	state->fidx = isrc->get_current_format(isrc);

    	printf("Image source formats:\n");
    	for (int i = 0; i < isrc->num_formats(isrc); i++) {
        	image_source_format_t ifmt;
        	isrc->get_format(isrc, i, &ifmt);
        	printf("\t%d\t%4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    	}

    	printf("Image source features:\n");
    	for (int i = 0; i < isrc->num_features(isrc); i++) {
        	const char *feature_name = isrc->get_feature_name(isrc, i);
        	char *feature_type = isrc->get_feature_type(isrc, i);
        	double v = isrc->get_feature_value(isrc, i);

        	printf("\t%-20s %10f     %s\n", feature_name, v, feature_type);
        	free(feature_type);
    }
       
	//variables to store the pic 
	image_source_data_t isdata;

        pthread_mutex_lock(&state->mutex);
        //take pic
	int res = isrc->get_frame(isrc, &isdata);
	while(res)
		res = isrc->get_frame(isrc, &isdata);
	
	//create name for outputfile
	char* part_dest = "/home/team1/467-A0/image";
	char* number;
	number = malloc(3);
	sprintf(number,"%d", num);
	char* format = ".ppm";
	
	char* dest;
	dest = malloc(strlen(part_dest) + 1 + 2 + 4);
	strcpy(dest, part_dest);
	strcat(dest, number);
	strcat(dest, format);

printf(dest);
	//convert pic to proper format and output
	image_u32_t* img = image_convert_u32(&isdata);
	image_u32_write_pnm(img, dest);

        isrc->release_frame(isrc, &isdata);
	isrc->stop(isrc);
     
  	pthread_mutex_unlock(&state->mutex);


	printf("made it here!!!\n\n");
	return 0;
}

#endif
