#ifndef TAKE_A_PIC_H
#define TAKE_A_PIC_H

#include <pthread.h>

#include "../imagesource/image_source.h"

typedef struct state state_t;
struct state {
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;
    pthread_mutex_t mutex;

};

void take_a_pic (int num);

#endif
