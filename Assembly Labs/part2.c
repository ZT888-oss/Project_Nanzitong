#include <stdio.h>
#include <stdint.h>
struct audio_t {

      volatile unsigned int control;

      volatile unsigned char rarc;

      volatile unsigned char ralc;

      volatile unsigned char wsrc;

      volatile unsigned char wslc;

      volatile unsigned int ldata;

      volatile unsigned int rdata;

};

struct audio_t *const audiop = ((struct audio_t *)0xff203040);

void audio_connect() {
    audiop->control = 0x8; // Clear output FIFOs
    audiop->control = 0x0; // Resume input conversion

    while (1) {
        // Wait for data to be available in the input FIFO
        if (audiop->ralc > 0 && audiop->rarc > 0) {  
            unsigned int left_input = audiop->ldata;  // Read left sample
            unsigned int right_input = audiop->rdata; // Read right sample

            // Wait for space in the output FIFOs
            while (audiop->wslc == 0);
            audiop->ldata = left_input;
						
            while (audiop->wsrc == 0);
            audiop->rdata = right_input;
        }
    }
}


int main(void){
	audio_connect();
	while(1);
}
	