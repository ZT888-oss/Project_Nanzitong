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
#define DAMPING 0.6 // Echo damping factor to make echo sounds different from original sound
struct audio_t *const audiop = ((struct audio_t *)0xff203040);

void delay() { //since delay is 0.4s -> 2.5Hz
    int cycle = 50000000 / 2.5; //clcok on DE1-Soc board frequency is 50MHz
    while (cycle--) {} 
}

void echo() {
    audiop->control = 0x8; // Clear output FIFOs
    audiop->control = 0x0; // Resume output conversion

    int sample_count = 0;
/*The DE1-SoC audio system (and CPUlator) uses a sampling rate of 8000 Hz (8 kHz).
This means the microphone captures 8000 samples per second.*/
    int record_duration = 3 * 8000;

    while (1) {
        if (sample_count < record_duration) { 
            if (audiop->rarc > 0 && audiop->ralc > 0) {
                unsigned int right_input = audiop->rdata;
                unsigned int left_input = audiop->ldata;

                while (audiop->wsrc == 0);
                audiop->rdata = right_input; // Play original sound

                while (audiop->wslc == 0);
                audiop->ldata = left_input;

                sample_count++; // Count samples
            }
        }

        // Delay loop for playing echo
        delay();

        // Replay the last recorded input with damping
        audiop->rdata = audiop->rdata * DAMPING; 
        audiop->ldata = audiop->ldata * DAMPING;
    }
}

int main() {
    echo();
    while (1);
}
