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
volatile unsigned int* const switches = ((volatile unsigned *)0xff200040);
void delay(unsigned int freq){
	int cycle = 50000000/freq;   //assume clock running at frequency 50MHz, use 50MHz divide frequency of square wave to get number of iterations for dealy loop
	while(cycle != 0){
		cycle --;
	}
}
void generate_square_wave(){
	audiop->control = 0x8; // Clear output FIFOs
    audiop->control = 0x0; // Resume input conversion
	 int switch_value = *switches & 0x03ff; //get stats of switch using and gate, only consider lower 10 bits:1111111111
	 int frequency = 100; //audible range starts at 100Hz;
	 int wave_state =0; //0 is lowest value

/*linear scalling:  linearly maps the 10-bit switch value (which is between 0 and 1023) to 
a new value between 100 Hz and 2000 Hz. why divide 1023: largest switches values is 1111111111, which is 1023*/
	while(1){
	frequency = 100 + (switch_value * (2000 - 100) / 1023); //get the frequency when changing switches.
	wave_state = wave_state^1; //toogle square wave state by xor gate
		
	if(wave_state == 0){
		audiop->wsrc = 0x00;
		audiop->wslc = 0x00;
	}else{
		audiop->wsrc = 0xff;
		audiop->wslc = 0xff;
	}
	//use a delay loop to show change of frequency in square_wave
	delay(frequency);
	}
}
int main(){
 generate_square_wave();
	while(1);
}