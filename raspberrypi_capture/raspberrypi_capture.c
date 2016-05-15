/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>

#include <time.h>
#include <wiringPi.h>

#define TRIG 0
#define ECHO 2

#define RED_LED 5
#define GREEN_LED 4
 
#define FLIR_CS 6

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

void pin_setup(void) {
        wiringPiSetup();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);

	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	pinMode(FLIR_CS, OUTPUT);
 
        //TRIG pin must start LOW
        digitalWrite(TRIG, LOW);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(FLIR_CS, HIGH);
        delay(30);
        digitalWrite(FLIR_CS, LOW);
}

void loop(void);
void pulse_flir_cs(void);

static const char *device = "/dev/spidev0.1";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 16000000;
static uint16_t my_delay;

#define VOSPI_FRAME_SIZE (164)
uint8_t lepton_frame_packet[VOSPI_FRAME_SIZE];
static unsigned int lepton_image[80][80];

static void save_pgm_file(void)
{
	int i;
	int j;
	unsigned int maxval = 0;
	unsigned int minval = UINT_MAX;
	char image_name[32];
	int image_index = 0;

	time_t timer;
	char datebuffer[26];
	struct tm* tm_info;

	time(&timer);
	tm_info = localtime(&timer);

	strftime(datebuffer, 26, "%Y%m%d_%H%M%S", tm_info);

	do {
		sprintf(image_name, "IMG_%s_%.4d.pgm", datebuffer, image_index);
		image_index += 1;
		if (image_index > 9999) 
		{
			image_index = 0;
			break;
		}

	} while (access(image_name, F_OK) == 0);

	FILE *f = fopen(image_name, "w");
	if (f == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	unsigned long row_average[60] = {0};

	printf("Calculating min/max values for proper scaling...\n");
	for(i=0;i<60;i++)
	{
		unsigned long row_total = 0;
		for(j=0;j<80;j++)
		{
			if (lepton_image[i][j] > maxval) {
				maxval = lepton_image[i][j];
			}
			if (lepton_image[i][j] < minval) {
				minval = lepton_image[i][j];
			}
			row_total += lepton_image[i][j];
		}
		row_average[i] = row_total / 80;
	}
	unsigned long total = 0;
	for(i=0; i<60;i++) {
		total += row_average[i];
	}
	printf("average = %lu\n", total / 60);
	
	printf("maxval = %u\n",maxval);
	printf("minval = %u\n",minval);
	
	fprintf(f,"P2\n80 60\n%u\n",maxval-minval);
	for(i=0;i<60;i++)
	{
		for(j=0;j<80;j++)
		{
			fprintf(f,"%d ", lepton_image[i][j] - minval);
		}
		fprintf(f,"\n");
	}
	fprintf(f,"\n\n");

	fclose(f);
}

int transfer(int fd)
{
	int ret;
	int frame_number = -1;
	uint8_t tx[VOSPI_FRAME_SIZE] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)lepton_frame_packet,
		.len = VOSPI_FRAME_SIZE,
		.delay_usecs = my_delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	if(((lepton_frame_packet[0]&0xf) != 0x0f))
	{
		frame_number = lepton_frame_packet[1];

		if(frame_number < 60 )
		{
			for(int i=0;i<80;i++)
			{
				lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);
			}
		}
	}
	return frame_number;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;


	fd = open(device, O_RDWR);
	if (fd < 0)
	{
		pabort("can't open device");
	}

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't get spi mode");
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't get bits per word");
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't get max speed hz");
	}

	pin_setup();

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	//pulse_flir_cs();

	int timeout = 0;

	while(transfer(fd)!=59 && (timeout < 2000)){timeout++;}
	if(timeout > 500) {
		printf("timeout higher than 500 ?? %d \n", timeout);
	}

	close(fd);

	loop(); // run the loop once

//	save_pgm_file();

	return ret;
}

int getCM() {
        //Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
	// TODO: Timeout faster on >100cm echoes
        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;
 
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}

long getEchoMicroseconds() {
        //Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
	// TODO: Timeout faster on >100cm echoes
        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;

	return travelTime;
}
 
 
typedef enum grillStatus_t {
	tooCold,
	tooSmall,
	justRight
} grillStatus_t;

void getFLIR(void){
	// nothing here
	// TODO: Copy FLIR setup/capture from main()
	;
}

typedef enum tempRating_t {
	ice,
	human,
	hot
} tempRating_t;

grillStatus_t processFLIR(void){
	// Image data in  lepton_image[80][80];
	// Look at the middle row.  
	tempRating_t temp_guess[80] = {0};
	unsigned int max = 0;
	unsigned int min = INT_MAX;
	
	for(int i=0; i<80; i++) {
		unsigned int raw_value = lepton_image[29][i];
		if(raw_value > max) max = raw_value;
		if(raw_value < min) min = raw_value;
		if(raw_value < 7500) { // Less than human body temp
			temp_guess[i] = ice;
		} else if (raw_value < 8500) { // Between 20-40 C
			temp_guess[i] = human;
		} else { // Greater than 40C, assume that's our heat source
			temp_guess[i] = hot;
		}
	}
	unsigned int left = temp_guess[0];
	unsigned int middle = temp_guess[39];
	unsigned int right = temp_guess[79];
	//Assume we're looking at circular heat source
	if(max < 8500) { // Not enough heat in image
		return tooCold;
	} else if (left==ice && right==ice && middle == hot) { // Heat detected, but not wide enough
		return tooSmall;
	} else if (left==hot || right==hot) { // Heat detected, goes off edge
		// Assume it's too big
		return justRight;
	} else {  // Too Cold check failed, assume heat was seen but not enough
		return tooSmall;
	}
}

void setLEDs(grillStatus_t gs){
	switch(gs){
		case tooCold: 
			digitalWrite(RED_LED, HIGH);
			digitalWrite(GREEN_LED, LOW);
			break;
		case tooSmall: 
			digitalWrite(RED_LED, HIGH);
			digitalWrite(GREEN_LED, HIGH);
			break;
		case justRight: 
			digitalWrite(RED_LED, LOW);
			digitalWrite(GREEN_LED, HIGH);
			break;
		default: 
			digitalWrite(RED_LED, LOW);
			digitalWrite(GREEN_LED, LOW);
			break;
	}
}

void storeResults(void) {
	//save_pgm_image()
	// Save thermally-adjusted distance measurement
	// Estimate size of grillable surface & store
	float distance = (float) getCM();
	// FOV is 51 degrees, so horizontal measurement is tan(51/2) * h
	double view_radius = 0.47697553f * distance;
	
}

void loop(void) {
	// Ultrasonic range
	long echoTime = getEchoMicroseconds();
	if(echoTime > 1200) { // 1200usec = 20+ cm distance
		return; // don't talk to Thermal Camera on short distance
	}
	
	// Something's close, get a Thermal Image
	// TODO: Pass FLIR image around
	getFLIR();
	
	// Process TI to determine grill status
	grillStatus_t grillStatus = processFLIR();

	// Indicate status to user
	setLEDs(grillStatus);


	// Store Thermal Image and Distance
	storeResults();
}

	

void pulse_flir_cs(void) {
        digitalWrite(FLIR_CS, HIGH);
        delay(200);
        digitalWrite(FLIR_CS, LOW);
        delay(100);
}
