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
#include <stdbool.h>
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

typedef enum grillStatus_t {
    noGrill,
    tooCold,
    tooSmall,
    justRight,
    tooHot,
    tooClose,
    invalid
} grillStatus_t;

static const char* grillStatusStrings[] = {
	"noGrill", "tooCold", "tooSmall", "justRight", "tooHot", "tooClose", "invalid"};


typedef enum displayColor {
    DISPLAYCOLOR_OFF,
    DISPLAYCOLOR_RED,
    DISPLAYCOLOR_GREEN,
    DISPLAYCOLOR_ORANGE
} displayColor;

void loop(void);
void pulse_flir_cs(void);
uint32_t get_distance_cm_at_temp_c(uint32_t temperature_c, uint32_t pulse_us);
void showStatus(grillStatus_t new_status);
void setDisplay(displayColor color);

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
		sprintf(image_name, "images/IMG_%s_%.4d.pgm", datebuffer, image_index);
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

int fd;

int main(int argc, char *argv[])
{
	int ret = 0;


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

	while (1)
	{
		loop(); // run the loop once
		delay(200);
	}

	close(fd);
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
 
 
void getFLIR(void){
	int timeout = 0;
	while(transfer(fd)!=59 && (timeout < 2000)){timeout++;}
	if(timeout > 500) {
		printf("timeout higher than 500 ?? %d \n", timeout);
	}
	if (timeout >= 2000) {
		// Gave up on image, puls chip select to reset Lepton and delay 13 frames
		pulse_flir_cs();
		delay(500);
	}

}

// Image processing function
// General approach is to compute a histogram of temperature values
// If there's enough "hot" pixels, signal that the fire's ready
// If there aren't, but the 'edges' are hot, user may need to reposition
// A pixel on the edge is worth two in the middle

unsigned int bins[10] = {0};
// Bins are 0-7k, 500 each up to 9000, then 9000+

unsigned int temp_c = 0;

grillStatus_t processFLIR(uint32_t echoTime){
	// Image data in  lepton_image[80][80];
	// Look at the middle row.  
	unsigned int max = 0;
	unsigned int min = INT_MAX;
	for(int garbage=0; garbage<10; garbage++) {
		bins[garbage] = 0;
	}

	unsigned long total = 0;
	
	for(int i=0; i<60; i++) {
		for(int j=0; j<80; j++) {
			unsigned int raw_value = lepton_image[i][j];
			total += raw_value;
			if(raw_value > max) max = raw_value;
			if(raw_value < min) min = raw_value;
			if(raw_value < 7000) {
				bins[0]++;
			} else if (raw_value > 9000) {
				bins[5]++;
			} else {
				bins[(raw_value-7000)/500 + 1]++;
			}
		}
	}
	unsigned int average = total/4800;
	printf("Max: %d Min: %d Avg: %d Total: %d\n", max, min, average, total);
	temp_c = (average - 7143) / 29;
	int distance = get_distance_cm_at_temp_c(temp_c, echoTime);
	if (distance > 80) {
		return noGrill;
	}
	printf("Temperature: %d Corrected distance is: %d  Raw: %d \n", temp_c, distance, echoTime/58);
	if(min==0 || max>50000) {
		// Lepton wasn't ready, pulse CS and consider image invalid
		return invalid;
	}
	// "Hot" bins are > 8500, meaning bins 8 & 9
	unsigned int hotCount = bins[4]+bins[5];
	if(max < 8500 || (hotCount < 400)) { // Not enough heat in image
		return tooCold;
	//} else if (hotCount < 1500) {
	} else {
		if(distance <10) {
			//Some heat, kinda close, flash red
			return tooClose;
		}
		// Might be hot enough, edge check can push to green
		double sqcm_per_pixel = ((0.47697553 * distance * 2) * (0.47697553 * distance * 2)) / 4800;
		double hot_sqcm = hotCount * sqcm_per_pixel;
		printf("Hot area: %f \n", hot_sqcm);
		if(hot_sqcm > 100) {
			return justRight;
		} else {
			return tooSmall;
		}
	}
	return tooCold;
}

// TODO Handle invalid and too close... Invalid means "hold what I was doing, pray it starts working again"

// Drive the user interface, make the "good" states persist longer than the
// bad, except for tooHot, whis is important to show quickly.
//
// noGrill      -> ALL OFF
// tooCold      -> RED SOLID
// tooSmall     -> ORANGE SOLID
// justRight    -> GREEN SOLID
// tooHot       -> RED BLINK
// tooClose     -> RED BLINK
//
// Flash red if in good state for > 10 seconds
void showStatus(grillStatus_t new_status){
    static const uint32_t blink_pulsewidth_us = 500;
    static const uint32_t ux_change_threshold_us = 100000; // TODO work out a reasonable constant

    static grillStatus_t pending_status = noGrill;
    static grillStatus_t displayed_status = noGrill;
    static uint32_t status_change_timeout;
    static uint32_t next_blink_update_us;
    static bool toggle;

    printf("new_status: %s\n", grillStatusStrings[new_status]);

    uint32_t now = micros();

    // Set new timeout if pending status changes
    if (new_status != pending_status) {
        status_change_timeout = ux_change_threshold_us + now;
        pending_status = new_status;
    }

    bool timeout_expired = now >= status_change_timeout;

    // Handle priority statuses immediately
    if (new_status == justRight)
    {
        setDisplay(DISPLAYCOLOR_GREEN);
        displayed_status = justRight;
        return;
    }
    // Blink red
    else if (new_status == tooHot || new_status == tooClose)
    {
        if (now >= next_blink_update_us)
        {
            next_blink_update_us = now + blink_pulsewidth_us;
            setDisplay(toggle ? DISPLAYCOLOR_RED : DISPLAYCOLOR_OFF);
            toggle = !toggle;
        }
        displayed_status = new_status;
        return;
    }

    // Handle regular statuses
    if (timeout_expired) {
        switch(pending_status){
            case noGrill:
                setDisplay(DISPLAYCOLOR_OFF);
                break;

            case tooCold: 
                setDisplay(DISPLAYCOLOR_RED);
                break;

            case tooSmall: 
                setDisplay(DISPLAYCOLOR_ORANGE);
                break;

            default:
                break;
        }

        displayed_status = pending_status;
    }
}

void setDisplay(displayColor color)
{
    switch (color)
    {
        case DISPLAYCOLOR_OFF:
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            break;

        case DISPLAYCOLOR_RED:
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, LOW);
            break;

        case DISPLAYCOLOR_GREEN:
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);
            break;

        case DISPLAYCOLOR_ORANGE:
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
            break;
    }
}

void storeResults(uint32_t temperature_c, uint32_t pulse_us, grillStatus_t gs) {
	// Save thermally-adjusted distance measurement
	// Estimate size of grillable surface & store
	if(gs != noGrill) {
		// Store the Thermal Image for later review
		save_pgm_file();
		float distance = (float) get_distance_cm_at_temp_c(temperature_c, pulse_us);
		// FOV is 51 degrees, so horizontal measurement is tan(51/2) * h
		double view_radius = 0.47697553f * distance;
	}
	
}

void loop(void) {
	// Ultrasonic range
	long echoTime = getEchoMicroseconds();
	grillStatus_t grillStatus  = noGrill;
	if(echoTime < 4000) { // 1200usec = 20+ cm distance
		// Something's close, get a Thermal Image
		// TODO: Pass FLIR image around
		getFLIR();
		
		// Process TI to determine grill status
		grillStatus = processFLIR(echoTime);
	}

	// Indicate status to user
	showStatus(grillStatus);

	// Store Thermal Image and Distance
	storeResults(temp_c, echoTime, grillStatus);
}


void pulse_flir_cs(void) {
        digitalWrite(FLIR_CS, HIGH);
        delay(200);
        digitalWrite(FLIR_CS, LOW);
        delay(100);
}

uint32_t get_distance_cm_at_temp_c(uint32_t temperature_c, uint32_t pulse_us)
{
    return (pulse_us * (331300 + 606 * temperature_c)) / 20000000;
}

