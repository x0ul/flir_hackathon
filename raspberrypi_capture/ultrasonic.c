#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
 
#define TRUE 1
 
#define TRIG 0
#define ECHO 1

#define GREEN_LED 5
#define RED_LED 4
 
void setup() {
        wiringPiSetup();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);
 
        //TRIG pin must start LOW
        digitalWrite(TRIG, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED, LOW);
        delay(30);
}
 
int getCM() {
        //Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;
 
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}
 
int main(void) {
        setup();
 
	while(1) {
		int distance = getCM();
        	printf("Distance: %dcm\n", distance);
		
		if(distance < 20) {
			digitalWrite(RED_LED, HIGH);
			digitalWrite(GREEN_LED, LOW);
		} else {
			digitalWrite(RED_LED, LOW);
			digitalWrite(GREEN_LED, HIGH);
		}
	}
 
        return 0;
}
