//This program is used for bidirectional communication between PC and AVR/Atmega88.

#include "serialport.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>

int sp = -1;

/* ------------------------------------------------------------(      FUNCTIONS      )------------------------------------------------------------ */

void reset_AVR_without_confirmation(void){
    tcflush(sp, TCIOFLUSH);
    uint8_t reset_AVR = 222;
    write(sp, &reset_AVR, 1);
    tcflush(sp, TCIOFLUSH);
    printf("\nReset AVR without confirmation.\n");
}

void reset_AVR(void){
    tcflush(sp, TCIOFLUSH);
    uint8_t reset_AVR = 222;
    uint8_t reset_AVR_confirmed;
    write(sp, &reset_AVR, 1);
    read(sp, &reset_AVR_confirmed, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
    printf("                                    reset_AVR: %hhu, reset_AVR_confirmed: %hhu\n", reset_AVR, reset_AVR_confirmed);
    sleep(1);
    tcflush(sp, TCIOFLUSH);
    printf("AVR should now be reset.\n");
}

/* Function for signal handler for SIGINT (Ctrl+C). */
void forced_exit(int sig) {
    if (sp != -1) {
        reset_AVR_without_confirmation();
        printf("Exiting program through Ctrl+C. Closing serial port...\n\n");
        printf("-----------------------------------------------------------------------------------------------\n\n");
        serial_cleanup(sp);
    }
    exit(0);
}

/* Function to initiate serial port and signal handler. */
int init_all(){
    // Register the signal handler for SIGINT (Ctrl+C)
    signal(SIGINT, forced_exit);
    printf("Info: Program is configured to close serial port when exiting through Ctrl+C.\n\n");

    //Initiate serial port
	sp = serial_init("/dev/ttyS0", 0); // Non-canonical mode.
	if(sp == 0) {
		printf("Error! Serial port could not be opened.\n\n");
        return -1;
	}
	else {
		printf("Serial port open with identifier %d \n\n",sp);
	}
    return 0;
}


/* ----------------------------------------------------------(      FUNCTIONS END      )---------------------------------------------------------- */




/* --------------------------------------------------------------(      MAIN      )-------------------------------------------------------------- */

int main(void){

    if(init_all() != 0){
        return -1;
    }

    reset_AVR();

    while(1){

        printf("-------------------------------------------------------------------------------------------------\n");
        printf("\nOptions Menu:\n1) Set speed\n2) Read speed once\n3) Read speed continuously\n4) Get duty cycle\n5) Speed reading test\n6) Toggle fine-tuning on/off\n7) Get fine-tuned rpm setpoint\n8) Get rpm setpoint\n222) Reset AVR\nENTER option: ");
        uint8_t option;
        uint8_t option_confirmed;
        scanf("%hhu", &option);
        write(sp, &option, 1);
        read(sp, &option_confirmed, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
        printf("                                    option: %hhu, option_confirmed: %hhu\n", option, option_confirmed);

        switch (option){
            case 1: {// SET SPEED
                uint8_t speed;
                uint8_t speed_confirmed;
                printf("ENTER desired RPM (5-120): ");
                scanf("%hhu", &speed);
                write(sp, &speed, 1);
                read(sp, &speed_confirmed, 1); // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                    speed: %hhu, speed_confirmed: %hhu\n", speed, speed_confirmed);
                printf("                                                                           Speed updated to %hhu.\n", speed_confirmed);
                break;
            }
            case 2: {// READ SPEED ONCE
                uint8_t measured_speed;
                read(sp, &measured_speed, 1); // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                                                           Current speed is %hhu\n", measured_speed);
                break;
            }
            case 3: {// READ SPEED CONTINUOUSLY
                uint8_t measured_speed_cont;
                printf("                                                                           Current speed is:\n");
                while(1) {
                    read(sp, &measured_speed_cont, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                    printf("                                                                                            %hhu\n", measured_speed_cont);
                }
                break;
            }
            case 4: {// GET DUTY CYCLE
                uint8_t duty;
                read(sp, &duty, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                                                           Duty cycle is %hhu\n", duty);
                break;
            }
            case 5: {// SPEED READING TEST (was used to study motor behaviour before implementing PI-controller)
                uint8_t range_lower;
                uint8_t range_upper;
                uint8_t increment;
                uint8_t readings;
                uint8_t readings_confirmed;
                printf("ENTER range lower: ");
                scanf("%hhu", &range_lower);
                printf("ENTER range upper: ");
                scanf("%hhu", &range_upper);
                printf("ENTER increment: ");
                scanf("%hhu", &increment);
                printf("ENTER readings per increment: ");
                scanf("%hhu", &readings);

                write(sp, &readings, 1);
                read(sp, &readings_confirmed, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                    readings: %hhu, readings_confirmed: %hhu\n", readings, readings_confirmed);

                printf("TEST STARTED\n");

                for (uint8_t speed = range_lower; speed <= range_upper; speed += increment) {
                    uint8_t speed_confirmed;
                    write(sp, &speed, 1);
                    read(sp, &speed_confirmed, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                    printf("                                    speed: %hhu, speed_confirmed: %hhu\n", speed, speed_confirmed);
                    printf("                                                                           Speed updated to %hhu.\n", speed_confirmed);
                    
                    printf("                                                                           Current speed is\n");
                    uint8_t measured_speed;
                        for (uint8_t current_readings = 0; current_readings < readings; current_readings += 1) {
                            read(sp, &measured_speed, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                            printf("                                                                                       %hhu\n", measured_speed);
                        }
                }

                reset_AVR();
                printf("TEST FINISHED\n");
                break;
            }
            case 6: {// TOGGLE FINE-TUNING ON/OFF
                uint8_t fine_tuning;
                read(sp, &fine_tuning, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                                                           Fine tuning is %hhu", fine_tuning);
                if(fine_tuning == 0){
                    printf(" = OFF\n");
                }
                else if(fine_tuning == 1){
                    printf(" = ON\n");
                }
                break;
            }
            case 7: {// GET FINE-TUNED RPM SETPOINT
                uint8_t tuned_rpm;
                read(sp, &tuned_rpm, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                                                           Fine-tuned RPM is %hhu\n", tuned_rpm);
                break;
            }
            case 8: {// GET RPM SETPOINT
                uint8_t set_rpm;
                read(sp, &set_rpm, 1);  // Serial port is configured to make read() block until exactly 1 byte is available.
                printf("                                                                           Setpoint RPM is %hhu\n", set_rpm);
                break;
            }
            case 222: {
                reset_AVR();
            }
        }

        printf("Returning to menu...\n\n");
    }

    return -1;

}


/* ------------------------------------------------------------(      MAIN END      )------------------------------------------------------------ */



