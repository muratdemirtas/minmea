#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include "minmea.h"

#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define SERIAL_BAUDRATE B9600           /* 9600 baudrate will bu use */
#define SERIAL_DEVICE "/dev/ttyUSB0"    /* serial device address definition */
#define SERIAL_BUFFER_SIZE 1024         /* usb serial buffer size definition*/
#define INDENT_SPACES "  "
void signal_handler_IO (int status);    /* definition of signal handler */

int wait_flag=true;                     /* TRUE while no signal received */
volatile bool STOP=false;               /* definition of test variable */

/**
 * @brief main main program entry point, async serial port reader application
 * @return none
*/
int main()
{
   int fd, available_bytes;                 /* definition of file descriptor and bytes counter*/
   struct termios oldtio,newtio;            /* definition of port structs*/
   struct sigaction saio;                   /* definition of signal action */
   char Serial_Buffer[SERIAL_BUFFER_SIZE];  /* definition of serial buffer*/

   //open the device to be non-blocking (read will return immediatly)
   fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

   //if file descriptor < 0 then exit
   if (fd <0) {

#ifdef CONSOLE_DEBUG
  perror(MODEMDEVICE); //print error code if available
#endif
       exit(-1);  //exit with -1 code
   }

   /* install the signal handler before making the device asynchronous */
   saio.sa_handler = signal_handler_IO;     //define callback function of signal handler
   saio.sa_flags = 0;                       //we dont use special flags(e.g SA_NOCLDSTOP),
   saio.sa_restorer = NULL;                 //restorer callback for signal handler, we will not use

   /* register signal handler callback for standart I/O operations, null for old sigaction struct*/
   sigaction(SIGIO, &saio, NULL);

   /* allow the process to receive SIGIO */
   fcntl(fd, F_SETOWN, getpid());

   /* Make the file descriptor asynchronous (the manual page says only
      O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
   fcntl(fd, F_SETFL, FASYNC);

   tcgetattr(fd,&oldtio); /* save current port settings */

   /* set new port settings for canonical input processing */
   newtio.c_cflag = SERIAL_BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
   newtio.c_iflag = IGNPAR | ICRNL;
   newtio.c_oflag = 0;
   newtio.c_lflag = ICANON;
   newtio.c_cc[VMIN]=1;
   newtio.c_cc[VTIME]=0;

   /* flush data descriptor for new application*/
   tcflush(fd, TCIFLUSH);

   /* set new port settings*/
   tcsetattr(fd,TCSANOW,&newtio);

   /* loop while waiting for input. normally we would do something
      useful here */
   while (STOP==false) {

     /* after receiving SIGIO, wait_flag = FALSE, input is available
        and can be read */
     if (wait_flag==false) {
       available_bytes = read(fd,Serial_Buffer,SERIAL_BUFFER_SIZE);
       if(available_bytes > 2) {
          Serial_Buffer[available_bytes] = 0;
          printf("Available :%d: Data is : %s", available_bytes,(char*) Serial_Buffer );

         Parse_Json_Data(&Serial_Buffer);
        }

       wait_flag = true;      /* wait for new input */
     }

   }
   /* restore old port settings */
   tcsetattr(fd,TCSANOW,&oldtio);
 }

/**
 * @brief Parse_Json_Data
 * @param GPS_Raw_Data
 */
int Parse_Json_Data(char* GPS_Raw_Data)
{

            switch (minmea_sentence_id(GPS_Raw_Data, false)) {
                case MINMEA_SENTENCE_RMC: {
                    struct minmea_sentence_rmc frame;
                    if (minmea_parse_rmc(&frame,GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                                frame.latitude.value, frame.latitude.scale,
                                frame.longitude.value, frame.longitude.scale,
                                frame.speed.value, frame.speed.scale);
                        printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                                minmea_rescale(&frame.latitude, 1000),
                                minmea_rescale(&frame.longitude, 1000),
                                minmea_rescale(&frame.speed, 1000));
                        printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                minmea_tocoord(&frame.latitude),
                                minmea_tocoord(&frame.longitude),
                                minmea_tofloat(&frame.speed));
                    }
                    else {
                        printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
                    }
                    else {
                        printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GST: {
                    struct minmea_sentence_gst frame;
                    if (minmea_parse_gst(&frame,GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                                frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                                frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                                frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                        printf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
                               " scaled to one decimal place: (%d,%d,%d)\n",
                                minmea_rescale(&frame.latitude_error_deviation, 10),
                                minmea_rescale(&frame.longitude_error_deviation, 10),
                                minmea_rescale(&frame.altitude_error_deviation, 10));
                        printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                                minmea_tofloat(&frame.latitude_error_deviation),
                                minmea_tofloat(&frame.longitude_error_deviation),
                                minmea_tofloat(&frame.altitude_error_deviation));
                    }
                    else {
                        printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_GSV: {
                    struct minmea_sentence_gsv frame;
                    if (minmea_parse_gsv(&frame, GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                        printf(INDENT_SPACES "$xxGSV: sattelites in view: %d\n", frame.total_sats);
                        for (int i = 0; i < 4; i++)
                            printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                                frame.sats[i].nr,
                                frame.sats[i].elevation,
                                frame.sats[i].azimuth,
                                frame.sats[i].snr);
                    }
                    else {
                        printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
                    }
                } break;

                case MINMEA_SENTENCE_VTG: {
                   struct minmea_sentence_vtg frame;
                   if (minmea_parse_vtg(&frame, GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
                               minmea_tofloat(&frame.true_track_degrees));
                        printf(INDENT_SPACES "        magnetic track degrees = %f\n",
                               minmea_tofloat(&frame.magnetic_track_degrees));
                        printf(INDENT_SPACES "        speed knots = %f\n",
                                minmea_tofloat(&frame.speed_knots));
                        printf(INDENT_SPACES "        speed kph = %f\n",
                                minmea_tofloat(&frame.speed_kph));
                   }
                   else {
                        printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
                   }
                } break;

                case MINMEA_SENTENCE_ZDA: {
                    struct minmea_sentence_zda frame;
                    if (minmea_parse_zda(&frame, GPS_Raw_Data)) {
                        printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                               frame.time.hours,
                               frame.time.minutes,
                               frame.time.seconds,
                               frame.date.day,
                               frame.date.month,
                               frame.date.year,
                               frame.hour_offset,
                               frame.minute_offset);
                    }
                    else {
                        printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
                    }
                } break;

                case MINMEA_INVALID: {
                    printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
                } break;

                default: {
                    printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
                } break;
            }


        return 0;
}



/**
 * @brief signal_handler_IO this function used to indicate main loop for new
 * characters have been received by operating system
 * @param status  signal handler status code
 */
void signal_handler_IO (int status)
{

  //print debug message if it necessary for you
#ifdef CONSOLE_DEBUG
    printf("Signal Handler Received Status Code %d\r\n",(int)status );
#endif

  wait_flag = false;   //set flag false to continue main loop
  return;
}
