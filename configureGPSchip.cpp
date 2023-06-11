#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h>

//g++ -Wall -o configureGPSchip -g configureGPSchip.cpp -lwiringPi
using namespace std;

template <size_t N>
void inplace_checksum( unsigned char (&arr)[N])
{
    //checksum
    uint32_t CK_A = 0, CK_B = 0;
    int len = sizeof(arr)/sizeof(arr[0]);
    for(int i=2; i<(len-2); i++) //skip first two, end two early if we include spots for checksum
    {
        CK_A = CK_A + (unsigned char)arr[i];
        CK_B = CK_B + CK_A;
    }

    uint8_t sum1=0x00, sum2=0x00;
    sum1 = CK_A & 0xff;//Mask the checksums to be one byte
    sum2 = CK_B & 0xff;

    arr[len-2] = (char)sum1;
    arr[len-1] = (char)sum2;
}

int validate_ack( unsigned char (&arr)[10])
{
//this chunk will adjust for up to 2 byte read of lag on the ACK message from receiver
   int i = 0;
   if( arr[2] == 0xFF ){
      printf("Did not receive good ACK message from receiver.\n");
      return 1;
   }else if( arr[1] == 0xFF ){
      i = 2;
   }else if( arr[0] == 0xFF ){
      i = 1;
   }

// validate ACK header and message acknowledgement for the configuration we support in this binary
   //validate header
   if( !( arr[i] == 0xB5 && arr[i+1] == 0x62 ) ){
      printf("Invalid ACK header, something went wrong, expect configuration not applied.\n");
      return 1; 
   }

   //handle NAKs
   if( arr[i+2] == 0x05 && arr[i+3] == 0x00 ){
      printf("NAK received.\n");
      return 1;
   }else if( arr[i+2] == 0x05 && arr[i+3] == 0x01 ){
   //all valid, parse the message ID that has been ACK'ed
      printf("UBX Message acknowledged for: ");
      uint16_t class_id = (arr[i+6] << 8) | arr[i+7];
      switch( class_id ) {
         case 0x0600:
            printf("Baud Rate config\n");
            break;
         case 0x0608:
            printf("Measurement rate config\n");
            break;
         case 0x0624:
            printf("Dynamic mode config\n");
            break;
         case 0x0601:
            printf("Messaging rate config\n");
            break;
         default:
            printf("ACK message not linkable to an expected configuration message.\n");
            printf("Something went wrong. Assume GPS receiver configuration is undetermined.\n");
            return 1;
      }
      return 0;
   }

   printf("Major error, you shouldn't be here o.O\n");
   return 2;
}

unsigned char ack_buf[10] = {
   0x00, 0x00,   // header
   0x00,         // class
   0x00,         // id
   0x00, 0x00,   // length of payload
   0x00,       // class of ack msg
   0x00,       // msgID of ack msg
   0x00, 0x00   // checkSum
};

unsigned char baudrate_buf[28] = {
   0xB5, 0x62,                       // header
   0x06,                             // class
   0x00,                             // id
   0x14, 0x00,                       // length of payload
   0x01,                             // portID (0x01 = UART1 [serial])
   0x00,                             // reserved1
   0x00, 0x00,                       // txReady
   0xC0, 0x08, 0x00, 0x00,           // mode (default: 8 bit char len, no parity, see ublox manual)
   0x00, 0xC2, 0x01, 0x00,           // baud rate (default 0x0001C200 = 115200)
   0x07, 0x00,                       // inMask (default: inUbx, inNmea, inRtcm2)
   0x03, 0x00,                       // outMask (default: outUbx, outNmea)
   0x00, 0x00,                       // flags
   0x00, 0x00,                       // reserved2
   0x00, 0x00                        // checksum
};


#define DTM_ID    0x0A
#define GBQ_ID    0x44
#define GBS_ID    0x09
#define GGA_ID    0x00
#define GLL_ID    0x01
#define GLQ_ID    0x43
#define GNQ_ID    0x42
#define GNS_ID    0x0D
#define GPQ_ID    0x40
#define GRS_ID    0x06
#define GSA_ID    0x02
#define GST_ID    0x07
#define GSV_ID    0x03
#define THS_ID    0x0E
#define TXT_ID    0x41
#define VLW_ID    0x0F
#define VTG_ID    0x05
#define ZDA_ID    0x08
#define RMC_ID    0x04
int changeIDrate(char currentID, char rate, int handle)
{
   unsigned char messagingrate_buf[16] = {
      0xB5, 0x62,                           // header
      0x06,                                 // class
      0x01,                                 // id
      0x08, 0x00,                           // length of payload
      0xF0,                                 // msgClass (NMEA messages all have class ID 0xF0)
      0x00,                                 // msgID (see above defines)
      0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   // rate (rpt for measurement)
      0x00, 0x00                            // checksum
   };

   messagingrate_buf[7] = currentID;
   messagingrate_buf[9] = rate;

   inplace_checksum(messagingrate_buf);

   int rv = -1;
   rv = write(handle, messagingrate_buf, 16);
   if( rv != 16 )
   {
      printf("Failed to write messaging rate change for %x.\n", currentID);
      return 1;
   }

   return 0;
};


unsigned char measurementrate_buf[14] = {
   0xB5, 0x62,   // header
   0x06,         // class
   0x08,         // id
   0x06, 0x00,   // length of payload
   0xFA, 0x00,   // measurement rate (500ms = 0x01F4) (250ms = 0x00FA)
   0x01, 0x00,   // nav rate 1 meas = 1 report
   0x01, 0x00,   // time reference (UTC time)
   0x00, 0x00    // checksum
};

unsigned char dynamicmode_buf[44] = {
   0xB5, 0x62,                          // header
   0x06,                               // class
   0x24,                               // id
   0x24, 0x00,                           // length of payload

   0x01, 0x00,                         // mask attempt
   0x04,                               // dynModel (0x04 = automotive)
   0x00,                               // fixMode
   0x00, 0x00, 0x00, 0x00,             // fixedAlt
   0x00, 0x00, 0x00, 0x00,             // fixedAltVar
   0x00,                               // minElev
   0x00,                               // drLimit
   0x00, 0x00,                         // pDop
   0x00, 0x00,                         // tDop
   0x00, 0x00,                         // pAcc
   0x00, 0x00,                         // tAcc
   0x00,                               // staticHoldThresh
   0x00,                               // dgnssTimeout
   0x00,                               // cnoThreshNumSVs
   0x00,                               // cnoThresh
   0x00, 0x00,                         // reserved1
   0x00, 0x00,                         // staticHoldMaxDist
   0x00,                               // utcStandard
   0x00, 0x00, 0x00, 0x00, 0x00,       // reserved2

   0x00, 0x00                          // checksum
};

typedef struct
{
   uint32_t baudRate;
   bool rmcOnly; 
   uint16_t measurementRate; // in ms
   int8_t dynamicMode;

} CfgOptions;

void printUsage()
{
   cout << "Usage:" << std::endl;
   cout << "./configGPS [options]" << std::endl;
   cout << "-h - help. Prints usage." << std::endl;
   cout << "-b - baud rate, int" << std::endl;
   cout << "-t - toggle rmcOnly, (1/0) bool " << std::endl;
   cout << "-r - measurementRate, int (in ms)" << std::endl;
   cout << "-m - dynamicMode, int (4 = automotive, otherwise, see ublox chip manual)" << std::endl;
}

void processOptions(int argc, char** argv, CfgOptions* opt)
{
   for(int i = 1; i < argc; i++)
   {
      if(argv[i][0] == '-')
      {
         switch(argv[i][1])
         {
            case 'h':
               printUsage();
               exit(-1);
               break;
            case 'b':
               i++;
               opt->baudRate = atoi(argv[i]);
               break;
            case 't':
               i++;
               opt->rmcOnly = (bool) (atoi(argv[i]));
               break;
            case 'r':
               i++;
               opt->measurementRate = atoi(argv[i]);
               break;
            case 'm':
               i++;
               opt->dynamicMode = atoi(argv[i]);
               break;
            default:
               printUsage();
               exit(-1); 
               break;
         }
      }
   }
}

void reviewOptions(CfgOptions* opt)
{
   //no options are mandatory for this config script
   //so this harness for user args sanity checks is empty
   bool toExit = false;

   if(toExit)
   {
      exit(-1);
   }

}

  
int main(int argc, char** argv)
{
//handle command line args
   CfgOptions opt;
   opt.baudRate = 0;
   opt.rmcOnly = false;
   opt.measurementRate = 0;
   opt.dynamicMode = -1;

   processOptions(argc, argv, &opt);
   reviewOptions(&opt);

//configure messages
   if( opt.baudRate > 0 )
   {
      uint8_t b1, b2, b3, b4;
      b1 = opt.baudRate & 0xFF;
      b2 = (opt.baudRate >> 8) & 0xFF;
      b3 = (opt.baudRate >> 16) & 0xFF;
      b4 = (opt.baudRate >> 24) & 0xFF;
      baudrate_buf[14] = b1;
      baudrate_buf[15] = b2;
      baudrate_buf[16] = b3;
      baudrate_buf[17] = b4;
   }

   if( opt.measurementRate > 0 )
   {
      uint8_t b1, b2;
      b1 = opt.measurementRate & 0xFF; 
      b2 = (opt.measurementRate >> 8) & 0xFF; 
      measurementrate_buf[6] = b1;
      measurementrate_buf[7] = b2;
   }

   if( opt.dynamicMode >= 0 )
   {
      dynamicmode_buf[8] = opt.dynamicMode;
   }

//open i2c device handle
   int handle = -1;
   handle = wiringPiI2CSetup(0x42);
   if (handle == -1) {
      printf("Failed to open I2C device.\n");
      return 1;
   }

//Baud Rate Set and ACK
   //compute checksum and write message
   inplace_checksum(baudrate_buf);
   int rv = -1;
   rv = write(handle, baudrate_buf, 28);
   if( rv != 28 )
   {
      printf("Failed to write baud rate configuration.\n");
      return 1;
   }
   //receive and verify ack
   rv = -1;
   rv = read(handle, ack_buf, 10);
   if( rv != 10 )
   {
      printf("Failed to read ACK for baud rate configuration.\n");
      return 1;
   }
   rv = validate_ack(ack_buf);
   if( rv != 0 ){ return 1; }

   sleep(2);

//Measurement Rate Set and ACK
   //compute checksum and write message
   inplace_checksum(measurementrate_buf);
   rv = -1;
   rv = write(handle, measurementrate_buf, 14);
   if( rv != 14 )
   {
      printf("Failed to write measurement rate configuration.\n");
      return 1;
   }
   //receive and verify ack
   rv = -1;
   rv = read(handle, ack_buf, 10);
   if( rv != 10 )
   {
      printf("Failed to read ACK for measurement rate configuration.\n");
      return 1;
   }
   rv = validate_ack(ack_buf);
   if( rv != 0 ){ return 1; }

   sleep(2);

//Dynamic Mode Set and ACK
   //compute checksum and write message
   inplace_checksum(dynamicmode_buf);
   rv = -1;
   rv = write(handle, dynamicmode_buf, 44);
   if( rv != 44 )
   {
      printf("Failed to write dynamic mode configuration.\n");
      return 1;
   }
   //receive and verify ack
   rv = -1;
   rv = read(handle, ack_buf, 10);
   if( rv != 10 )
   {
      printf("Failed to read ACK for dynamic mode configuration.\n");
      return 1;
   }
   rv = validate_ack(ack_buf);
   if( rv != 0 ){ return 1; }

   sleep(2);

//Handle RMC only toggle
   if( opt.rmcOnly )
   {
      rv = changeIDrate(GGA_ID, 0x00, handle); 
      if( rv != 0){ return 1; }
      rv = -1;
      rv = read(handle, ack_buf, 10);
      if( rv != 10 )
      {
         printf("Failed to read ACK for GGA message shutoff.\n");
         return 1;
      }
      rv = validate_ack(ack_buf);
      if( rv != 0 ){ return 1; }
      sleep(2);

      rv = changeIDrate(GLL_ID, 0x00, handle); 
      if( rv != 0){ return 1; }
      rv = -1;
      rv = read(handle, ack_buf, 10);
      if( rv != 10 )
      {
         printf("Failed to read ACK for GLL message shutoff.\n");
         return 1;
      }
      rv = validate_ack(ack_buf);
      if( rv != 0 ){ return 1; }
      sleep(2);

      rv = changeIDrate(GSA_ID, 0x00, handle); 
      if( rv != 0){ return 1; }
      rv = -1;
      rv = read(handle, ack_buf, 10);
      if( rv != 10 )
      {
         printf("Failed to read ACK for GSA message shutoff.\n");
         return 1;
      }
      rv = validate_ack(ack_buf);
      if( rv != 0 ){ return 1; }
      sleep(2);

      rv = changeIDrate(GSV_ID, 0x00, handle); 
      if( rv != 0){ return 1; }
      rv = -1;
      rv = read(handle, ack_buf, 10);
      if( rv != 10 )
      {
         printf("Failed to read ACK for GSV message shutoff.\n");
         return 1;
      }
      rv = validate_ack(ack_buf);
      if( rv != 0 ){ return 1; }
      sleep(2);

      rv = changeIDrate(VTG_ID, 0x00, handle); 
      if( rv != 0){ return 1; }
      rv = -1;
      rv = read(handle, ack_buf, 10);
      if( rv != 10 )
      {
         printf("Failed to read ACK for VTG message shutoff.\n");
         return 1;
      }
      rv = validate_ack(ack_buf);
      if( rv != 0 ){ return 1; }
      sleep(2);
   }

   return 0;
}
