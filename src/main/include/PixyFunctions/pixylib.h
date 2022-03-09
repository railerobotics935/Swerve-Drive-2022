/*
PIXY SPI Reader for Teensy 3.5 and 3.6
Written By: randomvibe
Date: 9/7/2017

Main Functions:
   pixy_spi_read_16bit  ~ Reads PIXY data via SPI connection using Teensy 3.5/3.6 SPI functions
   pixy_parse_data      ~ Parses SPI data stream and saves to array of PIXY struct
*/
#include <stdint.h>

// Teensy 3.5/3.6 SPI Pins
#define     MOSIX   11
#define     SCKX    13
#define     MISOX   12
SPISettings PIXYSPI(1000000, MSBFIRST, SPI_MODE0); 

// Pixy Parameters
#define  PIXY_SPI_CS             15        // SPI Chip Select
#define  PIXY_SPI_TRY            24
#define  PIXY_SYNC_BYTE_DATA     0x5b      // Sync with Pixy Device
#define  PIXY_START_WORD         0xaa55    // Start word in Pixy Data Block
#define  PIXY_START_WORD_CC      0xaa56    // Stard word in Pixy Data Block with Color Codes
#define  PIXY_MAX_BLOCKS         20        // Max No. of detectable blocks 

// PIXY data block struct array
typedef struct 
{
  uint16_t  obj;   // Object type: Normal (0xaa55), Color Code (0xaa56)
  uint16_t  sig;   // Signature Number
  uint16_t  x;     // x center of object
  uint16_t  y;     // y center of object
  uint16_t  wd;    // width of object
  uint16_t  ht;    // height of object
  int16_t   phi;   // angle of object (applies to Color-Code objects only)
} PIXY;

PIXY pixy[PIXY_MAX_BLOCKS];

uint16_t pixy_spi_read_16bit(uint16_t target, SPISettings SETX, uint8_t CSX)
{
  uint8_t   a, b;
  uint16_t  c;

  // Read First Byte
  SPI.beginTransaction(SETX);      
  digitalWrite(CSX,LOW);    
  a = SPI.transfer(target);    
  SPI.endTransaction();

  // Read Second Byte
  SPI.beginTransaction(SETX);
  b = SPI.transfer(0x00);              
  digitalWrite(CSX,HIGH); 
  SPI.endTransaction();

  // Build Pixy 16-bit Word 
  c = (uint16_t)(b | (a<<8));

  return  c;
}

uint16_t  pixy_parse_data( uint16_t target, SPISettings SETX, uint8_t CSX, PIXY *pixy)
{
    int       ii, jj;
    uint16_t  word1, word2, wordx, checksum_spi, checksum_verify;
    uint16_t  nblock, data[8];
    bool      SYNC;

    // Initialize variables
    for (ii=0; ii<8; ii++)  data[ii]=0;

    // Sync with Pixy
    SYNC  = 0;
    word1 = 0;
    word2 = pixy_spi_read_16bit( target, SETX, CSX);  
    jj    = 0; 
    while ( SYNC==0 && jj<PIXY_SPI_TRY)
    {
        jj++;
        word1 = word2;
        word2 = pixy_spi_read_16bit( target, SETX, CSX);

        if (word1==PIXY_START_WORD && (word2==PIXY_START_WORD || word2==PIXY_START_WORD_CC) )  {
            SYNC  = 1;
            wordx = word2;
        }
    }
   
    // Read Pixy Data Blocks 
    nblock=0;

    while (SYNC==1)
    {                       
        SYNC=0;

        if ( wordx==PIXY_START_WORD || wordx==PIXY_START_WORD_CC )
        {
            data[0] = wordx;                                      // Object type: Normal (0xaa55), Color Code (0xaa56)
            data[1] = pixy_spi_read_16bit( target, SETX, CSX);    // Checksum
            data[2] = pixy_spi_read_16bit( target, SETX, CSX);    // Signature No.
            data[3] = pixy_spi_read_16bit( target, SETX, CSX);    // X Center of object
            data[4] = pixy_spi_read_16bit( target, SETX, CSX);    // Y Center of object
            data[5] = pixy_spi_read_16bit( target, SETX, CSX);    // Width of object
            data[6] = pixy_spi_read_16bit( target, SETX, CSX);    // Height of object

            if (wordx==PIXY_START_WORD)     data[7]=0;                                        // Angle not available for Normal Objects
            if (wordx==PIXY_START_WORD_CC)  data[7]=pixy_spi_read_16bit( target, SETX, CSX);  // Angle of object for Color-Coded-Objects

            // Verify CHECKSUM
            checksum_spi    = data[1];
            checksum_verify = data[2] + data[3] + data[4] + data[5] + data[6] + data[7];


            // Capture Block Data
            if (checksum_spi==checksum_verify  && nblock<PIXY_MAX_BLOCKS)
            {
                SYNC = 1;                
                nblock++;
                                
                pixy->sig = data[2];
                pixy->x   = data[3];
                pixy->y   = data[4];
                pixy->wd  = data[5];
                pixy->ht  = data[6];
                pixy->phi = (int16_t)data[7];
    
                pixy++;  // Increment pointer to PIXY struct array

                wordx = pixy_spi_read_16bit( target, SETX, CSX);    // Advance to next data block
            }             
            
        }  // if ( wordx==PIXY_START_WORD || wordx==PIXY_START_WORD_CC )
        
    }  // while (SYNC==1)

    return  nblock;
}
