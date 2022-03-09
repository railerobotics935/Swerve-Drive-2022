/*
 PRmsgParser.h - Library for parsing PR prefixed NMEA-0183 style messages.
 Created by Jaap van Bergeijk, February 18, 2019
 FIRST Team 935
*/

#ifndef PRMSG_PARSER_H
#define PRMSG_PARSER_H

#include "PixyDefines.h"

#define NMEA_SDL_MAX_ITEM_LEN   (uint8_t)12
#define NMEA_SDL_MAX_LINE_LEN   (uint8_t)250

// Supported NMEA Style Messages
enum
{
  NMEA_PRCFG,	// configuration message
  NMEA_PRAWS,	// actual wheel speed
  NMEA_PRLED,	// control LEDs
  NMEA_PRBLK, // pixy camera blocks
  NMEA_PRTUR, // turret sensors
  NR_OF_NMEA_MSGS
};

// defined user handler trigger types
enum
{
  NMEA_EV_CONFIG,
  NMEA_EV_SPEED,
  NMEA_EV_LED,
  NMEA_EV_BLOCKS,
  NMEA_EV_TURRET,
  NR_OF_NMEA_HANDLERS
};

// data structure definitions
typedef struct
{
  float left;
  float right;
  uint8_t auto_drive;
  float arm_angle;
  float wrist_angle;
} PRmsgWheelSpeed;

typedef void (*prawshandlerptr)(float left, float right, uint8_t auto_drive, float arm_angle, float wrist_angle);
typedef void (*prledhandlerptr)(uint8_t led_ring_on, uint8_t upper_on, uint8_t lower_on);
typedef void (*prblkhandlerptr)(uint8_t camera_number, uint32_t camera_timestamp, uint8_t n_blocks, PIXY_BLOCK_TYPE *blocks);
typedef void (*prturhandlerptr)(uint32_t turret_timestamp, uint16_t kickwheel_speed, uint16_t hood_angle_1, uint16_t hood_angle_2, int target_distance);


class PRmsgParser
{
private:
  uint8_t CfgUseChecksum;
  bool BReadSentence;
  bool BReadChecksum;
  bool BChecksumAvailable;
  bool BStartSentence;

  char ItemBuf[NMEA_SDL_MAX_ITEM_LEN];
  uint8_t ItemBufIndex;
  uint8_t CurrentMessageType;
  uint8_t CurrentMessageItem;
  char SentenceBuf[NMEA_SDL_MAX_LINE_LEN];
  uint8_t SentenceBufIndex;
  char ChecksumBuf[4];
  uint8_t ChecksumBufIndex;
  uint8_t Checksum;
  uint8_t SendChecksum;

  PRmsgWheelSpeed actualWheelSpeed;
  uint8_t m_led_cmd;
  uint8_t m_upper_cmd;
  uint8_t m_lower_cmd;

  uint8_t block_parse_index;
  uint8_t camera_number;
  uint32_t camera_timestamp;
  uint8_t n_parsed_blocks;
  PIXY_BLOCK_TYPE parsed_blocks[MAX_NR_OF_BLOCKS];

  uint32_t turret_timestamp;
  uint16_t kickwheel_speed;
  uint16_t hood_angle_1;
  uint16_t hood_angle_2;
  int target_distance;

//  handlerptr nmeaHandlerPtr[NR_OF_NMEA_HANDLERS];
  prawshandlerptr prawsHandlerPtr;
  prledhandlerptr prledHandlerPtr;
  prblkhandlerptr prblkHandlerPtr;
  prturhandlerptr prturHandlerPtr;

  void parseSentenceChar(char inChar);

public:
  PRmsgParser();
  void setConfiguration(uint8_t cfg, uint8_t value);
//  setEventHandler(uint8_t event, handlerptr userHandlerPtr);
  void setPRAWSEventHandler(prawshandlerptr userHandlerPtr);
  void setPRLEDEventHandler(prledhandlerptr userHandlerPtr);
  void setPRBLKEventHandler(prblkhandlerptr userHandlerPtr);
  void setPRTUREventHandler(prturhandlerptr userHandlerPtr);

  void resetEventHandler(uint8_t event);
  void parseChar(char inChar);
};


#endif // PRMSG_PARSER_H

/****************************************************************************/
/**                                                                        **/
/**                              EOF                                       **/
/**                                                                        **/
/****************************************************************************/
