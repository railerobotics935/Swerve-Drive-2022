/*
 PRmsgParser.cpp - Library for parsing PR prefixed NMEA-0183 style messages.
 Created by Jaap van Bergeijk, February 18, 2019
 FIRST Team 935
*/

#include "PixyFunctions/PRmsgParser.h"
#include <iostream>
#include <stdlib.h>
#include <string.h>

/****************************************************************************/
PRmsgParser::PRmsgParser()
/****************************************************************************/
{
  CfgUseChecksum = false;
  BReadSentence = false;
  BReadChecksum = false;
  BChecksumAvailable = false;
  BStartSentence = false;
  SentenceBufIndex = 0;
  CurrentMessageType = NR_OF_NMEA_MSGS;
  CurrentMessageItem = 0;

  m_led_cmd = 0;
  m_upper_cmd = 0;
  m_lower_cmd = 0;

  camera_number = 0;
  camera_timestamp = 0;
  n_parsed_blocks = 0;

  turret_timestamp = 0;

  prawsHandlerPtr = NULL;
  prledHandlerPtr = NULL;
  prblkHandlerPtr = NULL;
  prturHandlerPtr = NULL;
  prtgaHandlerPtr = NULL;
}

/****************************************************************************/
void PRmsgParser::setConfiguration(uint8_t cfg, uint8_t value)
/****************************************************************************/
{
/*
  switch (cfg)
  {
  case NMEA_CFG_CHECKSUM:
    CfgUseChecksum = value;
    break;
  }
*/
}

/****************************************************************************/
//void PRmsgParser::setEventHandler(uint8_t event, handlerptr userHandlerPtr)
void PRmsgParser::setPRAWSEventHandler(prawshandlerptr userHandlerPtr)
/****************************************************************************/
{
  prawsHandlerPtr = userHandlerPtr;
}

void PRmsgParser::setPRLEDEventHandler(prledhandlerptr userHandlerPtr)
/****************************************************************************/
{
  prledHandlerPtr = userHandlerPtr;
}

void PRmsgParser::setPRBLKEventHandler(prblkhandlerptr userHandlerPtr)
/****************************************************************************/
{
  prblkHandlerPtr = userHandlerPtr;
}

void PRmsgParser::setPRTUREventHandler(prturhandlerptr userHandlerPtr)
/****************************************************************************/
{
  prturHandlerPtr = userHandlerPtr;
}

void PRmsgParser::setPRTGAEventHandler(prtgahandlerptr userHandlerPtr)
/****************************************************************************/
{
  prtgaHandlerPtr = userHandlerPtr;
}

/****************************************************************************/
void PRmsgParser::resetEventHandler(uint8_t event)
/****************************************************************************/
{
//  if (event < NR_OF_NMEA_HANDLERS)
//    nmeaHandlerPtr[event] = (handlerptr)(NULL);
}

/****************************************************************************/
void PRmsgParser::parseChar(char inChar)
/****************************************************************************/
{
uint8_t i;
  // Parse NMEA sentences
  switch (inChar)
  {
  case '$' : // start of new sentence
    BReadSentence = true;
    BReadChecksum = false;
    BChecksumAvailable = false;
    SentenceBufIndex = 0;
    Checksum = 0;
    break;

  case 10 : // line feed
    BReadSentence = false;
    BReadChecksum = false;

    // use Checksum?
    if (BChecksumAvailable && CfgUseChecksum)
    {
      if (ChecksumBuf[0] < 58)
        SendChecksum = (ChecksumBuf[0] - 48) << 4;
      else
        SendChecksum = (ChecksumBuf[0] - 55) << 4;
      
      if (ChecksumBuf[1] < 58)
        SendChecksum += (ChecksumBuf[1] - 48);
      else
        SendChecksum += (ChecksumBuf[1] - 55);

      if (Checksum != SendChecksum)
        break;
    }

    // parseSentence
	BStartSentence = true;
	ItemBufIndex = 0;
    for (i=0; i<SentenceBufIndex; i++)
      parseSentenceChar(SentenceBuf[i]);
    parseSentenceChar(',');
  
    switch (CurrentMessageType)
	{
 /*
	case NMEA_PRCFG:
      // check new Configuration Data
      if (GpsData.utc != PrevPositionUTC)
	  {
        PrevPositionUTC = GpsData.utc;
        if (nmeaHandlerPtr[NMEA_EV_POSITION] != NULL)
          (*(nmeaHandlerPtr[NMEA_EV_POSITION]))();
      }
      break;
*/

    case NMEA_PRAWS:
      // check new Speed Data
      if (prawsHandlerPtr != NULL)
        prawsHandlerPtr(actualWheelSpeed.left, actualWheelSpeed.right, actualWheelSpeed.auto_drive, actualWheelSpeed.arm_angle, actualWheelSpeed.wrist_angle);
/*
      if (GpsData.utc != PrevSpeedUTC)
      {
        PrevSpeedUTC = GpsData.utc;
        if (nmeaHandlerPtr[NMEA_EV_SPEED] != NULL)
          (*(nmeaHandlerPtr[NMEA_EV_SPEED]))();
      }
*/
      break;

    case NMEA_PRLED:
      // check new LED commands
      if (prledHandlerPtr != NULL)
        prledHandlerPtr(m_led_cmd, m_upper_cmd, m_lower_cmd);
      break;

    case NMEA_PRBLK:
      // call handler for further processing of pixy blocks
      if (prblkHandlerPtr != NULL)
        prblkHandlerPtr(camera_number, camera_timestamp, n_parsed_blocks, parsed_blocks);
      break;

    case NMEA_PRTUR:
      // call handler for turret data processing
      if (prturHandlerPtr != NULL)
        prturHandlerPtr(turret_timestamp, kickwheel_speed, hood_angle_1, hood_angle_2, target_distance);
      break;
    
    case NMEA_PRTGA:
      // call handler for turret data processing
      if (prtgaHandlerPtr != NULL)
        prtgaHandlerPtr(valid_blocks, target_angle_offset, target_distance);
      break;
    
  }
  break;

  case 13 : // return
    BReadSentence = false;
    BReadChecksum = false;
    break;
    
  case '*' : // checksum
    BReadSentence = false;
    BReadChecksum = true;
    BChecksumAvailable = true;
    ChecksumBufIndex = 0;
    break;

  default:
    if (BReadSentence)
    {
      SentenceBuf[SentenceBufIndex] = inChar;
      Checksum = Checksum ^ (uint8_t)inChar;
      if (SentenceBufIndex < (NMEA_SDL_MAX_LINE_LEN - 1))
        SentenceBufIndex++;
    }
    if (BReadChecksum)
    {
      ChecksumBuf[ChecksumBufIndex] = inChar;
      if (ChecksumBufIndex < 2)
        ChecksumBufIndex++;
    }
    break;
  }
}

/****************************************************************************/
/**                                                                        **/
/**                     LOCAL FUNCTIONS                                    **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
void PRmsgParser::parseSentenceChar(char inChar)
/****************************************************************************/
{
  // parse NMEA sentence buffer
  if (inChar == ',')
  {
    // new item is read, terminate item buffer
    ItemBuf[ItemBufIndex] = '\0';

    // parse item
    if (BStartSentence)
    {
      BStartSentence = false;
      ItemBufIndex = 0;
      CurrentMessageItem = 1;

      if (strcmp(ItemBuf,"PRCFG") == 0) 
      {
        CurrentMessageType = NMEA_PRCFG;
        ItemBuf[0] = '\0';
        return;
      }

      if (strcmp(ItemBuf,"PRAWS") == 0)
      {
        CurrentMessageType = NMEA_PRAWS;
        ItemBuf[0] = '\0';
        return;
      }

      if (strcmp(ItemBuf,"PRLED") == 0)
      {
        CurrentMessageType = NMEA_PRLED;
        ItemBuf[0] = '\0';
        return;
      }

      if (strcmp(ItemBuf,"PRBLK") == 0)
      {
        CurrentMessageType = NMEA_PRBLK;
        block_parse_index = 0;
        ItemBuf[0] = '\0';
        return;
      }

      if (strcmp(ItemBuf,"PRTUR") == 0)
      {
        CurrentMessageType = NMEA_PRTUR;
        block_parse_index = 0;
        ItemBuf[0] = '\0';
        return;
      }

      if (strcmp(ItemBuf,"PRTGA") == 0)
      {
        CurrentMessageType = NMEA_PRTGA;
        block_parse_index = 0;
        ItemBuf[0] = '\0';
        return;
      }

      CurrentMessageType = NR_OF_NMEA_MSGS;
      CurrentMessageItem = 0;
    }
    else
    {
      switch (CurrentMessageType)
      {
      case NMEA_PRCFG:
/*
		switch (CurrentMessageItem)
        {
        case 1: // UTC of Position Fix
					if (ItemBufIndex > 0)
						GpsData.numSVs = atoi(ItemBuf);
					else
						GpsData.numSVs = 0;
          break;
		case 2:
					if (ItemBufIndex > 0)
						GpsData.difCorAge = atof(ItemBuf);
					else
						GpsData.difCorAge = 0.0;
					break;
        }
*/
        break;
		
      case NMEA_PRAWS:
        switch (CurrentMessageItem)
		    {
        case 1: // Left Wheel Speed
          if (ItemBufIndex > 0)
            actualWheelSpeed.left = atof(ItemBuf);
          else
            actualWheelSpeed.left = 0.0;
          break;
     	  }
		    break;

      case NMEA_PRLED:
        switch (CurrentMessageItem)
		    {
        case 1: // LED ring command
          if (ItemBufIndex > 0) m_led_cmd = atoi(ItemBuf);
          break;
        case 2: // Upper LED command
          if (ItemBufIndex > 0) m_upper_cmd = atoi(ItemBuf);
          break;
        case 3: // Lower LED command
          if (ItemBufIndex > 0) m_lower_cmd = atoi(ItemBuf);
          break;
  		  }
	    	break;

      case NMEA_PRBLK:
        switch (CurrentMessageItem)
		    {
        case 1: // Camera number
          if (ItemBufIndex > 0) camera_number = atoi(ItemBuf);
          break;
        case 2: // timestamp
          if (ItemBufIndex > 0) camera_timestamp = atoi(ItemBuf);
          break;
        case 3: // Number of Blocks
          if (ItemBufIndex > 0) n_parsed_blocks = atoi(ItemBuf);
          break;
        default:
          // parse list of blocks
          if ((CurrentMessageItem > 3) && (CurrentMessageItem < (n_parsed_blocks * 5 + 4)))
          {
            if ((((CurrentMessageItem-4) % 5) == 0) && (ItemBufIndex > 0))
              parsed_blocks[block_parse_index].sig = atoi(ItemBuf);
            else if ((((CurrentMessageItem-4) % 5) == 1) && (ItemBufIndex > 0))
              parsed_blocks[block_parse_index].x = atoi(ItemBuf);
            else if ((((CurrentMessageItem-4) % 5) == 2) && (ItemBufIndex > 0))
              parsed_blocks[block_parse_index].y = atoi(ItemBuf);
            else if ((((CurrentMessageItem-4) % 5) == 3) && (ItemBufIndex > 0))
              parsed_blocks[block_parse_index].w = atoi(ItemBuf);
            else if ((((CurrentMessageItem-4) % 5) == 4) && (ItemBufIndex > 0))
            {
              parsed_blocks[block_parse_index].h = atoi(ItemBuf);
              block_parse_index++;
            }
          }
          break;
        }
        break;

      case NMEA_PRTUR:
        switch (CurrentMessageItem)
		    {
        case 1: // Turret timestamp
          if (ItemBufIndex > 0) turret_timestamp = atoi(ItemBuf);
          break;
        case 2: // kickwheel speed
          if (ItemBufIndex > 0) kickwheel_speed = atoi(ItemBuf);
          break;
        case 3:
          if (ItemBufIndex > 0) hood_angle_1 = atoi(ItemBuf);
          break;
        case 4:
          if (ItemBufIndex > 0) hood_angle_2 = atoi(ItemBuf);
          break;
        case 5:
          if (ItemBufIndex > 0) target_distance = atoi(ItemBuf);
          break;
        }
        break;

      case NMEA_PRTGA:
        switch (CurrentMessageItem)
		    {
        case 1: // Valid Blocks
          if (ItemBufIndex > 0)
            valid_blocks = atoi(ItemBuf);
          break;
        case 2: // Target Angle offset
          if (ItemBufIndex > 0)
            target_angle_offset = atof(ItemBuf);
          else
            target_angle_offset = 0.0;
          break;
        case 3: // Target Distance
          if (ItemBufIndex > 0)
            target_distance = atoi(ItemBuf);
          break;
        }
        break;
      }
      
      CurrentMessageItem++;
    }
    ItemBufIndex = 0;
    ItemBuf[0] = '\0';
  }
  else
  {
    // any character of current item is read
    ItemBuf[ItemBufIndex] = inChar;
    if (ItemBufIndex < (NMEA_SDL_MAX_ITEM_LEN - 1)) ItemBufIndex++;
  }
}
