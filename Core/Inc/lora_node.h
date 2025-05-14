#ifndef _LORANODE_H_
#define _LORANODE_H_

#include "main.h"
#include "log.h"
#include "sx1278.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"

// Response Codes
typedef enum
{
  Unknown,
  Connected,
  AckSet,
  Ack,
  NackSet,
  Nack
} Resp_Code_t;

// Node States
typedef enum
{
  Disconnect_State = 0x01,
  Connect_State_I  = 0x02,
  Connect_State_II = 0x03
} Node_State_t;

// Packet structures
typedef struct
{
  uint16_t NodeID;
  uint16_t PayloadID;
  uint16_t Period;
  uint16_t Battery;
} Connect_Packet_t;

typedef struct
{
  uint16_t NodeID;
  uint16_t PayloadID;
  uint16_t Distance1;
  uint16_t Distance2;
} Post_Packet_t;

typedef struct
{
  uint16_t NodeID;
  uint16_t PayloadID;
  Resp_Code_t RespCode;
  uint16_t setPeriod;
} Response_Packet_t;


#define CONNECT     0xAABB
#define POST        0xABBA
#define RESPONSE    0xABAB

#define NODEID      0x2332
#define GATEID      0xAAAA
#define PERIOD      20

#endif /* LORA_NODE_H_ */
