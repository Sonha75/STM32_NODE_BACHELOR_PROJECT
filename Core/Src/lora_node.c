#include <multi_hcsr04.h>

#include "lora_node.h"
#include "main.h"


static const char *TAG = "LORA: ";

extern UART_HandleTypeDef huart2;


static uint8_t DataArray[9] = {0};

static Connect_Packet_t Connect_Pack =
{
    .NodeID = NODEID,
    .PayloadID = CONNECT,
    .Period = PERIOD,
    .Battery = 0,
};

static Post_Packet_t Post_Pack =
{
    .NodeID = NODEID,
    .PayloadID = POST,
    .Distance1 = 0,
    .Distance2 = 0,
};

static Response_Packet_t Resp_Pack =
{
    .NodeID = NODEID,
    .PayloadID = RESPONSE,
    .RespCode = 0,
};


static void __Connect_Packet_set(uint16_t period, uint16_t battery)
{
    Connect_Pack.Period = period;
    Connect_Pack.Battery = battery;

    memset(DataArray, 0, sizeof(Connect_Packet_t) + 1);
	memcpy(DataArray, &Connect_Pack, sizeof(Connect_Packet_t));

    DataArray[sizeof(Connect_Packet_t)] = DataArray[0];
    for(int i=1; i<sizeof(Connect_Packet_t); i++)
    {
        DataArray[sizeof(Connect_Packet_t)] ^= DataArray[i];
    }
}

static void __Post_Packet_set(uint16_t distance1, uint16_t distance2)
{
    Post_Pack.Distance1 = distance1;
    Post_Pack.Distance2 = distance2;

    memset(DataArray, 0, 9);
	memcpy(DataArray, &Post_Pack, sizeof(Post_Packet_t));

    DataArray[sizeof(Post_Packet_t)] = DataArray[0];
    for(int i=1; i<sizeof(Post_Packet_t); i++)
    {
        DataArray[sizeof(Post_Packet_t)] ^= DataArray[i];
    }
}

static void __Resp_Packet_set(Resp_Code_t respcode)
{
    Resp_Pack.NodeID = NODEID;
    Resp_Pack.RespCode = respcode;

    memset(DataArray, 0, 9);
	memcpy(DataArray, &Post_Pack, sizeof(Response_Packet_t));

    DataArray[sizeof(Response_Packet_t)] = DataArray[0];
    for(int i=1; i<sizeof(Response_Packet_t); i++)
    {
        DataArray[sizeof(Response_Packet_t)] ^= DataArray[i];
    }
}

bool Connect_Packet_send(uint16_t period, uint16_t battery)
{
    __Connect_Packet_set(period, battery);
    int nTry = 0;
    while ((sx1278_listen_to_talk() == false) && (nTry < 10))
    {
        nTry++;
        HAL_Delay(50);
    }
    if (nTry == 10)
    {
        //myStatus = RECONNECT_MODE;
        //HAL_SPI_DeInit(&hspi1);

        return false;
    }
    sx1278_send_data(DataArray, sizeof(Connect_Packet_t) + 1);
    return true;
}

bool Post_Packet_send(uint16_t distance1, uint16_t distance2)
{
    __Post_Packet_set(distance1, distance2);
    int nTry = 0;
    while ((sx1278_listen_to_talk() == false) && (nTry < 10))
    {
        nTry++;
        HAL_Delay(150);
    }
    if (nTry == 10)
    {
        //myStatus = RECONNECT_MODE;
        HAL_SPI_DeInit(&hspi1);

        return false;
    }
    sx1278_send_data(DataArray, sizeof(Post_Packet_t) + 1);
    return true;
}

bool Resp_Packet_send(Resp_Code_t respcode)
{
    __Resp_Packet_set(respcode);
    int nTry = 0;
    while ((sx1278_listen_to_talk() == false) && (nTry < 10))
    {
        nTry++;
        HAL_Delay(50);
    }
    if (nTry == 10)
    {
        //myStatus = RECONNECT_MODE;
        HAL_SPI_DeInit(&hspi1);

        return false;
    }
    sx1278_send_data(DataArray, sizeof(Response_Packet_t) + 1);
    return true;
}

bool Resp_Packet_Receive(void)
{
    static uint32_t nByteRx = 0;
    static int rssi = -1;
    static float snr = -1;
    sx1278_start_recv_data();
    memset(DataArray, '\0', sizeof(Response_Packet_t) + 1);



    uint32_t timeOut = HAL_GetTick();
    while ((HAL_GetTick() - timeOut) < 2000U)
    {
        if (Is_LoRa_EXTI() == true)
        {
            if (sx1278_recv_data((uint8_t *)DataArray, &nByteRx, &rssi, &snr, false) == SX1278_OK)
            {

                if (nByteRx == sizeof(Response_Packet_t) + 1)
                {
                    memcpy(&Resp_Pack, DataArray, sizeof(Response_Packet_t));
                }
                else
                {
                    memcpy(&Resp_Pack, '\0', sizeof(Response_Packet_t));
                    sx1278_start_recv_data();
                    continue;
                }

                if ((Resp_Pack.PayloadID != RESPONSE) || (Resp_Pack.NodeID != NODEID))
                {
                    // this is not what I want OR this is not for me
                    memcpy(&Resp_Pack, '\0', sizeof(Response_Packet_t));
                    sx1278_start_recv_data();
                    continue;
                }
                else
                {
                    // Yep ok this is my packet and ID
                    return true;
                }
            }
        }
    }

    return false;
}

//void Disconnect_Handle(void)
//{
//    if(Connect_Packet_send((uint16_t)Sleep_Period_Get(), 0) == false)
//    {
//        State_Bkup_Set(Disconnect_State);
//        StandbyMode_Set(3);
//
//        return;
//    }
//
//    if(Resp_Packet_Receive() == false)
//    {
//        State_Bkup_Set(Disconnect_State);
//        StandbyMode_Set(3);
//
//        return;
//    }
//
//    switch(Resp_Pack.RespCode)
//    {
//        case Ack:
//            HAL_UART_Transmit(&huart2, "ACK response\n", 14, 1000);
//            State_Bkup_Set(Connect_State_I);
//            StandbyMode_Set(Sleep_Period_Get());
//            break;
//
//        case Nack:
//            HAL_UART_Transmit(&huart2, "NACK response\n", 14, 1000);
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(3);
//            break;
//
//        default:
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(3);
//    }
//}
//
//void Connect_I_Handle(void)
//{
//    /*measure distances from HCSR04 sensors*/
//    char str[30];
//    uint16_t distance1, distance2;
//
//    // Measure distance from first sensor
//    distance1 = measureDistance(triggerPorts[0], triggerPins[0], echoPorts[0], echoPins[0]);
//    // Measure distance from second sensor
//    distance2 = measureDistance(triggerPorts[1], triggerPins[1], echoPorts[1], echoPins[1]);
//
//    sprintf(str, "Distances: %d %d mm\n", distance1, distance2);
//    HAL_UART_Transmit(&huart2, str, strlen(str), 1000);
//
//    if(Post_Packet_send(distance1, distance2) == false)
//    {
//        State_Bkup_Set(Disconnect_State);
//        StandbyMode_Set(5);
//
//        return;
//    }
//
//    if(Resp_Packet_Receive() == false)
//    {
//        Distance1_Bkup_Set(distance1);
//        Distance2_Bkup_Set(distance2);
//        State_Bkup_Set(Connect_State_II);
//        StandbyMode_Set(5);
//
//        return;
//    }
//
//    switch(Resp_Pack.RespCode)
//    {
//        case Ack:
//            HAL_UART_Transmit(&huart2, "ACK response\n", 14, 1000);
//            State_Bkup_Set(Connect_State_I);
//            StandbyMode_Set(Sleep_Period_Get());
//            break;
//
//        case AckSet:
//            Sleep_Period_Set(Resp_Pack.setPeriod);
//            State_Bkup_Set(Connect_State_I);
//            StandbyMode_Set(Sleep_Period_Get());
//            break;
//
//        case Nack:
//            HAL_UART_Transmit(&huart2, "NACK response\n", 14, 1000);
//            Distance1_Bkup_Set(distance1);
//            Distance2_Bkup_Set(distance2);
//            State_Bkup_Set(Connect_State_II);
//            StandbyMode_Set(5);
//            break;
//
//        case NackSet:
//            Sleep_Period_Set(Resp_Pack.setPeriod);
//            State_Bkup_Set(Connect_State_II);
//            StandbyMode_Set(5);
//            break;
//
//        case Unknown:
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(5);
//            break;
//
//        default:
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(5);
//    }
//}
//
//void Connect_II_Handle(void)
//{
//    if(Post_Packet_send(Distance1_Bkup_Get(), Distance2_Bkup_Get()) == false)
//    {
//        State_Bkup_Set(Disconnect_State);
//        StandbyMode_Set(5);
//
//        return;
//    }
//
//    if(Resp_Packet_Receive() == false)
//    {
//        State_Bkup_Set(Connect_State_II);
//        StandbyMode_Set(5);
//
//        return;
//    }
//
//    switch(Resp_Pack.RespCode)
//    {
//        case Ack:
//            HAL_UART_Transmit(&huart2, "ACK response\n", 14, 1000);
//            State_Bkup_Set(Connect_State_I);
//            StandbyMode_Set(Sleep_Period_Get());
//            break;
//
//        case AckSet:
//            State_Bkup_Set(Connect_State_I);
//            StandbyMode_Set(Sleep_Period_Get());
//            break;
//
//        case Nack:
//            HAL_UART_Transmit(&huart2, "NACK response\n", 14, 1000);
//            State_Bkup_Set(Connect_State_II);
//            StandbyMode_Set(5);
//            break;
//
//        case NackSet:
//            State_Bkup_Set(Connect_State_II);
//            StandbyMode_Set(5);
//            break;
//
//        case Unknown:
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(5);
//            break;
//
//        default:
//            State_Bkup_Set(Disconnect_State);
//            StandbyMode_Set(5);
//    }
//}
