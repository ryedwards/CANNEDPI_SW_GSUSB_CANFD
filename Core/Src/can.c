/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair
Copyright (c) 2022 Ryan Edwards (changes for STM32G4 and CAN-FD)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "usbd_gs_can.h"

static uint8_t can_rx_data_buff[64];
static uint8_t can_tx_data_buff[64];

extern TIM_HandleTypeDef htim6;
extern osMessageQueueId_t queue_to_hostHandle;
extern USBD_HandleTypeDef hUSB;

static uint8_t can_covert_stm32dlc_to_len(uint32_t DataLength);
static uint32_t can_covert_len_to_stm32dlc(uint8_t len);
static void can_parse_error_status(FDCAN_ProtocolStatusTypeDef *status, struct gs_host_frame *frame);

void can_init(FDCAN_HandleTypeDef *hcan, FDCAN_GlobalTypeDef *instance)
{
  if (instance == FDCAN1) {
    MX_FDCAN1_Init();
  }
  else if(instance == FDCAN2) {
    MX_FDCAN2_Init();
  }
  else if(instance == FDCAN3) {
    MX_FDCAN3_Init();
  }
  else {
    /* invalid channel */
  }
}

void can_set_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
  if ((brp > 0) && (brp <= 1024)
    && (phase_seg1 > 0) && (phase_seg1 <= 16)
    && (phase_seg2 > 0) && (phase_seg2 <= 8)
    && (sjw > 0) && (sjw <= 4))
  {
    hcan->Init.NominalPrescaler = brp;
    hcan->Init.NominalTimeSeg1 = phase_seg1;
    hcan->Init.NominalTimeSeg2 = phase_seg2;
    hcan->Init.NominalSyncJumpWidth = sjw;
  }
}

void can_set_data_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
  hcan->Init.DataPrescaler = brp;
  hcan->Init.DataTimeSeg1 = phase_seg1;
  hcan->Init.DataTimeSeg2 = phase_seg2;
  hcan->Init.DataSyncJumpWidth = sjw;
}

void can_enable(FDCAN_HandleTypeDef *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd)
{
  FDCAN_FilterTypeDef sFilterConfig;

  hcan->Init.AutoRetransmission = ENABLE;
  if (one_shot)
  {
    hcan->Init.AutoRetransmission = DISABLE;
  }

  hcan->Init.Mode = FDCAN_MODE_NORMAL;
  hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  if (loop_back && listen_only)
  {
    hcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  }
  else if (loop_back)
  {
    hcan->Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
  }
  else if (listen_only)
  {
    hcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
  }
  else
  {
    //normal is good
  }

  /* we should also add a flag for non-BRS */
  if (can_mode_fd) {
    hcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  }

  hcan->Init.AutoRetransmission = DISABLE;

  HAL_FDCAN_Init(hcan);

  /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);

  /* Configure global filter on both FDCAN instances:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // Start CAN using HAL
  HAL_FDCAN_Start(hcan);

  HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                 | FDCAN_IT_LIST_BIT_LINE_ERROR
                                 | FDCAN_IT_LIST_PROTOCOL_ERROR, 0);
}

void can_disable(FDCAN_HandleTypeDef *hcan)
{
  //Stop can using HAL
  HAL_FDCAN_Stop(hcan);
  HAL_FDCAN_DeactivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                   | FDCAN_IT_LIST_BIT_LINE_ERROR
                                   | FDCAN_IT_LIST_PROTOCOL_ERROR);

}

bool can_is_enabled(FDCAN_HandleTypeDef *hcan)
{
  if(hcan->State == HAL_FDCAN_STATE_BUSY) {
    return true;
  }
  else {
    return false;
  }
}

bool can_send(FDCAN_HandleTypeDef *hcan, struct gs_host_frame *frame)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  if (frame->can_id & CAN_RTR_FLAG) {
      TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
  }
  else {
      TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  }

  TxHeader.DataLength = frame->can_dlc << 16;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (frame->can_id & CAN_EFF_FLAG) {
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
  }
  else {
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.Identifier = frame->can_id & 0x7FF;
  }

  if (frame->flags & GS_CAN_FLAG_FD) {
    TxHeader.FDFormat = FDCAN_FD_CAN;
    if (frame->flags & GS_CAN_FLAG_BRS) {
      TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    }
    if (frame->can_dlc > 8) {
      TxHeader.DataLength = can_covert_len_to_stm32dlc(frame->can_dlc);
    }
    memcpy(can_tx_data_buff, frame->canfd.data, frame->can_dlc);
  }
  else {
    memcpy(can_tx_data_buff, frame->classic_can.data, frame->can_dlc);
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, can_tx_data_buff) != HAL_OK) {
      return false;
  }
  else {
      return true;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef RxHeader;
  struct gs_host_frame frame;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, can_rx_data_buff) != HAL_OK) {
    //Error_Handler();
  }

  frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hfdcan);
  frame.can_id = RxHeader.Identifier;

  if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
    frame.can_id |= CAN_EFF_FLAG;
  }

  if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
    frame.can_id |= CAN_RTR_FLAG;
  }

  frame.can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;
  frame.echo_id = 0xFFFFFFFF; // not a echo frame
  frame.reserved = 0;
  frame.flags = 0;

  if (RxHeader.FDFormat == FDCAN_FD_CAN) {
    /* this is a CAN-FD frame */
    frame.flags = GS_CAN_FLAG_FD;
    if (RxHeader.BitRateSwitch == FDCAN_BRS_ON) {
      frame.flags |= GS_CAN_FLAG_BRS;
    }
    /* need to handle the fact that DLCs over 8 bytes are enumerated */
    if (RxHeader.DataLength > FDCAN_DLC_BYTES_8) {
      /* convert the enumeration to size */
      frame.can_dlc = can_covert_stm32dlc_to_len(RxHeader.DataLength);
    }

    memcpy(frame.canfd.data, can_rx_data_buff, frame.can_dlc);
  }
  else {
    memcpy(frame.classic_can.data, can_rx_data_buff, frame.can_dlc);
  }

  /* put this CAN message into the queue to send to host */
  osMessageQueuePut(queue_to_hostHandle, &frame, 0, 0);
}

static uint8_t can_covert_stm32dlc_to_len(uint32_t DataLength)
{

  uint8_t len = 0;

  switch (DataLength) {
    case FDCAN_DLC_BYTES_12:
      len = 12;
      break;
    case FDCAN_DLC_BYTES_16:
      len = 16;
      break;
    case FDCAN_DLC_BYTES_20:
      len = 20;
      break;
    case FDCAN_DLC_BYTES_24:
      len = 24;
      break;
    case FDCAN_DLC_BYTES_32:
      len = 32;
      break;
    case FDCAN_DLC_BYTES_48:
      len = 48;
      break;
    case FDCAN_DLC_BYTES_64:
      len = 64;
      break;
  }

  return len;
}

static uint32_t can_covert_len_to_stm32dlc(uint8_t len)
{

  uint32_t DataLength = 0;

  switch (len) {
    case 12:
      DataLength = FDCAN_DLC_BYTES_12;
      break;
    case 16:
      DataLength = FDCAN_DLC_BYTES_16;
      break;
    case 20:
      DataLength = FDCAN_DLC_BYTES_20;
      break;
    case 24:
      DataLength = FDCAN_DLC_BYTES_24;
      break;
    case 32:
      DataLength = FDCAN_DLC_BYTES_32;
      break;
    case 48:
      DataLength = FDCAN_DLC_BYTES_48;
      break;
    case 64:
      DataLength = FDCAN_DLC_BYTES_64;
      break;
  }

  return DataLength;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  FDCAN_ProtocolStatusTypeDef status;
  struct gs_host_frame frame;

  frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hfdcan);

  HAL_FDCAN_GetProtocolStatus(hfdcan, &status);
  can_parse_error_status(&status, &frame);

  /* QueuePut will only grab the bytes that match the size defined at initialization */
  osMessageQueuePut(queue_to_hostHandle, &frame, 0, 0);
}

static void can_parse_error_status(FDCAN_ProtocolStatusTypeDef *status, struct gs_host_frame *frame)
{
  frame->echo_id = 0xFFFFFFFF;
  frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
  frame->can_dlc = CAN_ERR_DLC;
  frame->classic_can.data[0] = CAN_ERR_LOSTARB_UNSPEC;
  frame->classic_can.data[1] = CAN_ERR_CRTL_UNSPEC;
  frame->classic_can.data[2] = CAN_ERR_PROT_UNSPEC;
  frame->classic_can.data[3] = CAN_ERR_PROT_LOC_UNSPEC;
  frame->classic_can.data[4] = CAN_ERR_TRX_UNSPEC;
  frame->classic_can.data[5] = 0;
  frame->classic_can.data[6] = 0;
  frame->classic_can.data[7] = 0;

  if (status->BusOff == 1) {
    frame->can_id |= CAN_ERR_BUSOFF;
  }

  if (status->ErrorPassive == 1) {
    frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
  }
  else if (status->Warning) {
    frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
  }

  uint8_t lec = status->LastErrorCode;
  if (lec != 0) { /* protocol error */
    switch (lec) {
      case 0x01: /* stuff error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_STUFF;
        break;
      case 0x02: /* form error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_FORM;
        break;
      case 0x03: /* ack error */
        frame->can_id |= CAN_ERR_ACK;
        break;
      case 0x04: /* bit recessive error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_BIT1;
        break;
      case 0x05: /* bit dominant error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_BIT0;
        break;
      case 0x06: /* CRC error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
        break;
      default:
        break;
    }
  }
}