/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @version V2.4.0
  * @date    28-February-2015
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "commands.h"

static int8_t UsbVcp_Init     (void);
static int8_t UsbVcp_DeInit   (void);
static int8_t UsbVcp_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t UsbVcp_Receive  (uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
 UsbVcp_Init,
 UsbVcp_DeInit,
 UsbVcp_Control,
 UsbVcp_Receive
};

USBD_CDC_LineCodingTypeDef linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/* Private functions ---------------------------------------------------------*/

/**
  * @brief UsbVcp_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
extern USBD_HandleTypeDef USBD_Device;

char g_VCPInitialized;

static int8_t UsbVcp_Init(void)
{
	//USBD_CDC_SetRxBuffer(&USBD_Device, usbVcpRxBuffer.Buffer);
	USBD_CDC_SetRxBuffer(&USBD_Device, rxBuf);
	g_VCPInitialized = 1;
	return (0);
}

/**
  * @brief UsbVcp_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UsbVcp_DeInit(void)
{
  /*
     Add your deinitialization code here 
  */  
  return (0);
}


/**
  * @brief UsbVcp_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t UsbVcp_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    linecoding.format     = pbuf[4];
    linecoding.paritytype = pbuf[5];
    linecoding.datatype   = pbuf[6];
    
    /* Add your code here */
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(linecoding.bitrate);
    pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
    pbuf[4] = linecoding.format;
    pbuf[5] = linecoding.paritytype;
    pbuf[6] = linecoding.datatype;     
    
    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }

  return (0);
}

//static int8_t UsbVcp_Receive(uint8_t* Buf, uint32_t *Len)
//{
//	usbVcpRxBuffer.Position = 0;
//	usbVcpRxBuffer.Size = *Len;
//	usbVcpRxBuffer.ReadDone = 1;
//	return (0);
//}

int8_t UsbVcp_Receive(uint8_t* Buf, uint32_t *Len)
{
	USBD_CDC_SetRxBuffer(&USBD_Device, &Buf[0]);
	USBD_CDC_ReceivePacket(&USBD_Device);
	rxLen = *Len;
	return (USBD_OK);
}

//enum { kMaxOutPacketSize = CDC_DATA_FS_OUT_PACKET_SIZE };

int UsbVcp_write(const void *pBuffer, int size)
{
	//Looks like this is breaking a large packet up into smaller ones, 
	//instead just decide that all packets will be small.
//    if (size > kMaxOutPacketSize)
//	{
//		int offset;
//    	int done = 0;
//    	for (offset = 0; offset < size; offset += done)
//		{
//    		int todo = MIN(kMaxOutPacketSize, size - offset);
//			done = UsbVcp_write(((char *)pBuffer) + offset, todo);
//			if (done != todo)
//				return offset + done;
//		}
//
//		return size;
//	}

	USBD_CDC_HandleTypeDef *pCDC =
	        (USBD_CDC_HandleTypeDef *)USBD_Device.pClassData;
	//while (pCDC->TxState) {} //Wait for previous transfer

	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *)pBuffer, size);
	if (USBD_CDC_TransmitPacket(&USBD_Device) != USBD_OK)
		return 0;

	//while (pCDC->TxState) {} //Wait until transfer is done
	return size;
}
