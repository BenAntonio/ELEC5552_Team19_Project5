#include "usbh_usr.h"
#include "usb_hcd_int.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"  
#include "delay.h"  
#include "lcd.h"  
#include "string.h"  


vu8 bDeviceState=0;		  


extern USB_OTG_CORE_HANDLE USB_OTG_Core_dev;
extern void USBH_Msg_Show(u8 msgx);
u8 USB_FIRST_PLUGIN_FLAG=0;

//USB OTG 
//
void OTG_FS_IRQHandler(void)
{ 
	USBH_OTG_ISR_Handler(&USB_OTG_Core_dev);
}  

//USB HOST 
USBH_Usr_cb_TypeDef USR_Callbacks =
{
  USBH_USR_Init,
  USBH_USR_DeInit,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  NULL,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};
 
//USB HOST  
void USBH_USR_Init(void)
{
	printf("USB OTG FS MSC Host\r\n");
	printf("> USB Host library started.\r\n");
	printf("  USB Host Library v2.1.0\r\n\r\n");
	
}
//
void USBH_USR_DeviceAttached(void)
{ 
	printf("检测到USB设备插入!\r\n");
}
/
void USBH_USR_DeviceDisconnected (void)/
{ 
	printf("USB out!\r\n");
	bDeviceState=0;	
	USBH_Msg_Show(0);
}  

void USBH_USR_ResetDevice(void)
{
	printf("...\r\n");
}
//
//DeviceSpeed:(0,1,2 / )
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		printf("(HS)USB!\r\n");
 	}  
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		printf("(FS)USB!\r\n"); 
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		printf("(LS)USB!\r\n");  
	}
	else
	{
		printf("error!\r\n");  
	}
}

//DeviceDesc:
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{ 
	USBH_DevDesc_TypeDef *hs;
	hs=DeviceDesc;   
	printf("VID: %04Xh\r\n" , (uint32_t)(*hs).idVendor); 
	printf("PID: %04Xh\r\n" , (uint32_t)(*hs).idProduct); 
}

void USBH_USR_DeviceAddressAssigned(void)
{
	printf("success!\r\n");   
}

void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
	USBH_InterfaceDesc_TypeDef *id; 
	id = itfDesc;   
	if((*id).bInterfaceClass==0x08)
	{
		printf("removable!\r\n"); 
	}else if((*id).bInterfaceClass==0x03)
	{
		printf("HID !\r\n"); 
	}    
}
//Manufacturer String
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	printf("Manufacturer: %s\r\n",(char *)ManufacturerString);
}
//Product String 
void USBH_USR_Product_String(void *ProductString)
{
	printf("Product: %s\r\n",(char *)ProductString);  
}
//SerialNum String 
void USBH_USR_SerialNum_String(void *SerialNumString)
{
	printf("Serial Number: %s\r\n",(char *)SerialNumString);    
} 

void USBH_USR_EnumerationDone(void)
{ 
	printf("finish!\r\n\r\n");    
} 
//
void USBH_USR_DeviceNotSupported(void)
{ 
	USBH_Msg_Show(3);
	//printf("!\r\n\r\n");    
}  
//
USBH_USR_Status USBH_USR_UserInput(void)
{ 
	printf("!\r\n");
	bDeviceState=1;
	return USBH_USR_RESP_OK;
} 

void USBH_USR_OverCurrentDetected (void)
{
	printf("!!!\r\n");
}  
//
void USBH_USR_DeInit(void)
{
	printf("!!!\r\n");
}
//
void USBH_USR_UnrecoveredError (void)
{
	printf("!!!\r\n\r\n");	
}

u8 USBH_Check_EnumeDead(USBH_HOST *phost)
{
	static u16 errcnt=0;
	
	if(phost->gState==HOST_CTRL_XFER&&(phost->EnumState==ENUM_IDLE||phost->EnumState==ENUM_GET_FULL_DEV_DESC))
	{
		errcnt++;
		if(errcnt>2000)//
		{ 
			errcnt=0;
			RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,ENABLE);//USB OTG FS 
			delay_ms(5);
			RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,DISABLE);	 
			return 1;
		} 
	}else errcnt=0;
	return 0;
} 

u8 USBH_Check_HIDCommDead(USB_OTG_CORE_HANDLE *pcore,HID_Machine_TypeDef *phidm)
{
 	if(pcore->host.HC_Status[phidm->hc_num_in]==HC_DATATGLERR)//
	{  
		return 1;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////



void USR_MOUSE_Init	(void)
{
 	USBH_Msg_Show(2);		//USB
	USB_FIRST_PLUGIN_FLAG=1;
}
//键盘初始化
void  USR_KEYBRD_Init(void)
{ 
 	USBH_Msg_Show(1);		
	USB_FIRST_PLUGIN_FLAG=1;
}


__align(4) u8 tbuf[2017]; 


void USR_MOUSE_ProcessData(HID_MOUSE_Data_TypeDef *data)
{  
	static signed short x,y,z; 
	if(USB_FIRST_PLUGIN_FLAG)
	{
		USB_FIRST_PLUGIN_FLAG=0;
		x=y=z=0;
	}
	x+=(signed char)data->x;
	if(x>9999)x=9999;
	if(x<-9999)x=-9999;
	y+=(signed char)data->y;
	if(y>9999)y=9999;
	if(y<-9999)y=-9999;
	z+=(signed char)data->z;
	if(z>9999)z=9999;
	if(z<-9999)z=-9999;
	POINT_COLOR=BLUE;
	sprintf((char*)tbuf,"BUTTON:");
	if(data->button&0X01)strcat((char*)tbuf,"LEFT");
	if((data->button&0X03)==0X02)strcat((char*)tbuf,"RIGHT");
	else if((data->button&0X03)==0X03)strcat((char*)tbuf,"+RIGHT");
	if((data->button&0X07)==0X04)strcat((char*)tbuf,"MID");
	else if((data->button&0X07)>0X04)strcat((char*)tbuf,"+MID");  
 	LCD_Fill(30+56,180,lcddev.width,180+16,WHITE);	
	LCD_ShowString(30,180,210,16,16,tbuf);	
	sprintf((char*)tbuf,"X POS:%05d",x); 
	LCD_ShowString(30,200,200,16,16,tbuf);	
	sprintf((char*)tbuf,"Y POS:%05d",y);
	LCD_ShowString(30,220,200,16,16,tbuf);	
	sprintf((char*)tbuf,"Z POS:%05d",z);
	LCD_ShowString(30,240,200,16,16,tbuf);	 
	//printf("btn,X,Y,Z:0x%x,%d,%d,%d\r\n",data->button,(signed char)data->x,(signed char)data->y,(signed char)data->z);   
} 



void  USR_KEYBRD_ProcessData (uint8_t data)
{ 
	static u16 pos; 
	static u16 endx,endy;
	static u16 maxinputchar;
	
	u8 buf[4];
	if(USB_FIRST_PLUGIN_FLAG)/
	{
		USB_FIRST_PLUGIN_FLAG=0;
		endx=((lcddev.width-30)/8)*8+30;		
		endy=((lcddev.height-220)/16)*16+220;	
		maxinputchar=((lcddev.width-30)/8);
		maxinputchar*=(lcddev.height-220)/16;	
 		pos=0; 
	}
	POINT_COLOR=BLUE;
	sprintf((char*)buf,"%02X",data);
	LCD_ShowString(30+56,180,200,16,16,buf);
	if(data>=' '&&data<='~')
	{
		tbuf[pos++]=data;
		tbuf[pos]=0;		 
		if(pos>maxinputchar)pos=maxinputchar;
	}else if(data==0X0D)	
	{
		if(pos)pos--;
		tbuf[pos]=0;		
	}
	if(pos<=maxinputchar)	
	{
		LCD_Fill(30,220,endx,endy,WHITE);
		LCD_ShowString(30,220,endx-30,endy-220,16,tbuf);
	}		
	//printf("KEY Board Value:%02X\r\n",data);
	//printf("KEY Board Char:%c\r\n",data); 
}













