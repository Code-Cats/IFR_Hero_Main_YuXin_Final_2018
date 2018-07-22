
#ifndef __USART_JUDGE_H__
#define __USART_JUDGE_H__


#include "bsp.h"

#define JUDGEMENTSENDDATA_DEFAULT {0,SOF_FIXED,13,0,0,0,0,0,0} 
#define  SOF_FIXED                          0xA5    //固定帧头字节
#define  FRAMEHEADER_SIZE                   0x04      //帧头长度
#define  FRAMEHEADER_LEN                    0x05    //帧头长度
#define  BSP_USART3_DMA_RX_BUF_LEN          100u  
#define  RC_JUDGEMENT_LENGTH                44u

#define  RESTART      1
#define  NO_RESTART   0

#define  ON           1
#define  OFF          0

#define  HURT         					 1
#define  SMALLAMROR_FRAME_CHANGE 2
#define  BIGARMOR_FRAME_CHANGE   2
#define  BIGARMOR_LOST           3

#define  GENERAL	    			   0
#define  LITTLE_ADVANTAGE		   1
#define  LARGE_ADVANTAGE	  	 2
#define  LITTLE_DISADVANTAGE   3
#define  LARGE_DISADVANTAGE		 4

#define  START        0
#define  PROCESSING   1
#define  END          2

#define  LEVEL_0    0
#define  LEVEL_1    1
#define  LEVEL_2    2
#define  LEVEL_4    4
#define  LEVEL_5    10
#define  LEVEL_BIG  20

typedef enum{
  GameRobotStateId  = 0x0001, //比赛机器人状态         8 Byte           10HZ
  RobotHurtId       = 0x0002,  //机器人伤害数据        1 Byte            ---
  ShootDataId       = 0x0003, //实时射击数据           6 Byte            ---
	PowerHeatDataId   = 0x0004, //实时功率热量数据      20 Byte            50HZ
	RfidDetectId      = 0x0005, //场地交互数据           2 Byte            10HZ
	GameResultId      = 0x0006, //比赛胜负数据           2 Byte            ---
	GetBuffId         = 0x0007, //Buff获取数据           3 Byte            ---
	GameRobotPosId    = 0x0008,	//机器人位置朝向信息    16 Byte            50HZ
	StudentSend       = 0x0100, //学生自定义数据
}comIdType;


typedef __packed struct
{
  uint8_t   sOF;           //165
  uint16_t  dataLenth;     //20
	uint8_t   cSeq;
  uint8_t   cRC8;
}tFrameHeader;

typedef __packed struct 
{ 
   uint16_t  stageRemainTime;  //当前阶段剩余时间
   uint8_t   gameProgress;     //当前比赛阶段  
   uint8_t   robotLevel;       //机器人当前等级
   uint16_t   remainHP;         //机器人当前血量
   uint16_t   maxHP;            //机器人满血量
}tGameRobotState; 

typedef __packed struct
{
  uint8_t armorType:4;    /*0-3bits: 若变化类型为装甲伤害时：标识装甲 ID
                        0x00: 0 号装甲面 （前）
                        0x01： 1 号装甲面 （左）
                        0x02： 2 号装甲面 （后）
                        0x03： 3 号装甲面 （ 右）
                        0x04: 4 号装甲面 （上 1）
                        0x05: 5 号装甲面（ 上 2）*/
  
  uint8_t hurtType:4;   /*4-7bits: 血量变化类型
                        0x0: 装甲伤害（受到攻击）    
                        0x1: 模块掉线*/
}tRobotHurt;

typedef __packed struct
{
  uint8_t bulletType;        //弹丸类型   1-> 17mm弹丸 ；2-> 42mm弹丸
  uint8_t bulletFreq;       //弹丸射频（ 发/s）
  float   bulletSpeed;     //弹丸射速(m/s 英雄机器人)
}tShootData;

typedef __packed struct  
{  
   float chassisVolt;        //底盘输出电压
   float chassisCurrent;     //底盘输出电流
   float chassisPower;       //底盘输出功率
   float chassisPowerBuffer; //底盘功率缓冲
	 uint16_t shooterHeat0;    //17mm枪口热量
	 uint16_t shooterHeat1;    //42mm枪口热量
}tPowerHeatData;

typedef __packed struct 
{
  uint8_t cardType;  //卡类型   0：攻击加成卡  1：防御加成卡  2：红方加血卡 3：蓝方加血卡 4：红方大能量机关卡 5：蓝方大能量机关卡
	uint8_t cardIdx;   //卡索引号，可用于区分不同区域
}tRfidDetect;

typedef __packed struct
{
  uint8_t winner;
}tGameResult;

typedef __packed struct
{
  uint8_t enemySmallBuff;
	uint8_t enemyBigBuff;
	uint8_t myselfSmallBuff;
	uint8_t myselfBigBuff;
}tGetBuff;

typedef __packed struct
{
  float x;   //X坐标
	float y;   //Y坐标
	float z;   //Z坐标
	float yaw; //枪口朝向角度值，单位：度
}tGameRobotPos;

typedef __packed struct
{
  float data1;    //自定义数据1
	float data2;    //自定义数据2
	float data3;    //自定义数据3
	uint8_t mask;   //自定义数据4
}tShowData;

typedef union{
	
	u8		U8[4];
	float F32;
}Send2PcDataConvertUnion;

typedef __packed struct
{
  char flag; 
  uint8_t   sOF;
  uint16_t  dataLenth;
	uint8_t   cSeq;
  uint8_t   cRC8;	
  float Data_A; 
	float Data_B;
	float Data_C;
	uint8_t Data_D;
}JudgementSendData;

typedef struct  __JUDGEMENT_DATA__
{
  char flag;	
  char shootflag;	

}JUDGEMENT_DATA;
extern u8 JudgeSendBuff[22];
void Judgement_DataSend(float a,float b,float c,uint8_t d);
uint8_t Judagement_Send_Guiding_lights(u8 stateA, u8 stateB, u8 stateC, u8 stateD, u8 stateE, u8 stateF);
//void Judagement_Send_Change_hero(float *a,float *b,float *c,uint8_t *d);
extern uint8_t judgementBuf[];
extern JUDGEMENT_DATA judgementData;
extern tFrameHeader            testFrameHeader;
extern tGameRobotState         testGameRobotState;      //比赛机器人状态
extern tRobotHurt              testRobotHurt;          //机器人伤害数据
extern tShootData              testShootData;          //实时射击数据
extern tPowerHeatData 				  testPowerHeatData;      //实时功率热量数据
extern tRfidDetect             testRfidDetect;          //场地交互数据
extern tGameResult             testGameResult;          //比赛胜负数据
extern tGetBuff                testGetBuff;             //Buff获取数据
extern tGameRobotPos           testGameRobotPos;        //机器人位置朝向信息
extern tShowData               testShowData;            //自定义数据
extern JudgementSendData     testJudgementSendData;

extern uint16_t  testcmdId;
extern int16_t testNum;
extern JUDGEMENT_DATA Judgement_Data;
//void judgementDataHandler(void);
void USART3_Configuration(uint32_t baud_rate);
void USART3_IRQHandler(void);
void Frameheader_Prcess(uint8_t *pData);
void BinocularVisionDataPrcess(uint8_t *pData);
float get_speed(void);
void judgementDataHandler(void);
#endif
