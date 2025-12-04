/*********************************************************************************************************
* 模块名称: XXX.c
* 摘    要: 模块实现具体功能
* 当前版本: 1.0.0
* 作    者: XXX
* 完成日期: 2024年12月14日
* 内    容:
* 注    意:
**********************************************************************************************************
* 取代版本:
* 作    者:
* 完成日期:
* 修改内容:
* 修改文件:
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "ECG.h"
#include "ADC.h"
#include "UART1.h"
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "Filter.h" 
#include "stm32f10x_conf.h"
#include "ECG_HeartRate_Calculate.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define ECG_LEAD_ON 0
#define ECG_LEAD_OFF 1
#define ECG_START 0
#define ECG_STOP 1

#define ECG_LEADOFF_PIN (BitAction)GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)
#define ECG_ZERO_PIN_START  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
#define ECG_ZERO_PIN_STOP  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
float ECG_WaveData[ECG_ADC_arrMAX] = {0}; //初始化数组
u16 WAVE_NUM = 0;         //波形数据包的点计数器
static u8 ECG_START_INFO = ECG_STOP;           //上位机发来的命令（待补充）
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static u8 ECG_LEADOFF_Check(void);              //检查导联脱落（这里引脚待补充）
static u8 ECG_Start_Check(void);                //检查开始测量信号Flag
static void ECG_ADC_Read(void);          //存入单个读取的数据，返回目前数组数据量
static void ECG_Wave_Send(void);                //发送数据
static void  ConfigECGGPIO(void);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/

/*********************************************************************************************************
* 函数名称: ECG_LEADOFF_Check
* 函数功能: 心电导联脱落Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:脱落：PA7高电平1；连接：PA7低电平0                                                          
*********************************************************************************************************/
static u8 ECG_LEADOFF_Check()
{
  static char ECG_LEAD_ERROR_FLAG = FALSE;
  if(ECG_LEADOFF_PIN==ECG_LEAD_OFF)
  {
    if(ECG_LEAD_ERROR_FLAG == FALSE)//更新ERROR标志并发送ERROR消息
    {
      ECG_LEAD_ERROR_FLAG = TRUE;
      printf("ERROR：心电导联没接上哦\n");
    }
  }
  else if(ECG_LEADOFF_PIN==ECG_LEAD_ON)
  {
    if(ECG_LEAD_ERROR_FLAG == TRUE)//更新ERROR标志并发送ERROR消息
    {
      ECG_LEAD_ERROR_FLAG = FALSE;
      printf("INFO:导联已连接\n");
    }  
  }
  else
  {
    printf("WARNING:导联引脚状态检测异常\n");
  }
  return ECG_LEAD_ERROR_FLAG;
}

/*********************************************************************************************************
* 函数名称: ECG_Start_Check
* 函数功能: 心电开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static u8 ECG_Start_Check()
{
  static char ECG_START_FLAG = ECG_STOP;
  if(ECG_START_INFO == ECG_START)
  {
    if(ECG_START_FLAG == ECG_STOP)//更新FLAG标志并发送开始消息
    {
      ECG_START_FLAG = ECG_START;
      ECG_ZERO_PIN_START
      printf("INFO：心电信号展示开始\n");
    }
  }
  else if(ECG_START_INFO == ECG_STOP)
  {
    if(ECG_START_FLAG == ECG_START)//更新FLAG标志并发送开始消息
    {
      ECG_START_FLAG = ECG_STOP;
      ECG_ZERO_PIN_STOP
      printf("INFO：心电信号展示停止\n");    
    }
  }
  return ECG_START_FLAG;
}

/*********************************************************************************************************
* 函数名称: ECG_ADC_Read
* 函数功能: 心电ADC值读取存入
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void ECG_ADC_Read()
{
  u16 adcData;                      //队列数据
  float  waveData;                  //波形数据
  
  static u8 ECG_TM_counter = 0;            //计数器
  ECG_TM_counter++;                        //计数增加

  if(ECG_TM_counter >= ECG_ADC_TM)                  //达到2ms
  {
    if(ReadADCBuf(&adcData))        //从缓存队列中取出1个数据
    {
      //waveData = (adcData * 3.3) / 4095;      //计算获取点的位置
      waveData = adcData * 1.0;      //计算获取点的位置
      //printf("%lf\n",waveData);      //测试：是否能取出信号；正确是输出心电电压信号
      ECG_WaveData[WAVE_NUM] = waveData;  //存放到数组
    }
    ECG_TM_counter = 0;                              //准备下次的循环
  }
}


/*********************************************************************************************************
* 函数名称: ECG_Wave_Send
* 函数功能: 心电波形数据发送
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
static void ECG_Wave_Send()
{
  static u8 ECG_TM_counter = 0;            //计数器
  ECG_TM_counter++;                        //计数增加

  if(ECG_TM_counter >= ECG_ADC_TM)     //达到2ms
  {  
    printf("%f\n",ECG_WaveData[WAVE_NUM]);
    ECG_TM_counter = 0;                              //准备下次的循环
  }
}
/*********************************************************************************************************
* 函数名称：ConfigLEDGPIO
* 函数功能：配置LED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigECGGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //GPIO_InitStructure用于存放GPIO的参数
                                                                     
  //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;           //设置引脚 PC13 ECG_ZERO
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化GPIO
  
    //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;           //设置引脚 PA7 ECG_LEAD_OFF
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;     //设置模式
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据参数初始化GPIO
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: ECG_Init
* 函数功能: 心电测量初始化函数
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void ECG_Init()
{
  ConfigECGGPIO();
  ECG_START_INFO = ECG_STOP;
  printf("{{2,HR}}\r\n");
  printf("{{3,HRFlag}}\r\n");
  printf("{{4,Filter}}\r\n");
}

/*********************************************************************************************************
* 函数名称: ECG_Task
* 函数功能: 心电测量任务主入口
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void ECG_Task()
{  
  static u8 test_point = '0';
  if(ECG_LEADOFF_Check()==TRUE)//检查导联脱落（PA7）
  {
    return;    
  }
  
  if(ECG_Start_Check()==ECG_STOP) //检查开始测量信号Flag
  {
    return;    
  }
  
  ECG_ADC_Read();          //存入单个读取的数据，返回目前数组数据量
  
  ECG_Filter(&ECG_WaveData[WAVE_NUM]);                   //滤波

  ECG_Wave_Send();                //发送数据
  
  WAVE_NUM++;                          //波形数据包的点计数器加1操作
  
  if(WAVE_NUM >= ECG_ADC_arrMAX)      //当存满了
  {
    ECG_HeartRate_Calculate();      //计算心率
    WAVE_NUM=0;
    switch(test_point)
    {
      case '0':
        test_point ++;
        printf("[[3,%c]]\r\n",test_point); //设置测量结果显示
      break;
      case '1':
        test_point = '0';
        printf("[[3,%c]]\r\n",test_point); //设置测量结果显示
      break;
    }
  }
}
/*********************************************************************************************************
* 函数名称: ECG_Start_Check
* 函数功能: 心电开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void ECG_StartInfo_Change(char value)
{
  switch(value)
  {
    case ECG_START:ECG_START_INFO=ECG_START;break;
    case ECG_STOP:ECG_START_INFO=ECG_STOP;break;
    default:break;
  }
}

/*********************************************************************************************************
* 函数名称: ECG_Start_Check
* 函数功能: 心电开始测量Flag检测
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
u8 ECG_StartInfo_Get()
{
  return ECG_START_INFO;
}


