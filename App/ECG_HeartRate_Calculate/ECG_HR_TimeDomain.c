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
#include "ECG_HR_TimeDomain.h"
#include "arm_math.h"
#include "ECG.h"
#include "UART1.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define ECG_Statistic_Num 2

// 滤波参数定义（0.5-5Hz带通IIR滤波器，500Hz采样率）
#define BANDPASS_ORDER 2

// 峰值检测参数
#define WINDOW_SIZE 50  // 0.2秒窗口(500Hz*0.2)
#define MIN_RR_INTERVAL 300  // 最小RR间期(300bpm对应200ms)
#define MAX_RR_INTERVAL 1200 // 最大RR间期(40bpm对应1500ms)

#define RR_MS_TO_SAMPLES(ms)  ((ms) * 500.0f / 1000.0f)
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
// 滤波参数定义（0.5-5Hz带通IIR滤波器，500Hz采样率）
static float32_t bandpass_coeffs[BANDPASS_ORDER*5] = {
    0.0018, 0.0035, 0.0018, -1.9506, 0.9512  // b0,b1,b2,a1,a2
};
static arm_biquad_casd_df1_inst_f32 bandpass_inst;
static float32_t bandpass_state[4];  // 2阶滤波器需要4个状态变量

// 峰值检测参数
static float32_t ecg_filtered[ECG_ADC_arrMAX] = {0};
static uint32_t r_peak_indices[ECG_Statistic_Num] = {0};
static uint16_t r_peak_count = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
float ECG_HR_TimeDomain_Calc(void);
void ECG_HR_Send1(float ECG_HeartRate);

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: ECG_HR_TimeDomain_Calc
* 函数功能: 心电心率计算
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
float ECG_HR_TimeDomain_Calc(void)
{
    uint16_t i, j;
    float32_t max_val;
    uint16_t max_idx;
    uint32_t rr_interval_sum = 0;
        float ECG_HeartRate = 0;
        float32_t avg_val = 0;

  // 1. 带通滤波预处理
    arm_biquad_cascade_df1_f32(&bandpass_inst, (float32_t*)ECG_WaveData, ecg_filtered, ECG_ADC_arrMAX);
    
    // 2. 滑动窗口峰值检测
    // 修改滑动窗口循环
    r_peak_count = 0;
    // 从 WINDOW_SIZE/2 开始，到 ECG_ADC_arrMAX - WINDOW_SIZE/2 结束，步长=1
    for(i = WINDOW_SIZE/2; i < ECG_ADC_arrMAX - WINDOW_SIZE/2; i++)
    {
        // 窗口内找最大值（范围：i-25 ~ i+25）
        max_val = ecg_filtered[i];
        max_idx = i;
        for(j = i - WINDOW_SIZE/2; j < i + WINDOW_SIZE/2; j++)
        {
            if(ecg_filtered[j] > max_val)
            {
                max_val = ecg_filtered[j];
                max_idx = j;
            }
        }
        
        // 峰值验证：增加“峰值突出度”判断（抗噪）
        // 计算窗口内平均值（作为基线参考）
        for(j = i - WINDOW_SIZE/2; j < i + WINDOW_SIZE/2; j++)
        {
            avg_val += ecg_filtered[j];
        }
        avg_val /= WINDOW_SIZE;
        // 要求峰值比窗口平均值高 50 以上（可根据你的数据调整，突出 R 波）
        if(ecg_filtered[max_idx] > avg_val + 50 &&
           ecg_filtered[max_idx] > ecg_filtered[max_idx-1] && 
           ecg_filtered[max_idx] > ecg_filtered[max_idx+1])
        {
            // RR 间期判断（用第一步修改后的逻辑）
            if(r_peak_count == 0 || 
               ((max_idx - r_peak_indices[r_peak_count-1]) >= RR_MS_TO_SAMPLES(MIN_RR_INTERVAL) &&
                (max_idx - r_peak_indices[r_peak_count-1]) <= RR_MS_TO_SAMPLES(MAX_RR_INTERVAL)))
            {
                r_peak_indices[r_peak_count++] = max_idx;
                if(r_peak_count >= ECG_Statistic_Num) break;
            }
        }
    }
    
    // 3. 计算心率
    if(r_peak_count < 2) 
    {
          ECG_HR_Send1(-1);
      return -1;  // 峰值不足
    }
    
    for(i = 0; i < r_peak_count - 1; i++)
    {
        rr_interval_sum += r_peak_indices[i+1] - r_peak_indices[i];
    }
    ECG_HeartRate = 60.0 / ((rr_interval_sum / (r_peak_count - 1)) * 0.002f);
    ECG_HR_Send1(ECG_HeartRate);
    // 采样间隔2ms，转换为bpm
    return ECG_HeartRate;
}

/*********************************************************************************************************
* 函数名称: ECG_HR_Send
* 函数功能: 心电心率计算
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void ECG_HR_Send1(float ECG_HeartRate)

{
  //printf("INFO:ECG_HeartRate:%lf",ECG_HeartRate);
  //printf("{{2,HR}}\r\n");    //设置参数名显示
  printf("[[2,%f]]\r\n",ECG_HeartRate); //设置测量结果显示
  
}
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: ECG_HR_TimeDomain_Init
* 函数功能: 心电心率计算初始化
* 输入参数: void
* 输出参数: void
* 返 回 值: void
* 创建日期: 2025年11月26日
* 注    意:
*********************************************************************************************************/
void ECG_HR_TimeDomain_Init(void)
{
  arm_biquad_cascade_df1_init_f32(&bandpass_inst, BANDPASS_ORDER, 
                                 bandpass_coeffs, bandpass_state);
}
