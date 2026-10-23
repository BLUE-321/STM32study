#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "queue.h"
#include "OLED.h"

//volatile int16_t g_ax = 0; 
QueueHandle_t xQueue_AX;

// 纯 CPU 空循环延时，替代江协的 SysTick 延时
void RTOS_Delay_us(uint32_t us)
{
    uint32_t i;
    for(i = 0; i < us * 8; i++);
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

void Hardware_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // PC13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    // PA9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200; 
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
    
    // PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 19999; 
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 1500; 
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_Cmd(TIM2, ENABLE);
    
    MyI2C_Init();
    MPU6050_Init();
	
	OLED_Init(); // 初始化 OLED
    OLED_Clear(); // 清屏
    OLED_ShowString(1, 1, "System V2.0"); // 开机画面
    OLED_ShowString(2, 1, "Init OK...");
}

// 读任务（低优先级）
void Task_ReadSensor(void *pvParameters)
{
    int16_t ax, ay, az, gx, gy, gz;
    float filtered_ax = 0; // 用来保存滤波后的结果
    int16_t send_ax = 0;
    
    while(1)
    {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        
        // 一阶低通滤波算法 (平滑底噪)
        // 0.8 表示 80% 信任历史数据，0.2 表示 20% 信任新数据
        filtered_ax = (filtered_ax * 0.8) + (ax * 0.2);
        send_ax = (int16_t)filtered_ax;
        
        // 发送滤波后的数据
        xQueueSend(xQueue_AX, &send_ax, pdMS_TO_TICKS(10)); 
        
        vTaskDelay(20); 
    }
}

// 控舵机任务（最高优先级！救火队长）
void Task_ControlServo(void *pvParameters)
{
    int pwm_value = 1500;
    int last_pwm = 1500; // 记录上一次的 PWM
    int16_t received_ax; 
    
    const int PWM_MIN = 500;
    const int PWM_MAX = 2500;
    const int SENSOR_MIN = -2000; 
    const int SENSOR_MAX = 2000;
    
    OLED_ShowString(1, 1, "AX:  "); 
    OLED_ShowString(2, 1, "PWM: ");
    
    while(1)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
        
        if(xQueueReceive(xQueue_AX, &received_ax, portMAX_DELAY) == pdTRUE)
        {
            // 限制范围
            if(received_ax < SENSOR_MIN) received_ax = SENSOR_MIN;
            if(received_ax > SENSOR_MAX) received_ax = SENSOR_MAX;

            // 线性映射
            pwm_value = PWM_MIN + (received_ax - SENSOR_MIN) * (PWM_MAX - PWM_MIN) / (SENSOR_MAX - SENSOR_MIN);

            // --- 核心防抖：PWM 增量死区 ---
            // 如果新算出来的 PWM 和上次的差距绝对值小于 20，就忽略它！
            int diff = pwm_value - last_pwm;
            if(diff > 20 || diff < -20) 
            {
                // 差距大，说明是真实倾斜，更新 PWM
                TIM_SetCompare3(TIM2, pwm_value); 
                last_pwm = pwm_value; // 记录这次的 PWM
                
                // 更新 OLED
                OLED_ShowSignedNum(1, 5, received_ax, 5); 
                OLED_ShowNum(2, 6, pwm_value, 4);
                
                printf("AX: %d, PWM: %d\r\n", received_ax, pwm_value);
            }
        }
        
        vTaskDelay(50); 
    }
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    Hardware_Init();
    
	// 创建一个队列：能装 5 个数据，每个数据的大小是 int16_t
	xQueue_AX = xQueueCreate(5, sizeof(int16_t));
	if(xQueue_AX == NULL) 
		{
			printf("队列创建失败！可能是内存不够\r\n");
		}

    // 关键修改：控制舵机的优先级设为 4，读传感器设为 2
    xTaskCreate(Task_ReadSensor, "Read", 256, NULL, 2, NULL);   
    xTaskCreate(Task_ControlServo, "Servo", 256, NULL, 4, NULL); 
    
    vTaskStartScheduler();
    while(1);
}

