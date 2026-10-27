#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"
#include "math.h"

#include "MyI2C.h"
#include "MPU6050.h"
#include "OLED.h"

#define PI 3.1415926f

volatile uint8_t who_am_i = 0;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
} SensorData_t;

QueueHandle_t xQueue_Sensor;

/* 软件 I2C 使用的微秒延时 */
void RTOS_Delay_us(uint32_t us)
{
    uint32_t i;

    for(i = 0; i < us * 8; i++)
    {
        __NOP();
    }
}

/* printf 重定向到 USART1 */
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);

    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }

    return ch;
}

/* 浮点数限幅 */
float LimitFloat(float value, float min, float max)
{
    if(value < min)
    {
        return min;
    }

    if(value > max)
    {
        return max;
    }

    return value;
}

/* 硬件初始化 */
void Hardware_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_USART1,
        ENABLE
    );

    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2,
        ENABLE
    );

    /* PC13：板载 LED */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    /* PA9：USART1_TX */
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

    /*
     * TIM2_CH3 -> PA2
     *
     * 72 MHz / (71 + 1) = 1 MHz
     * 所以一个计数约等于 1 us
     *
     * 周期 20000 us，即 50 Hz
     */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
//时基单元初始化
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

    /* MPU6500 使用当前兼容 MPU6050 的驱动 */
    MyI2C_Init();
    MPU6050_Init();

    /* OLED 使用 PB8/PB9 */
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(1, 1, "Attitude");
    OLED_ShowString(2, 1, "Init OK");
}

/*
 * 传感器采集任务
 *
 * 约 50 Hz 采样。
 * 队列长度为 1，只保留最新数据。
 */
void Task_ReadSensor(void *pvParameters)
{
    SensorData_t sensor_data;
    int16_t gx;
    int16_t gy;
    int16_t gz;
	
	TickType_t current_tick;
    TickType_t last_tick = 0;
    volatile TickType_t 
	debug_delta_tick = 0;

    uint8_t first_sample = 1;
	
	volatile TickType_t tick_history[10] = {0};
	uint8_t history_index = 0;
	
	TickType_t min_tick;
	TickType_t max_tick;

	uint32_t sum_tick = 0;
	volatile float avg_tick = 0.0f;

	uint8_t i;
	uint8_t statistics_done = 0;
	
	
	
	
	TickType_t read_start;
	TickType_t read_end;
	volatile TickType_t read_duration = 0;
	
	volatile TickType_t read_history[10] = {0};
	uint8_t read_index = 0;
	
	TickType_t read_min;
	TickType_t read_max;
	uint32_t read_sum = 0;
	volatile float read_avg = 0.0f;
	uint8_t read_statistics_done = 0;
//	uint8_t history_count = 0;
	
    while(1)
    {
		current_tick = xTaskGetTickCount();
		
		if(first_sample != 1)
		{
			debug_delta_tick = current_tick - last_tick;
			if(history_index < 10)
			{
				tick_history[history_index] = debug_delta_tick;
				history_index++;
				if (history_index == 10 && statistics_done == 0)
				{
					min_tick = tick_history[0];
					max_tick = tick_history[0];
					
					for (i = 0; i < 10; i++)
					{
						if(tick_history[i]<min_tick)
						{
							min_tick = tick_history[i];
						}
						if (tick_history[i] > max_tick)
						{
							max_tick = tick_history[i];
						}
					sum_tick += tick_history[i];
					}
					avg_tick = sum_tick / 10.0f;
					statistics_done = 1;
				}
			
			}
		}
		
		last_tick = current_tick;
		
		first_sample = 0;
		read_start = xTaskGetTickCount();	
        MPU6050_GetData(
            &sensor_data.ax, 
            &sensor_data.ay, 
            &sensor_data.az, 
			&gx,
			&gy,
			&gz
		);
        /*
         * 新数据覆盖旧数据。
         * 对姿态数据来说，最新值比历史值更重要。
         */
		read_end = xTaskGetTickCount();
		read_duration = read_end - read_start;
		if(read_index < 10)
			{
				read_history[read_index] = read_duration;
				read_index++;
				if (read_index == 10 && read_statistics_done == 0)
				{
					read_min = read_history[0];
					read_max = read_history[0];
					
					for (i = 0; i < 10; i++)
					{
						if(read_history[i]< read_min)
						{
							read_min = read_history[i];
						}
						if (read_history[i] > read_max)
						{
							read_max = read_history[i];
						}
					read_sum += read_history[i];
					}
					read_avg  = read_sum / 10.0f;
					read_statistics_done = 1;
				}
			}
        xQueueOverwrite(
            xQueue_Sensor,
            &sensor_data
        );

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
 * 姿态计算、滤波和舵机控制任务
 */
void Task_ControlServo(void *pvParameters)
{
	TickType_t lastDisplayTick = 0;
	const TickType_t displayInterval = pdMS_TO_TICKS(100);
    SensorData_t sensor_data;

    float raw_pitch;
    float filtered_pitch;
    float denominator;

    int target_pwm;
    int output_pwm;
    int pwm_diff;

    uint8_t filter_initialized;

    const int PWM_MIN = 500;
    const int PWM_MAX = 2500;

    const int PWM_DEADBAND = 8;
    const int PWM_MAX_STEP = 80;

    filtered_pitch = 0.0f;
    output_pwm = 1500;
    filter_initialized = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, "P:");
    OLED_ShowString(2, 1, "PWM:");

    while(1)
    {
        if(xQueueReceive(
            xQueue_Sensor,
            &sensor_data,
            portMAX_DELAY
        ) == pdTRUE)
        {
            /*
             * 计算 Pitch
             *
             * 水平时一般接近 0°
             * 竖直时一般接近 +90° 或 -90°
             */
            denominator = sqrtf(
                (float)sensor_data.ay * sensor_data.ay +
                (float)sensor_data.az * sensor_data.az
            );

            raw_pitch = atan2f(
                -(float)sensor_data.ax,
                denominator
            ) * 180.0f / PI;

            raw_pitch = LimitFloat(
                raw_pitch,
                -90.0f,
                90.0f
            );

            /*
             * 第一次采样时直接赋值，避免上电缓慢爬升
             */
            if(filter_initialized == 0)
            {
                filtered_pitch = raw_pitch;
                filter_initialized = 1;
            }
            else
            {
                /*
                 * 一阶低通滤波
                 *
                 * 0.85：保留历史值
                 * 0.15：使用新值
                 *
                 * 如果仍然抖动，可改成：
                 * 0.90 和 0.10
                 */
                filtered_pitch =
                    filtered_pitch * 0.65f +
                    raw_pitch * 0.35f;
            }

            filtered_pitch = LimitFloat(
                filtered_pitch,
                -90.0f,
                90.0f
            );

            /*
             * 将 -90° 到 +90°
             * 映射到 500 到 2500 us
             */
            target_pwm = PWM_MIN +
                (int)(
                    (filtered_pitch + 90.0f)
                    * (PWM_MAX - PWM_MIN)
                    / 180.0f
                );

            if(target_pwm < PWM_MIN)
            {
                target_pwm = PWM_MIN;
            }

            if(target_pwm > PWM_MAX)
            {
                target_pwm = PWM_MAX;
            }

            /*
             * 目标 PWM 与当前 PWM 的差值
             */
            pwm_diff = target_pwm - output_pwm;

            /*
             * PWM 死区
             *
             * 小于 15 us 的变化被认为是噪声
             */
            if(
                pwm_diff > PWM_DEADBAND ||
                pwm_diff < -PWM_DEADBAND
            )
            {
                /*
                 * PWM 限速
                 *
                 * 每次最多只变化 20 us，
                 * 避免舵机突然大幅跳动
                 */
                if(pwm_diff > PWM_MAX_STEP)
                {
                    pwm_diff = PWM_MAX_STEP;
                }

                if(pwm_diff < -PWM_MAX_STEP)
                {
                    pwm_diff = -PWM_MAX_STEP;
                }

                output_pwm = output_pwm + pwm_diff;

                if(output_pwm < PWM_MIN)
                {
                    output_pwm = PWM_MIN;
                }

                if(output_pwm > PWM_MAX)
                {
                    output_pwm = PWM_MAX;
                }

                TIM_SetCompare3(
                    TIM2,
                    output_pwm
                );
            }

            /*
             * OLED 显示滤波后的 Pitch 和实际输出 PWM
             */
           if ((xTaskGetTickCount() - lastDisplayTick) >= displayInterval)
		   {
				OLED_ShowSignedNum(1, 3, (int32_t)filtered_pitch, 3);
				OLED_ShowNum(2, 5, output_pwm, 4);

				printf("RawPitch:%d FilterPitch:%d PWM:%d\r\n",
					   (int)raw_pitch,
					   (int)filtered_pitch,
					   output_pwm);

				lastDisplayTick = xTaskGetTickCount();
			}
		}
	}
}

int main(void)
{
    BaseType_t result;

    NVIC_PriorityGroupConfig(
        NVIC_PriorityGroup_2
    );

    Hardware_Init();

    /*
     * 队列长度必须为 1，
     * 因为使用 xQueueOverwrite()
     */
    xQueue_Sensor = xQueueCreate(
        1,
        sizeof(SensorData_t)
    );

    if(xQueue_Sensor == NULL)
    {
        /*
         * 队列创建失败，不启动任务
         */
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);

        while(1)
        {
        }
    }

    result = xTaskCreate(
        Task_ReadSensor,
        "Read",
        256,
        NULL,
        2,
        NULL
    );

    if(result != pdPASS)
    {
        while(1)
        {
        }
    }

    /*
     * 控制任务需要使用 sqrtf、atan2f，
     * 栈空间比普通任务稍大
     */
    result = xTaskCreate(
        Task_ControlServo,
        "Control",
        512,
        NULL,
        4,
        NULL
    );

    if(result != pdPASS)
    {
        while(1)
        {
        }
    }
	
	who_am_i = MPU6050_ReadReg(0x75);

    vTaskStartScheduler();

    while(1)
    {
    }
}
