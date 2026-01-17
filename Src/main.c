/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
osThreadId LCD_TaskHandle;
osThreadId DDS_AD9850_TaskHandle;

xQueueHandle Global_Queue_Handle = 0;

uint32_t Global_Freq = 11000000;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void LCD_Task(const void * arg);
void DDS_AD9850_Task(const void * arg);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define  FREQ_CONST  4294967296    //AD9850 calculation constant
#define  XTAL_MHZ    125000000     //cristal freq. on AD9850's pcb
#define  INTERMED    4000000       //intermediate frequency

// D7 D6 D5 D4 BL CS RW RS
// x  x  x  x  1  1  0  0 = XC (hex)
#define LCD_RS         (uint8_t) (0x01)//(0b00000001)  //P0 - PCF8574T Pin connected to RS
#define LCD_RW         (uint8_t) (0x02)//( 0b00000010)  //P1 - PCF8574T Pin connected to RW
#define LCD_EN         (uint8_t) (0x04)//( 0b00000100)  //P2 - PCF8574T Pin connected to EN
#define LCD_BACKLIGHT  (uint8_t) (0x08)//( 0b00001000)  //P3 - PCF8574T Pin connected to BACKLIGHT

// LCD Command
#define LCD_CUR_OFFSET 0x80      // 0x80 + 5 moves cursor to position 5 in a line

#define LCD_HOME 0x02
#define LCD_NEXT_LINE 0xC0
#define LCD_CLEAR 0x01
#define LCD_1CYCLE 0
#define LCD_2CYCLE 1

/******************************************************************************/
/* Calculation for frequency.It converts decimal freq. in HZ to AD9850 format */
/* @param the frequency                                                       */
/* @return calculated freq                                                    */
/******************************************************************************/
int calculate_frequency(int frequency) {
    return frequency * FREQ_CONST / XTAL_MHZ;
}

/* USER CODE END 0 */

int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();

    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    //LCD_Task
    osThreadDef(LCD, LCD_Task, osPriorityNormal, 0, 128);
    LCD_TaskHandle = osThreadCreate(osThread(LCD), NULL);

    // DDS_AD9850_Task
    osThreadDef(DDS_AD9850, DDS_AD9850_Task, osPriorityNormal, 0, 128);
    DDS_AD9850_TaskHandle = osThreadCreate(osThread(DDS_AD9850), NULL);

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    Global_Queue_Handle = xQueueCreate(10, 10 * sizeof(uint8_t));

    /* USER CODE END RTOS_QUEUES */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    __PWR_CLK_ENABLE()
    ;

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void) {

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    HAL_I2C_Init(&hi2c1);

}

/* SPI1 init function */
void MX_SPI1_Init(void) {

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void) {

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi2.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi2);

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PC3   ------> I2S2_SD
 PA4   ------> I2S3_WS
 PB10   ------> I2S2_CK
 PC7   ------> I2S3_MCK
 PA9   ------> USB_OTG_FS_VBUS
 PA10   ------> USB_OTG_FS_ID
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 PC10   ------> I2S3_CK
 PC12   ------> I2S3_SD
 */
void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __GPIOE_CLK_ENABLE()
    ;
    __GPIOC_CLK_ENABLE()
    ;
    __GPIOH_CLK_ENABLE()
    ;
    __GPIOA_CLK_ENABLE()
    ;
    __GPIOB_CLK_ENABLE()
    ;
    __GPIOD_CLK_ENABLE()
    ;

    /*Configure GPIO pin : PE2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : CS_I2C_SPI_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PE5 MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA6 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_IN_Pin */
    GPIO_InitStruct.Pin = CLK_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PB12 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
     Audio_RST_Pin */
    GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PC7 I2S3_SCK_Pin PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | I2S3_SCK_Pin | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : VBUS_FS_Pin */
    GPIO_InitStruct.Pin = VBUS_FS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
    GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB5 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

//void LCD_putcmd(unsigned char data,unsigned char cmdtype)
void LCD_putcmd(uint8_t data, uint8_t cmdtype) {
    //D7 D6 D5 D4 BL CS RW RS
    // x  x  x  x  x  x  x  x

    uint8_t i2c_bytes[1];

    // Put the Upper 4 bits data
    //D7 D6 D5 D4 BL CS RW RS
    // x  x  x  x  1  1  0  0
    i2c_bytes[0] = (data & 0xF0) | LCD_BACKLIGHT | LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(2);

    // Write Enable Pulse E: Hi -> Lo
    //D7 D6 D5 D4 BL CS RW RS
    // x  x  x  x  1  0  0  0
    i2c_bytes[0] &= (~LCD_EN );
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(1);

    // cmdtype = 0; One cycle write, cmdtype = 1; Two cycle writes
    if (cmdtype) {

        //D7 D6 D5 D4 BL CS RW RS
        // x  x  x  x  1  1  0  0
        i2c_bytes[0] = ((data << 4) & 0xF0) | LCD_BACKLIGHT | LCD_EN;
        HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
        osDelay(2);

        /// Write Enable Pulse E: Hi -> Lo
        //D7 D6 D5 D4 BL CS RW RS
        // x  x  x  x  1  0  0  0
        i2c_bytes[0] &= (~LCD_EN );
        HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
        osDelay(1);
    }
}

void LCD_init(void) {
    // 3v 40ms wait
    osDelay(41);

    // Send Command 0x30
    LCD_putcmd(0x30, LCD_1CYCLE);

    // Wait for more than 4.1 ms
    osDelay(5);

    // Send Command 0x30
    LCD_putcmd(0x30, LCD_1CYCLE);

    // Wait for more than 100 us
    osDelay(1);

    // Send Command 0x30
    LCD_putcmd(0x30, LCD_1CYCLE);

    // Function set: Set interface to be 4 bits long (only 1 cycle write).
    LCD_putcmd(0x20, LCD_1CYCLE);

    // Function set: DL=0;Interface is 4 bits, N=1; 2 Lines, F=0; 5x8 dots font)
    LCD_putcmd(0x28, LCD_2CYCLE);

    // Display on D=1; , C=1 Cursor on; B=0 Blinking off
    LCD_putcmd(0x0C, LCD_2CYCLE);

    // Display Clear
    LCD_putcmd(LCD_CLEAR, LCD_2CYCLE);

    // Entry Mode Set: I/D=1; Increament, S=0; No shift
    LCD_putcmd(0x06, LCD_2CYCLE);

    // Display On, Cursor Off
    //LCD_putcmd(0x0B, LCD_2CYCLE);
}

void LCD_putch(uint8_t data) {

    uint8_t i2c_bytes[1];

    // Put the Upper 4 bits data
    i2c_bytes[0] = (data & 0xF0) | LCD_BACKLIGHT | LCD_RS | LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(2);

    i2c_bytes[0] &= ~LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(1);

    // Put the Lower 4 bit data
    i2c_bytes[0] = ((data << 4) & 0xF0) | LCD_BACKLIGHT | LCD_RS | LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(2);

    i2c_bytes[0] &= ~LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
    osDelay(1);

}

/*
 * prints string on lCD
 */
void LCD_puts(uint8_t *s) {

    while (*s != 0) {      // While not Null
        if (*s == '\n')
            LCD_putcmd(LCD_NEXT_LINE, LCD_2CYCLE);  // Goto Second Line
        else
            LCD_putch(*s);
        s++;
    }

}

/*
 * converts integer to string
 */
int itoa(uint32_t value, uint8_t *ptr) {
    int count = 0, temp;
    if (ptr == NULL)
        return 0;
    if (value == 0) {
        *ptr = '0';
        return 1;
    }

    if (value < 0) {
        value *= (-1);
        *ptr++ = '-';
        count++;
    }
    for (temp = value; temp > 0; temp /= 10, ptr++)
        ;
    *ptr = '\0';
    for (temp = value; temp > 0; temp /= 10) {
        *--ptr = temp % 10 + '0';
        count++;
    }
    return count;

}

/*
 * sends frequency to AD9850
 */
void DDS_AD9850_send_freq(uint32_t freq) {
    freq = calculate_frequency(freq);
    uint8_t bytes[5];

    bytes[0] = (uint8_t) (freq);
    bytes[1] = (uint8_t) (freq >> 8);
    bytes[2] = (uint8_t) (freq >> 16);
    bytes[3] = (uint8_t) (freq >> 24);
    bytes[4] = 0;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, bytes, 5, 10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}

/*
 * this
 */
void DDS_AD9850_Task(const void * arg) {
    static uint32_t old_Global_Freq;
    uint32_t freq = Global_Freq;

    uint8_t buff[10];

    //freq += 1000;
    itoa(freq, buff);
    DDS_AD9850_send_freq(freq); //send frequency AD9850
    xQueueSend(Global_Queue_Handle, buff, 100); //send frequency to lcd

    for (;;) {
        if (old_Global_Freq != Global_Freq) {
            itoa(Global_Freq, buff);
            DDS_AD9850_send_freq(Global_Freq); //send frequency AD9850
            xQueueSend(Global_Queue_Handle, buff, 100); //send frequency to lcd

            old_Global_Freq = Global_Freq;
        }
        vTaskDelay(100);
    }

}

/*
 * this task prints information  to the LCD
 */
void LCD_Task(const void * arg) {

    LCD_init();
//LCD_puts("Hi people WOW!!!\n");
//LCD_puts("HEllO WORLD!!!");

    //uint16_t i = 0;

    uint8_t buff[10];

    for (;;) {
        //i += 1000;
        //itoa(i, buff);

        //LCD_putcmd(LCD_HOME, LCD_2CYCLE);
        //LCD_putcmd(LCD_CUR_OFFSET + 7, LCD_2CYCLE);

        //LCD_puts(buff);

        //HAL_Delay(300);

        if (xQueueReceive(Global_Queue_Handle, buff, 1000)) {
            LCD_putcmd(LCD_CUR_OFFSET + 7, LCD_2CYCLE);
            LCD_puts(buff);
        }

    }
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

    /* USER CODE BEGIN 5 */
    /*uint32_t init_freq = 5000000;
     uint32_t freq = calculate_frequency(init_freq);
     //uint32_t delta = 10;

     uint8_t bytes[5];

     bytes[0] = (uint8_t) (freq);
     bytes[1] = (uint8_t) (freq >> 8);
     bytes[2] = (uint8_t) (freq >> 16);
     bytes[3] = (uint8_t) (freq >> 24);
     bytes[4] = 0;

     osDelay(10);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

     HAL_SPI_Transmit(&hspi1, bytes, 5, 10);

     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
     */
//lcd test
//D7 D6 D5 D4 BL CS RW RS
// x  x  x  x  1  1  0  0 = XC (hex)
//uint8_t i2c_bytes[5];
//HAL_Delay(20);
    /*LCD_init();
     LCD_puts("Hi people WOW!!!\n");
     LCD_puts("HEllO WORLD!!!");*/

//i2c_bytes[0] = (0x30 & ~LCD_RS & ~LCD_RW ) | LCD_EN | LCD_BACKLIGHT;
//i2c_bytes[0] = 0x01<<4;
//HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
//HAL_Delay(10);
//HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
//HAL_Delay(10);
    /*i2c_bytes[0] |= 0x28 | ~LCD_RS | LCD_EN | ~LCD_RW;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);

     i2c_bytes[0] &= ~LCD_EN;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(1000);

     i2c_bytes[0] |= 0x30 | LCD_RS | LCD_EN | ~LCD_RW;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);

     i2c_bytes[0] &= ~LCD_EN;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(1000);*/

    /*i2c_bytes[0] = 0Xc6;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);
     i2c_bytes[0] = 0Xcc;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);
     i2c_bytes[0] = 0X06;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);
     i2c_bytes[0] = 0X80;
     HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
     HAL_Delay(100);*/

    /*uint16_t i = 0;

     uint8_t buff[10];*/

    /* Infinite loop */
    for (;;) {
        //LCD_putch('h');
        //LCD_putch('H');
        //HAL_I2C_Master_Transmit(&hi2c1, 0x27, i2c_bytes, 3, 100);
        //osDelay(1);
        //osDelay(1);
        //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //Toggle the state of LED2
        //HAL_Delay(500); //delay 13ms
        /*init_freq += delta;
         freq = calculate_frequency(init_freq);
         uint8_t bytes[5];

         bytes[0] = (uint8_t) (freq);
         bytes[1] = (uint8_t) (freq >> 8);
         bytes[2] = (uint8_t) (freq >> 16);
         bytes[3] = (uint8_t) (freq >> 24);
         bytes[4] = 0;

         osDelay(10);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

         HAL_SPI_Transmit(&hspi1, bytes, 5, 10);

         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);*/

        //TM_SPI_Send(SPI_TypeDef* SPIx, byte);
        //i2c_bytes[0] = 0x01 << 3;
        //HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
        //HAL_Delay(100);
        //i2c_bytes[0] = 0x00;
        //HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1, i2c_bytes, 1, 100);
        //HAL_Delay(100);
        /*i += 1000;
         itoa(i, buff);

         //LCD_putcmd(LCD_HOME, LCD_2CYCLE);
         LCD_putcmd(LCD_CUR_OFFSET + 7, LCD_2CYCLE);

         LCD_puts(buff);

         HAL_Delay(300);*/
        //LCD_puts("WWW 9999");
        //LCD_putcmd(LCD_HOME, LCD_2CYCLE);
        //HAL_Delay(100);
    }
    /* USER CODE END 5 */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


/*
 * main.h
 *
 *  Created on: Dec 26, 2015
 *      Author: os
 */

#ifndef APPLICATION_USER_MAIN_H_
#define APPLICATION_USER_MAIN_H_

extern uint32_t Global_Freq;

#endif /* APPLICATION_USER_MAIN_H_ */


/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "main.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern void xPortSysTickHandler(void);

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    osSystickHandler();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) {
    /* USER CODE BEGIN EXTI4_IRQn 0 */

    /* USER CODE END EXTI4_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

    /* USER CODE BEGIN EXTI4_IRQn 1 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
        Global_Freq += 50;
    }else {
        Global_Freq -= 50;
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //Toggle the state of LED2

    /* USER CODE END EXTI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/