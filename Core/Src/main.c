/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#define BL_DEBUG_MSG_EN
#define D_UART   &huart3
#define C_UART   &huart2
#define BL_RX_LEN 200
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void printmsg(char *format, ...);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t bl_rx_buffer[BL_RX_LEN];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CRC_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    /* Lets check whether button is pressed or not, if not pressed jump to user application */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
        printmsg("BL_DEBUG_MSG:Button is pressed .. going to BL mode\n\r");

        //we should continue in bootloader mode
        bootloader_uart_read_data();

    }
    else
    {
        printmsg(
                "BL_DEBUG_MSG:Button is not pressed .. executing user app\n\r");

        //jump to user application
        bootloader_jump_to_user_app();

    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
     */
    __HAL_RCC_PWR_CLK_ENABLE()
    ;

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time 
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOD_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOC_CLK_ENABLE()
    ;

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD,
    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bootloader_uart_read_data(void)
{
    uint8_t rcv_len = 0;

    while (1)
    {
        memset(bl_rx_buffer, 0, 200);
        // here we will read and decode the commands coming from host
        // first read only one byte from the host, which is the "length" field of the command
        HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
        rcv_len = bl_rx_buffer[0];
        HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

        switch (bl_rx_buffer[1])
        {
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer);
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(bl_rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);
                break;
            case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);
                break;
            default:
                printmsg(
                        "BL_DEBUG_MSG:Invalid command code received from host \n");
                break;
        }
    }
}

/*code to jump to user application
 *Here we are assuming FLASH_SECTOR2_BASE_ADDRESS
 *is where the user application is stored
 */
void bootloader_jump_to_user_app(void)
{
    //just a function pointer to hold the address of the reset handler of the user app.
    void (*app_reset_handler)(void);

    printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\n\r");

    // 1. configure the MSP by reading the value from the base address of the sector 2
    uint32_t msp_value = *(volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS;
    printmsg("BL_DEBUG_MSG:MSP value : %#x\n\r", msp_value);

    //This function comes from CMSIS.
    __set_MSP(msp_value);

    //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

    /* 2. Now fetch the reset handler address of the user application
     * from the location FLASH_SECTOR2_BASE_ADDRESS+4
     */
    uint32_t resethandler_address =
            *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;

    printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n\r",
            app_reset_handler);

    //3. jump to reset handler of the user application
    app_reset_handler();
}

/* prints formatted string to console over UART */
void printmsg(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
    char str[80];
    memset(str, 0x00, sizeof(str));

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format, args);
    HAL_UART_Transmit(D_UART, (uint8_t *) str, strlen(str), HAL_MAX_DELAY);
    va_end(args);
#endif
}

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) Verify the checksum
    printmsg("BL_DEBUG_MSG: bootloader_handle_getver_cmd\r\n");

    // Total length of the command packet
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 sent by the Host
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4,
            host_crc))
    {
        printmsg("BL_DEBUG_MSG: checksum success!!\r\n");
        bootloader_send_ack(bl_rx_buffer[0], 1);
        bl_version = get_bootloader_version();
        printmsg("BL_DEBUG_MSG: BL_VER --> %d %#x\n\r", bl_version, bl_version);
        bootloader_uart_write_data(&bl_version, 1);

    }
    else
    {
        printmsg("BL_DEBUG_MSG: checksum failed!!\r\n");
        bootloader_send_nack();
    }
}

void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_go_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{

}

void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{

}

void bootloader_handle_mem_read(uint8_t *pBuffer)
{

}

void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{

}

void bootloader_handle_read_otp(uint8_t *pBuffer)
{

}

void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{

}

// This function sends ACK if CRC matches along with "len to follow"
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
    // Here we sent 2 bytes...first byte ACK and the second len value
    uint8_t ack_buf[2];
    ack_buf[0] = BL_ACK;
    ack_buf[1] = follow_len;
    HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack(void)
{
    uint8_t nack = BL_NACK;
    HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue = 0xff;

    for (uint32_t i = 0; i < len; i++)
    {
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
    }

    /* Reset CRC Calculation Unit */
    __HAL_CRC_DR_RESET(&hcrc);

    if (uwCRCValue == crc_host)
    {
        return VERIFY_CRC_SUCCESS;
    }

    return VERIFY_CRC_FAIL;
}

uint8_t get_bootloader_version(void)
{
    return (uint8_t) BL_VERSION;
}

void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
    HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
