#include "BMI088Middleware.h"

#include "main.h"
#include "spi.h"

#define BMI088_SPI_HANDLE hspi2

void BMI088_GPIO_init(void)
{
    /* GPIO and SPI2 are generated and initialized by CubeMX. */
}

void BMI088_com_init(void)
{
    /* SPI2 is initialized by MX_SPI2_Init() before the scheduler starts. */
}

void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    const uint32_t ticks_per_us = SystemCoreClock / 1000000U;
    const uint32_t reload = SysTick->LOAD + 1U;
    uint32_t previous = SysTick->VAL;
    uint32_t elapsed = 0U;

    while (elapsed < ((uint32_t)us * ticks_per_us))
    {
        uint32_t current = SysTick->VAL;
        elapsed += (previous >= current) ? (previous - current)
                                         : (previous + reload - current);
        previous = current;
    }
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0U;
    (void)HAL_SPI_TransmitReceive(&BMI088_SPI_HANDLE, &txdata, &rxdata, 1U, 1000U);
    return rxdata;
}
