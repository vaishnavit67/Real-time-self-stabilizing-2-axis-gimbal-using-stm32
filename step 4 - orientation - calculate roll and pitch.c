#include "stm32f4xx.h"
#include <stdint.h>
#include <math.h>

/* =========================================================
   MPU6050
   ========================================================= */

#define MPU6050_ADDR        0x68
#define MPU6050_WRITE_ADDR  (MPU6050_ADDR << 1)       // 0xD0
#define MPU6050_READ_ADDR   ((MPU6050_ADDR << 1) | 1) // 0xD1

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1          0x6B
#define SMPLRT_DIV          0x19
#define CONFIG_REG          0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B


/* =========================================================
   Global variables - WATCH THESE IN KEIL
   ========================================================= */

volatile uint8_t  who_am_i = 0;
volatile uint8_t  mpu_status = 0;

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;

volatile int32_t accel_x_offset = 0;
volatile int32_t accel_y_offset = 0;
volatile int32_t accel_z_offset = 0;

volatile int32_t gyro_x_offset = 0;
volatile int32_t gyro_y_offset = 0;
volatile int32_t gyro_z_offset = 0;

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

volatile float roll_accel_deg = 0.0f;
volatile float pitch_accel_deg = 0.0f;

volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;


/* =========================================================
   Simple delay
   System clock = 16 MHz
   ========================================================= */

void delay_ms(uint32_t ms)
{
    volatile uint32_t i;

    while(ms--)
    {
        for(i = 0; i < 8000; i++)
        {
            __NOP();
        }
    }
}


/* =========================================================
   I2C1 GPIO
   PB8 = SCL
   PB9 = SDA
   AF4 = I2C1
   ========================================================= */

void I2C1_GPIO_Init(void)
{
    /* Enable GPIOB clock */
    RCC->AHB1ENR |= (1U << 1);

    /* PB8 and PB9 -> Alternate Function */
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  ((2U << 16) | (2U << 18));

    /* Open drain */
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);

    /* High speed */
    GPIOB->OSPEEDR |= (3U << 16) | (3U << 18);

    /*
       Internal pull-up.
       The GY-521 normally also has I2C pull-up resistors.
    */
    GPIOB->PUPDR &= ~((3U << 16) | (3U << 18));
    GPIOB->PUPDR |=  ((1U << 16) | (1U << 18));

    /*
       PB8/PB9 are in AFR[1]
       AF4 = I2C1
    */
    GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->AFR[1] |=  ((4U << 0) | (4U << 4));
}


/* =========================================================
   I2C1 Initialization
   PCLK1 = 16 MHz
   Standard Mode = 100 kHz
   ========================================================= */

void I2C1_Init(void)
{
    /* Enable I2C1 clock */
    RCC->APB1ENR |= (1U << 21);

    I2C1_GPIO_Init();

    /* Reset I2C1 */
    I2C1->CR1 |= (1U << 15);
    I2C1->CR1 &= ~(1U << 15);

    /*
       PCLK1 = 16 MHz

       Standard mode:
       CCR = 16 MHz / (2 * 100 kHz)
           = 80
    */
    I2C1->CR2   = 16;
    I2C1->CCR   = 80;
    I2C1->TRISE = 17;

    /* Enable I2C */
    I2C1->CR1 |= (1U << 0);
}


/* =========================================================
   Wait for I2C flag with timeout
   ========================================================= */

uint8_t I2C_WaitFlag(uint32_t flag)
{
    uint32_t timeout = 1000000;

    while(!(I2C1->SR1 & flag))
    {
        if(--timeout == 0)
            return 0;
    }

    return 1;
}


/* =========================================================
   Generate STOP
   ========================================================= */

void I2C_Stop(void)
{
    I2C1->CR1 |= (1U << 9);
}


/* =========================================================
   Write one byte to MPU6050 register
   ========================================================= */

uint8_t MPU6050_WriteByte(uint8_t reg, uint8_t data)
{
    /* Wait until bus is free */
    uint32_t timeout = 1000000;

    while(I2C1->SR2 & (1U << 1))
    {
        if(--timeout == 0)
            return 0;
    }

    /* START */
    I2C1->CR1 |= (1U << 8);

    /* Wait SB */
    if(!I2C_WaitFlag(1U << 0))
    {
        I2C_Stop();
        return 0;
    }

    /* Send MPU6050 write address */
    I2C1->DR = MPU6050_WRITE_ADDR;

    /* Wait ADDR */
    if(!I2C_WaitFlag(1U << 1))
    {
        I2C_Stop();
        return 0;
    }

    /* Clear ADDR */
    (void)I2C1->SR2;

    /* Send register address */
    I2C1->DR = reg;

    /* Wait until register address is transferred */
    if(!I2C_WaitFlag(1U << 2))       // BTF
    {
        I2C_Stop();
        return 0;
    }

    /* Send data */
    I2C1->DR = data;

    /* Wait until data transfer is complete */
    if(!I2C_WaitFlag(1U << 2))       // BTF
    {
        I2C_Stop();
        return 0;
    }

    /* STOP */
    I2C_Stop();

    return 1;
}


/* =========================================================
   Read one byte from MPU6050 register
   ========================================================= */

uint8_t MPU6050_ReadByte(uint8_t reg, uint8_t *data)
{
    uint32_t timeout;

    /* -----------------------------------------
       Wait for bus free
       ----------------------------------------- */

    timeout = 1000000;

    while(I2C1->SR2 & (1U << 1))
    {
        if(--timeout == 0)
        {
            mpu_status = 61;
            return 0;
        }
    }


    /* -----------------------------------------
       START
       ----------------------------------------- */

    I2C1->CR1 |= (1U << 8);

    if(!I2C_WaitFlag(1U << 0))        // SB
    {
        I2C_Stop();
        mpu_status = 62;
        return 0;
    }


    /* -----------------------------------------
       Send WRITE address
       ----------------------------------------- */

    I2C1->DR = MPU6050_WRITE_ADDR;

    if(!I2C_WaitFlag(1U << 1))        // ADDR
    {
        I2C_Stop();
        mpu_status = 63;
        return 0;
    }

    /* Clear ADDR */
    (void)I2C1->SR2;


    /* -----------------------------------------
       Send register address
       ----------------------------------------- */

    I2C1->DR = reg;

    if(!I2C_WaitFlag(1U << 2))        // BTF
    {
        I2C_Stop();
        mpu_status = 64;
        return 0;
    }


    /* -----------------------------------------
       REPEATED START
       ----------------------------------------- */

    I2C1->CR1 |= (1U << 8);

    if(!I2C_WaitFlag(1U << 0))        // SB
    {
        I2C_Stop();
        mpu_status = 65;
        return 0;
    }


    /* -----------------------------------------
       Send READ address
       ----------------------------------------- */

    I2C1->DR = MPU6050_READ_ADDR;

    if(!I2C_WaitFlag(1U << 1))        // ADDR
    {
        I2C_Stop();
        mpu_status = 66;
        return 0;
    }


    /* -----------------------------------------
       Single-byte receive
       ----------------------------------------- */

    /* Disable ACK for single-byte reception */
		I2C1->CR1 &= ~(1U << 10);

		/* Clear ADDR */
		(void)I2C1->SR2;

		/* Generate STOP */
		I2C1->CR1 |= (1U << 9);


    /* -----------------------------------------
       Wait for received byte
       ----------------------------------------- */

    if(!I2C_WaitFlag(1U << 6))        // RXNE
    {
        mpu_status = 67;
        return 0;
    }

    *data = I2C1->DR;

    /* Re-enable ACK */
    I2C1->CR1 |= (1U << 10);

    return 1;
}


/* =========================================================
   Read multiple bytes
   Used for accelerometer + gyro data
   ========================================================= */

uint8_t MPU6050_ReadBytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    uint32_t timeout;

    if(length == 0)
        return 0;

    /* Bus free */
    timeout = 1000000;

    while(I2C1->SR2 & (1U << 1))
    {
        if(--timeout == 0)
            return 0;
    }

    /* START */
    I2C1->CR1 |= (1U << 8);

    if(!I2C_WaitFlag(1U << 0))
    {
        I2C_Stop();
        return 0;
    }

    /* WRITE address */
    I2C1->DR = MPU6050_WRITE_ADDR;

    if(!I2C_WaitFlag(1U << 1))
    {
        I2C_Stop();
        return 0;
    }

    (void)I2C1->SR2;

    /* Register */
    I2C1->DR = reg;

    if(!I2C_WaitFlag(1U << 2))
    {
        I2C_Stop();
        return 0;
    }

    /* Repeated START */
    I2C1->CR1 |= (1U << 8);

    if(!I2C_WaitFlag(1U << 0))
    {
        I2C_Stop();
        return 0;
    }

    /* READ address */
    I2C1->DR = MPU6050_READ_ADDR;

    if(!I2C_WaitFlag(1U << 1))
    {
        I2C_Stop();
        return 0;
    }

    /*
       Multiple-byte reception:
       ACK each byte except the final one.
    */
    (void)I2C1->SR2;

    for(i = 0; i < length; i++)
    {
        if(i == length - 1)
        {
            /* Last byte */
            I2C1->CR1 &= ~(1U << 10);   // ACK = 0
            I2C1->CR1 |=  (1U << 9);    // STOP
        }
        else
        {
            I2C1->CR1 |= (1U << 10);    // ACK = 1
        }

        if(!I2C_WaitFlag(1U << 6))       // RXNE
        {
            I2C1->CR1 |= (1U << 10);
            return 0;
        }

        buffer[i] = I2C1->DR;
    }

    /* Re-enable ACK */
    I2C1->CR1 |= (1U << 10);

    return 1;
}


/* =========================================================
   MPU6050 Initialization
   ========================================================= */

uint8_t MPU6050_Init(void)
{
    uint8_t id;

    /* Wake MPU6050 */
    if(!MPU6050_WriteByte(PWR_MGMT_1, 0x00))
    {
        mpu_status = 1;
        return 0;
    }

    delay_ms(100);


    /* Digital low-pass filter */
    if(!MPU6050_WriteByte(CONFIG_REG, 0x03))
    {
        mpu_status = 2;
        return 0;
    }


    /* Sample rate = 100 Hz */
    if(!MPU6050_WriteByte(SMPLRT_DIV, 9))
    {
        mpu_status = 3;
        return 0;
    }


    /* Gyroscope ±250 °/s */
    if(!MPU6050_WriteByte(GYRO_CONFIG, 0x00))
    {
        mpu_status = 4;
        return 0;
    }


    /* Accelerometer ±2g */
    if(!MPU6050_WriteByte(ACCEL_CONFIG, 0x00))
    {
        mpu_status = 5;
        return 0;
    }


    /* Read WHO_AM_I */
    if(!MPU6050_ReadByte(WHO_AM_I_REG, &id))
    {
        return 0;
    }

    who_am_i = id;


    /* Expected MPU6050 ID */
    if(id != 0x68)
    {
        mpu_status = 7;
        return 0;
    }


    /* SUCCESS */
    mpu_status = 8;

    return 1;
}


/* =========================================================
   Read accelerometer and gyro
   ========================================================= */

uint8_t MPU6050_ReadSensorData(void)
{
    uint8_t buffer[14];

    if(!MPU6050_ReadBytes(ACCEL_XOUT_H, buffer, 14))
        return 0;

    /* Accelerometer */
    accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    /* buffer[6], buffer[7] = temperature */

    /* Gyroscope */
    gyro_x = (int16_t)((buffer[8]  << 8) | buffer[9]);
    gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
		
		/* Convert raw values to physical units */

		/* Apply calibration offsets */

		accel_x_g = (accel_x - accel_x_offset) / 16384.0f;
		accel_y_g = (accel_y - accel_y_offset) / 16384.0f;
		accel_z_g = (accel_z - accel_z_offset) / 16384.0f;

		gyro_x_dps = (gyro_x - gyro_x_offset) / 131.0f;
		gyro_y_dps = (gyro_y - gyro_y_offset) / 131.0f;
		gyro_z_dps = (gyro_z - gyro_z_offset) / 131.0f;
		
		/* Calculate roll and pitch from accelerometer */

		roll_accel_deg =
				atan2f(accel_y_g, accel_z_g) * (180.0f / 3.14159265f);

		pitch_accel_deg =
				atan2f(-accel_x_g,
							 sqrtf(accel_y_g * accel_y_g +
										 accel_z_g * accel_z_g))
				* (180.0f / 3.14159265f);

    return 1;
}

void MPU6050_Calibrate(void)
{
    int32_t sum_ax = 0;
    int32_t sum_ay = 0;
    int32_t sum_az = 0;

    int32_t sum_gx = 0;
    int32_t sum_gy = 0;
    int32_t sum_gz = 0;

    uint16_t i;

    /* Take 500 stationary samples */
    for(i = 0; i < 500; i++)
    {
        MPU6050_ReadSensorData();

        sum_ax += accel_x;
        sum_ay += accel_y;
        sum_az += accel_z;

        sum_gx += gyro_x;
        sum_gy += gyro_y;
        sum_gz += gyro_z;

        delay_ms(5);
    }

    /* Calculate average */
    accel_x_offset = sum_ax / 500;
    accel_y_offset = sum_ay / 500;

    /* Z-axis should contain +1g when flat */
    accel_z_offset = (sum_az / 500) - 16384;

    gyro_x_offset = sum_gx / 500;
    gyro_y_offset = sum_gy / 500;
    gyro_z_offset = sum_gz / 500;
}


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    I2C1_Init();

    delay_ms(100);

    if(!MPU6050_Init())
    {
        /*
           Stop here if MPU6050 initialization fails.
           Watch mpu_status in Keil.
        */

        while(1)
        {
        }
    }
		
		/* Calibrate MPU6050 while it is stationary */
		MPU6050_Calibrate();


    while(1)
    {
        MPU6050_ReadSensorData();

        delay_ms(100);
    }
}
