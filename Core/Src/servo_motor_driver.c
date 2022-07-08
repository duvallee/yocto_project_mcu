/*
 * File: servo_motor_driver.c
 *
 * Written by duvallee in 2022
 *
 */
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

// ==========================================================================
#define DEFAULT_PCA9865_I2C_ADDR                         0x80

// ==========================================================================
#define PCA9685_I2C_TIMEOUT                              1

// ==========================================================================
#define PCA9685_SET_BIT_MASK(BYTE, MASK)                 ((BYTE) |= (uint8_t)(MASK))
#define PCA9685_CLEAR_BIT_MASK(BYTE, MASK)               ((BYTE) &= (uint8_t)(~(uint8_t)(MASK)))
#define PCA9685_READ_BIT_MASK(BYTE, MASK)                ((BYTE) & (uint8_t)(MASK))

// ==========================================================================
// MODE1
#define PCA9685_MODE1                                    0x00
#define PCA9685_MODE1_SLEEP                              0x10
#define PCA9685_MODE1_RESTART                            0x80

// MODE2
#define PCA9685_MODE2                                    0x01
#define PCA9685_SUBADDR1                                 0x02
#define PCA9685_SUBADDR2                                 0x03
#define PCA9685_SUBADDR3                                 0x04
#define PCA9685_ALLCALLADDR                              0x05

#define PCA9685_LEDX_ON_L                                0x06
#define PCA9685_LEDX_ON_H                                0x07
#define PCA9685_LEDX_OFF_L                               0x08
#define PCA9685_LEDX_OFF_H                               0x09

#define PCA9685_ALL_LED_ON_L                             0xFA
#define PCA9685_ALL_LED_ON_H                             0xFB
#define PCA9685_ALL_LED_OFF_L                            0xFC
#define PCA9685_ALL_LED_OFF_H                            0xFD
#define PCA9685_PRESCALE                                 0xFE

#define PCA9685_PRESCALE_MIN                             0x03                    // => max. frequency of 1526 Hz
#define PCA9685_PRESCALE_MAX                             0xFF                    // => min. frequency of 24 Hz

#define PCA9685_COUNTER_RANGE                            4096
#define PCA9685_DEFAULT_PERIOD                           5000000                 // Default period_ns = 1/200 Hz
#define PCA9685_OSC_CLOCK_MHZ                            25                      // Internal oscillator with 25 MHz

#define PCA9685_NUMREGS                                  0xFF
#define PCA9685_MAXCHAN                                  0x10

#define LED_FULL                                         (1 << 4)
#define MODE1_SLEEP                                      (1 << 4)
#define MODE2_INVRT                                      (1 << 4)
#define MODE2_OUTDRV                                     (1 << 2)

// start address for channel
#define PCA9685_CH_0_REG                                 0x06
#define PCA9685_CH_1_REG                                 0x0A
#define PCA9685_CH_2_REG                                 0x0E
#define PCA9685_CH_3_REG                                 0x12
#define PCA9685_CH_4_REG                                 0x16
#define PCA9685_CH_5_REG                                 0x1A
#define PCA9685_CH_6_REG                                 0x1E
#define PCA9685_CH_7_REG                                 0x22
#define PCA9685_CH_8_REG                                 0x26
#define PCA9685_CH_9_REG                                 0x2A
#define PCA9685_CH_10_REG                                0x2E
#define PCA9685_CH_11_REG                                0x32
#define PCA9685_CH_12_REG                                0x36
#define PCA9685_CH_13_REG                                0x3A
#define PCA9685_CH_14_REG                                0x3E
#define PCA9685_CH_15_REG                                0x42

// ==========================================================================
// definition of internal functions
static int pca9685_write_reg(uint8_t reg, uint8_t value);
static int pca9685_write_data(uint8_t reg, uint8_t* data, size_t length);
static int pca9685_read_reg(uint8_t reg, uint8_t* value);
static int pca9685_is_sleeping(uint8_t *sleeping);
static int pca9685_sleep();
static int pca9685_wakeup();
static int pca9685_set_pwm_frequency(float frequency);

// ==========================================================================
// APIs

/* --------------------------------------------------------------------------
 * Name : pca9685_motor_driver_init()
 *
 *
 * -------------------------------------------------------------------------- */
void pca9685_motor_driver_init()
{
   uint8_t pwm_data[4]                                   = {0, };

   // --------------------------------------------------------------------
   // MODE1 (0x00)
   // bit[7] : 0 : Restart disabled
   // bit[6] : 0 : Used internal clock (25MHz)
   // bit[5] : 0 : Register Auto-increment disabled
   // bit[4] : 0 : Normal mode
   // bit[3-1] : 0 : PCA9685 responds to I2C-bus subaddress 1, 2, 3
   // bit[0] : 0 : PCA9685 responds to LED All Call I2C-bus address
   if (pca9685_write_reg(PCA9685_MODE1, 0x30) < 0)                             // Auto-increment & sleep
   {
      printf("pca9685_write_reg failed !!!\r\n");
   }

   // --------------------------------------------------------------------
   // MODE2 (0x01)
   // bit[7-5] : 0 : reserved
   // bit[4] : 0 : Output logic state not inverted
   // bit[3] : 0 : Outputs change on STOP command
   // bit[2] : 0 : The 16 LEDn outputs are configured with an open-drain structure.
   // bit[1-] :
   //          00 : When nOE = 1 (output drivers not enabled), LEDn = 0
   //          01 : When nOE = 1 (output drivers not enabled)
   if (pca9685_write_reg(PCA9685_MODE2, 0x04) < 0)
   {
      printf("pca9685_write_reg failed !!!\r\n");
   }

   // --------------------------------------------------------------------
   // ALL off
   // PCA9685_ALL_LED_ON_L (0xFA)
   // 4 bytes OFF, OFF(high 4bits), ON, ON(high 4bits)

   pwm_data[0]                                           = 0;
   pwm_data[1]                                           = 0;
   pwm_data[2]                                           = 0;
   pwm_data[3]                                           = 0x10;                 // bit[4] : Write only (ALL_LED Full OFF)
   if (pca9685_write_data(PCA9685_ALL_LED_ON_L, pwm_data, 4) < 0)
   {
      printf("pca9685_write_data failed !!!\r\n");
   }

   // --------------------------------------------------------------------
   // 200 Hz 
   if (pca9685_set_pwm_frequency(200.0) < 0)
   {
      printf("pca9685_set_pwm_frequency failed !!!\r\n");
   }

   // --------------------------------------------------------------------
   if (pca9685_wakeup() < 0)
   {
      printf("pca9685_wakeup failed !!!\r\n");
   }

   // --------------------------------------------------------------------
//   set_motor_init_position();

}


/* --------------------------------------------------------------------------
 * Name : set_pca9685_motor_driver_pwm()
 *
 *
 * -------------------------------------------------------------------------- */
int set_pca9685_motor_driver_pwm(int channel, int pwm_us)
{
   uint8_t motor_data[4]                                 = {0, };                                  // ON(L), ON(H 4bits), OFF(L), OFF(H 4bits)
   uint8_t reg                                           = 0;

   int on_counter                                        = 0;
   int off_counter                                       = 0;

   // --------------------------------------------------------------------
   if (channel < 0 || channel > 15)
   {
      printf("wrong channel number in pca9685 !!!\r\n");
      return -1;
   }
   reg                                                   = (channel * 4) + PCA9685_CH_0_REG;

   // --------------------------------------------------------------------
   // 200Hz -> 1 clock = 5 ms, totoal counter 4096
   // value (us) of counter value
   //     = (4096 * value) / 5000 (ms -> us)
   on_counter                                            = ((int) roundf((4096.0 * pwm_us) / 5000.0)) + off_counter;

   // debug_printf("(pwm = %d), on_counter : %d(0x%x) us \r\n", value, on_counter, on_counter);

   // --------------------------------------------------------------------
   // 0...41...on_counter...4096
   // OFF...ON...........OFF....

   // off counter
   motor_data[0]                                         = (uint8_t) (off_counter & 0xFF);
   motor_data[1]                                         = (uint8_t) ((off_counter >> 8) & 0xF);
   // on counter
   motor_data[2]                                         = (uint8_t) (on_counter & 0xFF);
   motor_data[3]                                         = (uint8_t) ((on_counter >> 8) & 0xF);

   if (pca9685_write_data(reg, motor_data, 4) < 0)
   {
      printf("pca9685_write_data failed !!!\r\n");
      return -1;
   }

   return 0;
}


// ==========================================================================
// static functions

/* --------------------------------------------------------------------------
 * Name : pca9685_write_reg()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_write_reg(uint8_t reg, uint8_t value)
{
#if 0
   uint8_t data[]                                        = {address, value};
   if (HAL_I2C_Master_Transmit(&pLegMotor->i2c_handle, pLegMotor->i2c_addr, data, 2, PCA9685_I2C_TIMEOUT) != HAL_OK)
   {
      debug_output_error("HAL_I2C_Master_Transmit failed (addr = 0x%x, value = 0x%x) !!!\r\n", address, value);
      return -1;
   }



   if (BSP_I2C4_WriteReg(DEFAULT_PCA9865_I2C_ADDR, reg, &value, 1) != BSP_ERROR_NONE)
   {
      printf("BSP_I2C4_WriteReg failed (addr = 0x%x, value = 0x%x) !!!\r\n", reg, value);
      return -1;
   }
#endif
   return 0;
}

/* --------------------------------------------------------------------------
 * Name : pca9685_write_data()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_write_data(uint8_t reg, uint8_t* data, size_t length)
{
#if 0
   if (BSP_I2C4_WriteReg(DEFAULT_PCA9865_I2C_ADDR, reg, data, length) != BSP_ERROR_NONE)
   {
      printf("BSP_I2C4_WriteReg failed (reg = 0x%x) !!!\r\n", reg);
      return -1;
   }
#endif
   return 0;
}

/* --------------------------------------------------------------------------
 * Name : pca9685_read_reg()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_read_reg(uint8_t reg, uint8_t* value)
{
#if 0
   if (HAL_I2C_Master_Transmit(&pLegMotor->i2c_handle, pLegMotor->i2c_addr, &address, 1, PCA9685_I2C_TIMEOUT) != HAL_OK)
   {
      debug_output_error("HAL_I2C_Master_Transmit failed (Device = 0x%x, addr = 0x%x !!!\r\n", DEFAULT_PCA9865_I2C_ADDR, address);
      return -1;
   }

   if (HAL_I2C_Master_Receive(&pLegMotor->i2c_handle, pLegMotor->i2c_addr, value, 1, PCA9685_I2C_TIMEOUT) != HAL_OK)
   {
      debug_output_error("HAL_I2C_Master_Receive failed (addr = 0x%x) !!!\r\n", address);
      return -1;
   }

   if (BSP_I2C4_ReadReg(DEFAULT_PCA9865_I2C_ADDR, reg, value, 1) != BSP_ERROR_NONE)
   {
      printf("BSP_I2C4_ReadReg failed (addr = 0x%x) !!!\r\n", reg);
      return -1;
   }
#endif
   return 0;
}

/* --------------------------------------------------------------------------
 * Name : pca9685_is_sleeping()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_is_sleeping(uint8_t *sleeping)
{
   uint8_t mode1_reg                                     = 0;

   if (pca9685_read_reg(PCA9685_MODE1, &mode1_reg) < 0)
   {
      printf("pca9685_read_reg failed (addr = 0x%x) !!!\r\n", PCA9685_MODE1);
      return -1;
   }

   // Check if the sleeping bit is set.
   *sleeping                                             = (mode1_reg & PCA9685_MODE1_SLEEP);
   return 0;
}

/* --------------------------------------------------------------------------
 * Name : pca9685_sleep()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_sleep()
{
   uint8_t mode1_reg                                     = 0;
   if (pca9685_read_reg(PCA9685_MODE1, &mode1_reg) < 0)
   {
      printf("pca9685_read_reg failed (addr = 0x%x) !!!\r\n", PCA9685_MODE1);
      return -1;
   }

   if ((mode1_reg & PCA9685_MODE1_SLEEP) == 0)
   {
      mode1_reg                                          |= PCA9685_MODE1_SLEEP;
      if (pca9685_write_reg(PCA9685_MODE1, mode1_reg) < 0)
      {
         printf("pca9685_write_reg failed !!!\r\n");
         return -1;
      }
   }
   return 0;
}

/* --------------------------------------------------------------------------
 * Name : pca9685_wakeup()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_wakeup()
{
   uint8_t mode1_reg                                     = 0;
   uint8_t restart_required                              = 0;

   if (pca9685_read_reg(PCA9685_MODE1, &mode1_reg) < 0)
   {
      printf("pca9685_read_reg failed (addr = 0x%x) !!!\r\n", PCA9685_MODE1);
      return -1;
   }
   restart_required                                      = (mode1_reg & PCA9685_MODE1_RESTART);

   // --------------------------------------------------------------------
   PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_MODE1_SLEEP);
   PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_MODE1_RESTART);

   printf("mode1 reg : 0x%x \r\n", mode1_reg);

   if (pca9685_write_reg(PCA9685_MODE1, mode1_reg) < 0)
   {
      printf("pca9685_write_reg failed !!!\r\n");
      return -1;
   }

   // --------------------------------------------------------------------
   if (restart_required != 0)
   {
      // Oscillator requires at least 500us to stabilise, so wait 1ms.
      HAL_Delay(1);

      PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_MODE1_RESTART);
      if (pca9685_write_reg(PCA9685_MODE1, mode1_reg) < 0)
      {
         printf("pca9685_write_reg failed !!!\r\n");
         return -1;
      }
   }
   return 0;
}


/* --------------------------------------------------------------------------
 * Name : pca9685_set_pwm_frequency()
 *
 *
 * -------------------------------------------------------------------------- */
static int pca9685_set_pwm_frequency(float frequency)
{
   uint8_t prescaler                                     = 0;
   uint8_t sleeping                                      = 0;

   // --------------------------------------------------------------------
   if (frequency < 24.0 || frequency > 1526.0)
   {
      printf("wrong frequency : %f Hz \r\n", frequency);
      return -1;
   }

   // --------------------------------------------------------------------
   // #define PCA9685_PRESCALE_MIN                             0x03                    // => max. frequency of 1526 Hz
   // #define PCA9685_PRESCALE_MAX                             0xFF                    // => min. frequency of 24 Hz

   // Prescale value = round (osc_clock / (4096 * update_rate)) - 1
   // 200 Hz (Prescale Value) = round (25 MHz / (4096 * 200)) - 1 = 30 (0x1E)

   // Calculate the prescaler value (see datasheet page 25)
   prescaler = (uint8_t) roundf(25000000.0f / (4096 * frequency)) - 1;

   printf("prescaler : 0x%x \r\n", prescaler);

   // --------------------------------------------------------------------
   if (pca9685_is_sleeping(&sleeping) < 0)
   {
      printf("pca9685_is_sleeping failed !!!\r\n");
      return -1;
   }

   if (sleeping != 0)
   {
      if (pca9685_sleep() < 0)
      {
         printf("pca9685_sleep failed !!!\r\n");
         return -1;
      }
   }

   // --------------------------------------------------------------------
   // The prescaler can only be changed in sleep mode.
   if (pca9685_write_reg(PCA9685_PRESCALE, prescaler) < 0)
   {
      printf("pca9685_write_reg failed !!!\r\n");
      return -1;
   }

   if (pca9685_wakeup() < 0)
   {
      printf("pca9685_wakeup failed !!!\r\n");
      return -1;
   }

   return 0;
}





