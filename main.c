/*
===============================================================================
 Name        : main.c
 Author      : Yonghan Ching
 Version     :
 Description : main definition
 Code Size   : 24.24kb
===============================================================================


TODO: everything looks good so far... now implement UART output for the sensors
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.

  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up

  Transformation order: first yaw then pitch then roll.
*/

/*
  Commands that the firmware understands:

  "#o<param>" - Set output parameter. The available options are:
      "#o0" - Disable continuous streaming output.
      "#o1" - Enable continuous streaming output.
      "#ob" - Output angles in binary format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in text format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      "#os" - Output (calibrated) sensor data of all 9 axes in text format. One frame consist of
              three lines - one for each sensor.
      "#oc" - Go to calibration output mode.
      "#on" - When in calibration mode, go on to calibrate next sensor.
      "#oe0" - Disable error message output.
      "#oe1" - Enable error message output.

  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only.
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.

  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.
*/

//#include "stm32f4xx.h"
//#include "stm32f4xx_it.h"
#include "motor_control.h"
#include "stickIMU.h"
//#include "uart_int.h"
//#include "debug_output.h"


// Global variables
uint32_t current_time = 0;
IMUData_t imu_data;
eulerAngle_t body_Tilt;
uint32_t timestamp = 0;
uint32_t timestamp_Old = 0;
float G_Dt = 0.0f;
Bool processRoutine = FALSE;


//void TIM3_IRQHandler(void)
//{
//    // flash on update event
//    if (TIM3->SR & TIM_FLAG_Update)
//        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
//
//    // Clear interrupt flag
//    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//}
//
//void TimeOut_UserCallback(void);

void PeriphInit(void)
{
    // Setup SysTick interrupts at 10kHz (0.1ms)
    SysTick_Config(SystemCoreClock / 10000);

    #ifdef AHRS_TEST
    // Set up USART1 to use 115200 baud, 8N1 with Ring Buffer
    USART_RingInit(USART2, 115200, USART_Parity_No, USART_WordLength_8b, USART_StopBits_1);
    #endif

    // Initialize I2C with CPAL to be used with StickIMU
    StickIMU_Init(100000); // 100kHz or 400kHz

    // Initialize Timer3 to generate PWM signals to control motors
    motor_init();
}

void main(void)
{
    // Local variables
    uint8_t i[2];
    uint32_t iteration = 0;
    char header;
    char command;
    char output_param;
    char error_param;

    #ifdef DEBUG_ENABLE
    // Initialize flags
    reset_calibrationSessionFlag = TRUE;
    output_mode = OUTPUT__MODE_ANGLES_TEXT;
    output_streamOn = FALSE;
    #endif

    PeriphInit();
    #ifdef AHRS_TEST
    StickIMU_ReadAccelerometer(&imu_data);
    StickIMU_ReadMagnetometer(&imu_data);
    StickIMU_ReadGyroscope(&imu_data);
    USARTSendString(USART2, "Hello,\nI tried your code");
    /*while(1)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE))
        {
            USART_SendData(USART2, '0');
        }
    }
    USART_SendData(USART2, 'h');
    USART_SendData(USART2, 'a');*/
    #endif /* AHRS_TEST */
    motor_start();

    while (1);
}