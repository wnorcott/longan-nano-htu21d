#include "lcd/lcd.h"
#include "gd32v_pjt_include.h"
#include "gd32vf103_libopt.h"
#include "gd32vf103.h"
#include "gd32vprofile.h"

extern uint32_t enable_mcycle_minstret();

#define ERROR_I2C_TIMEOUT 	998
#define ERROR_BAD_CRC		999

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

#define USER_REGISTER_RESOLUTION_MASK 0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14 0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12 0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13 0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11 0x81

#define USER_REGISTER_END_OF_BATTERY 0x40
#define USER_REGISTER_HEATER_ENABLED 0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD 0x02

#define HTU21D_ADDRESS          0x40  //Unshifted 7-bit I2C address for the sensor
#define I2C0_OWN_ADDRESS7       0x72

#define I2C0_SLAVE_ADDRESS7    HTU21D_ADDRESS

#define MAX_WAIT 100
#define DELAY_INTERVAL 10
#define MAX_COUNTER (MAX_WAIT/DELAY_INTERVAL)

#define I2C_DATA_WRITE 0
#define I2C_DATA_READ 1

// asm volatile("csrr %0, 0xb00 : =r(__tmp));
#define read_csr(csr_reg) ({ \
    unsigned long __tmp; \
    asm volatile ("csrr %0, " #csr_reg : "=r"(__tmp)); \
    __tmp; \
})
// typedefs

typedef uint8_t byte;

// forward declarations
void rcu_config(void);
void gpio_config(void);
void i2c_config(void);
float readTemperature(void);
float readHumidity(void);
float readValue(uint8_t data);

// global variables
uint8_t reading[3];
uint16_t rawData;
float tempTemperature, realTemperature;

/*
   ----------------------------------------------------------------------
   From: GD32VF103_Firmware_Library/Examples/TIMER/TIMER1_timebase/main.c
   ----------------------------------------------------------------------
*/
void timer_config(void)
{
    /* ----------------------------------------------------------------------------
       TIMER1 Configuration: 
       TIMER1CLK = SystemCoreClock/5400 = 20KHz.
       TIMER1 configuration is timing mode, and the timing is 0.2s(4000/20000 = 0.2s).
       CH0 update rate = TIMER1 counter clock/CH0CV = 20000/4000 = 5Hz.
       ---------------------------------------------------------------------------- */
    timer_oc_parameter_struct   timer_ocinitpara;
    timer_parameter_struct      timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 0x5399;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0x4000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);

    /* CH0,CH1 and CH2 configuration in OC timing mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);

    /* CH0 configuration in OC timing mode */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 2000);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    timer_enable(TIMER1);
}

char hexval(unsigned char val)
{
    if(val < 10)
        return '0' + val;
    return 'A' + (val - 10);
}

/*
 * Print the 32-bit hex value to LCD at the specified line number.
 * The value is shown in big-endian form.
 */
void printHex(unsigned char line, unsigned long val) {

    char buf[10];

    for(int i=0; i < 4; i++) {
        buf[i]   = hexval((val >> (28 - (i << 2))) & 0xf);
        buf[i+5] = hexval((val >> (12 - (i << 2))) & 0xf);
    }
    buf[4] = ':';
    buf[9] = 0;

    LCD_ShowString8(0, (line << 3), (u8 *)(buf), YELLOW);
}

unsigned long g_count = 0; // Displays starting at 2 - why?
unsigned long t_count = 0; 

// asm volatile("csrr %0, 0xb00 : =r(__tmp));
#define read_csr(csr_reg) ({ \
    unsigned long __tmp; \
    asm volatile ("csrr %0, " #csr_reg : "=r"(__tmp)); \
    __tmp; \
})

int main(void)
{
    /* RCU config */
    rcu_config();
    /* GPIO config */
    gpio_config();
    /* I2C config */
    i2c_config();
    // RGB LED: LED_R->PORTC_13, LED_G->PORTA_1, LED_B->PORTA_2
    // Enable PORTA and PORTC peripherals.
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    // Configure bits 1 & 2 of PORTA, and bit 13 in PORTC
    // Note, the SDK defining GPIO 'PIN' is a misnomer.
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1|GPIO_PIN_2);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    timer_config();

    eclic_global_interrupt_enable();
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
    eclic_irq_enable(TIMER1_IRQn,1,0);
    eclic_irq_enable(EXTI5_9_IRQn,1,0);

    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_8);
    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_8);

    Lcd_Init();         // init OLED
    BACK_COLOR=BLUE;
    LCD_Clear(BACK_COLOR);

    LEDR(1);
    LEDG(1);
    LEDB(1);

    g_count = 0;
    int i=0;

    enable_mcycle_minstret();

    uint16_t porta, pa8;

    int loop = 1;
    while (loop)
    {
        float temperature, humidity;
        uint64_t instructions_at_start, instr2, overhead, instructions_per_tick = 0;
        instructions_at_start = instructionsExecuted();
    /*  Call a functions that calls instructionsExecuted twice in a row and
        returns the difference as a 64 bit count that is the number of instructions
        spent by the instructionsExecuted method iteself.  It should be a 
        small number */
        overhead = instructionsExecutedOverhead();
        LCD_ShowString8(90, (5 << 3), (u8 *)"overhead", RED);
        printHex(5, (uint32_t) overhead);
        pa8 = gpio_input_bit_get(GPIOA, GPIO_PIN_8);
        porta = gpio_input_port_get(GPIOA);

        BACK_COLOR = pa8 ? BRED : BLUE;

        // LCD_ShowString8(90, (5 << 3), (u8 *)"g_count", RED);
        // printHex(5, g_count);
        unsigned long instret = read_csr(0xC02);
        unsigned long instreth = read_csr(0xC82);
        LCD_ShowString8(90, (6 << 3), (u8 *)"instreth", GREEN);
        printHex(6, instreth);
        LCD_ShowString8(90, (7 << 3), (u8 *)"instret", GREEN);
        printHex(7, instret);
        LCD_ShowString8(90, (9 << 3), (u8 *)"t_count", RED);
        printHex(9, t_count);

        i++;
        if(i % 2 == 0)
            continue;

        switch( (i / 2) % 3 ) {
            case 0:
                LEDR_TOG;
                break;
            case 1:
                LEDG_TOG;
                break;
            case 2:
                LEDB_TOG;
                break;
        }
        // get temperature in degrees Celsius, convert to Fahrenheit and print
        temperature = readTemperature();
        temperature = 1.8 * temperature + 32;
        LCD_ShowNum1(0, (1 << 3), temperature, 4, YELLOW);
        LCD_ShowString8(40, (1 << 3), (u8 *)" F", YELLOW);
        humidity = readHumidity();
        LCD_ShowNum1(90, (1 << 3), humidity, 4, YELLOW);
        LCD_ShowString8(130, (1 << 3), (u8 *)" RH", YELLOW);
        //  compute # of instructions in this loop thus far
        instructions_per_tick = instructionsExecuted() - instructions_at_start;
        LCD_ShowString8(90, (4 << 3), (u8 *)"ins/loop", GREEN);
        printHex(4, instructions_per_tick);    
        //  compute ratio of clock cycles to instructions executed
        uint32_t clocks_per_instruction = 
            instructionCycles() / instructionsExecuted();
        LCD_ShowString8(90, (8 << 3), (u8 *)"cycl/ins", WHITE);
        printHex(8, clocks_per_instruction);
        pmu_to_sleepmode(WFI_CMD);
    }
}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* I2C0 and I2C1 GPIO ports */
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}


void softReset()
{
    if (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
    {
        i2c_stop_on_bus(I2C0);
        /* wait until stop condition generate */
        while(I2C_CTL0(I2C0)&0x0200);
    } 
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    while (!i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));    
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, (I2C0_SLAVE_ADDRESS7 << 1), I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

    // Send the command to soft reset the HTU21D
    i2c_data_transmit(I2C0, SOFT_RESET );
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
    // HTU21D spec says allow 15 milliseconds for a soft reset
    delay_1ms(15);
    i2c_stop_on_bus(I2C0);
}

float readTemperature()
{
    // Return the temperature in degrees Celcius
    uint16_t rawTemperature = readValue(TRIGGER_TEMP_MEASURE_HOLD);
        //Given the raw temperature data, calculate the actual temperature
    float tempTemperature = rawTemperature * (175.72 / 65536.0); //2^16 = 65536
    float realTemperature = tempTemperature - 46.85; //From page 14
    return realTemperature;
}


float readHumidity()
{
    // return the relative humidity as a percentage
    uint16_t rawHumidity = readValue(TRIGGER_HUMD_MEASURE_HOLD);
        //Given the raw Humidity data, calculate the actual Humidity

  //Given the raw humidity data, calculate the actual relative humidity
  float tempHumidity = rawHumidity * (125.0 / 65536.0); //2^16 = 65536
  float realHumidity = tempHumidity - 6.0; //From page 14
  return realHumidity;
}


// Return a 16 bit value (temperature or humidity) from the HTU21D
float readValue(uint8_t data)
{    

    /* RCU config */
    rcu_config();
    /* GPIO config */
    gpio_config();
    /* I2C config */
    i2c_config();
    // LCD_ShowString8(90, (0 << 3), (u8 *)"one   ", RED);
    /* wait until I2C bus is idle */
    if (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
    {
        i2c_stop_on_bus(I2C0);
        /* wait until stop condition generate */
        while(I2C_CTL0(I2C0)&0x0200);
    }
 
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    while (!i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, (I2C0_SLAVE_ADDRESS7 << 1), I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    // /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(I2C0, I2C_FLAG_TBE));
  
    // Send the command to read data from peripheral
    i2c_data_transmit(I2C0, data);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

    /* GD32VF103 User manual, page 362:
        After sending the last byte I2C master sets the BTC bit,
        because both shift register and I2C_DATA are now empty/
        When the BTC bit is set we should send a stop on bus, which
        will clear both the BTC bit and the DTE bit.
        wait until the BTC bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    while (!i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus as receiver mode */
    i2c_master_addressing(I2C0, (I2C0_SLAVE_ADDRESS7 << 1), I2C_RECEIVER);
    while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    // /* wait until the transmit data buffer is empty */
    // while (!i2c_flag_get(I2C0, I2C_FLAG_TBE));

    /* Receive data */
    /*
        As soon as the first byte is received RBNE flag is set by
        hardware.  Software can now ready data from I2C_DATA and
        RBNE flag is cleared as well.   Any time RBNE is set, software
        can read a byte from I2C_DATA
    */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    int idx = 0;
    while(idx < 3) {
        // wait for "a byte has been read" flag RBNE
        while (!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
        reading[idx++] = i2c_data_receive(I2C0);
        if(idx == 2) {
            // next byte will be reading[3], so NACK when it arrives
            i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            reading[idx++] = i2c_data_receive(I2C0);
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0)&0x0200);
    rawData = (((uint16_t) reading[0]) << 8) | + reading[1];
    return rawData;
}


//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes
byte checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, checkvalue is 0x79
  //message = 0x683A, checkvalue is 0x7C
  //message = 0x4E85, checkvalue is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    //Serial.print("remainder: ");
    //Serial.println(remainder, BIN);
    //Serial.print("divsor:    ");
    //Serial.println(divsor, BIN);
    //Serial.println();

    if ( remainder & (uint32_t)1 << (23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return (byte)remainder;
}
