#include "lcd/lcd.h"
#include "gd32vprofile.h"
#include "gd32v_pjt_include.h"
#include "gd32vf103_libopt.h"
#include "gd32vf103.h"


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




// forward declarations
void rcu_config(rcu_periph_enum periph);
void gpio_config(uint32_t i2c_periph);
void i2c_config(uint32_t i2c_periph, uint32_t addr);
void softReset(uint32_t i2c_periph, uint32_t addr);
float readTemperature(void);
float readHumidity(void);
float readValue(uint8_t data, uint32_t i2c_periph, uint32_t addr);
void blinky(uint32_t sequencer);
void timer_config(void);
uint32_t lutHexString(uint32_t num, char *s);
void printHex(unsigned char line, uint32_t val);
uint8_t checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

// global variables

float tempTemperature, realTemperature;
uint32_t g_count = 0;  
uint32_t t_count = 0; 




int main(void)
{
    /* RCU config */
    rcu_config(RCU_I2C0);
    /* GPIO config */
    gpio_config(I2C0);
    /* I2C config */
    i2c_config(I2C0, I2C0_SLAVE_ADDRESS7);
    // RGB LED: LED_R->PORTC_13, LED_G->PORTA_1, LED_B->PORTA_2
    // Enable PORTA and PORTC peripherals.
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);

    // Configure bits 1 & 2 of PORTA, and bit 13 in PORTC
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

    // Enable reading of the instret and cycle hardware registers
    enable_mcycle_minstret();

    // Send a soft reset command to the HTU21D which is a software initiated
    // reboot of the device.
    softReset(I2C0, I2C0_SLAVE_ADDRESS7);

    while (TRUE)
    {
        float temperature, humidity;
        uint64_t instructions_at_start, overhead, instructions_per_loop = 0;
        instructions_at_start = instructionsExecuted();
    /*  Call a functions that calls instructionsExecuted twice in a row and
        returns the difference as a 64 bit count that is the number of instructions
        spent by the instructionsExecuted method iteself.  It should be a 
        small number */
        overhead = instructionsExecutedOverhead();
        LCD_ShowString8(90, (5 << 3), (u8 *)"overhead", RED);
        printHex(5, (uint32_t) overhead);
        uint32_t instret = read_csr(0xC02);
        uint32_t instreth = read_csr(0xC82);
        LCD_ShowString8(90, (6 << 3), (u8 *)"instreth", GREEN);
        printHex(6, instreth);
        LCD_ShowString8(90, (7 << 3), (u8 *)"instret", GREEN);
        printHex(7, instret);
        LCD_ShowString8(90, (9 << 3), (u8 *)"t_count", RED);
        printHex(9, t_count);

        blinky(instret);

        /*  Get temperature in degrees Celsius, convert to Fahrenheit and print
        it on the top line of the OLED display, and also the humidity.
        Pressing the BOOT button toggles between  Celsius and Fahrenheit */

        temperature = readTemperature();
        char *scale, *banner;
        u16 color;
        if (temperature == ERROR_BAD_CRC)
        {
            scale = " X";
            banner = "BAD CRC";
            color = RED;
        }
        else if (g_count % 2)
        {
            temperature = 1.8 * temperature + 32;
            scale = " F";
            banner = "FAHRENHEIT";
            color = GREEN;
        } 
        else
        {
            scale = " C";
            banner = "CELSIUS   ";
            color = YELLOW;
        }
        humidity = readHumidity();
        if (humidity == ERROR_BAD_CRC)
        {
            banner = "BAD CRC";
            color = RED;
        }
        LCD_ShowString8(80, (3 << 3), (u8 *) banner, color);
        printHex(3, g_count);
        LCD_ShowString8(40, (0 << 3), (u8 *)scale, color);
        LCD_ShowNum1(0, (0 << 3), temperature, 4, color);
       
        LCD_ShowNum1(90, (0 << 3), humidity, 4, YELLOW);
        LCD_ShowString8(130, (0 << 3), (u8 *)" RH", YELLOW);
    
        //  compute ratio of clock cycles to instructions executed
        uint32_t clocks_per_instruction = 
        instructionCycles() / instructionsExecuted();
        LCD_ShowString8(90, (8 << 3), (u8 *)"cycl/ins", WHITE);
        printHex(8, clocks_per_instruction);

        //  compute # of instructions in this loop thus far
        instructions_per_loop = instructionsExecuted() - instructions_at_start;
        LCD_ShowString8(90, (4 << 3), (u8 *)"ins/loop", GREEN);
        printHex(4, instructions_per_loop);

        // Sleep until timer fires:every about once per 3.5 seconds
        // This thing sleeps more 98 percent of the elapsed time
        pmu_to_sleepmode(WFI_CMD);
    }
}

/*!
    \brief      enable the peripheral clock
    \param[in]  rcu_periph_enum periph
    \param[out] none
    \retval     none
*/
void rcu_config(rcu_periph_enum periph)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C clock */
    rcu_periph_clock_enable(periph);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  rcu_periph_enum periph
    \param[out] none
    \retval     none
*/
void gpio_config(uint32_t i2c_periph)
{
    uint32_t pin;
    /* I2C0 and I2C1 GPIO ports */
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    /* connect PB10 to I2C0_SCL */
    /* connect PB11 to I2C0_SDA */
    pin = i2c_periph == I2C0 ? GPIO_PIN_6 | GPIO_PIN_7 : GPIO_PIN_10 | GPIO_PIN_11;
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, pin);
}

/*!
    \brief      cofigure the I2C0 interfaces
    \param[in]  uint32_t i2c_periph, uint32_t addr
    \param[out] none
    \retval     none
*/
void i2c_config(uint32_t i2c_periph, uint32_t addr)
{
    /* I2C clock configure */
    i2c_clock_config(i2c_periph, 100000, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, addr);
    /* enable I2C0 */
    i2c_enable(i2c_periph);
    /* nable acknowledge */
    i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
}

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

void blinky(uint32_t sequencer)
{
    switch( sequencer % 7 ) {
        case 0:
            LEDR_TOG;
            break;
        case 1:
            LEDG_TOG;
            break;
        case 2:
            LEDB_TOG;
            break;
        case 3:
            LEDR_TOG;
            LEDG_TOG;
            break;
        case 4:
            LEDG_TOG;
            LEDB_TOG;
            break;
        case 5:
            LEDB_TOG;
            LEDR_TOG;
            break;
        case 6:
            LEDB_TOG;
            LEDR_TOG;
            LEDG_TOG;
            break;                        
        }
}
/* Send a "soft reset" command to the HTU21D
  this is equivalent to a software initiated power up sequence
  See datasheet for more information:
  https://cdn.sparkfun.com/assets/6/a/8/e/f/525778d4757b7f50398b4567.pdf

  void softReset(uint32_t i2c_periph, uint32_t addr)
  periph = {I2C0|I2C1}   the I2C hardware device
  addr = the unshifted 7 bit address of the slave: for HTU21D this is 0x40

  */
void softReset(uint32_t i2c_periph, uint32_t addr)
{
    if (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        i2c_stop_on_bus(i2c_periph);
        /* wait until stop condition generate */
        while(I2C_CTL0(i2c_periph)&0x0200);
    } 
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY));    
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, (addr << 1), I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

    // Send the command to soft reset the HTU21D
    i2c_data_transmit(i2c_periph, SOFT_RESET );
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
    // HTU21D spec says allow 15 milliseconds for a soft reset
    delay_1ms(15);
    i2c_stop_on_bus(i2c_periph);
}

float readTemperature()
{
    // Return the temperature in degrees Celcius
    uint16_t rawTemperature = readValue(TRIGGER_TEMP_MEASURE_HOLD, I2C0, I2C0_SLAVE_ADDRESS7);
        //Given the raw temperature data, calculate the actual temperature
    if (rawTemperature == ERROR_BAD_CRC)
        return ERROR_BAD_CRC;
    float tempTemperature = rawTemperature * (175.72 / 65536.0); //2^16 = 65536
    float realTemperature = tempTemperature - 46.85; //From page 14
    return realTemperature;
}


float readHumidity()
{
    // return the relative humidity as a percentage
    uint16_t rawHumidity = readValue(TRIGGER_HUMD_MEASURE_HOLD, I2C0, I2C0_SLAVE_ADDRESS7);
    if (rawHumidity == ERROR_BAD_CRC)
        return ERROR_BAD_CRC;
    //Given the raw humidity data, calculate the actual relative humidity
    float tempHumidity = rawHumidity * (125.0 / 65536.0); //2^16 = 65536
    float realHumidity = tempHumidity - 6.0; //From page 14
    return realHumidity;
}


// Return a 16 bit value (temperature or humidity) from the HTU21D
float readValue(uint8_t data, uint32_t i2c_periph, uint32_t addr)
{    
    uint8_t reading[3];
    #define CRC_BYTE reading[2]
    uint16_t rawData;
    // LCD_ShowString8(90, (0 << 3), (u8 *)"one   ", RED);
    /* wait until I2C bus is idle */
    if (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        i2c_stop_on_bus(i2c_periph);
        /* wait until stop condition generate */
        while(I2C_CTL0(i2c_periph)&0x0200);
    }
 
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY));

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, (addr << 1), I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    // /* wait until the transmit data buffer is empty */
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));
  
    // Send the command to read data from peripheral
    i2c_data_transmit(i2c_periph, data);
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));

    /* GD32VF103 User manual, page 362:
        After sending the last byte I2C master sets the BTC bit,
        because both shift register and I2C_DATA are now empty/
        When the BTC bit is set we should send a stop on bus, which
        will clear both the BTC bit and the DTE bit.
        wait until the BTC bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY));

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus as receiver mode */
    i2c_master_addressing(i2c_periph, (addr << 1), I2C_RECEIVER);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND));
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    // /* wait until the transmit data buffer is empty */
    // while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE));

    /* Receive data */
    /*
        As soon as the first byte is received RBNE flag is set by
        hardware.  Software can now ready data from I2C_DATA and
        RBNE flag is cleared as well.   Any time RBNE is set, software
        can read a byte from I2C_DATA
    */
    i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE));
    reading[0] = i2c_data_receive(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE));
    reading[1] = i2c_data_receive(i2c_periph);
    // last byte read responds with a NACK instad of ACK
    i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE));
    reading[2] = i2c_data_receive(i2c_periph);

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c_periph);
    /* wait until stop condition generate */
    while(I2C_CTL0(i2c_periph)&0x0200);
    rawData = (((uint16_t) reading[0]) << 8) | + reading[1];
    if (checkCRC(rawData, CRC_BYTE))
        return ERROR_BAD_CRC;
    else
        return rawData;
}

/*
 * Return 8 bit CRC value 
 * */
uint8_t checkCRC8(uint16_t data)
{
#define HTU21D_CRC8_POLYNOMINAL      0x13100   //crc8 polynomial for 16bit value, CRC8 -> x^8 + x^5 + x^4 + 1    
  for (uint8_t bit = 0; bit < 16; bit++)
  {
    if   (data & 0x8000) data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    else data <<= 1;
  }
  return data >>= 8;
}

uint8_t checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
    if (checkCRC8(message_from_sensor) != check_value_from_sensor)
        return 1;  // bad CRC
    else
        return 0;  // good CRC
 }
        

/*
    Fast lookup table based uint32_t to hex string conversion.
    Modified from original code by Johnny Lee, thanks.
    modified by wnorcott to print XXXX.XXXX formatted hex and to 
    use uint32_t num width instead of num64_t, and to only us upper case
    hex digits.
    https://johnnylee-sde.github.io/Fast-unsigned-integer-to-hex-string/
*/
uint32_t lutHexString(uint32_t num, char *s)
{
	static const char digits[513] =
		"000102030405060708090A0B0C0D0E0F"
		"101112131415161718191A1B1C1D1E1F"
		"202122232425262728292A2B2C2D2E2F"
		"303132333435363738393A3B3C3D3E3F"
		"404142434445464748494A4B4C4D4E4F"
		"505152535455565758595A5B5C5D5E5F"
		"606162636465666768696A6B6C6D6E6F"
		"707172737475767778797A7B7C7D7E7F"
		"808182838485868788898A8B8C8D8E8F"
		"909192939495969798999A9B9C9D9E9F"
		"A0A1A2A3A4A5A6A7A8A9AAABACADAEAF"
		"B0B1B2B3B4B5B6B7B8B9BABBBCBDBEBF"
		"C0C1C2C3C4C5C6C7C8C9CACBCCCDCECF"
		"D0D1D2D3D4D5D6D7D8D9DADBDCDDDEDF"
		"E0E1E2E3E4E5E6E7E8E9EAEBECEDEEEF"
		"F0F1F2F3F4F5F6F7F8F9FAFBFCFDFEFF";
	
	uint32_t x = (uint32_t)num;
	int i = 3;
	char *lut = (char *)(digits);
    s[4] = ':';
	while (i >= 0)
	{
		int pos = (x & 0xFF) * 2;
		char ch = lut[pos];
        if (i<=1)
        {
		    s[i * 2] = ch;
		    ch = lut[pos + 1];
		    s[i * 2 + 1] = ch;
        }
        else
        {
         	s[i * 2 + 1] = ch;
		    ch = lut[pos + 1];
		    s[i * 2 + 2] = ch;   /* code */
        }
        

		x >>= 8;
		i -= 1;
	}

	return 0;
}


/*
 * Print the 32-bit hex value to LCD at the specified line number.
 * Format is XXXX.XXXX
 */
void printHex(unsigned char line, uint32_t val) 
{
    char buf[10];
    lutHexString(val, buf);
    buf[9] = 0;

    LCD_ShowString8(0, (line << 3), (u8 *)(buf), YELLOW);
}
