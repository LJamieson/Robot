/* University of York Robotics Laboratory PsiSwarm Library: I2C Setup Source File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: i2c_setup.cpp
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 *
 */

#include "psiswarm.h"

char gpio_byte0;
char gpio_byte1;
char user_id_set = 0;
char wheel_enc_set = 0;
char switch_set = 0;

char emitter_byte = 0x00;

Timeout update_timeout;

char test;

char Setup::get_dc_status()
{
    IF_read_aux_ic_data();
    return status_dc_in;
}

void Setup::IF_set_IR_emitter_output(char emitter, char state)
{
    if(emitter <3) {
        if(state == 0) {
            char shift = 1 << emitter;
            emitter_byte &= (0xFF - shift);
        }
        if(state == 1) {
            char shift = 1 << emitter;
            emitter_byte |= shift;
        }
        char data[2];
        data [0] = 0x0A;  //Write to OLAT register
        data [1] = emitter_byte;  //GP0-3 are outputs on aux expansion IC
        //pc.printf("%c\n", emitter_byte);
        primary_i2c.write(AUX_IC_ADDRESS,data,2,false);
    }
}

void Setup::IF_set_base_LED(char state)
{
    if(state == 0) {
        emitter_byte &= 0xF7;
    } else emitter_byte |= 0x08;
    char data[2];
    data [0] = 0x0A;  //Write to OLAT register
    data [1] = emitter_byte;  //GP0-3 are outputs on aux expansion IC
    primary_i2c.write(AUX_IC_ADDRESS,data,2,false);

}

unsigned short Setup::IF_read_IR_adc_value(char adc, char index)
{
    char address = ADC1_ADDRESS;
    if(adc == 2) address=ADC2_ADDRESS;
    // Returns the raw sensor value for the IR sensor defined by index (range 0-7).
    short value = 0;
    // Read a single value from the ADC
    if(index<8) {
        char apb[1];
        char data[2];
        switch(index) {
            case 0:
                apb[0]=0x80;
                break;
            case 1:
                apb[0]=0x90;
                break;
            case 2:
                apb[0]=0xA0;
                break;
            case 3:
                apb[0]=0xB0;
                break;
            case 4:
                apb[0]=0xC0;
                break;
            case 5:
                apb[0]=0xD0;
                break;
            case 6:
                apb[0]=0xE0;
                break;
            case 7:
                apb[0]=0xF0;
                break;
        }
        primary_i2c.write(address,apb,1,false);
        primary_i2c.read(address,data,2,false);
        value=((data[0] % 16)<<8)+data[1];
        if(value > 4096) value=4096;
        value=4096-value;
    }
    return value;
}

char Setup::IF_setup_led_expansion_ic(void)
{
    //LED expansion IC is PCA9555
    //Address is 0100 001x (0x42) {defined by LED_IC_ADDRESS}
    //All 16 entries are outputs as they drive LEDs; the relevant registers are 2&3 (output port registers) and 6&7 (config. registers: a 0=output)
    //Message structure: {Address-RW}{Command}{Port 0}{Port 1}
    //Command bytes: 00000010 (0x02) = Write to output port
    //Command bytes: 00000110 (0x06) = Write to config registers
    //Note that for the LEDs, 0 = on, 1 = off
    //Port 0 = LED 1:4 Red:Green
    //Port 1 = LED 5:8 Red:Green
    char data [3];
    data [0] = 0x06;    //Write config registers
    data [1] = 0x00;    //All 8 pins in port 0 are outputs (0)
    data [2] = 0x00;    //All 8 pins in port 1 are outputs (0)
    primary_i2c.write(LED_IC_ADDRESS,data,3,false);

    //Turn all LEDs on
    data [0] = 0x02;    //Write to output port
    data [1] = 0x00;    //Enable LED1-4 (both colours)
    data [2] = 0x00;    //Enable LED5-8 (both colours)
    primary_i2c.write(LED_IC_ADDRESS,data,3,false);

    wait(0.05);
    //Turn all LEDs off
    data [0] = 0x02;    //Write to output port
    data [1] = 0xFF;    //Enable LED1-4 (both colours)
    data [2] = 0xFF;    //Enable LED5-8 (both colours)
    return primary_i2c.write(LED_IC_ADDRESS,data,3,false);
}

//Returns 0 if successful, 1 if test mode button pressed
void Setup::IF_setup_gpio_expansion_ic(void)
{
    //Main GPIO expansion IC is PCA9555
    //Address is 0100 000x (0x40) {defined by GPIO_IC_ADDRESS}
    //All 16 entries are inputs; the relevant registers are 0&1 (input port registers), 4&5 (polarity inv. registers) and 6&7 (config. registers: a 0=output)
    //Message structure: {Address-RW}{Command}{Port 0}{Port 1}
    //Command bytes: 00000010 (0x02) = Write to output port
    //Command bytes: 00000110 (0x06) = Write to config registers
    //Note that for the LEDs, 0 = on, 1 = off
    //Port 0 = PGDL; PGDR; PGDIR; UP; DOWN; LEFT; RIGHT; CENTER
    //Port 1 = ENC_LA; ENC_LB; ENC_RA; ENC_RB; ID0; ID1; ID2; ID3
    char data [3];
    char okay = 1;
    data [0] = 0x06;    //Write config registers
    data [1] = 0xFF;    //All 8 pins in port 0 are inputs (1)
    data [2] = 0xFF;    //All 8 pins in port 1 are inputs (1)
    if(primary_i2c.write(GPIO_IC_ADDRESS,data,3,false) != 0) {
        system_warnings += 2;
        okay = 0;
        psi.debug("- WARNING: No I2C acknowledge for main GPIO IC\n");
        if(HALT_ON_GPIO_ERROR){
            psi.debug("- PROGRAM HALTED.  Check that robot is switched on!\n");
            while(1){
               mbed_led1=1;
               mbed_led2=1;
               mbed_led3=0;
               mbed_led4=0;
               wait(0.25); 
               mbed_led1=0;
               mbed_led2=0;
               mbed_led3=1;
               mbed_led4=1;
               wait(0.25);   
            }   
        }
    }
    //Set all inputs to polarity-inverted (so a logic low = 1)
    data [0] = 0x04;    //Write to polarity inversion ports
    data [1] = 0xF8;    //Invert polarity of all switch input bits in input port 0 [but not power-good inputs]
    data [2] = 0xFF;    //Invert polarity of all bits in input port 1
    primary_i2c.write(GPIO_IC_ADDRESS,data,3,false);

    wait(0.01);

    //Read data
    char read_data[2];
    char command[1]; //Command to read from input port 0
    command[0]=0;
    primary_i2c.write(GPIO_IC_ADDRESS,command,1,false);
    primary_i2c.read(GPIO_IC_ADDRESS,read_data,2,false);
    gpio_byte0 = read_data[0];
    //char ret_val = (gpio_byte0 & 0xF8) >> 3;  //Returns a >0 value if a button is being pushed
    gpio_byte1 = read_data[1];
    if(okay && testing_voltage_regulators_flag)psi.debug("- Checking 3.3V voltage regulators\n");
    IF_parse_gpio_byte0(gpio_byte0);
    IF_parse_gpio_byte1(gpio_byte1);
    testing_voltage_regulators_flag = 0;
    //Setup interrupt handler for GPIO interrupts
    gpio_interrupt.mode(PullUp);
    gpio_interrupt.rise(this,&Setup::IF_handle_gpio_interrupt);
    //pc.printf("%c %c",gpio_byte0,gpio_byte1);

    //Secondary GPIO expansion IC is MCP23009
    //Address is 0100 111 (0x4E) {defined by AUX_IC_ADDRESS}
    //GP0,1,2,3 are outputs for driving infrared emitters and the base LED
    //IODIR register wants to be 0xF0 (1=input, 0=output)
    data [0] = 0x00;  //Write to IODIR register
    data [1] = 0xF0;  //Set GP0-3 as outputs
    primary_i2c.write(AUX_IC_ADDRESS,data,2,false);

    if(primary_i2c.write(AUX_IC_ADDRESS,data,2,false) != 0) {
        system_warnings += 4;
        psi.debug("- WARNING: No I2C acknowledge for aux GPIO IC\n");
    }
    data [0] = 0x06;  //Write to GPPU register
    data [1] = 0x3F;  //Set GP0-3 as active pull-up outputs and P4,P5 as pull-up inputs
    primary_i2c.write(AUX_IC_ADDRESS,data,2,false);

    //My interrupt is not so reliable: poll with a 50ms timeout in case interrupts aren't handled
    update_timeout.attach_us(this,&Setup::IF_update_gpio_inputs,50000);
    //return ret_val;
}

void Setup::IF_read_aux_ic_data()
{
    //Read the values of the input pins on the auxilliary GPIO expander
    char write_data [1];
    char read_data [1];
    write_data[0] = 0x09;
    primary_i2c.write(AUX_IC_ADDRESS,write_data,1,false);
    primary_i2c.read(AUX_IC_ADDRESS,read_data,1,false);
    char old_charging_state = status_dc_in;
    status_dc_in = 1-((read_data[0] & 0x10) >> 4);
    if(status_dc_in!=old_charging_state){
        if(status_dc_in == 0)psi.debug("No DC input\n");
        else psi.debug("DC input to charge pins\n");
    }
    //pc.printf("Aux IC Data:%X Charge:%d\n",read_data[0],charge_in);
}

void Setup::IF_parse_gpio_byte0(char byte)
{
    gpio_byte0 = byte;
    //GPIO byte zero contains the power line traces and the switch states
    char current_switch = ((gpio_byte0 & 0xF8) >> 3);
    if(switch_set == 1) {
        if(current_switch != switch_byte) {
            previous_switch_byte = switch_byte;
            switch_byte = current_switch;
            event++;
            switch_event = 1;
        }
    } else {
        switch_byte = current_switch;
        switch_set = 1;
    }
    if(((gpio_byte0 & 0x01)) != power_good_motor_left){
        power_good_motor_left = (gpio_byte0 & 0x01);
        if(!power_good_motor_left){
            if(testing_voltage_regulators_flag || SHOW_VR_WARNINGS)psi.debug("- WARNING: Voltage regulator left motor low\n");    
        }
        else if(testing_voltage_regulators_flag)psi.debug("- Power good left motor v.reg\n");
    } 
    if(((gpio_byte0 & 0x02) >> 1) != power_good_motor_right){
        power_good_motor_right = (gpio_byte0 & 0x02) >> 1;
        if(!power_good_motor_right){
            if(testing_voltage_regulators_flag || SHOW_VR_WARNINGS)psi.debug("- WARNING: Voltage regulator right motor low\n");
        }
        else if(testing_voltage_regulators_flag)psi.debug("- Power good right motor v.reg\n");
    } 
    if(((gpio_byte0 & 0x04) >> 2) != power_good_infrared){
        power_good_infrared = (gpio_byte0 & 0x04) >> 2;
        if(!power_good_infrared){
            if(testing_voltage_regulators_flag || SHOW_VR_WARNINGS)psi.debug("- WARNING: Voltage regulator infrared low\n");
        }
        else if(testing_voltage_regulators_flag)psi.debug("- Power good infrared and aux v.reg\n");
    } 
    if(USE_LED4_FOR_VR_WARNINGS){
         mbed_led4 = (!power_good_motor_left || !power_good_motor_right || !power_good_infrared);
    }
    //Halt the system if settings flag is set and all v-regs are bad [usually this means robot is switched off!]
    if(HALT_ON_ALL_VREGS_LOW && !power_good_motor_left && !power_good_motor_right && !power_good_infrared){
        psi.debug("- PROGRAM HALTED.  Check that robot is switched on!\n");
        while(1){
             mbed_led1=1;
             mbed_led2=0;
             mbed_led3=1;
             mbed_led4=0;
             wait(0.25); 
             mbed_led1=0;
             mbed_led2=1;
             mbed_led3=0;
             mbed_led4=1;
             wait(0.25); 
        }
    }
}

void Setup::IF_parse_gpio_byte1(char byte)
{
    gpio_byte1 = byte;
    //GPIO byte one contains the wheel encoders and the ID switch
    char current_id = ((gpio_byte1 & 0xF0)>> 4);
    if(user_id_set == 1) {
        if(robot_id != current_id) {
            previous_robot_id = robot_id;
            robot_id = current_id;
            event++;
            change_id_event = 1;
        }
    } else {
        robot_id = current_id;
        user_id_set = 1;
    }
    char current_encoder = (gpio_byte1 & 0x0F);
    if(wheel_enc_set == 1) {
        if(wheel_encoder_byte != current_encoder) {
            previous_wheel_encoder_byte = wheel_encoder_byte;
            wheel_encoder_byte = current_encoder;
            event++;
            encoder_event = 1;
        }
    } else {
        wheel_encoder_byte = current_encoder;
        wheel_enc_set = 1;
    }
}

void Setup::IF_handle_gpio_interrupt()
{
    test = 1-test;
    if(USE_LED3_FOR_INTERRUPTS) mbed_led3 = test;
    IF_update_gpio_inputs();
}

char Setup::IF_is_switch_pressed()
{
    //Read data
    char data[1];
    char command[1] = {0}; //Command to read from input port 0
    primary_i2c.write(GPIO_IC_ADDRESS,command,1,false);
    primary_i2c.read(GPIO_IC_ADDRESS,data,1,false);
    return (data[0] & 0x80);  //Returns a 1 if the center button is being pushed
}


char Setup::IF_get_switch_state()
{
    //Read data
    char data[1];
    char command[1] = {0}; //Command to read from input port 0
    primary_i2c.write(GPIO_IC_ADDRESS,command,1,false);
    primary_i2c.read(GPIO_IC_ADDRESS,data,1,false);
    return (data[0] & 0xF8) >> 3;  //Returns the current switch state
}

void Setup::IF_update_gpio_inputs()
{
    update_timeout.detach();
    //Read data
    char data[2];
    char command[1] = {0}; //Command to read from input port 0
    primary_i2c.write(GPIO_IC_ADDRESS,command,1,false);
    primary_i2c.read(GPIO_IC_ADDRESS,data,2,false);
    if(data[0]!=gpio_byte0) {
        IF_parse_gpio_byte0(data[0]);
    }
    if(data[1]!=gpio_byte1) {
        IF_parse_gpio_byte1(data[1]);
    }
    update_timeout.attach_us(this,&Setup::IF_update_gpio_inputs,50000);
}


void Setup::IF_write_to_led_ic(char byte_0, char byte_1)
{
    //Set LEDs
    char data[3];
    data [0] = 0x02;    //Write to output port
    data [1] = byte_0;
    data [2] = byte_1;
    primary_i2c.write(LED_IC_ADDRESS,data,3,false);
}


void Setup::IF_setup_temperature_sensor()
{
    char data[3];
    data[0] = 0x04; //Set critical temp limit
    data[1] = TEMPERATURE_CRITICAL_HI;
    data[2] = TEMPEARTURE_CRITICAL_LO;
    primary_i2c.write(TEMPERATURE_ADDRESS,data,3,false);
    data[0] = 0x02; //Set high temp limit
    data[1] = TEMPERATURE_HIGH_HI;
    data[2] = TEMPEARTURE_HIGH_LO;
    primary_i2c.write(TEMPERATURE_ADDRESS,data,3,false);
    data[0] = 0x03; //Set low temp limit
    data[1] = TEMPERATURE_LOW_HI;
    data[2] = TEMPEARTURE_LOW_LO;
    primary_i2c.write(TEMPERATURE_ADDRESS,data,3,false);
}

float Setup::IF_read_from_temperature_sensor()
{
    char command[1] = {0x05};  //Write to Ta Register
    char data[3];
    signed int temp;
    float temperature;
    primary_i2c.write(TEMPERATURE_ADDRESS,command,1,false);
    primary_i2c.read(TEMPERATURE_ADDRESS,data,2,false);

    //Convert the temperature data
    //First Check flag bits
    char UpperByte = data[0];
    char LowerByte = data[1];
    if ((UpperByte & 0x80) == 0x80) {
        psi.debug("- WARNING: Temperature sensor reports critical temperature\n");
    }
    if ((UpperByte & 0x40) == 0x40) {
        psi.debug("- WARNING: Temperature sensor reports above upper limit\n");
    }
    if ((UpperByte & 0x20) == 0x20) {
        psi.debug("- WARNING: Temperature sensor reports below lower limit\n");
    }
    UpperByte = UpperByte & 0x1F;       //Clear flag bits
    if ((UpperByte & 0x10) == 0x10) {
        UpperByte = UpperByte & 0x0F;   //Clear SIGN
        temp = (UpperByte * 256) + LowerByte;
        temperature = - (temp / 16.0f);
    } else   {
        temp = (UpperByte * 256) + LowerByte;
        temperature = (temp / 16.0f);
    }
    return temperature;
}