/*
 */

#include <avr/io.h>
#include <util/delay.h>

#define H1 PD1
#define L1 PD5

#define H2 PC3
#define L2 PD4

#define H3 PD7
#define L3 PD3

#define LED1 PB0
#define LED2 PB1



void setLED(uint8_t i);
void error();
void setMotor(uint8_t i);
void initPorts();

void error()
{
    PORTD &= ~((1<<H1)|(1<<L1)|(1<<L2)|(1<<H3)|(1<<L3));
    PORTC &= ~(1<<H2);

    DDRD &= ~((1<<H1)|(1<<L1)|(1<<L2)|(1<<H3)|(1<<L3));
    DDRC &= ~((1<<H2));

    while (1)
    {
        uint8_t i;
        for(i=0; i<4; i++)
        {
            setLED(i);
            _delay_ms(10);
        }
    }
}

void setLED(uint8_t i)
{

    switch(i)
    {
    case 0:
        PORTB &= ~((1<<LED1)|(1<<LED2));
        break;

    case 1:
        PORTB &= ~(1<<LED1);
        PORTB |= (1<<LED2);
        break;

    case 2:
        PORTB &= ~(1<<LED2);
        PORTB |= (1<<LED1);
        break;

    case 3:
        PORTB |= ((1<<LED2)|(1<<LED1));
        break;

    default:
        error();
    }


}

void setMotor(uint8_t i)
{

    PORTD &= ~((1<<H1)|(1<<L1)|(1<<L2)|(1<<H3)|(1<<L3));
    PORTC &= ~(1<<H2);

    switch(i)
    {
    case 0:
        PORTD |=((1<<H1)|(1<<L2));


        break;

    case 1:
        PORTD |=((1<<L1)|(1<<H3));

        break;

    case 2:
        PORTC |=(1<<H2);
        PORTD |=(1<<L3);

        break;

    default:
        error();
    }

}
void initPorts()
{
    //Motor
    DDRD |= ((1<<H1)|(1<<L1)|(1<<L2)|(1<<H3)|(1<<L3));
    DDRC |= ((1<<H2));

    //LEDs
    DDRB |=((1<<LED1)|(1<<LED2));



}
void test()
{

    setLED(0);
    _delay_ms(300);
    setLED(1);
    _delay_ms(300);
    setLED(2);
    _delay_ms(300);
    setLED(3);
    _delay_ms(800);


     setLED(0);
     setMotor(0);
    _delay_ms(300);
    setLED(1);
    setMotor(1);
    _delay_ms(300);
    setLED(2);
    setMotor(2);
    _delay_ms(300);
    setLED(3);
    _delay_ms(300);


}
int main(void)
{
    initPorts();

    while(1)
    {
        test();
    }



    return 0;
}
