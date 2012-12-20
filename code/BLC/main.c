/*
 */

#include <avr/io.h>
#include <util/delay.h>

#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdint.h>


#define H1 PD1
#define L1 PD5

#define H2 PC3
#define L2 PD4

#define H3 PD7
#define L3 PD3

#define LED1 PB0
#define LED2 PB1

uint8_t aktPos;


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



    }

}


/**
    Einstellung des Vergleichspins für AC
*/
void setMUX(int mux)
{


    //AC Interrupt aus, damit kein falsches Interrupt ausgelöst wird
    ACSR &= ~(1<<ACIE);


    switch(mux)
    {

        /**
        PHASE2
        ADC1 */
    case 1:
        ADMUX &= ~(1<<MUX2);
        ADMUX &= ~(1<<MUX1);
        ADMUX |= (1<<MUX0);

        //AC Interrupt ein
        ACSR |= (1<<ACIE);
        break;



        /**
        PHASE3
        ADC0 */
    case 0:
        ADMUX &= ~(1<<MUX2);
        ADMUX &= ~(1<<MUX1);
        ADMUX &= ~(1<<MUX0);

        //AC Interrupt ein
        ACSR |= (1<<ACIE);

        break;


        /**
        PHASE1
        ADC2 */
    case 2:
        ADMUX &= ~(1<<MUX2);
        ADMUX |= (1<<MUX1);
        ADMUX &= ~(1<<MUX0);

        //AC Interrupt ein
        ACSR |= (1<<ACIE);

        break;


    }



}


/**
    Initialisierung Analog Comparator
*/
void init_ANALOG_COMPARATOR()
{

    //init_AC
    ACSR &= ~(1<<ACD);   //AnalogComparator ein
    ACSR |= (1<<ACIE);  //Interrupt enable ein



    /**Flanke einstellen */
    /**steigende Flanke*/
    // ACSR  |= (1<<ACIS1);
    // ACSR  |= (1<<ACIS0);
    /**fallende Flanke*/
    ACSR  |= (1<<ACIS1);
    ACSR  &= ~(1<<ACIS0);
    /** Toggle*/
    //  ACSR  &= ~(1<<ACIS1);
    //  ACSR  &= ~(1<<ACIS0);


    SFIOR |= (1<<ACME);     //Multiplexer für Analogkomparator ein
    ADCSRA &= ~(1<<ADEN);

}
/**
Analog Comparator
    Erkennung des Nulldurchgangs
*/
ISR(ANA_COMP_vect)
{

    if(aktPos>1)
    {
        aktPos=0;
    }
    else
    {
        aktPos++;

    }


    setMUX(aktPos);

}

void initPorts()
{
    //Motor
    DDRD |= ((1<<H1)|(1<<L1)|(1<<L2)|(1<<H3)|(1<<L3));
    DDRC |= ((1<<H2));

    //LEDs
    DDRB |=((1<<LED1)|(1<<LED2));



}
int main(void)
{


    initPorts();


 //   setLED(1);while(1);

    init_ANALOG_COMPARATOR();

    aktPos = 0;
    setMUX(0);
    setMotor(0);


setLED(1);
_delay_ms(200);
setLED(0);
_delay_ms(200);
setLED(1);
_delay_ms(200);
setLED(0);
_delay_ms(200);
setLED(2);
_delay_ms(200);
setLED(0);




    sei();


    while(1)
    {
         setMotor(aktPos);
         _delay_us(5);
         setMotor(3);
        _delay_us(20);




    }



    return 0;
}
