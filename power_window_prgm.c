

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SET_Bit(PORT,BIT) PORT|=(1<<BIT)
#define CLR_Bit(PORT,BIT) PORT&=~(1<<BIT)
#define SET_bit(PORT,BIT)  PORT|=(1<<BIT)
#define CLR_bit(PORT,BIT) PORT&=~(1<<BIT)

   volatile unsigned int FLAG_ISR1=0;
   volatile unsigned int FLAG_ISR2=0;


long readUltrasonicDistance(int pin)

{

  SET_bit(DDRD,pin);

  CLR_bit(PORTD,pin);

  _delay_ms(2);

  SET_bit(PORTD,pin);

  _delay_ms(5);

  CLR_bit(PORTD,pin);

  CLR_bit(DDRD,pin);

  return pulseIn(pin, HIGH);

}
 

int main()
{
  long  m, cm;
  SET_Bit(DDRB,PB1);
  SET_Bit(DDRB,PB2);
  CLR_Bit(PORTB,PB1);
  CLR_Bit(PORTB,PB2);
  CLR_Bit(DDRD,PD2);
  CLR_Bit(DDRD,PD3);
  CLR_Bit(DDRC,PC0);
  
  EICRA&=~(1<<ISC01);
  EICRA|=(1<<ISC00);
  EICRA&=~(1<<ISC11);
  EICRA|=(1<<ISC10);
    
  EIMSK|=(1<<INT0);
  EIMSK|=(1<<INT1);
  SREG|=(1<<7);

  
  cm = 0.01723 * readUltrasonicDistance(7);
  //m = cm/100;
  Serial.begin(9600);
 //Serial.println(cm);
   while(1)
  {  cm = 0.01723 * readUltrasonicDistance(7);  
   if(FLAG_ISR1==1&&FLAG_ISR2==0 && cm>50){
     FLAG_ISR1=0;
     FLAG_ISR2=0;
     SET_bit(PORTB,PB1);     //Motor rotates in Clockwise direction(Window moves upwards) 
      CLR_Bit(PORTB,PB2);
     //Serial.println("1");
     _delay_ms(5000);
     
       }
   else if((FLAG_ISR1==0&&FLAG_ISR2==1)){
     FLAG_ISR1=0;
     FLAG_ISR2=0;
     
     SET_bit(PORTB,PB2);     //Motor rotates in anti-clockwise direction(Window moves downwards)
     CLR_Bit(PORTB,PB1);
     //Serial.println("4");
     _delay_ms(5000);
       }
   
      else if(FLAG_ISR1==0||FLAG_ISR2==0)
     { if(cm<50){
       SET_bit(PORTB,PB2);    //Obstcle detected and motor reverses the direction and window moves downwards)
      CLR_Bit(PORTB,PB1);
     //Serial.println("5");
     _delay_ms(5000);
     }
    
       CLR_bit(PORTB,PB2);    //Motor stops running after the given delay
       CLR_Bit(PORTB,PB1);
      _delay_ms(5000);
       //Serial.println("3");
     }
   
     }
  }
      

ISR(INT0_vect)
{
  FLAG_ISR1=1;
   FLAG_ISR2=0;//to avoid more processing inside ISR.
}
ISR(INT1_vect)
{
  FLAG_ISR2=1;
   FLAG_ISR1=0;//this complement happens,only in the case of logical interrupts.
}

  
   
   
