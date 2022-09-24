
#include <mega32a.h>
#include <delay.h>

typedef enum{
SS_STOP=0,
SS_START
} system_state_e;

system_state_e SS=SS_STOP;
unsigned char speed_div=1;
unsigned char f_shift=1;

unsigned char f1=0,f2=0,f3=0;

// Sine Wave D=127 Center=127 Degree=360
const unsigned char sinewave[256]={
0x7F,0x82,0x85,0x88,0x8B,0x8F,0x92,0x95,0x98,0x9B,0x9E,0xA1,0xA4,0xA7,0xAA,0xAD,
0xB0,0xB2,0xB5,0xB8,0xBB,0xBE,0xC0,0xC3,0xC6,0xC8,0xCB,0xCD,0xD0,0xD2,0xD4,0xD7,
0xD9,0xDB,0xDD,0xDF,0xE1,0xE3,0xE5,0xE7,0xE9,0xEA,0xEC,0xEE,0xEF,0xF0,0xF2,0xF3,
0xF4,0xF5,0xF7,0xF8,0xF9,0xF9,0xFA,0xFB,0xFC,0xFC,0xFD,0xFD,0xFD,0xFE,0xFE,0xFE,
0xFE,0xFE,0xFE,0xFE,0xFD,0xFD,0xFD,0xFC,0xFC,0xFB,0xFA,0xF9,0xF9,0xF8,0xF7,0xF5,
0xF4,0xF3,0xF2,0xF0,0xEF,0xEE,0xEC,0xEA,0xE9,0xE7,0xE5,0xE3,0xE1,0xDF,0xDD,0xDB,
0xD9,0xD7,0xD4,0xD2,0xD0,0xCD,0xCB,0xC8,0xC6,0xC3,0xC0,0xBE,0xBB,0xB8,0xB5,0xB2,
0xB0,0xAD,0xAA,0xA7,0xA4,0xA1,0x9E,0x9B,0x98,0x95,0x92,0x8F,0x8B,0x88,0x85,0x82,
0x7F,0x7C,0x79,0x76,0x73,0x6F,0x6C,0x69,0x66,0x63,0x60,0x5D,0x5A,0x57,0x54,0x51,
0x4E,0x4C,0x49,0x46,0x43,0x40,0x3E,0x3B,0x38,0x36,0x33,0x31,0x2E,0x2C,0x2A,0x27,
0x25,0x23,0x21,0x1F,0x1D,0x1B,0x19,0x17,0x15,0x14,0x12,0x10,0x0F,0x0E,0x0C,0x0B,
0x0A,0x09,0x07,0x06,0x05,0x05,0x04,0x03,0x02,0x02,0x01,0x01,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x02,0x02,0x03,0x04,0x05,0x05,0x06,0x07,0x09,
0x0A,0x0B,0x0C,0x0E,0x0F,0x10,0x12,0x14,0x15,0x17,0x19,0x1B,0x1D,0x1F,0x21,0x23,
0x25,0x27,0x2A,0x2C,0x2E,0x31,0x33,0x36,0x38,0x3B,0x3E,0x40,0x43,0x46,0x49,0x4C,
0x4E,0x51,0x54,0x57,0x5A,0x5D,0x60,0x63,0x66,0x69,0x6C,0x6F,0x73,0x76,0x79,0x7C
};

// Functions prototype
void timer_pwm_stop();

// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{
  PORTC.0=1;
  timer_pwm_stop();
}

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
  f3=f1+170;
  OCR0=sinewave[f3]/speed_div;
}

// Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
  f1+=f_shift;
  f2=f1+85;
  OCR1AL=sinewave[f1]/speed_div;
  OCR1BL=sinewave[f2]/speed_div;
}

void timer_pwm_stop()
{
  TIMSK=0;

  TCCR1A=0;
  TCCR1B=0;
  TCCR0=0;

  OCR1AL=0;
  OCR1BL=0;
  OCR0=0;

  SS=SS_STOP;
}

void timer_pwm_start()
{
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 1000/000 kHz
  // Mode: Fast PWM top=0xFF
  // OC0 output: Non-Inverted PWM
  // Timer Period: 0/256 ms
  // Output Pulse(s):
  // OC0 Period: 0/256 ms Width: 0 us
  TCCR0=(1<<WGM00) | (1<<COM01) | (0<<COM00) | (1<<WGM01) | (0<<CS02) | (1<<CS01) | (0<<CS00);
  TCNT0=0x00;
  OCR0=0x00;

  // Timer/Counter 1 initialization
  // Clock source: System Clock
  // Clock value: 1000/000 kHz
  // Mode: Fast PWM top=0x00FF
  // OC1A output: Non-Inverted PWM
  // OC1B output: Non-Inverted PWM
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 0/256 ms
  // Output Pulse(s):
  // OC1A Period: 0/256 ms Width: 0 us
  // OC1B Period: 0/256 ms Width: 0 us
  // Timer1 Overflow Interrupt: On
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: Off
  // Compare B Match Interrupt: Off
  TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x00;
  ICR1L=0x00;
  OCR1AH=0x00;
  OCR1AL=0x00;
  OCR1BH=0x00;
  OCR1BL=0x00;

  // Timer(s)/Counter(s) Interrupt(s) initialization
  TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);

  SS=SS_START;
}

void main(void)
{
  // Declare your local variables here

  // Input/Output Ports initialization
  // Port A initialization
  // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
  DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
  // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
  PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

  // Port B initialization
  // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In 
  DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
  // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T 
  PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

  // Port C initialization
  // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
  DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (1<<DDC0);
  // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
  PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

  // Port D initialization
  // Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
  DDRD=(0<<DDD7) | (0<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
  // State: Bit7=T Bit6=T Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T 
  PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);


  // External Interrupt(s) initialization
  // INT0: On
  // INT0 Mode: Falling Edge
  // INT1: Off
  // INT2: Off
  GICR|=(0<<INT1) | (1<<INT0) | (0<<INT2);
  MCUCR=(0<<ISC11) | (0<<ISC10) | (1<<ISC01) | (0<<ISC00);
  MCUCSR=(0<<ISC2);
  GIFR=(0<<INTF1) | (1<<INTF0) | (0<<INTF2);

  // Global enable interrupts
  #asm("sei")

  timer_pwm_start();
  
  while (1)
  {

  }
}
