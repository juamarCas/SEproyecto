
#include <avr/io.h>
#include <avr/interrupt.h>

const double sound = 34300; // velocidad del sonido en cm/s
const double periodSecond = 0.0000005; // periodo del ciclo de reloj en segundos
int timer; // tendrá a TCNT1
uint16_t distance; // se da en cm
int counter;

unsigned int result = 0;

#define LED PD4
/*Timer 1, usado para el sensor de ultrasonido*/
#define StartTimer2 TCCR2B = (1 << CS22) | (1 << CS21)  | (1 << CS20); // prescaller 256;
#define StopTimer2 TCCR2B &= 0B11111000;
#define timerA2 TCCR2A = 0x00;
/*Sensor ultrasonido*/
#define Echo PC1
#define trigger PC0
#define error -1
#define noObs -2
/*Servomotor*/
/*Control de motores*/

/*0 entrada, 1 salida*/
void setup() {
  Serial.begin(9600);
  DDRD = (1 << LED); // pd7 salida
  DDRC = (1 << trigger); // PC0 es trigger PC1 es echo
  DDRB = (1 << PB2);
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS02) | (1 << CS00); //prescaller 64
  TIMSK0 = (1 << OCIE0A);
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  OCR1A = 39999;  // 50Hz pwm
  OCR1B = 3000; // t_on = 1.5 ms, port B as output. PB2 (pin 16) is OC1B

 
  OCR0A = 157; // 0.01 seg

  //TCCR1B = (1 << CS11); // prescaller de 8
  TCNT1 = 0;
  sei();
}

void loop() {

  if(timer <= 26){
    OCR1B = 2000;
  }else{
    OCR1B = 3000;
  }
   //Serial.println(timer);
   

}


void TakeDistance() {
  PORTC |= (1 << trigger);
  _delay_us(15);
  PORTC &= ~(1 << trigger);
}

uint16_t GetPulseWidth() {
  uint32_t i, result;
  // flanco de subida

  for (i = 0; i < 60000; i++) {
    if (!(PINC & (1 << Echo))) {
      continue;
    } else {
      break;
    }
  }

  if ( i == 60000) {
    return error; // se acabó el tiempo
  }
  timerA2;
  StartTimer2;
  TCNT2 = 0x00; // comienza el conteo

  for (i = 0; i < 60000; i++) {
    if (PINC & (1 << Echo)) {
      if (TCNT2 > 200) {
        break;
      } else {
        continue;
      }

    } else {
      break;
    }
  }

  if (i == 60000) {
    return noObs;
  }

  result = TCNT2;

  StopTimer2;

  return TCNT2;
}


ISR(TIMER0_COMPA_vect) {
  counter++;
  if (counter >= 10) {
    TakeDistance();
    timer = GetPulseWidth();
   

  }
}
