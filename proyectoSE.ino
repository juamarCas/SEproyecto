#include <avr/io.h>
#include <avr/interrupt.h>


int timer; // tendrá a TCNT2
int counter;
bool right;
bool left;
bool canMove = true;
unsigned int result = 0;

/*Timer 2, usado para el sensor de ultrasonido*/
#define StartTimer2 TCCR2B = (1 << CS22) | (1 << CS21)  | (1 << CS20); // prescaller 1024;
#define StopTimer2 TCCR2B &= 0B11111000;
#define timerA2 TCCR2A = 0x00;
/*Timer 0 usado con interrupción para constantemente checar el estado*/
#define StartTimer0 TCCR0B = (1 << CS02) | (1 << CS00); //prescaller 64
#define StopTimer0 TCCR0B = 0B11111000;
#define timerA0 TCCR0A = 0x00;
#define configTimerA0 TCCR0A = (1 << WGM01);
#define activateTimeInt  TIMSK0 = (1 << OCIE0A);
#define stopTimeInt TIMSK0 = 0x00;
/*Sensor ultrasonido*/
#define Echo PC1
#define trigger PC0
#define error -1
#define noObs -2
/*Servomotor*/
#define pos1 3000
#define pos2 2000 // 3000 es el centro
#define pos3 4000
/*Control de motores*/
#define n1 PD0
#define n2 PD1
#define n3 PD3
#define n4 PD4
#define l 1
#define r 0 // l -> izquierda, r -> derecha
/*0 entrada, 1 salida*/
void setup() {
  DDRD = (1 << n1) | (1 << n2) | (1 << n3) | (1 << n4) ; //salidas hacia el puente h
  DDRC = (1 << trigger); // PC0 es trigger PC1 es echo
  DDRB = (1 << PB2);
  init_Timer0();
  motor_forward();
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  OCR1A = 39999;  // 50Hz pwm
  OCR1B = pos1; // t_on = 1.5 ms, port B as output. PB2 (pin 16) is OC1B
  // 2000 izquierda
  // 4000 derecha

  //TCCR1B = (1 << CS11); // prescaller de 8
  TCNT1 = 0;
  sei();
}

void loop() {

  if (timer <= 26) {

    stop_Timer0();
    Stop();
    _delay_ms(700);
    canMove = false;
    left = CheckSides(pos2);
    right = CheckSides(pos3);

    if (left && !right) {
      GoBack(200);//retrocede un poco
      TurnSide(l);
      
    } else if (left && right) {
      //si ambos son verdad que escoja por defecto la izquierda
      GoBack(200);
      TurnSide(l);
    } else if (!left && right) {
      GoBack(200);  //retrocede un poco
      TurnSide(r);
    
    } else {
      while (left == false && right == false) {
        GoBack(500);
        left = CheckSides(pos2);
        right = CheckSides(pos3);
      }
      if ( left ) {
        // va a la izquierda
        TurnSide(l);
      } else if ( right ) {
        // va a la derecha
        TurnSide(r);
      } else if ( right && left) {
        // si en ambos casos es verdad que escoja la derecha por defecto
        TurnSide(r);
      }
    }
    init_Timer0();
    motor_forward();
    //OCR1B = 2000;
  } else {
    OCR1B = pos1;
  }



}

bool CheckSides(int pos){
   OCR1B = pos;
  TakeDistance();
  timer = GetPulseWidth();
  _delay_ms(1000);

  return timer >= 26;
}

void motor_forward() { // ir adelante
  PORTD = (1 << n1) | (1 << n3);
  PORTD &= ~((1 << n2) | (1 << n4));
}

void GoBack(int y) { // y es el tiempo activo de los motores
  OCR1B = pos1;
  _delay_ms(500);
  PORTD = (1 << n2) | (1 << n4);
  PORTD &= ~((1 << n1) | (1 << n3));
  while(y--){
    _delay_ms(1);
  }
  Stop();

}

void TurnSide(int dir){ //girar
  if(dir == l){ // dirección izquierda
     PORTD &= ~((1 << n1) | (1 << n2) | (1 << n3));
    PORTD = (1 << n4);
    _delay_ms(250);
  }else if (dir == r){ // dirección derecha
    PORTD &= ~((1 << n3) | (1 << n4) | (1 << n1));
    PORTD = (1 << n2);
    _delay_ms(250);
    
  }

  Stop();
}


void Stop() {
  PORTD &= ~((1 << n1) | (1 << n2) | (1 << n3) | (1 << n4));
}

void stop_Timer0() {
  //no cuente
  StopTimer0;
  timerA0;
  stopTimeInt;


}

void init_Timer0() {
  //empieza a contar
  StartTimer0;
  configTimerA0;
  activateTimeInt;
  OCR0A = 157; // 0.01 seg
}



void TakeDistance() {
  PORTC |= (1 << trigger);
  _delay_us(15);
  PORTC &= ~(1 << trigger);
}



uint16_t GetPulseWidth() {
  uint32_t i, result;
  // flanco de subida

  for (i = 0; i < 600000; i++) {
    if (!(PINC & (1 << Echo))) {
      continue;
    } else {
      break;
    }
  }

  if ( i == 600000) {
    return error; // se acabó el tiempo
  }
  timerA2;
  StartTimer2;
  TCNT2 = 0x00; // comienza el conteo

  for (i = 0; i < 600000; i++) {
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

  if (i == 600000) {
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
    counter = 0;
    //Serial.println(timer);


  }
}
