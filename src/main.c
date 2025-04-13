#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds3231.h"
#include <stdio.h>

#define PRESCALER 1024
#define HOLD_TIME_SET_CLK 2 //seconds
#define TIME_QUIT_SET_CLK 4 //seconds

enum State
{
  DISPLAY,
  SET_CLOCK
};

enum State current_state = DISPLAY;

static uint8_t button1_pressed = 0;
static uint8_t button2_pressed = 0;
static uint8_t button_pressed = 0;

static uint8_t timer_initialised = 0;

void init_timer(uint8_t seconds);
void stop_timer(void);

// Redirection de stdout vers le port série
int serial_putchar(char c, FILE *stream) {
  // Attente que le buffer de transmission soit prêt
  while (!(UCSR0A & (1 << UDRE0))) {
      // Attente active
  }

  // Envoi du caractère au registre de transmission
  UDR0 = c;

  return 0;
}

void uart_init() {
  // Calcul de la valeur de UBRR pour une communication à 9600 bauds
  unsigned int baudrate = 9600;
  unsigned int ubrr_value = F_CPU / 16 / baudrate - 1;

  // Configurer le registre UBRR pour ajuster la vitesse de transmission
  UBRR0H = (unsigned char)(ubrr_value >> 8);
  UBRR0L = (unsigned char)(ubrr_value);

  // Activer l'émetteur (TX)
  UCSR0B = (1 << TXEN0);

  // Configurer la taille des données à 8 bits
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

int main(void)
{
  uint8_t new_button1_pressed = 0;
  uint8_t new_button2_pressed = 0;
  uint8_t new_button_pressed  = 0;

  TCCR1A &= ~(1 << WGM11) & ~(1 << WGM10); //nomral mode
  TCCR1B &= ~(1 << WGM13) & ~(1 << WGM12);
  TCCR1B |=  (1 << CS10)  |  (1 << CS12); //prescaler /1024


  DDRB = (1 << DDB1) | (1 << DDB0) | (1 << DDB2) | (1 << DDB3); //portB1,2 output
  
  PCICR  |= (1 << PCIE1); //enable pin change int1
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);
  PORTC  |= (1 << PINC0)  | (1 << PINC1); //active pull-up

  sei();

  // Initialisation de l'USART pour le port série
  uart_init();

  // Rediriger stdout vers le port série
  fdevopen(&serial_putchar, 0);


  struct ds3231_clock_t myClock = { 0 };
  printf("Il est %2.2i:%2.2i:%2.2i \n", myClock.hours, myClock.minutes, myClock.seconds);
  while (1)
  {
    new_button1_pressed = button1_pressed;
    new_button2_pressed = button2_pressed;
    new_button_pressed  = button_pressed;

    ds3231_read_clock(&myClock);
    printf("Il est %2.2i:%2.2i:%2.2i \n", myClock.hours, myClock.minutes, myClock.seconds);
    _delay_ms(1000);

    if(current_state == DISPLAY)
    {
      PORTB |=  (1 << PIND2);
      PORTB &= ~(1 << PIND3);

      if(new_button_pressed)
      {
        if(!timer_initialised)  init_timer(HOLD_TIME_SET_CLK);
      }
      else stop_timer();
    }
    else //SET_CLOCK state
    {
      PORTB &= ~(1 << PIND2);
      PORTB |=  (1 << PIND3);

      if(!timer_initialised)                          init_timer(TIME_QUIT_SET_CLK);
      if(new_button1_pressed || new_button2_pressed)  stop_timer();
    }
  }

  return 0;
}

ISR(PCINT1_vect)
{
  stop_timer();

  if(!(PINC & (1 << PINC0)))
  {
    PORTB |= (1 << PINB0);
    button1_pressed = 1;
  }
  else
  {
    PORTB &= ~(1 << PINB0);
    button1_pressed = 0;
  }

  if(!(PINC & (1 << PINC1)))
  {
    PORTB |= (1 << PINB1);
    button2_pressed = 1;
  }
  else
  {
    PORTB &= ~(1 << PINB1);
    button2_pressed = 0;
  }

  if(button1_pressed && button2_pressed) button_pressed = 1;
  else                                   button_pressed = 0;
}

ISR(TIMER1_COMPA_vect)
{
  stop_timer();
  if(button_pressed && (current_state == DISPLAY))
  {
    current_state = SET_CLOCK;
  }
  else if(current_state == SET_CLOCK)
  {
    current_state = DISPLAY;
  }
}

void init_timer(uint8_t seconds)
{
  timer_initialised = 1;
  OCR1A = (uint16_t) seconds * F_CPU / PRESCALER;
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A); //enable oc int
}

void stop_timer(void)
{
  timer_initialised = 0;
  TIFR1  |=  (1 << OCF1A);
  TIMSK1 &= ~(1 << OCIE1A);
}