#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds3231.h"
#include <stdio.h>

#define PRESCALER 1024
#define HOLD_TIME_SET_CLK 2 //seconds
#define TIME_QUIT_SET_CLK 4 //seconds

#define HOURS_IN_DAY 24
#define MINUTES_IN_HOUR 60

enum State
{
  DISPLAY,
  SET_CLOCK
};

static enum State current_state = DISPLAY;

//bool
static uint8_t button1_active  = 0;
static uint8_t button2_active  = 0;
static uint8_t button1_pressed = 0;
static uint8_t button2_pressed = 0;
static uint8_t buttons_pressed = 0;

static uint8_t timer_initialised = 0;

static uint8_t display_time = 0;

static struct ds3231_clock_t my_clock = {0};

void init_RTC(struct ds3231_control_t *ctrl_reg);

void init_board(void);

void init_timer(uint8_t seconds);
void stop_timer(void);

void display_time_nixie(void);

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
  uint8_t new_buttons_pressed = 0;

  uint8_t add_hour = 0;
  uint8_t add_minute = 0;

  struct ds3231_control_t control_register = {0};
  init_RTC(&control_register);

  init_board();
  
  sei();

  // Initialisation de l'USART pour le port série
  uart_init();
  // Rediriger stdout vers le port série
  fdevopen(&serial_putchar, 0);

  while (1)
  {
    new_button1_pressed = button1_pressed;
    new_button2_pressed = button2_pressed;
    new_buttons_pressed = buttons_pressed;

    if(display_time) display_time_nixie();

    if(add_hour && new_button1_pressed && button1_active)
    {
        add_hour = 0;
        button1_active = 0;

        my_clock.seconds = 0;
        ++my_clock.hours;
        if(my_clock.hours >= HOURS_IN_DAY) my_clock.hours -= HOURS_IN_DAY;

        if(ds3231_write_clock(&my_clock)) printf("error write clock \n");
        display_time_nixie();
    }
    if(add_minute && new_button2_pressed && button2_active)
    {
        add_minute = 0;
        button2_active = 0;

        my_clock.seconds = 0;
        ++my_clock.minutes;
        if(my_clock.minutes >= MINUTES_IN_HOUR) my_clock.minutes -= MINUTES_IN_HOUR;

        if(ds3231_write_clock(&my_clock)) printf("error write clock \n");
        display_time_nixie();
    }

    if(current_state == DISPLAY)
    {
      PORTD |=  (1 << PIND5);
      PORTD &= ~(1 << PIND4);

      button1_active = 0;
      button2_active = 0;

      if(new_buttons_pressed)
      {
        if(!timer_initialised)  init_timer(HOLD_TIME_SET_CLK);
      }
      else stop_timer();
    }
    else //SET_CLOCK state
    {
      PORTD &= ~(1 << PIND5);
      PORTD |=  (1 << PIND4);

      if(new_button1_pressed) add_hour = 1;
      else button1_active = 1;

      if(new_button2_pressed) add_minute = 1;
      else button2_active = 1;

      if(!timer_initialised)                          init_timer(TIME_QUIT_SET_CLK);
      if(new_button1_pressed || new_button2_pressed)  stop_timer();
    }
  }

  return 0;
}


/*
 * Initialisation of the RTC module and RTC related features.
 * Set the control register to allow oscillation when using battery.
 * Enable interrupt truggered by SQW.
 */ 
void init_RTC(struct ds3231_control_t *ctrl_reg)
{
    ctrl_reg->intcn = 0;
    ctrl_reg->rs1 = 0;
    ctrl_reg->rs2 = 0;
    ctrl_reg->bbsqw = 1;
    ctrl_reg->neg_eosc = 0;

    if(ds3231_write_control(ctrl_reg)) printf("error write ctrl reg \n");

    //SQW 1Hz
    PORTD |= (1 << PIND2);            //activate pull-up
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);
}

void init_board(void)
{
    TCCR1A &= ~(1 << WGM11) & ~(1 << WGM10); //nomral mode
    TCCR1B &= ~(1 << WGM13) & ~(1 << WGM12);
    TCCR1B |=  (1 << CS10)  |  (1 << CS12); //prescaler /1024
    
    PCICR  |= (1 << PCIE0); //enable pin change int0
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
    PORTB  |= (1 << PINB0)  | (1 << PINB1); //activate pull-up
  
    DDRD = (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7); //portD4.5.6.7 output
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

void display_time_nixie(void)
{
  display_time = 0;
  if(ds3231_read_clock(&my_clock)) printf("error read clock \n");
  printf("Il est %2.2i:%2.2i:%2.2i \n", my_clock.hours, my_clock.minutes, my_clock.seconds);
}

ISR(PCINT0_vect)
{
  stop_timer();

  if(!(PINB & (1 << PINB0)))
  {
    PORTD |= (1 << PIND7);
    button1_pressed = 1;
  }
  else
  {
    PORTD &= ~(1 << PIND7);
    button1_pressed = 0;
  }

  if(!(PINB & (1 << PINB1)))
  {
    PORTD |= (1 << PIND6);
    button2_pressed = 1;
  }
  else
  {
    PORTD &= ~(1 << PIND6);
    button2_pressed = 0;
  }

  if(button1_pressed && button2_pressed) buttons_pressed = 1;
  else                                   buttons_pressed = 0;
}

ISR(TIMER1_COMPA_vect)
{
  stop_timer();
  if(buttons_pressed && (current_state == DISPLAY))
  {
    current_state = SET_CLOCK;
  }
  else if(current_state == SET_CLOCK)
  {
    current_state = DISPLAY;
  }
}

ISR(INT0_vect)
{
  display_time = 1;
}
