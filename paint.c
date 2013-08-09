// 3D Paint developed by Will Myers and Guo Jie Chin 2012, Cornell University
// This is code for an AtMega644 Microcontroller. The 644 interfaces over a serial
// port to any PC running Matlab. The PC running Matlab will need to run frontend.m
// after this code is properly running on the 644 to begin drawing. If everything is
// setup correctly, the Matlab GUI will display 'Open' in green letters in the 'Connection
// Status' box.
//
// Pin Description on the 644
// B[2] - Rx 2 Interrupt (Z)
// D[0:1] - UART
// D[2] - Rx 0 Interrupt (X)
// D[3] - Rx 1 Interrupt (Y)
// D[4] - Brush Button
// D[7] - 40KHz Output

// Standard includes
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"


// Defines to simplify code
#define t1 40               // Button Check frequecy - every 10ms
#define t2 40               // Pulse Train interpulse period - every 20ms
#define t3 2                // Pulse Train time length - 1ms
#define t4 15               // Pulse timeout - 7.5ms
#define NoPush 1            // For button state machine
#define MaybePush 2         // For button state machine
#define Pushed 3            // For button state machine
#define MaybeNoPush 4       // For button state machine
#define MedianLength 15     // The length of the sliding window median filter on chip
#define MeanLength 35       // The length of the sliding window mean filter on chip


// Helper Functions
void initialize(void);
void check_button(void);
int compare (const void * a, const void * b);

// General Variables
volatile unsigned char timer_flag[3];         // TimerFlag[i] = 1 means receiver i is primed to received a sample
volatile unsigned char delay_flag[3];         // DelayFlag[i] = 1 means receiver i has locked onto a sample
volatile unsigned char r_char;                // Char received from Matlab to signal a new transmission
volatile int time1;                           // Timer to dispatch button checking
volatile int time2;                           // Timer to trigger a new pulse train and reset the system
volatile int time3;                           // Timer to stop the pulse train
volatile int time4;                           // Timer to timeout a given pulse train if 3 samples weren't received
volatile unsigned long cycle_delay[3];        // cycle_delay[i] is the delay for receiver i in cycles at 16MHz
volatile unsigned long cycle_delay_overflow;  // Varible used for higher precision counting with overflow vector
unsigned char push_state;                     // Button checking state variable for debouncing
unsigned char i;                              // Used for generic for-loops
unsigned char j;                              // Used for generic for-loops
unsigned char timeout;                        // Signal that the system is ready to begin the next pulse train
unsigned char got_point;                      // Signal that the system has logged a point and processed it
unsigned char median_filter_index;            // Index for the sliding window median filter
unsigned char mean_filter_index;              // Index for the sliding window mean filter
unsigned long total;                          // Accumulator variable for mean calculation
unsigned long output_delay[3];                // output_delay[i] is the filtered delay in cycles for receiver i
unsigned long median_window[3][MedianLength]; // The median window itself - 1 row for each data stream
unsigned long sorted_median[3][MedianLength]; // The sorted median window for finding the median
unsigned long mean_window[3][MeanLength];     // The mean window itself - 1 row for each data steam

// Setup for UART communication
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// Rx Interrupt 0 - X Position
ISR (INT0_vect)
{
  // If timer is armed (waiting for a delay and has not received it yet), then store the delay.
  if(timer_flag[0] == 1){
    cycle_delay[0] = TCNT1 + cycle_delay_overflow;    // Record total delay time
    timer_flag[0] = 0;                                // De-arm the timer
    delay_flag[0] = 1;                                // Flag that we're ready to enter this ino processing
  }
}

// Rx Interrupt 1 - Y Position
ISR (INT1_vect)
{
  // If timer is armed (waiting for a delay and has not received it yet), then store the delay.
  if(timer_flag[1] == 1){
    cycle_delay[1] = TCNT1 + cycle_delay_overflow;    // Record total delay time
    timer_flag[1] = 0;                                // De-arm the timer
    delay_flag[1] = 1;                                // Flag that we're ready to enter this ino processing
  }
}

// Rx Interrupt 2 - Z Position
ISR (INT2_vect)
{
  // If timer is armed (waiting for a delay and has not received it yet), then store the delay.
  if(timer_flag[2] == 1){
    cycle_delay[2] = TCNT1 + cycle_delay_overflow;    // Record total delay time
    timer_flag[2] = 0;                                // De-arm the timer
    delay_flag[2] = 1;                                // Flag that we're ready to enter this ino processing
  }
}

// This interrupt executes everytime TCNT1 overflows, so we accumulate the overflow values here
ISR (TIMER1_OVF_vect) {
  cycle_delay_overflow += 256 ;
}

// Task Dispatcher Interrupt
ISR (TIMER0_COMPA_vect){
  if(time1 > 0) --time1;     // Button dispatcher
  if(time2 > 0) --time2;     // Pulse Train IPP
  if(time3 > 0) --time3;     // Pulse train length
  if(time4 > 0) --time4;     // Pulse train timeout
}

// When we get a message from matlab, record the result here
ISR(USART0_RX_vect){
  r_char = UDR0;
}

int main(void)
{
  // Standard initialization procedues to set up ports and variables
    initialize();

    while(1)
    {
    if(time1 == 0){ time1 = t1; check_button();}  // Dispatch button checker
    if(time2 == 0 || timeout == 1){               // If the IPP is up OR we've timed out, re-arm the system
      cycle_delay[0] = 0;                         // Reset delay
      cycle_delay[1] = 0;                         // Reset delay
      cycle_delay[2] = 0;                         // Reset delay
      timer_flag[0] = 1;                          // Re-arm interrupt logging
      timer_flag[1] = 1;                          // Re-arm interrupt logging
      timer_flag[2] = 1;                          // Re-arm interrupt logging
      TCNT1 = 0;                                  // Reset timer
      cycle_delay_overflow = 0;                   // Reset timer overflow values
      time2 = t2;                                 // Reset process dispatcher
      time3 = t3;                                 // Reset process dispatcher
      time4 = t4;                                 // Reset process dispatcher
      timeout = 0;                                // Reset flag for ready
      got_point = 0;                              // Reset flag for point
      TCCR2B = 1;                                 // Turn on the 40KHz train
    }

    if(time3 == 0){                               // Turn off the pulse train
      TCCR2B = 0;
    }

    if(time4 == 0 && got_point == 0){       // Timeout event
      delay_flag[0] = 0;                    // Reset interrupt finished flag
      delay_flag[1] = 0;                    // Reset interrupt finished flag
      delay_flag[2] = 0;                    // Reset interrupt finished flag
      timeout = 1;                          // Signal a timeout
    }

    // Here we have received three delays all corresponding to a similar pulse train, process them
    if((delay_flag[0] == 1) && (delay_flag[1] == 1) && (delay_flag[2] == 1)){
      got_point = 1;                                                // Signal that we've recieved the points
      for(i = 0; i < 3; i++){                                       // Loop over all three delay streams
        if(cycle_delay[i] > 100){                                   // Any delay result less than 100 is faulty
          median_window[i][median_filter_index] = cycle_delay[i];   // Cyclically add the new delay value to the median window
          sorted_median[i][0] = median_window[i][0];                // Store the window in a temporary array for sorting
          sorted_median[i][1] = median_window[i][1];                // We don't want to lose the order that we're placing entries into the window
          sorted_median[i][2] = median_window[i][2];
          sorted_median[i][3] = median_window[i][3];
          sorted_median[i][4] = median_window[i][4];
          sorted_median[i][5] = median_window[i][5];
          sorted_median[i][6] = median_window[i][6];
          sorted_median[i][7] = median_window[i][7];
          sorted_median[i][8] = median_window[i][8];
          sorted_median[i][9] = median_window[i][9];
          sorted_median[i][10] = median_window[i][10];
          sorted_median[i][11] = median_window[i][11];
          sorted_median[i][12] = median_window[i][12];
          sorted_median[i][13] = median_window[i][13];
          sorted_median[i][14] = median_window[i][14];
          qsort(sorted_median[i], MedianLength, sizeof(unsigned long), compare);  // Sort temp window
          mean_window[i][mean_filter_index] = sorted_median[i][MedianLength/2];   // Extract the median
          total = 0;                                                              // Initialize an accumulator for the mean
          for(j = 0; j < MeanLength; j++){total += mean_window[i][j];}            // Total up all delays..
          output_delay[i] = total/MeanLength;                                     // And average
          delay_flag[i] = 0;                                                      // Signal that we've processed each stream
        }
      }
      median_filter_index++;                                            // Increment median index
      mean_filter_index++;                                              // Increment mean index
      if(median_filter_index == MedianLength){median_filter_index = 0;} // wrap around mod style
      if(mean_filter_index == MeanLength){mean_filter_index = 0;}       // wrap around mod style

    }

    // If Matlab is prepared to receive a new dataset...
    if(r_char == 'y'){
      // Matlab terminator: \n\r
      // The data formatting is tricky and very particular, this matches up with the variable lengths in the
      // Matlab fscanf(). Comma delimiters help out Matlab with parsing as well.
      // Each packet contains the button state and three delays.
      fprintf(stdout,"%u,%ld, %ld, %ld\r\n",push_state, output_delay[0], output_delay[1], output_delay[2]);
      r_char = 'n'; // reset for next send
    }


    }

}

// State machine for button pressing debouncing
void check_button(){

  switch (push_state){
       case NoPush:
          if(~PIND & 0x10) push_state = MaybePush;    // Got push..
          else push_state = NoPush;                   // No push..
          break;
       case MaybePush:
          if(~PIND & 0x10){ push_state = Pushed;}     // Got 2 pushes..
          else push_state = NoPush;                   // False push
          break;
       case Pushed:
          if(~PIND & 0x10) push_state = Pushed;       // Still pushed..
          else push_state = MaybeNoPush;              // Got release..
          break;
       case MaybeNoPush:
          if(~PIND & 0x10) push_state = Pushed;       // False release..
          else push_state = NoPush;                   // Got 2 releases..
       break;
  }
}

// Generic compare function used by qsort. Documentation found on cplusplus.com
int compare(const void * a, const void * b)
{
  return ( *(unsigned long*)a - *(unsigned long*)b );
}

void initialize(void)
{

    // Init the UART -- uart_init() is in uart.c
  // Ensure that uart.c has the proper baud set (38500)
    uart_init();
    stdout = stdin = stderr = &uart_str;
  fprintf(stdout,"running...\n\r");

    // set up the ports!
  DDRD = (0 << PIND2) | (0 << PIND3);                 // D2 and D3 are inputs for hardware intterupts
  DDRB = (0 << PINB2);                                // B2 is an input for hardware interrupts
  PORTD = (1 << PIND2) | (1 << PIND3) | (1 << PIND4); // D2 and D3 need pull-up resistor set for interrupting
                                                      // D4 needs pull-up resistor set for button
  PORTB = (1 << PINB2);                               // B2 needs pull-up resistor set for interrupting


  // Timer 0 for .5 mSec ticks
    TIMSK0 = 2;                               // Turn on timer 0 cmp match ISR
    OCR0A = 125;                              // Set the compare reg to 125 time ticks
    TCCR0A = 0b00000010;                      // Turn on clear-on-match
    TCCR0B = 0b00000011;                      // Clock prescalar to 64

  // Timer 1 for Delay Timing
  TCCR1B = 1 ;                                // Divide by 1, turn on Timer 1
  TIMSK1 = 1 ;                                // Turn on Timer 1 overflow ISR for float precision time

    // Timer 2 Generates a 40KHz Square Wave
    OCR2A = 199;                              // 200 cycles/half-period
    TCCR2B =  0;                              // Count at full rate
    TCCR2A = (1<<COM2A0) | (1<<WGM21);        // Set to toggle OC2A, clear on match
    DDRD = (1<<PIND7);                        // PORT D.7 is OC2A

  // Set up INT0, INT1, INT2 hardware driven interrupt and Timer 2 counting
  EICRA = 63 ;                                        // All interrupts triggered on rising edge
  EIMSK = (1 << INT0) | (1 << INT1) | (1 << INT2) ;   // Turn on INT0, INT1, INT2, rising edge for all

  // Init some values
  for(i = 0; i < 3; i ++){
    timer_flag[i] = 0;
    delay_flag[i] = 0;
    cycle_delay[i] = 0;
    output_delay[i] = 0;
  }

  r_char = 'n';     // No char yet
  time1 = t1;       // Prime timer
  time2 = t2;       // Prime timer
  time3 = t3;       // Prime timer
  time4 = t4;       // Prime timer
  cycle_delay_overflow = 0;
    push_state = NoPush;
  timeout = 1;
  got_point = 0;
  total = 0;

  for(median_filter_index = 0; median_filter_index < MedianLength; median_filter_index++){
    for(i = 0; i < 3; i ++){ median_window[i][0] = 0;}
  }
  for(mean_filter_index = 0; mean_filter_index < MeanLength; mean_filter_index++){
    for(i = 0; i < 3; i ++){ mean_window[i][0] = 0;}
  }

  median_filter_index = 0;
  mean_filter_index = 0;

  // Set up character receive complete interrupt
  UCSR0B |= (1 << RXCIE0);

    // Crank up the ISRs
    sei();
}
