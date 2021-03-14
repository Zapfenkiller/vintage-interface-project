/*****************************************************************************\
 * File:         vip.c
 * Project:      Vintage Interface Project - V.I.P.
 * Initial date: May / 7 / 2017
 * Release rev.: 2017jun28
 * Copyright:    (c) 2017 R. Trapp / H.A.R.R.Y.
 *               All rights reserved.
 * Contact:      h_a_r_r_y_@users.sourceforge.net
 *               <https://forum.ftcommunity.de> => send PN to H.A.R.R.Y.
 * Description:  Adapt vintage fischertechnik Computing Interfaces to I²C-bus.
 *               The Interface thus can be addressed and operated as a slave.
 *               See documentation in archive or ft:pedia 2/2017 ff.
 *               Adjusts automatically to all known interface types:
 *               30520, 30561, 30562, 30563, 30564, 30565, 30566, 30567, 39319,
 *               66843
 *               ATtiny2313 runs off 8MHz, bus speed maximum 100kHz w/o clock-
 *               stretching. 400kHz relies on clock-stretching to slow down the
 *               bus-master!
 * Toolchain:    WinAVR-20100110 (http://www.sourceforge.net)
 *               ATtiny2313 datasheet revision = Rev. 2543I–AVR–04/06
 * WARNING:      This code contains inline assembly and plays lots of other
 *               tricks to optimize for topmost speed where necessary. Due to
 *               this everything must be done here in *one* file. If you dislike
 *               such a coding style you better stay away and stop reading now.
 *               There is the pure 'teaching book' "how-things-work" C-code in
 *               the neighbourhood. Read this equivalent to get an idea of the
 *               functionality. That code also compiles, links and works but if
 *               you compile and use that code, you will get a penalty of
 *               increased execution time - leading also to clock-stretching at
 *               100kHz bus.
 * License:      Redistribution and use in source and binary forms, with or
 *               without modification, are permitted provided that the following
 *               conditions are met:
 *               * Redistributions of source code must retain the above
 *                 copyright notice, this list of conditions and the following
 *                 disclaimer.
 *               * Redistributions in binary form must reproduce the above
 *                 copyright notice, this list of conditions and the following
 *                 disclaimer in the documentation and/or other materials
 *                 provided with the distribution.
 *               * Neither the name of the copyright holders nor the names of
 *                 contributors may be used to endorse or promote products
 *                 derived from this software without specific prior written
 *                 permission.
 * Disclaimer:   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *               CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *               INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *               MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *               DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *               CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *               NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *               HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *               CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *               OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *               EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
\*****************************************************************************/


// ============================================================================
//                         Set MCU clock to 8MHz!
//                      Everything here relies on it!
// ============================================================================

#ifndef F_CPU
#define F_CPU                 8000000UL
#else
#if (F_CPU!=8000000)
#error: F_CPU not set to 8MHz! Readjust project properties!
#endif
#endif


// ============================================================================
//                   Check for ATtiny2313 to be selected
// ============================================================================

#ifndef __AVR_ATtiny2313__
#error: This code is exclusively dedicated to ATtiny2313!
#endif
// Very close relatives could eventually also be used without changes.


/******************************************************************************\
 ******************************************************************************
 *
 *                                Prologue
 *
 ******************************************************************************
\******************************************************************************/


// ============================================================================
// Necessary includes
// ============================================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// ============================================================================
//                      Highly Recommended settings
// ============================================================================

// CKSEL3..0      = 0100   = internal 8MHz
// CKDIV          = 1      = no prescaling
// SUT1..0        = 10     = 65ms (slow rising power)
// BODLEVEL2..0   = 101    = 2.7V
// According to documentation just list here those fuse bits which need to get
// programmed ('0'). Unprogrammed ('1') fuses shall not be listed.
   FUSES = 
   {
      .low = (FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL1 & FUSE_CKSEL0),
      .high = (FUSE_SPIEN & FUSE_BODLEVEL1),
      .extended = (FUSE_SELFPRGEN),
   };


// 'Security'     = Leave open since this code is released to the public.
   LOCKBITS = (LB_MODE_1);


// ============================================================================
// Assign signal names to ports
// ============================================================================

// ft Computing Interface:
// -----------------------
#define CLK                   PD5
#define COUNT_IN              PD4
#define DATA_IN               PD6
#define DATA_OUT              PD3
#define LOAD_IN               PD2
#define LOAD_OUT              PB2
#define MMV_PULSES            PA0
#define TRIGGER_X             PB4
#define TRIGGER_Y             PB3

// I²C bus:
// --------
#define TWIport               PORTB
#define TWIread               PINB
#define TWIddr                DDRB
#define TWIsclBit             PB7
#define TWIsdaBit             PB5

// Miscellaneous:
// --------------
#define MISO_ISP              PB6
#define SJ3                   PA1
#define SJ2                   PD1
#define SJ1                   PD0


// ============================================================================
// Reserve a few(!) registers - to speed up interrupt handling!
// ============================================================================
// Note: Need to do it here so that *everything* is aware of this reservation
//       at compile time!

/// \brief
/// Private storage register for the actual bus state.
// Use of r16 to r31 is mandatory due to CPI instructions
volatile register uint8_t  I2C_private_state asm ("r16");


/// \brief
/// Private storage register for the last received address+R/W field.
volatile register uint8_t  I2C_address_received asm ("r17");


// ============================================================================
// Unexpected Interrupts
// ============================================================================


/// \brief
/// Catches all unexpected interrupts.
/// \detail
/// What to do if any unexpected interrupt occurs? One could reset the appli-
/// cation immediately. And then? Unreliable functionality and really hard to
/// find root cause(s). Whom to report about the fault? No debug interface, no
/// user interface, ... so just ignore this unexpected interrupt. The ones we
/// are interested in are already handled.
EMPTY_INTERRUPT (BADISR_vect);
//  Up to here we assume 10 clocks (7 according to datasheet plus uncertainty)
//  So when RETI has passed we lost roughly 14 clocks @ 8MHz = 1750ns


/******************************************************************************\
 ******************************************************************************
 *
 *                       ft Computing Interface handling
 *                       ===============================
 *
 ******************************************************************************
\******************************************************************************/


// ============================================================================
// Some macros for easier coding
// ============================================================================
// Note: Do not rely on the compiler - force optimum code where necessary!

#define FT_IF_DEFAULT_DDRA    (0<<MMV_PULSES)
#define FT_IF_DEFAULT_PORTA   (1<<MMV_PULSES)
#define FT_IF_DEFAULT_DDRB    ((1<<TRIGGER_X) | (1<<TRIGGER_Y) | (1<<LOAD_OUT))
#define FT_IF_DEFAULT_PORTB   ((1<<TRIGGER_X) | (1<<TRIGGER_Y) | (0<<LOAD_OUT))
#define FT_IF_DEFAULT_DDRD    ((0<<DATA_IN) | (1<<CLK) | (0<<COUNT_IN) | (1<<DATA_OUT) | (1<<LOAD_IN))
#define FT_IF_DEFAULT_PORTD   ((1<<DATA_IN) | (1<<CLK) | (1<<COUNT_IN) | (0<<DATA_OUT) | (0<<LOAD_IN))

#define CLK_rise              asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (CLK))
#define CLK_fall              asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (CLK))
#define COMPARATOR_state      ACSR & (1<<ACO)
#define COUNT_IN_state        PIND & (1 << COUNT_IN)
#define COUNT_IN_count        TCNT0
#define DATA_IN_state         PIND & (1 << DATA_IN)
#define DATA_OUT_hi           asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (DATA_OUT))
#define DATA_OUT_lo           asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (DATA_OUT))
#define LOAD_IN_rise          asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (LOAD_IN))
#define LOAD_IN_fall          asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTD)), [bit] "I" (LOAD_IN))
#define LOAD_OUT_rise         asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTB)), [bit] "I" (LOAD_OUT))
#define LOAD_OUT_fall         asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(PORTB)), [bit] "I" (LOAD_OUT))
#define MMV_PULSES_state      PINA & (1 << MMV_PULSES)
#define TRIGGER_MMV(mmv)      PORTB &= (mmv); /*trigger_select;*/
#define TRIGGERs_release      PORTB |= ((1 << TRIGGER_X) | (1 << TRIGGER_Y))

#define DATA_IN_native        0b00000000
#define DATA_IN_inverted      0b11111111
#define no_motor_protection   0b11111111
#define protect_motors        0b00111111
#define MMV_worst_5k_ms       5
#define MMV_worst_100k_ms     75
#define MMV_trigger_width_us  5
#define ID_sampling_after_us  50

#define DELAY_1_MCU_CLOCK     asm volatile ("NOP"::)


// ============================================================================
// Enumerations for better code readability
// ============================================================================

enum
{  // Interface identification
   FT_IF_UNKNOWN   = 0,
   FT_IF_30520     = 9,
   FT_IF_30561     = 1,
   FT_IF_30562     = 2,
   FT_IF_30563     = 3,
   FT_IF_30564     = 4,
   FT_IF_30565     = 5,
   FT_IF_30566     = 6,
   FT_IF_30567     = 7,
// FT_IF_30568     = 8, RESERVATION!
   FT_IF_39319     = 6,
   FT_IF_66843     = 9
};


enum
{  // Channel selection for analog readback
   FT_IF_EX = (~(1<<TRIGGER_X)),
   FT_IF_EY = (~(1<<TRIGGER_Y))
};



// ============================================================================
// Variable definitions
// ============================================================================

uint8_t  ft_identifier = FT_IF_UNKNOWN;
uint8_t  ft_data_inversion_mask = DATA_IN_native;
uint8_t  ft_motor_protection = protect_motors;


// ============================================================================
// Resistance read back (analog inputs EX / EY)
// ============================================================================

/// \brief
/// Read the resistance value connected between EX or EY and COM.
/// \detail
/// Perform a full analog reading cycle with respect to the identified interface
/// properties. Readouts are adjusted to give similar results even with
/// different hardware properties.
/// \param channel defines which of the both is desired.
/// \returns byte proportional to resistance connected. 0 Ohms approx. = 20,
/// 5 kOhms approx = 237.
//  Runtime is of no concern since things take as long as they need. Bus is off
//  while getting the readings.
//  Time atom of 12250ns = 98 clocks is a fine fit for all but the 30563.
//  With 30563 we use 16625ns = 133 clocks as time atom.
static uint8_t FT_IF_get_resi (uint8_t trigger_select)
{
   // a software counter for getting the pulse length
   uint8_t my_counter = 0;
   // give one CLOCK pulse to WD and shifter preload
   CLK_fall;
   DELAY_1_MCU_CLOCK;
   DELAY_1_MCU_CLOCK;
   CLK_rise;
   // clear counter and its overflow flag
   TIFR = (1<<TOV0);
   TCNT0 = 0;
   // check individual response
   switch(ft_identifier)
   {
      case  FT_IF_30561:
            // trigger 556 by taking control line low
            TRIGGER_MMV(trigger_select);
            _delay_us(MMV_trigger_width_us);
            TRIGGERs_release;
            //  243 us @ 470 Ohm
            // 2823 us @ 5k Ohm
            // => 12250 ns / loop
   //       for (my_counter = 0; my_counter < 255; my_counter++)
   //       {
   //          if !(COUNT_IN_state)
   //             break;
   //          // coarse adjust
   //          _delay_us(xx);
   //          // fine adjust
   //          DELAY_1_MCU_CLOCK;
   //       }
            asm volatile (
            //                   for (my_counter = 0; ... ; ...)
            "\n\t"               "LDI  %[counter], 0"
            "\n\t"               "RJMP L_loopentry%="
            //                   {
            "\n\t" "L_loop%=:"
            //                   for (...; ...; count++)
            "\n\t"               "INC  %[counter]"
            "\n\t" "L_loopentry%=:"
            //                   if !(COUNT_IN_state)
            //                      break;
            "\n\t"               "SBIS %[port_addr], %[bit_num]"
            "\n\t"               "RJMP L_leave%="
            //                   _delay_us(11.25)  // coarse adjust
            "\n\t"               "LDI  r25, 30"
            "\n\t" "L_delay%=:"
            "\n\t"               "DEC  r25"
            "\n\t"               "BRNE L_delay%="
            "\n\t"               "NOP"             // fine adjust
            "\n\t"               "NOP"
            //                   for (...; count < 255; ...)
            "\n\t"               "CPI  %[counter], 255"
            "\n\t"               "BRLO L_loop%="
            //                   }
            "\n\t" "L_leave%=:"
            :  [counter]         "=&a" (my_counter)
            :  [port_addr]       "I"   (_SFR_IO_ADDR(PIND)),
               [bit_num]         "I"   (COUNT_IN)
            :  "r25"
            );
            return (my_counter);
            // NOTE: 30561 response can only be handled by software.
      case  FT_IF_30565:
      case  FT_IF_30567:
            // trigger 556 by taking control line low
            TRIGGER_MMV(trigger_select);
            _delay_us(MMV_trigger_width_us);
            TRIGGERs_release;
            //  243 us @ 470 Ohm
            // 2823 us @ 5k Ohm
            // => 12250 ns / loop
   //       for (my_counter = 0; my_counter < 255; my_counter++)
   //       {
   //          if (DATA_IN_state)
   //             break;
   //          // coarse adjust
   //          _delay_us(12);
   //          // fine adjust
   //          DELAY_1_MCU_CLOCK;
   //       }
            asm volatile (
            //                   for (my_counter = 0; ... ; ...)
            "\n\t"               "LDI  %[counter], 0"
            "\n\t"               "RJMP L_loopentry%="
            //                   {
            "\n\t" "L_loop%=:"
            //                   for (...; ...; count++)
            "\n\t"               "INC  %[counter]"
            "\n\t" "L_loopentry%=:"
            //                   if !(COUNT_IN_state)
            //                      break;
            "\n\t"               "SBIS %[port_addr], %[bit_num]"
            "\n\t"               "RJMP L_leave%="
            //                   _delay_us(11.25)  // coarse adjust
            "\n\t"               "LDI  r25, 30"
            "\n\t" "L_delay%=:"
            "\n\t"               "DEC  r25"
            "\n\t"               "BRNE L_delay%="
            "\n\t"               "NOP"             // fine adjust
            "\n\t"               "NOP"
            //                   for (...; count < 255; ...)
            "\n\t"               "CPI  %[counter], 255"
            "\n\t"               "BRLO L_loop%="
            //                   }
            "\n\t" "L_leave%=:"
            :  [counter]         "=&a" (my_counter)
            :  [port_addr]       "I"   (_SFR_IO_ADDR(PIND)),
               [bit_num]         "I"   (DATA_IN)
            :  "r25"
            );
            while (!(DATA_IN_state))
            {
               // no pot connected => wait until MMV expires or input read will
               // get jammed!
            }
            return (my_counter);
            // NOTE: Even if the hardware setup allows for usage of ICP features
            // of Timer1 there is currently a mismatch between timer resolution
            // and required output range. The software measurement thus takes
            // advantage of better controllability of the time atom to count.
            // Since there is just nothing else to do here this is considered
            // a valid solution to the problem.
            // Neue Möglichkeit: res = ICP / 97,5238...
            // 1) uint32_t res = 0;
            // 2) res += (ICP1 << 5);
            // 3) res += (ICP1 << 7);
            // 4) res += (ICP1 << 9);
            // 5) res = res >> 16;
            // Ergibt auch etwa 237 für 5kOhm (alles nominal)
            // Ob das auch so ginge, mit weniger Schiebereien und weniger
            // Registern?
            // 1) uint16_t res = 0;
            // 2) res += ICP1;
            //    res = res >> 2;
            // 3) res += ICP1;
            //    res = res >> 2;
            // 4) res += ICP1
            // 5) res = res >> 7;
            // bzw. res = res << 1 und nur das Hi-Byte nehmen
            // Aber nur theoretisch. Praktisch muss die CPU eh warten, da kann
            // sie auch gleich selber im richtigen Takt zählen.
      case  FT_IF_30562:
      case  FT_IF_30564:
      case  FT_IF_30566:   // also 39319 currently
      //case  FT_IF_39319:
      case  FT_IF_30520:   // also 66843 currently
      //case  FT_IF_66843:
            // trigger 556 by taking control line low
            TRIGGER_MMV(trigger_select);
            _delay_us(MMV_trigger_width_us);
            TRIGGERs_release;
            //  243 us @ 470 Ohm
            // 2823 us @ 5k Ohm
            // wait worst case for 5k-pulse to finish
            _delay_ms(MMV_worst_5k_ms);
            // fetch result from the counter
            if (TIFR & (1<<TOV0))
               return (255);   // overflow
            return (COUNT_IN_count);    // no overflow
            // NOTE: A full software based counting could also be an option. But
            // this way the timeout can be managed more elegant.
      case  FT_IF_30563:
            // trigger analog by taking control line low and keep it low!
            //  347 us @ 470 Ohm
            // 3945 us @ 5k Ohm
            // => 16625 ns / loop
            TRIGGER_MMV(trigger_select);
   //       for (my_counter = 0; my_counter < 255; my_counter++)
   //       {
   //          if (COMPARATOR_state)
   //             break;
   //          // coarse adjust
   //          _delay_us(12);
   //          // fine adjust
   //          DELAY_1_MCU_CLOCK;
   //       }
            asm volatile (
            //                   for (my_counter = 0; ... ; ...)
            "\n\t"               "LDI  %[counter], 0"
            "\n\t"               "RJMP L_loopentry%="
            //                   {
            "\n\t" "L_loop%=:"
            //                   for (...; ...; count++)
            "\n\t"               "INC  %[counter]"
            "\n\t" "L_loopentry%=:"
            //                   if !(COUNT_IN_state)
            //                      break;
            "\n\t"               "SBIS %[port_addr], %[bit_num]"
            "\n\t"               "RJMP L_leave%="
            //                   _delay_us(11.25)  // coarse adjust
            "\n\t"               "LDI  r25, 42"
            "\n\t" "L_delay%=:"
            "\n\t"               "DEC  r25"
            "\n\t"               "BRNE L_delay%="
            "\n\t"               "NOP"             // fine adjust
            //                   for (...; count < 255; ...)
            "\n\t"               "CPI  %[counter], 255"
            "\n\t"               "BRLO L_loop%="
            //                   }
            "\n\t" "L_leave%=:"
            :  [counter]         "=&a" (my_counter)
            :  [port_addr]       "I"   (_SFR_IO_ADDR(ACSR)),
               [bit_num]         "I"   (ACO)
            :  "r25"
            );
            // release control line - immediately discharge external cap
            TRIGGERs_release;
            return (my_counter);
            // NOTE: Even if the hardware setup allows for usage of ICP features
            // of Timer1 there is currently a mismatch between timer resolution
            // and required output range. The software measurement thus takes
            // advantage of better controllability of the time atom to count.
            // Since there is just nothing else to do here this is considered
            // a valid solution to the problem.
      case  FT_IF_UNKNOWN:
      default:
            return (0);
   }
}


// ============================================================================
// Read back switch states (E1..E8)
// ============================================================================

/// \brief
/// Generate LOAD_IN pulse.
/// \detail
/// Generate the LOAD_IN pulse to latch data from '4014 inputs to the
/// '4014 shifter chain simultanously. According to datasheet of '4014
/// the pulse width of each, CLOCK as well as LOAD_IN, needs to be at
/// least 400ns - worst case.
/// - Takes 1250ns @ 8MHz
static inline void FT_IF_sample_from_switches (void) __attribute__((always_inline));
void FT_IF_sample_from_switches (void)
{
   LOAD_IN_rise;
   CLK_fall;
   DELAY_1_MCU_CLOCK;
   DELAY_1_MCU_CLOCK;
   CLK_rise;
   LOAD_IN_fall;
   // 10 clocks @ 8MHz = 1250ns
}


/// \brief
/// Shift in one byte from the '4014 shifter chain.
/// \detail
/// Shifts in one byte from the '4014 shifter chain.
/// The MSB gets sampled first, all trailing bits down to the LSB thereafter.
/// The CLK line is assumed to be '1' when transfer starts and left at '1' 
/// when transfer is done.
/// The issue is that with C-code there is no control of the exact timing
/// possible. So inline assembly is used to nail things down exactly controlled.
/// \returns the byte shifted in. '0' = respective switch input is in open
/// state.
//  '4014: Relevant datasheet parameters vary with manufacturer!
//  Worst case values found in a Motorola specification from 1995 but there
//  might be worse around ...
//  Propagation delay from clock pulse to Q outputs = 800ns @ 5V/50pF
//  Fairchild says just 320ns @ 5V for same parameter
//  Clock pulse width = 400ns @ 5V
//  Parallel/Serial Control pulse width = 400ns @ 5V
//  Setup Time P/S to Clock rise = 200ns @ 5V
//  Clock frequency = 1.5MHz @ 5V
//  @ 8MHz:
//    CLOCK pulse high = 750ns
//    CLOCK pulse low = 750ns
//    CLOCK frequency = 667kHz
//    sample DATA_IN 875ns after posedge CLOCK
//  Even the worse Motorola ones will work reliably.
//  uint8_t FT_IF_shift_from_switches (void)
//  {
//     for(uint8_t count = 8; count > 0; count--)
//     {
//        DELAY_1_MCU_CLOCK;
//        CLK_fall;
//        DELAY_1_MCU_CLOCK;
//        data = data << 1;
//        if (DATA_IN_state != 0)
//           data = data | 0x01;
//        CLK_rise;
//     }
//     data = data ^ ft_data_inversion_mask;
//  }
static uint8_t FT_IF_shift_from_switches (void)
{
   uint8_t data;
   asm volatile (
   //                   for(uint8_t count = 8; ...; ...)
   //                   {
   "\n\t"               "LDI  r25, 8"
   //                   CLK_fall;
   "\n\t" "L_loop%=:"
   "\n\t"               "NOP"
   "\n\t"               "CBI  %[portd], %[clk_bit]"
   //                   DELAY_1_MCU_CLOCK;
   "\n\t"               "NOP"
   //                   data = data << 1;
   "\n\t"               "LSL  %[shifter]"
   //                   if (DATA_IN_state != 0)
   //                      data = data | 0x01;
   "\n\t"               "SBIC %[pind], %[di_bit]"
   "\n\t"               "ORI  %[shifter], 0x01"
   //                   CLK_rise;
   "\n\t"               "SBI  %[portd], %[clk_bit]"
   //                   for(...; ...; count--)
   "\n\t"               "DEC  r25"
   //                   for(...; count > 0; ...)
   "\n\t"               "BRNE L_loop%="
   //                   }
   :  [shifter]         "=&a" (data) // by convention r24 takes the 8-bit return value
   :  [pind]            "I"   (_SFR_IO_ADDR(PIND)),
      [portd]           "I"   (_SFR_IO_ADDR(PORTD)),
      [clk_bit]         "I"   (CLK),
      [di_bit]          "I"   (DATA_IN)
   :  "r25"
   );
   data = data ^ ft_data_inversion_mask;
   return (data);
   // 106 clocks @ 8MHz = 13250ns (incl. RCALL und RET)
   // Even at 400kHz bus speed this works fast enough to not cause extra waiting
   // time.
}


// ============================================================================
// Control motor driver stages (M1..M4)
// ============================================================================

/// \brief
/// Generate LOAD_OUT pulse.
/// \detail
/// Generate the LOAD_OUT pulse to latch data from the '4094 shifter chain
/// to motor stages simultanously. According to '4094 datasheet the LOAD_OUT
/// pulse just needs to be at least 200ns width.
/// - Takes  500ns @ 8MHz
static inline void FT_IF_latch_to_motors (void) __attribute__((always_inline));
void FT_IF_latch_to_motors (void)
{
   LOAD_OUT_rise;
   LOAD_OUT_fall;
   // 4 clocks @ 8MHz =  500ns
}


/// \brief
/// Shift out one byte to the '4094 shifter chain.
/// \detail
/// Shift out the byte given to the '4094 shifter chain.
/// The MSB gets out first, all trailing bits down to the LSB thereafter.
/// The CLK line is assumed to be '1' when transfer starts and left at '1' 
/// when transfer is done.
/// \param data gets shifted out.
//  '4094: Relevant datasheet parameters similar with Motorola and Fairchild.
//  Minimum Strobe Pulse time = 200ns @ 5V
//  Minimum Clock Pulse Width = 200ns @ 5V
//  Data Setup time D to Clock = 80ns @ 5V
//  Clock frequency = 1.5MHz @ 5V
//  @ 8MHz:
//    CLOCK pulse high = 625ns (750ns) / 1250ns (1375ns)
//    CLOCK pulse low = 500ns (625ns)
//    provide DATA_OUT 250ns before posedge CLOCK
static void FT_IF_shift_to_motors (uint8_t data)
{
   // for (uint8_t count = 4; count > 0; count--)
   // {
   //    if (data >= 0b11000000)
   //       data &= protect_motors;
   //    CLK_fall;
   //    if (data & 0x80)
   //       DATA_OUT_hi;
   //    else
   //       DATA_OUT_lo;
   //    data = data << 1;
   //    CLK_rise;
   //    DELAY_1_MCU_CLOCK;
   //    DELAY_1_MCU_CLOCK;
   //    CLK_fall;
   //    if (data & 0x80)
   //       DATA_OUT_hi;
   //    else
   //       DATA_OUT_lo;
   //    data = data << 1;
   //    CLK_rise;
   // }
   // Assembly code is optimized for speed and works a sligthly different way!
   // With this trick each clock high and low time is at least 2 clocks wide.
   // Also timing is kept balanced which reduces EMC issues.
   asm volatile (
   //                   for (uint8_t count = 4; ...; ...)
   "\n\t"               "LDI  r25, 4"
   "\n\t" "L_loop%=:"       
   "\n\t"               "CPI  %[shifter], 0b11000000"
   "\n\t"               "BRLO L_skip%="
   "\n\t"               "AND  %[shifter], %[mot_prot]"
   "\n\t" "L_skip%=:"
   "\n\t"               "SBRC %[shifter], 7"
   "\n\t"               "SBI  %[portd], %[do_bit]"
   "\n\t"               "CBI  %[portd], %[clk_bit]"
   "\n\t"               "SBRS %[shifter], 7"
   "\n\t"               "CBI  %[portd], %[do_bit]"
   "\n\t"               "SBI  %[portd], %[clk_bit]"
   "\n\t"               "ROL  %[shifter]"
   "\n\t"               "SBRC %[shifter], 7"
   "\n\t"               "SBI  %[portd], %[do_bit]"
   "\n\t"               "CBI  %[portd], %[clk_bit]"
   "\n\t"               "SBRS %[shifter], 7"
   "\n\t"               "CBI  %[portd], %[do_bit]"
   "\n\t"               "SBI  %[portd], %[clk_bit]"
   "\n\t"               "ROL  %[shifter]"
   //                   for (uint8_t ...; ...; count--)
   "\n\t"               "DEC  r25"
   //                   for (uint8_t ...; count > 0; ...)
   "\n\t"               "BRNE L_loop%="
   :
   :  [shifter]         "a"   (data),
      [mot_prot]        "r"   (ft_motor_protection),
      [pind]            "I"   (_SFR_IO_ADDR(PIND)),
      [portd]           "I"   (_SFR_IO_ADDR(PORTD)),
      [clk_bit]         "I"   (CLK),
      [do_bit]          "I"   (DATA_OUT)
   :  "r25"
   // 111 clocks @ 8MHz = 13875ns (incl. RCALL und RET)
   // Even at 400kHz bus speed this works fast enough to not cause extra waiting
   // time.
   );
}


// ============================================================================
// Initialization / Identification
// ============================================================================

/// \brief
/// Identify fischertechnik Computing Interface by its response. Also serves as
/// necessary init of some - but not all - properties.
/// \returns ID of interface detected.
uint8_t FT_IF_init_and_identify (void)
{
   // Basic hardware inits for analog readings
   // ----------------------------------------
   // Set up analog comparator to use external reference
   ACSR = 0;
   DIDR = (0b11<<AIN0D);
   // Set up timer 0 to just count external rising edges
   TCCR0A = 0;
   TCCR0B = (0b111<<CS00);
   TIFR = (1<<TOV0);
   TCNT0 = 0;
   // Set up '4014, assume a max of 8 interfaces in daisychain!
   // Since this also turns on WD and WD-LED in interface for 0.5s preload the
   // '4094s to drive '0' to the motors - stopping them all effectively.
   DATA_OUT_lo;
   for (uint8_t count = 8; count > 0; count--)
   {
      FT_IF_shift_from_switches();
   }
   FT_IF_latch_to_motors();

   // The 555 MMVs are said to be automatically triggered on power up. Since
   // there is no way to stop a running MMV in a fischertechnik Computing
   // Interface, we have to deal with the situation!

   // Find out about interface connected - at least try it
   // ----------------------------------------------------
   // Assume that the MMVs are *not* running
   // --------------------------------------
   //                   30561 30562 30563 30565 30566
   //                         30564       30567 30520
   //                                           39319
   //                                           66843
   // (ACSR & (1<<ACO))     0     0     0     0     0
   // DATA_IN               ?     ?     ?     *     ?
   // MMV_PULSES            1     1     1     1     0
   // COUNT_IN              0     0     1     1     0
   // * depends on '4014 status, shall be 1 if ready!
   //
   // And while triggered we have this
   // --------------------------------
   //                   30561 30562 30563 30565 30566
   //                         30564       30567 30520
   //                                           39319
   //                                           66843
   // (ACSR & (1<<ACO))     1     1     0     1     1
   // DATA_IN               ?     ?     ?     0     ?
   // MMV_PULSES            1     1     1     1     1
   // COUNT_IN              1     *     1     1     0
   // * due to pulses this will toggle and T0 gets counting up
   //
   // To not get too cryptical:
   // * Trigger EX reading (already done by power up, but we want to be sure!)
   // * Wait a certain time to allow for proper settling of inputs
   // * Check for properties expected and drop interfaces not matching
   // * Wait sufficiently long enough to get MMVs finished
   // * Then check for certain properties when untriggered
   // * Sort out interfaces not valid with their properties

   // * Trigger EX reading and wait ample time before doublechecking
   TRIGGER_MMV(FT_IF_EX);
   _delay_us(ID_sampling_after_us);
   uint8_t comp = COMPARATOR_state;
   TRIGGERs_release;

   // * Start identification trial
   if (!(comp))
   {  // Only 30563 provides this response when triggered by V. I. P.!
      ft_data_inversion_mask = DATA_IN_native;
      ft_motor_protection = protect_motors;
      ft_identifier = FT_IF_30563;
      // No further action required, immediately proceed with application
   }
   else
   {  // In no case it is the 30563 here, so sort out the remaining 9 ones
      if (COUNT_IN_count == 0)
      {  // No edges on COUNT_IN sorts out 30562, 30564, 30566, 39319, 30520 and 66843 just leaving 30561, 30565 or 30567
         if (DATA_IN_state)
         {  // DATA_IN at '1' here means we definitely do not have a 30565 and also no 30567
            // Just left: 30561
            ft_data_inversion_mask = DATA_IN_native;
            ft_motor_protection = protect_motors;
            ft_identifier = FT_IF_30561;
         }
         else
         {  // But we can not be sure what it tells if DATA_IN_state = '0'
            // For this to get we need to wait on the end of the MMV cycle
            _delay_ms(MMV_worst_100k_ms);
            if (DATA_IN_state)
            {  // Rising edge on DATA_IN kicks 30561 from list
               // Leaving 30565 or 30567 - can not tell better
               ft_data_inversion_mask = DATA_IN_inverted;
               ft_motor_protection = protect_motors;
               ft_identifier = FT_IF_30565;
            }
            else
            {  // No change at DATA_IN kicks 30565 and 30567
               // Just left: 30561
               ft_data_inversion_mask = DATA_IN_native;
               ft_motor_protection = protect_motors;
               ft_identifier = FT_IF_30561;
            }
         }
      } // if (COUNT_IN_count == 0)
      else
      {  // Edges on COUNT_IN kick off 30561, 30565 and 30567 (30563 we already
         // excluded) leaving 30562, 30564, 30566, 39319, 30520 or 66843
         // To get the difference between groups 30562 / 30564 and 30566 / 39319 / 30520 / 66843
         // we need to wait for the end of the MMV cycle and check MMV_PULSES state
         _delay_ms(MMV_worst_100k_ms);
         // 50566, 39319, 30520 and 66843 now are timed out
         if (MMV_PULSES_state)
         {  // MMV_PULSES still is at '1' => not 30566, 39319, 30520 or 66843
            // Leaving just 30562 or 30564 - can not tell better
            ft_data_inversion_mask = DATA_IN_native;
            ft_motor_protection = protect_motors;
            ft_identifier = FT_IF_30562;
         }
         else
         {  // MMV_PULSES at '0' is not valid with 30562 and 30564
            // Just leaving 30566, 39319, 30520 or 66843
            ft_data_inversion_mask = DATA_IN_native;
            ft_motor_protection = no_motor_protection;
            ft_identifier = FT_IF_30566;
         }
      }
   }
   return (ft_identifier);
}


/// \brief
/// Return interface identifier. Without new identification trial.
/// \returns ID of interface detected.
static inline uint8_t FT_IF_get_identifier (void) __attribute__((always_inline));
uint8_t FT_IF_get_identifier (void)
{
   return (ft_identifier);
}


/******************************************************************************\
 ******************************************************************************
 *
 *                              I²C bus response
 *                              ================
 *
 ******************************************************************************
\******************************************************************************/


// ============================================================================
// Some macros for easier coding
// ============================================================================
// Note: Do not rely on the compiler - force optimum code where necessary!

#define I2C_DEFAULT_DDRB      ((0<<TWIsdaBit) | (0<<TWIsclBit)))
#define I2C_DEFAULT_PORTB     ((0<<TWIsdaBit) | (0<<TWIsclBit)))
#define I2C_DEFAULT_DDRD      ((0<<SJ2) | (0<<SJ1))
#define I2C_DEFAULT_PORTD     ((1<<SJ2) | (1<<SJ1))

#define SCL_gets_input        asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(TWIddr)), [bit] "I" (TWIsclBit))
#define SCL_gets_output       asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(TWIddr)), [bit] "I" (TWIsclBit))
#define SDA_gets_input        asm volatile ("CBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(TWIddr)), [bit] "I" (TWIsdaBit))
#define SDA_gets_output       asm volatile ("SBI %[port], %[bit]"::[port] "I" (_SFR_IO_ADDR(TWIddr)), [bit] "I" (TWIsdaBit))

#define USI_START_IRQ_ONLY    (1<<USISIE) | (0<<USIOIE) | (0b10<<USIWM0) | (0b10<<USICS0) | (0b0<<USICLK)
#define USI_START_OVF_IRQ     (1<<USISIE) | (1<<USIOIE) | (0b11<<USIWM0) | (0b10<<USICS0) | (0b0<<USICLK)
#define USI_CLEAR_ALL         (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC)
#define USI_CLEAR_OVF_STOP_DC (0<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC)
#define USI_RELEASE_1_BIT     (1<<USIOIF) | 14
#define USI_RELEASE_8_BIT     (1<<USIOIF)

#define SREG_SAVE             GPIOR0
#define IRQ_STACK_1           GPIOR1
#define IRQ_STACK_2           GPIOR2


// ============================================================================
// Enumerations for better code readability
// ============================================================================

enum
{  // Bus states for application usage
   I2C_IDLE = 0,
   I2C_ACCESSING
};


enum
{  // Buffer stati
   I2C_BUF_EMPTY = 0,
   I2C_BUF_FULL
};


enum
{  // Register Addresses and R/W Control
   I2C_WR_REG_0 = 0,
   I2C_RD_REG_0 = 1,
   I2C_WR_REG_1 = 2,
   I2C_RD_REG_1 = 3
};


/// \brief
/// Some private states for I²C protocol handling.
enum
{
   I2C_STATE_is_off = 0,

   I2C_STATE_wait4start,

   I2C_STATE_receiving_address,
   I2C_STATE_confirm_read,
   I2C_STATE_confirm_write,

   I2C_STATE_wait4putdata,
   I2C_STATE_sending_data,
   I2C_STATE_reading_ack,

   I2C_STATE_receiving_data,
   I2C_STATE_wait4getdata,

   I2C_STATE_sending_ack = I2C_STATE_confirm_write,   // this is intended!
   I2C_STATE_addressed = I2C_STATE_confirm_read       // this is intended!
};


// ============================================================================
// Variable definitions
// ============================================================================

uint8_t  I2C_my_base_address = 0;

uint8_t  I2C_reg_0_1st_return = 0;
uint8_t  I2C_reg_1_1st_return = 0;

volatile uint8_t  I2C_RxFlag = I2C_BUF_EMPTY;
volatile uint8_t  I2C_TxFlag = I2C_BUF_EMPTY;
volatile uint8_t  I2C_RxBuf  = 0;
volatile uint8_t  I2C_TxBuf  = 0;


// ============================================================================
// Turning bus interface on and off
// ============================================================================

/// \brief
/// Sets up the USI to service the I²C physical layer as a slave device.
void I2C_Setup (void)
{
   // Please note: setup does not disrupt any I²C transfer
   cli();
   // Find out about the selected I²C base address
   switch(PIND & ((1<<SJ2) | (1<<SJ1)))
   {
      case  0:
            I2C_my_base_address = (0x3E<<1);
            break;
      case  (1<<SJ1):
            I2C_my_base_address = (0X3C<<1);
            break;
      case  (1<<SJ2):
            I2C_my_base_address = (0x26<<1);
            break;
      case  ((1<<SJ2) | (1<<SJ1)):
      default:
            I2C_my_base_address = (0x24<<1);
   }
   I2C_RxFlag = I2C_BUF_EMPTY;
   I2C_TxFlag = I2C_BUF_EMPTY;
   I2C_private_state = I2C_STATE_wait4start;
   TWIddr &= ~((1<<TWIsclBit) | (1<<TWIsdaBit));
   USICR = USI_START_IRQ_ONLY;
   USISR = USI_CLEAR_ALL;
   TWIport |= (1<<TWIsclBit) | (1<<TWIsdaBit);
   SCL_gets_output;
   sei();
}


/// \brief
/// Sets up the USI to not react on subsequent I²C transactions.
void I2C_Retire (void)
{
   cli();
   USICR = 0;
   USISR = USI_CLEAR_ALL;
   TWIddr &= ~((1<<TWIsdaBit) | (1<<TWIsclBit));
   TWIport &= ~((1<<TWIsclBit) | (1<<TWIsdaBit)); // prevent pull-up!
   I2C_private_state = I2C_STATE_is_off;
}


// ============================================================================
// Cooperation
// ============================================================================

/// \brief
/// Get the actual state of the bus protocol handling.
/// \detail
///
/// \returns one of the predefined states.
//  Compiler optimization already fine. No need to use assembly tricks.
//  But check for every release build to be sure or code inline!
uint8_t I2C_GetState (void)
{
   if (I2C_private_state < I2C_STATE_addressed)
      return (I2C_IDLE);
   else
      return (I2C_ACCESSING);
}


/// \brief
/// Set the very first byte to reply after addressed for reading reg 0.
/// \detail
/// 
/// \param data is provided as first reply to the bus when accessed reading.
static inline void I2C_Set_Early_Reply_Reg_0 (uint8_t data) __attribute__((always_inline));
void I2C_Set_Early_Reply_Reg_0 (uint8_t data)
{
   I2C_reg_0_1st_return = data;
}


/// \brief
/// Set the very first byte to reply after addressed for reading reg 1.
/// \detail
/// 
/// \param data is provided as first reply to the bus when accessed reading.
static inline void I2C_Set_Early_Reply_Reg_1 (uint8_t data) __attribute__((always_inline));
void I2C_Set_Early_Reply_Reg_1 (uint8_t data)
{
   I2C_reg_1_1st_return = data;
}


// ============================================================================
// Addressfield handling
// ============================================================================

/// \brief
/// Just get and return the relevant address field data.
/// \detail
/// Waits for an address to receive and returns back what the ISR provides.
/// Returns only in case a valid address has been received. Other addresses
/// are rejected by ISR and waiting commences.
/// \returns just the 2 necessary bits according to I²C definitions:
/// 0, 0, 0, 0, 0, 0, A0, R/W
static inline uint8_t I2C_GetAddress (void) __attribute__((always_inline));
uint8_t I2C_GetAddress (void)
{
   while (I2C_private_state < I2C_STATE_addressed) {}
   return (I2C_address_received);
   // 3 clocks per loop means it takes worst case 5 clocks to detect a match
   // after RETI from USIOIF-ISR
}


// ============================================================================
// Data exchange (*data* - not address )
// ============================================================================

/// \brief
/// Get the actual data byte from the bus.
/// \detail
/// This function shall be used to get each byte the bus master sends by bus
/// write accesses.
/// Waits for a byte to receive if there is not already one in
/// the receive buffer.
/// Waits for the receive buffer to get full. If so gets the databyte from the
/// buffer, sets the flag to empty. When done checks explicitly for state
/// I2C_STATE_wait4getdata. If this occurs the get has been too late and the
/// next transfer needs a manual trigger. Otherwise the ISR has done everything
/// already so leave it!
/// Due to the usage of this buffer the application has a full byte-transaction-
/// time for its response. This should be way enough, especially at 100kHz.
/// \returns the byte received from the bus master.
//   while (I2C_RxFlag == I2C_BUF_EMPTY)
//   {
//      // Return if state changed due to START cycle
//      if (I2C_private_state < I2C_STATE_addressed)
//         return (0);
//      // Check for STOP cycle
//      if ((USISR & (1<<USIPF)) != 0)
//      {
//         cli();
//         if (I2C_private_state != I2C_STATE_receiving_address)
//         {
//            USICR = USI_START_IRQ_ONLY;
//            USISR = USI_CLEAR_OVF_STOP_DC; // Leave USISIF if already set!!!
//            I2C_private_state = I2C_STATE_wait4start;
//         }
//         sei();
//         return (0);
//      }
//   }
//   uint8_t data = I2C_RxBuf;
//   I2C_RxFlag = I2C_BUF_EMPTY;
//   if (I2C_private_state == I2C_STATE_wait4getdata)
//   {
//      USIDR = 0; // prepare ACK reply
//      SDA_gets_output;
//      USISR = USI_RELEASE_1_BIT;
//      I2C_private_state = I2C_STATE_sending_ack;
//   }
//  Compiler optimization copies r16 to r?? once initially and then never
//  updates it while looping => WRONG! Also copying r16 to r?? is waste. So
//  here we need to do it the forcible way. This is very ugly but it does not
//  work out otherwise.
uint8_t I2C_GetData (void)
{
   uint8_t data;
   // r16 directly holds I2C_private_state refreshed by the ISRs
   asm volatile (
   "\n" "L_loop%=:"
   //                   while (I2C_RxFlag == I2C_BUF_EMPTY)
   "\n\t"               "LDS  r24, I2C_RxFlag"
   "\n\t"               "CPI  r24, %[empty_val]"
   "\n\t"               "BRNE L_leave%="
   //                   if (I2C_private_state < I2C_STATE_addressed)
   "\n\t"               "CPI  %[state], %[bus_addressed]"
   "\n\t"               "BRLO L_exit%="
   //                   if ((USISR & (1<<USIPF)) != 0)
   "\n\t"               "SBIS %[usisr_addr], %[usipf]"
   "\n\t"               "RJMP L_loop%="
   //                   cli();
   "\n\t"               "CLI"
   //                   if (I2C_private_state != I2C_STATE_receiving_address)
   "\n\t"               "CPI  %[state], %[bus_wait4addr]"
   "\n\t"               "BREQ L_exit_sei%="
   //                   USICR = USI_START_IRQ_ONLY;
   "\n\t"               "LDI  r24, %[usicr_val]"
   "\n\t"               "OUT  %[usicr_addr], r24"
   //                   USISR = USI_CLEAR_OVF_STOP_DC;
   "\n\t"               "LDI  r24, %[usisr_wipe2]"
   "\n\t"               "OUT  %[usisr_addr], r24"
   //                   I2C_private_state = I2C_STATE_wait4start;
   "\n\t"               "LDI  %[retstate], %[bus_wait4start]"
   "\n" "L_exit_sei%=:"
   //                   sei();
   "\n\t"               "SEI"
   "\n" "L_exit%=:"
   //                   return (0);
   "\n\t"               "LDI  r24, 0"
   "\n\t"               "RET"
   "\n" "L_leave%=:"
   :  [retstate]        "=a"  (I2C_private_state)
   :  [bus_wait4start]  "M"   (I2C_STATE_wait4start),
      [bus_wait4addr]   "M"   (I2C_STATE_receiving_address),
      [bus_addressed]   "M"   (I2C_STATE_addressed),
      [empty_val]       "M"   (I2C_BUF_EMPTY),
      [usisr_wipe2]     "M"   (USI_CLEAR_OVF_STOP_DC),
      [usicr_val]       "M"   (USI_START_IRQ_ONLY),
      [usicr_addr]      "I"   (_SFR_IO_ADDR(USICR)),
      [usisr_addr]      "I"   (_SFR_IO_ADDR(USISR)),
      [usipf]           "I"   (USIPF),
      [state]           "a"   (I2C_private_state)
   :  "r24"
   );
   data = I2C_RxBuf;
   I2C_RxFlag = I2C_BUF_EMPTY;
   if (I2C_private_state == I2C_STATE_wait4getdata)
   {
      USIDR = 0; // prepare ACK reply
      SDA_gets_output;
      USISR = USI_RELEASE_1_BIT;
      I2C_private_state = I2C_STATE_sending_ack;
   }

   return (data);
}


/// \brief
/// Define data to be sent on the bus.
/// \detail
/// This function shall be used to provide each byte the bus master requests by
/// bus read accesses.
/// Waits for the transmit buffer to get empty. If so puts the databyte to the
/// buffer, sets the flag to full. When done checks explicitly for state
/// I2C_STATE_wait4putdata. If this occurs the put has been too late and the
/// next transfer needs a manual trigger. Otherwise the ISR has done everything
/// already so leave it!
/// Due to the usage of this buffer the application has a full byte-transaction-
/// time for its response. This should be way enough, especially at 100kHz.
/// \param data is the byte put to the bus.
//   while (I2C_TxFlag == I2C_BUF_FULL)
//   {
//      // Return if state changed due to START cycle
//      if (I2C_private_state < I2C_STATE_addressed)
//         return;
//      // Check for STOP cycle
//      if ((USISR & (1<<USIPF)) != 0)
//      {
//         cli();
//         if (I2C_private_state != I2C_STATE_receiving_address)
//         {
//            USICR = USI_START_IRQ_ONLY;
//            USISR = USI_CLEAR_OVF_STOP_DC; // Leave USISIF if already set!!!
//            I2C_private_state = I2C_STATE_wait4start;
//         }
//         sei();
//         return;
//      }
//   }
//  Compiler optimization copies r16 to r?? once initially and then never
//  updates it while looping => WRONG! Also copying r16 to r?? is waste. So
//  here we need to do it the forcible way. This is very ugly but it does not
//  work out otherwise.
void I2C_PutData (uint8_t data)
{
   // By convention data arrives in r24? At least seems so.
   // r16 directly holds I2C_private_state refreshed by the ISRs
   asm volatile (
   "\n" "L_loop%=:"
   //                   while (I2C_TxFlag == I2C_BUF_FULL)
   "\n\t"               "LDS  r24, I2C_TxFlag"
   "\n\t"               "CPI  r24, %[full_val]"
   "\n\t"               "BRNE L_leave%="
   //                   if (I2C_private_state < I2C_STATE_addressed)
   "\n\t"               "CPI  %[state], %[bus_addressed]"
   "\n\t"               "BRLO L_exit%="
   //                   if ((USISR & (1<<USIPF)) != 0)
   "\n\t"               "SBIS %[usisr_addr], %[usipf]"
   "\n\t"               "RJMP L_loop%="
   //                   cli();
   "\n\t"               "CLI"
   //                   if (I2C_private_state != I2C_STATE_receiving_address)
   "\n\t"               "CPI  %[state], %[bus_wait4addr]"
   "\n\t"               "BREQ L_exit_sei%="
   //                   USICR = USI_START_IRQ_ONLY;
   "\n\t"               "LDI  r24, %[usicr_val]"
   "\n\t"               "OUT  %[usicr_addr], r24"
   //                   USISR = USI_CLEAR_OVF_STOP_DC;
   "\n\t"               "LDI  r24, %[usisr_wipe2]"
   "\n\t"               "OUT  %[usisr_addr], r24"
   //                   I2C_private_state = I2C_STATE_wait4start;
   "\n\t"               "LDI  %[retstate], %[bus_wait4start]"
   "\n" "L_exit_sei%=:"
   //                   sei();
   "\n\t"               "SEI"
   "\n" "L_exit%=:"
   //                   return;
   "\n\t"               "RET"
   "\n" "L_leave%=:"
   :  [retstate]        "=a"  (I2C_private_state)
   :  [bus_wait4start]  "M"   (I2C_STATE_wait4start),
      [bus_wait4addr]   "M"   (I2C_STATE_receiving_address),
      [bus_addressed]   "M"   (I2C_STATE_addressed),
      [full_val]        "M"   (I2C_BUF_FULL),
      [usisr_wipe2]     "M"   (USI_CLEAR_OVF_STOP_DC),
      [usicr_val]       "M"   (USI_START_IRQ_ONLY),
      [usicr_addr]      "I"   (_SFR_IO_ADDR(USICR)),
      [usisr_addr]      "I"   (_SFR_IO_ADDR(USISR)),
      [usipf]           "I"   (USIPF),
      [state]           "a"   (I2C_private_state)
   :  "r24"
   );
   I2C_TxBuf = data;
   I2C_TxFlag = I2C_BUF_FULL;
   if (I2C_private_state == I2C_STATE_wait4putdata)
   {
      USIDR = data;
      SDA_gets_output;
      USISR = USI_RELEASE_8_BIT;
      I2C_TxFlag = I2C_BUF_EMPTY;
      I2C_private_state = I2C_STATE_sending_data;
   }
}


// ============================================================================
// Interrupt services
// ============================================================================

/// \brief
/// Sync to the start bit and denote the following byte as addressfield.
/// \detail
/// When USISIF in USISR gets set this indicates a START condition. The bus is
/// blocked by clock stretching until USISIF is cleared. To avoid transfer rate
/// loss this is handled by an ISR: The USISIF is cleared, the USI is prepared
/// to receive one byte and block the transfer again thereafter. The internal
/// operation state is set accordingly to have the byte indicated as ADDRESS
/// information, not as payload data.
/// Note the change in USIWM[1:0] turning on USIOIF blocking!
//  =>
//  ISR(USI_START_vect)
//  {
//     USICR = USI_START_OVF_IRQ;
//     USISR = USI_CLEAR_ALL;
//     I2C_private_state = I2C_STATE_receiving_address;
//  }
//  Analysing the compiled assembly file:
//  => Compiler produces much overhead on pushing and popping registers never
//  used! Even the rare fact that the SREG is not touched is not taken for a
//  proper optimization effort. => Recode using assembly language!
//  Now we get:
//  ISR itself = 9 clock ticks
//  ISR call overhead = 7 clock ticks (MCU hardware!)
//  Uncertainty = 2 clock ticks (due to interruption of a running instruction)
//  Assume a total of 19 clocks to be sure!
//  @ 8MHz (worst case):
//    complete IRQ service takes 2375ns
//    SCL released after 14 clocks = 1750ns
//  400kHz-bus just gives 1250ns for SCL release w/o clock-stretching.
//  100kHz-bus will give 5000ns for SCL release w/o clock-stretching.
//  According to ATtiny2313 datasheet the USISIF reacts already on the falling
//  edge of SDA after a delay of max. 300ns. This in mind SCL needs to stay
//  high for at least those 300ns and usually it takes longer for SCL to fall.
//  So most likely the clock-stretching will not occur even at the 400kHz bus
//  clocking. But in the very worst case:
//  Run ATtiny @ 8MHz and bus @ 100kHz or clock-stretching is mandatory for the
//  master - assuming the USISIF gets set with SCL falling edge.
//
//  Note: 'state' is reserved for ISR usage. Use as clobber before finally
//  adjusting the state information. Saves PUSH and POP of another register.
ISR (USI_START_vect, ISR_NAKED)
{
   asm volatile (
   // up to here we assume 10 clocks (7 according to datasheet plus uncertainty)
   //                   USICR = USI_START_OVF_IRQ;
   "\n\t"               "LDI  %[clobber], %[usicr_val]"
   "\n\t"               "OUT  %[usicr_addr], %[clobber]"
   //                   USISR = USI_CLEAR_ALL;
   "\n\t"               "LDI  %[clobber], %[usisr_val]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 14 clocks until SCL release (total) @ 8MHz = 1750ns
   //(66 clocks margin until SCL rise - SDA is ahead roughly 5µs!!))
   //  5 clocks ahead until RETI done
   //----
   // 19 total => 661 clocks left until next IRQ @ 100kHz bus (at least 621)
   //                   I2C_private_state = I2C_STATE_receiving_address;
   "\n\t"               "LDI  %[retstate], %[next_state]"
   "\n\t"               "RETI"
   :  [retstate]        "=a"  (I2C_private_state)
   :  [usicr_val]       "M"   (USI_START_OVF_IRQ),
      [usicr_addr]      "I"   (_SFR_IO_ADDR(USICR)),
      [usisr_val]       "M"   (USI_CLEAR_ALL),
      [usisr_addr]      "I"   (_SFR_IO_ADDR(USISR)),
      [next_state]      "M"   (I2C_STATE_receiving_address),
      [clobber]         "a"   (I2C_private_state) // see note above!
   );
}


/// \brief
/// Control transfer handling, semi-automatic.
/// \detail
/// When USIOIF in USISR gets set this indicates that the transfer has been
/// finished. The internal state is adjusted accordingly to denote the event.
/// In some cases the subsequent action is automatically triggered. In other
/// cases nothing happens and the application has to react.
//  => The order inside the switch() is currently very important for timing!
//  ISR(USI_OVERFLOW_vect)
//  {
//    switch(I2C_private_state)
//    {
//       case  I2C_STATE_receiving_address:
//             I2C_address_received = USIDR;
//             if ((I2C_address_received & 0xFC) == I2C_my_base_address)
//             {
//                // accept matching address
//                USIDR = 0; // prepare ACK reply
//                SDA_gets_output;
//                USISR = USI_RELEASE_1_BIT;
//                I2C_address_received = I2C_address_received & 0x03
//                I2C_private_state = I2C_STATE_confirm_write;
//                if ((I2C_address_received & 0x01) != 0)
//                   I2C_private_state = I2C_STATE_confirm_read;
//             }
//             else
//             {
//             // reject other address
//                SDA_gets_input;
//                USICR = USI_START_IRQ_ONLY;
//                USISR = USI_CLEAR_ALL;
//                I2C_private_state = I2C_STATE_wait4start;
//             }
//             break;
//       case  I2C_STATE_confirm_read;
//             uint8_t data = I2C_reg_0_1st_return;
//             if ((I2C_address_received & 0x02) != 0)
//                data = I2C_reg_1_1st_return;
//             USIDR = data;
//             SDA_gets_output;
//             USISR = USI_RELEASE_8_BIT;
//             I2C_private_state = I2C_STATE_sending_data;
//             break;
//       case  I2C_STATE_reading_ack:
//             if ((USIDR & __twiNoAck__) == 0)
//             {
//                if (I2C_TxFlag != I2C_BUF_EMPTY)
//                {
//                   USIDR = I2C_TxBuf;
//                   SDA_gets_output;
//                   USISR = USI_RELEASE_8_BIT;
//                   I2C_TxFlag = I2C_BUF_EMPTY;
//                   I2C_private_state = I2C_STATE_sending_data;
//                }
//                else
//                   I2C_private_state = I2C_STATE_wait4putdata;
//             }
//             else
//             {  // Master sent NoACK to end communication!!
//                USICR = USI_START_IRQ_ONLY;
//                USISR = USI_CLEAR_ALL;
//                I2C_private_state = I2C_STATE_wait4start;
//             }
//       case  I2C_STATE_receiving_data:
//             if (I2C_RxFlag != I2C_BUF_FULL)
//             {
//                I2C_RxBuf = USIDR;
//                USIDR = 0; // prepare ACK reply
//                SDA_gets_output;
//                USISR = USI_RELEASE_1_BIT;
//                I2C_RxFlag = I2C_BUF_FULL;
//                I2C_private_state = I2C_STATE_sending_ack;
//             }
//             else
//                I2C_private_state = I2C_STATE_wait4getdata;
//             break;
//       case  I2C_STATE_sending_data:
//             SDA_gets_input;
//             USISR = USI_RELEASE_1_BIT;
//             I2C_private_state = I2C_STATE_reading_ack;
//             break;
//       case  I2C_STATE_sending_ack:
//       case  I2C_STATE_confirm_write:
//             SDA_gets_input;
//             USISR = USI_RELEASE_8_BIT;
//             I2C_private_state = I2C_STATE_receiving_data;
//             break;
//       default:
//             ;
//  }
//  Analysing the compiled assembly file:
//  => Compiler native code is too much loaded with ballast.
//  400kHz-bus just gives 1250ns for SCL release w/o clock-stretching.
//  100kHz-bus will give 5000ns for SCL release w/o clock-stretching.
//  => Manual coding gives improved timing and shortened response.
//  Run ATtiny @ min. 8MHz and bus @ max. 100kHz or clock-stretching is a 'must
//  have' for the master.
//
//  Note: reg 'state' is reserved for state handling. Use as 'clobber' before
//  finally adjusting the state information. Saves PUSH and POP of another
//  register.
//  Note: reg 'address' is reserved for address field storage!
//  Note: GPIOR are used as private miniature stack for ISR purposes. Takes less
//  cycles than PUSH / POP (1 cycle less each).
ISR (USI_OVERFLOW_vect, ISR_NAKED)
{
   asm volatile (
   // up to here we assume 10 clocks (7 according to datasheet plus uncertainty)
   "\n\t"               "OUT  %[my_stack1], r18"
   "\n\t"               "IN   r18, __SREG__"
   "\n\t"               "OUT  %[my_stack0], r18"
   // 13 clocks now have passed

   //                   switch(I2C_private_state)

   //                   case  I2C_STATE_receiving_address:======================
   "\n" "L_receive_address%=:"
   "\n\t"               "CPI  %[state], %[bus_wait4addr]"
   "\n\t"               "BRNE L_read_ack%="
   //                   I2C_address_received = USIDR;
   "\n\t"               "IN   %[address], %[usidr_addr]"
   //                   if ((I2C_address_received & 0xFC) == I2C_my_base_address)
   "\n\t"               "MOV  r18, %[address]"
   "\n\t"               "ANDI r18, 0b11111100"
   "\n\t"               "LDS  %[clobber], I2C_my_base_address"
   "\n\t"               "CP   %[clobber], r18"
   "\n\t"               "BRNE L_nono%="
   "\n" "L_match%=:"    // accept matching address
   //                   USIDR = 0; // prepare ACK reply
   "\n\t"               "LDI  %[clobber], 0"
   "\n\t"               "OUT  %[usidr_addr], %[clobber]"
   //                   SDA_gets_output;
   "\n\t"               "SBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_1_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_1bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 28 clocks until SCL release for ACK (total) @ 8MHz = 3500ns
   //(12 clocks margin until SCL rise)
   // 13 clocks ahead until RETI done in case of read cycle
   // 13 clocks ahead until RETI done in case of write cycle
   //----
   // 41 total =>  39 clocks left until next IRQ @ 100kHz bus (read)
   // 41 total =>  39 clocks left until next IRQ @ 100kHz bus (write)
   //                   I2C_address_received = I2C_address_received & 0x03
   "\n\t"               "ANDI %[address], 0b00000011"
   //                   I2C_private_state = I2C_STATE_confirm_write;
   "\n\t"               "LDI  %[retstate], %[bus_write]"
   //                   if ((I2C_address_received & 0x01) != 0)
   "\n\t"               "SBRC %[address], 0"
   //                   I2C_private_state = I2C_STATE_confirm_read;
   "\n\t"               "LDI  %[retstate], %[bus_read]"
   //                   break;
   "\n\t"               "RJMP L_leave%="
   "\n" "L_nono%=:"     // reject wrong address
   //                   SDA_gets_input;
   "\n\t"               "CBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USICR = USI_START_IRQ_ONLY;
   "\n\t"               "LDI  %[clobber], %[usicr_val]"
   "\n\t"               "OUT  %[usicr_addr], %[clobber]"
   //                   USISR = USI_CLEAR_ALL;
   "\n\t"               "LDI  %[clobber], %[usisr_wipe]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 29 clocks until SCL release for NACK (total) @ 8MHz = 3625ns
   //(11 clocks margin until SCL rise)
   // 10 clocks ahead until RETI done
   //----
   // 39 total =>  41 clocks left until end of NACK plus some 'til next START
   //                   I2C_private_state = I2C_STATE_wait4start;
   "\n\t"               "LDI  %[retstate], %[bus_wait4start]"
   //                   break;
   "\n\t"               "RJMP L_leave%="

   // 16 clocks now have passed
   //                   case  I2C_STATE_reading_ack:============================
   "\n" "L_read_ack%=:"
   "\n\t"               "CPI  %[state], %[bus_wait4ack]"
   "\n\t"               "BRNE L_receiving_data%="
   //                   if ((USIDR & 0x01) == 0)
   "\n\t"               "IN   %[clobber], %[usidr_addr]"
   "\n\t"               "ANDI %[clobber], 0b00000001"
   "\n\t"               "BRNE L_noAck%="
   // 21 clocks now have passed
   //                   if (I2C_TxFlag != I2C_BUF_EMPTY)
   "\n\t"               "LDS  %[clobber], I2C_TxFlag"
   "\n\t"               "CPI  %[clobber], %[empty_val]"
   "\n\t"               "BREQ L_tx_empty%="
   //                   USIDR = I2C_TxBuf;
   "\n\t"               "LDS  %[clobber], I2C_TxBuf"
   "\n\t"               "OUT  %[usidr_addr], %[clobber]"
   //                   SDA_gets_output;
   "\n\t"               "SBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_8_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_8bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 31 clocks until SCL release (total) @ 8MHz = 3875ns
   //( 9 clocks margin until SCL rise)
   // 13 clocks ahead until RETI done
   //----
   // 44 total => 596 clocks left until next IRQ @ 100kHz bus
   //                   I2C_TxFlag = I2C_BUF_EMPTY;
   "\n\t"               "LDI  %[clobber], %[empty_val]"
   "\n\t"               "STS  I2C_TxFlag, %[clobber]"
   //                   I2C_private_state = I2C_STATE_sending_data;
   "\n\t"               "LDI  %[retstate], %[bus_sending]"
   "\n\t"               "RJMP L_leave%="
   //                   else
   "\n\t" "L_tx_empty%=:"
   // 25 clocks now have passed
   // 10 clocks ahead until RETI done
   //----
   // 35 total => 5 clocks left until SCL rise on 100kHz bus.
   // Delay now depends on the application to provide the next byte to the bus!
   // The bus is halted by SCL kept '0' (clock-stretching).
   // Remember: Application needs to start the submission in this case!
   //                   I2C_private_state = I2C_STATE_wait4putdata;
   "\n\t"               "LDI  %[retstate], %[bus_sent]"
   //                   "break";
   "\n\t"               "RJMP L_leave%="
   "\n" "L_noAck%=:"
   //       else        // Master sent NACK to end communication!
   //                   USICR = USI_START_IRQ_ONLY;
   "\n\t"               "LDI  %[clobber], %[usicr_val]"
   "\n\t"               "OUT  %[usicr_addr], %[clobber]"
   //                   USISR = USI_CLEAR_ALL;
   "\n\t"               "LDI  %[clobber], %[usisr_wipe]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 26 clocks until SCL release (total) @ 8MHz = 3250ns
   //(14 clocks margin until SCL rise)
   // 10 clocks ahead until RETI done
   //----
   // 36 total => >43 clocks left until next IRQ @ 100kHz bus
   //                   I2C_private_state = I2C_STATE_wait4start;
   "\n\t"               "LDI  %[retstate], %[bus_wait4start]"
   //                   break;
   "\n\t"               "RJMP L_leave%="

   // 19 clocks now have passed
   //                   case  I2C_STATE_receiving_data:=========================
   "\n" "L_receiving_data%=:"
   "\n\t"               "CPI  %[state], %[bus_receiving]"
   "\n\t"               "BRNE L_confirm_read%="
   //                   if (I2C_RxFlag != I2C_BUF_FULL)
   "\n\t"               "LDS  %[clobber], I2C_RxFlag"
   "\n\t"               "CPI  %[clobber], %[full_val]"
   "\n\t"               "BREQ L_rx_full%="
   //                   I2C_RxBuf = USIDR;
   "\n\t"               "IN   %[clobber], %[usidr_addr]"
   "\n\t"               "STS  I2C_RxBuf, %[clobber]"
   //                   USIDR = 0; // prepare ACK reply
   "\n\t"               "LDI  %[clobber], 0"
   "\n\t"               "OUT  %[usidr_addr], %[clobber]"
   //                   SDA_gets_output;
   "\n\t"               "SBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_1_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_1bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 34 clocks until SCL release (total) @ 8MHz = 4250ns
   //( 6 clocks margin until SCL rise)
   // 13 clocks ahead until RETI done
   //----
   // 47 total =>  33 clocks left until next IRQ @ 100kHz bus
   //                   I2C_RxFlag = I2C_BUF_FULL;
   "\n\t"               "LDI  %[clobber], %[full_val]"
   "\n\t"               "STS  I2C_RxFlag, %[clobber]"
   //                   I2C_private_state = I2C_STATE_sending_ack;
   "\n\t"               "LDI  %[retstate], %[bus_acking]"
   //                   break;
   "\n\t"               "RJMP L_leave%="
   //       else        
   "\n\t" "L_rx_full%=:"
   // 22 clocks now have passed
   // 10 clocks ahead until RETI done
   //----
   // 32 total => 8 clocks left until SCL rise on 100kHz bus.
   // Delay now depends on the application to provide the next byte to the bus!
   // The bus is halted by SCL kept '0' (clock-stretching).
   // Remember: Application needs to start the submission in this case!
   //                   I2C_private_state = I2C_STATE_wait4getdata;
   "\n\t"               "LDI  %[retstate], %[bus_received]"
   //                   break;
   "\n\t"               "RJMP L_leave%="

   // 22 clocks now have passed
   //                   case  I2C_STATE_confirm_read:===========================
   "\n" "L_confirm_read%=:"
   "\n\t"               "CPI  %[state], %[bus_read]"
   "\n\t"               "BRNE L_send_data%="
   // 24 clocks now have passed
   // 16 clocks left until application must provide the 1st byte and release SCL
   //                   uint8_t data = I2C_reg_0_1st_return;
   "\n\t"               "LDS  %[clobber], I2C_reg_0_1st_return"
   //                   if ((I2C_address_received & 0x02) != 0)
   "\n\t"               "SBRC %[address], 1"
   //                   data = I2C_reg_1_1st_return;
   "\n\t"               "LDS  %[clobber], I2C_reg_1_1st_return"
   //                   USIDR = data;
   "\n\t"               "OUT  %[usidr_addr], %[clobber]"
   //                   SDA_gets_output;
   "\n\t"               "SBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_8_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_8bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 34 clocks until SCL release (total) @ 8MHz = 4250ns
   //( 6 clocks margin until SCL rise)
   // 10 clocks ahead until RETI done
   //----
   // 44 total => 599 clocks left until next IRQ @ 100kHz bus
   //                   I2C_private_state = I2C_STATE_sending_data;
   "\n\t"               "LDI  %[retstate], %[bus_sending]"
   //                   "break";
   "\n\t"               "RJMP L_leave%="

   // 25 clocks now have passed
   // case              I2C_STATE_sending_data:=================================
   "\n" "L_send_data%=:"
   "\n\t"               "CPI  %[state], %[bus_sending]"
   "\n\t"               "BRNE L_confirm_write%="
   //                   SDA_gets_input;
   "\n\t"               "CBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_1_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_1bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 31 clocks until SCL release (total) @ 8MHz = 3875ns
   //( 9 clocks margin until SCL rise)
   // 10 clocks ahead until RETI done
   //----
   // 41 total =>  39 clocks left until next IRQ @ 100kHz bus
   //                   I2C_private_state = I2C_STATE_reading_ack;
   "\n\t"               "LDI  %[retstate], %[bus_wait4ack]"
   //                   break;
   "\n\t"               "RJMP L_leave%="

   // 28 clocks now have passed
   //                   case  I2C_STATE_sending_ack:============================
   //                   case  I2C_STATE_confirm_write:
   "\n" "L_confirm_write%=:"
   "\n\t"               "CPI  %[state], %[bus_acking]"
   "\n\t"               "BRNE L_leave%="
   //                   SDA_gets_input;
   "\n\t"               "CBI  %[twi_ddr], %[twi_sda_bit]"
   //                   USISR = USI_RELEASE_8_BIT;
   "\n\t"               "LDI  %[clobber], %[usisr_8bit]"
   "\n\t"               "OUT  %[usisr_addr], %[clobber]"
   // 34 clocks until SCL release (total) @ 8MHz = 4250ns
   //( 6 clocks margin until SCL rise)
   //  8 clocks ahead until RETI done
   //----
   // 42 total => 598 clocks left until next IRQ @ 100kHz bus
   //                   I2C_private_state = I2C_STATE_receiving_data;
   "\n\t"               "LDI  %[retstate], %[bus_receiving]"

   // still 7 clocks ahead until RETI done
   "\n" "L_leave%=:"
   "\n\t"               "IN   r18, %[my_stack0]"
   "\n\t"               "OUT  __SREG__, r18"
   "\n\t"               "IN   r18, %[my_stack1]"
   "\n\t"               "RETI"
   :  [retstate]        "=a"  (I2C_private_state)
   :  [bus_wait4start]  "M"   (I2C_STATE_wait4start),
      [bus_wait4addr]   "M"   (I2C_STATE_receiving_address),
      [bus_read]        "M"   (I2C_STATE_confirm_read),
      [bus_write]       "M"   (I2C_STATE_confirm_write),
      [bus_acking]      "M"   (I2C_STATE_sending_ack),
      [bus_receiving]   "M"   (I2C_STATE_receiving_data),
      [bus_received]    "M"   (I2C_STATE_wait4getdata),
      [bus_sending]     "M"   (I2C_STATE_sending_data),
      [bus_wait4ack]    "M"   (I2C_STATE_reading_ack),
      [bus_sent]        "M"   (I2C_STATE_wait4putdata),
      [usicr_val]       "M"   (USI_START_IRQ_ONLY),
      [usisr_wipe]      "M"   (USI_CLEAR_ALL),
      [usisr_1bit]      "M"   (USI_RELEASE_1_BIT),
      [usisr_8bit]      "M"   (USI_RELEASE_8_BIT),
      [empty_val]       "M"   (I2C_BUF_EMPTY),
      [full_val]        "M"   (I2C_BUF_FULL),
      [my_stack0]       "I"   (_SFR_IO_ADDR(SREG_SAVE)),
      [my_stack1]       "I"   (_SFR_IO_ADDR(IRQ_STACK_1)),
      [my_stack2]       "I"   (_SFR_IO_ADDR(IRQ_STACK_2)),
      [usicr_addr]      "I"   (_SFR_IO_ADDR(USICR)),
      [usisr_addr]      "I"   (_SFR_IO_ADDR(USISR)),
      [usidr_addr]      "I"   (_SFR_IO_ADDR(USIDR)),
      [twi_ddr]         "I"   (_SFR_IO_ADDR(TWIddr)),
      [twi_scl_bit]     "I"   (TWIsclBit),
      [twi_sda_bit]     "I"   (TWIsdaBit),
      [address]         "a"   (I2C_address_received),
      [state]           "a"   (I2C_private_state),
      [clobber]         "a"   (I2C_private_state) // see note above!
   );
}


/******************************************************************************\
 ******************************************************************************
 *
 *                                  Main loop
 *                                  =========
 *
 ******************************************************************************
\******************************************************************************/


int main (void)
{


// ============================================================================
// Enumerations for better code readability
// ============================================================================

   enum
   {  // Commands to register 1
      VIP_READ_EX_cmd = 0x80,
      VIP_READ_EY_cmd = 0x40,
      VIP_READ_ID_cmd = 0xFF
   };


// ============================================================================
// Variable definitions
// ============================================================================

   uint8_t  answer = 0;
   uint8_t  received = 0;
   uint8_t  command = 0;
   uint8_t  flag = 0;


// ============================================================================
// Initializations
// ============================================================================

   // Init IO ports
   PORTD = I2C_DEFAULT_PORTD | FT_IF_DEFAULT_PORTD;
   DDRD  = I2C_DEFAULT_DDRD | FT_IF_DEFAULT_DDRD;
   PORTB = (1<<MISO_ISP) | FT_IF_DEFAULT_PORTB;
   DDRB  = (0<<MISO_ISP) | FT_IF_DEFAULT_DDRB;
   PORTA = (1<<SJ3) | FT_IF_DEFAULT_PORTA;
   DDRA  = (0<<SJ3) | FT_IF_DEFAULT_DDRA;

   // Identify ft-interface by its response
   FT_IF_init_and_identify();

   // Start bus interface
   I2C_Set_Early_Reply_Reg_0 (0xFF);
   I2C_Set_Early_Reply_Reg_1 (answer);
   I2C_Setup();


// ============================================================================
// Application main loop
// ============================================================================

   // And this is all that needs to be done from now on
   while (1)
   {
      switch(I2C_GetAddress())
      {
         case  I2C_RD_REG_0:
               FT_IF_sample_from_switches();
               while (I2C_GetState() == I2C_ACCESSING)
               {
                  I2C_PutData(FT_IF_shift_from_switches());
               }
               break;
         case  I2C_RD_REG_1:
               while (I2C_GetState() == I2C_ACCESSING)
               {
                  I2C_PutData(answer);
               }
               break;
         case  I2C_WR_REG_0:
               flag = 0;
               while (1)
               {
                  received = I2C_GetData();
                  if (I2C_GetState() == I2C_ACCESSING)
                  {
                     FT_IF_shift_to_motors(received);
                     flag = 1;
                  }
                  else
                     break;
               }
               if (flag != 0)
                  FT_IF_latch_to_motors();
               break;
         case  I2C_WR_REG_1:
               flag = 0;
               while (1)
               {
                  received = I2C_GetData();
                  if (I2C_GetState() == I2C_ACCESSING)
                  {
                     command = received;
                     flag = 1;
                  }
                  else
                     break;
               }
               if (flag != 0)
                  switch (command)
                  {
                     case  VIP_READ_ID_cmd:
                           answer = FT_IF_get_identifier();
                           I2C_Set_Early_Reply_Reg_1 (answer);
                           break;
                     case  VIP_READ_EX_cmd:
                           I2C_Retire();
                           answer = FT_IF_get_resi(FT_IF_EX);
                           I2C_Set_Early_Reply_Reg_1 (answer);
                           I2C_Setup();
                           break;
                     case  VIP_READ_EY_cmd:
                           I2C_Retire();
                           answer = FT_IF_get_resi(FT_IF_EY);
                           I2C_Set_Early_Reply_Reg_1 (answer);
                           I2C_Setup();
                           break;
                     default:
                           answer = 0;
                           I2C_Set_Early_Reply_Reg_1 (answer);
                  }
               break;
         default:
               ;  // <= ask the compiler about why!!!
      }  // switch(...)
   }  // while (1)
}  // main()


/*****************************************************************************\
 *
 * "$Id:$"
 *
 * "$Log:$"
 *
\*****************************************************************************/
