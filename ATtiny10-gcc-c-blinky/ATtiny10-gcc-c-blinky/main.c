/*
 * ATtiny10-gcc-c-blinky.c
 *
 * Created: 11/26/2020 8:03:55 PM
 * Author : Kwangwu Lee
 */

#include <avr/io.h>

// Import _delay_ms()
#define F_CPU 8000000UL     // ATtiny10 (ATTINY10-TSHR) can support up-to 12MHz
                            // with an external OSC, but the internal one is 8MHz
#include <util/delay.h>

#define TIMER_COUNT_9600_BPS    (F_CPU / 9600)

#include <avr/sfr_defs.h>
#include <avr/interrupt.h>

#define HIGH    1
#define LOW     0

#define PIN_UART    0

static volatile char        byteInBits;     // The LSb (least significant bit) represents the current bit to be sent
static volatile uint8_t     bitPos = 0;     // The bit position of the byte being sent over UART

void setPin(uint8_t port_B_pin, uint8_t is_on);

// UART Tick
ISR(TIM0_COMPA_vect)
{
#if TIMING_TEST
    PORTB = (byteInBits & 1) ? PORTB | 0b1 : PORTB & ~0b1;
    byteInBits ^= 1;
#endif

    switch (bitPos)
    {
        // Start bit in UART
        case 1:
            PORTB &= ~(_BV(PIN_UART));
            bitPos++;
            break;
        // UART 8-bit data
        case 2 ... 9:
            PORTB = (byteInBits & 1) ?
                    PORTB | (_BV(PIN_UART)) :
                    PORTB & (~(_BV(PIN_UART)));
            byteInBits >>= 1;
            bitPos++;
            break;
        // Stop bit without any parity bit in UART
        case 10:
            PORTB |= _BV(PIN_UART);
            bitPos = 0; // Done. Reset the bit position to 0 and let the sendByte() wait released.
            break;
    }
}

void sendByte(uint8_t val)
{
    byteInBits  = val;
    bitPos      = 1;

    while (bitPos); // Very important: wait until the byte is fully sent.
}


// Send decimal of up-to 16-bit unsigned. The max value is 65535
void outDec(uint16_t num)
{
    uint16_t    divider = 10000;
    uint8_t     zero    = 0;

    uint16_t    pos;

    for (pos = 0; pos < 5; pos++)
    {
        if (num >= divider)
        {
            zero = 1;
            sendByte((num / divider) + '0');
            num %= divider;
        }
        else if (zero)
        {
            sendByte('0');
        }

        divider /= 10;
    }
}


int main(void)
{
    CCP     = 0xD8;
    CLKPSR  = 0;

    DDRB    =   /* _BV(PORTB3) | */ _BV(PB2) | _BV(PB0); // PORTB2 = PB2, etc... Refer avr/portpin.h

    // Timer/Counter0 Control Register A
    /*
    Bits 1:0 - WGM0n: Waveform Generation Mode [n = 1:0]
    Combined with the WGM0[3:2] bits found in the TCCR0B Register, these bits control the counting
    sequence of the counter, the source for maximum (TOP) counter value, and what type of waveform
    generation to be used. Modes of operation supported by the Timer/Counter unit are: Normal mode
    (counter), Clear Timer on Compare match (CTC) mode, and three types of Pulse Width Modulation
    (PWM) modes. (See Modes of Operation).
    */
    // Waveform Generation Mode Bit Description
    // Mode = 4, WGM0[3:0] = 0100
    // Because WGM0 bit-fields are scattered into two different registers TCCR0A and TCCR0B
    // we need to set them separately, WGM0[3:2] = 0b10, WGM0[1:0] = 0b00
    // Timer/Counter Mode of Operation = CTC (Clear Timer on Compare)
    TCCR0A  = 0;        // Timer/Counter0 Control Register A
    // TCCR0B  = 0b1001;   // No pre-scaler. CTC mode (clear timer on compare) for WGM0[3:2]
    TCCR0B  = (0b01 << WGM02) | (0b001 << CS00); // More descriptive value settings. No pre-scaler. CTC mode 4 (clear timer on compare) for WGM0[3:2]
    // 8 MHz / 9600 bps = 833.3333...
    OCR0AH  = TIMER_COUNT_9600_BPS >> 8;    // High byte value
    OCR0AL  = TIMER_COUNT_9600_BPS & 0xFF;  // Low byte value

    TIMSK0  |= _BV(OCIE0A);
    sei();

    // Enable ADC on PB3
    DIDR0   = _BV(ADC3D);   // Disable digital input on PB3 (i.e., 0b1000)
    ADMUX   = 3;

    // ADC Control and Status Register A (ADCSRA)
    /*
    Bits 2:0 - ADPSn: ADC Prescaler Select [n = 2:0]
    These bits determine the division factor between the system clock frequency and the input clock to the
    ADC.
    */
    ADCSRA  = _BV(ADEN) | (0b111 << ADPS0); // ADEN: ADC Enable,
                                            // (ADPS1 | ADPS0): ADC Prescaler Select = Division Factor 128 (0b111)
                                            // 8MHz / 128 = 0.016 msec. That is, if we take a delay of 1 ms to print out ADC,
                                            // sampling is still 1.6%

    uint16_t boringCount = 0;
    /* Replace with your application code */
    while (1)
    {
        ADCSRA |= _BV(ADSC);    // Start ADC conversion

        // while (ADCSRA & _BV(ADSC)); // Wait until ADC conversion is done
        // while (bit_is_set(ADCSRA, ADSC)); // Wait until ADC conversion is done
        loop_until_bit_is_set(ADCSRA, ADSC); // Wait until ADC conversion is done

        outDec(ADCL);

        _delay_ms(1);
        //sendByte('A');

        //outDec(boringCount);

        sendByte('\r');
        sendByte('\n');

        boringCount++;

        /*
        setPin(PORTB2, HIGH);
        _delay_ms(500);

        setPin(PORTB2, LOW);
        _delay_ms(500);
        */
    }
}

void setPin(uint8_t port_B_pin, uint8_t is_on)
{
    if (is_on)
    {
        PORTB |= _BV(port_B_pin);
    }
    else
    {
        PORTB &= ~(_BV(port_B_pin));
    }
}

