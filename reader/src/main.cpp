#include "mke04z4.h"

/*
 * The goal is to convert rs485 stream to SPI commands for
 * NFC card reader + allow additional commands:
 *  - LED1 on/off
 *  - LED2 on/off
 *  - RESET reader
 *
 * It would be best if the protocol was device independent.
 * SPI buffer size at least 8B
 *
 * TODO I will probably use the following protocol:
 * > START
 * > 1B high level command [LED, SPI, RESET]
 * > X B SPI data
 * > END frame break
 * - delay X ms
 * < START
 * < 1B status
 * < X B SPI response
 * < END frame break
 *
 * control byte in data will be escaped by prefixing the byte with ESC and then
 * transmitting the real byte XOR 0x20
 *
 * START = 0x7E
 * END = 0x6D
 * ESC = 0x7D
 */

extern "C" {


void LowLevelInit(void) {
    SIM->SCGC |= SIM_SCGC_SWD_MASK
            | SIM_SCGC_FLASH_MASK
            ;

    /*
    OSC->CR |= OSC_CR_OSCEN_MASK
            | OSC_CR_RANGE_MASK
            | OSC_CR_OSCOS_MASK
            | OSC_CR_OSCSTEN_MASK
            | OSC_CR_HGO_MASK; // range high, crystal on, high gain
    while ((OSC->CR & OSC_CR_OSCINIT_MASK) == 0); // waiting until oscillator is ready

    ICS->C1 = (0b011 << ICS_C1_RDIV_SHIFT); // ref divide by 256
    */

    ICS->C1 = (0b000 << ICS_C1_RDIV_SHIFT | ICS_C1_IREFS_MASK); // ref divide by 1, internal

}

void SystemInit(void) { 
    SIM->SCGC |= SIM_SCGC_UART0_MASK
            | SIM_SCGC_SPI0_MASK
            ;
}

// UART status
volatile uint8_t received = 0;
volatile uint8_t buffer[8];
volatile uint8_t state = 0;

static const uint32_t PTC2 = 1 << 18;
static const uint32_t PTC3 = 1 << 19;

int main(void) {
    GPIOA->PIDR |= PTC2 | PTC3; // disable input
    GPIOA->PDDR |= PTC2 | PTC3; // set as output
    GPIOA->PSOR |= PTC2 | PTC3; // set output

    // UART on PTA2 / PTA3
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;

    // UART prescaler
    UART0->BDH = 0;
    UART0->BDL = 22; // 40Mhz, 115200 baud, error -1.36%

    // TX and RX enable
    UART0->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;

    SysTick->LOAD = 0xffffff;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
            | SysTick_CTRL_ENABLE_Msk
            | SysTick_CTRL_TICKINT_Msk;

    __enable_irq();

    GPIOA->PTOR = PTC2; // toggle output

    while (1) {
    }
}

void UART0_IRQHandler(void) {
  received = 1;
}

volatile uint8_t blink = 0;

void SysTick_Handler(void) {
    blink = (blink + 1) & 0b111;
    if (!blink) {
      GPIOA->PTOR = PTC2;
    }
}

}
