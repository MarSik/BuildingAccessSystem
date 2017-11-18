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
 * START = {
 * END = }
 * ESC = \
 */

// UART status
typedef struct {
    uint8_t buffer[8];
    uint8_t cur = 0;
    uint8_t size = 0;
} Buffer_t;

volatile Buffer_t TxBuffer;
volatile Buffer_t RxBuffer;

static const uint8_t FRAME_MARKER_START = '{';
static const uint8_t FRAME_MARKER_ESC = '\\';
static const uint8_t FRAME_MARKER_ESC_XOR = 0x20;
static const uint8_t FRAME_MARKER_END = '}';

volatile enum {
    FRAME_IDLE, // Line idle, both RX and TX can be started
    FRAME_ACTIVE, // Line is receiving frame
    FRAME_ESC, // Next received byte is escaped
    FRAME_DONE, // Whole frame received
    FRAME_TX, // Line is ready to send or sending data
    FRAME_TX_IDLE, // Line has no data to send, but frame was not terminated
    FRAME_TX_ENDING // Frame terminator is the last symbol in the queue
} FrameState = FRAME_IDLE;

static const uint32_t PTC0 = 1 << 16; // ~RESET for card reader
static const uint32_t PTC2 = 1 << 18; // Led 1
static const uint32_t PTC3 = 1 << 19; // Led 2
static const uint32_t PTA1 = 1 <<  1; // TXEN for RS485 driver

void _sendNextByte();
bool serialSend();
void serialQueueEnd();
void serialQueueBuffer(volatile Buffer_t* buffer);
void serialQueue(uint8_t byte);

extern "C" {


void LowLevelInit(void) {
    SIM->SCGC |= SIM_SCGC_SWD_MASK
            | SIM_SCGC_FLASH_MASK
            ;

    OSC->CR |= OSC_CR_OSCEN_MASK
            | OSC_CR_RANGE_MASK
            | OSC_CR_OSCOS_MASK
            | OSC_CR_OSCSTEN_MASK
            | OSC_CR_HGO_MASK; // range high, crystal on, high gain
    while ((OSC->CR & OSC_CR_OSCINIT_MASK) == 0); // waiting until oscillator is ready

    ICS->C1 = (0b011 << ICS_C1_RDIV_SHIFT); // ref divide by 256

    //ICS->C1 = (0b000 << ICS_C1_RDIV_SHIFT | ICS_C1_IREFS_MASK); // ref divide by 1, internal

}

void SystemInit(void) { 
    SIM->SCGC |= SIM_SCGC_UART0_MASK
            | SIM_SCGC_SPI0_MASK
            ;
}

int main(void) {
    GPIOA->PIDR |= PTC2 | PTC3 | PTA1 | PTC0; // disable input
    GPIOA->PDDR |= PTC2 | PTC3 | PTA1 | PTC0; // set as output

    GPIOA->PSOR = PTC2 | PTC3 | PTC0; // set output to 1
    GPIOA->PCOR = PTA1; // clear output to 0

    // UART on PTA2 / PTA3
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;

    // UART prescaler
    UART0->BDH = 0;
    UART0->BDL = 22; // 40Mhz, 115200 baud, error -1.36%

    // TX and RX enable, incl. interrupts
    UART0->C2 |= UART_C2_TE_MASK
                 | UART_C2_RE_MASK
                 | UART_C2_RIE_MASK;

    NVIC_EnableIRQ(UART0_IRQn);

    // Configure SysTick
    SysTick->LOAD = 0xffffff;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
            | SysTick_CTRL_ENABLE_Msk
            | SysTick_CTRL_TICKINT_Msk;

    GPIOA->PTOR = PTC2; // toggle output

    // Announce itself
    serialQueue('O');
    serialQueue('K');
    serialQueueEnd();
    serialSend();

    // Main echo loop
    while (1) {
        if (FrameState == FRAME_DONE) {
            // Process received data - echo it back
            FrameState = FRAME_IDLE;

            // Toggle leds based on commands
            if (RxBuffer.size == 1
                    && RxBuffer.buffer[0] == 0x01) {
                GPIOA->PTOR = PTC2;
            }

            if (RxBuffer.size == 1
                && RxBuffer.buffer[0] == 0x02) {
                GPIOA->PTOR = PTC3;
            }

            // Echo data back (DEBUG)
            serialQueueBuffer(&RxBuffer);
            serialQueueEnd();
            serialSend();
        }

        __WFI();
    }
}

void UART0_IRQHandler(void) {
    // Character received
    uint8_t s1 = UART0->S1;
    if (s1 & UART_S1_RDRF_MASK) {
        uint8_t data = UART0->D;
        switch (FrameState) {
            case FRAME_IDLE:
                if (data == FRAME_MARKER_START) {
                    FrameState = FRAME_ACTIVE;
                    RxBuffer.cur = RxBuffer.size = 0;
                }
                break;

            case FRAME_TX:
            case FRAME_TX_IDLE:
            case FRAME_TX_ENDING:
            case FRAME_DONE:
                break;

            case FRAME_ESC:
                data ^= FRAME_MARKER_ESC_XOR;
                RxBuffer.buffer[RxBuffer.size++] = data;
                break;

            case FRAME_ACTIVE:
                switch (data) {
                    case FRAME_MARKER_ESC:
                        FrameState = FRAME_ESC;
                        break;
                    case FRAME_MARKER_END:
                        FrameState = FRAME_DONE;
                        break;
                    default:
                        RxBuffer.buffer[RxBuffer.size++] = data;
                        break;
                }
                break;
        }
    }

    // TX buffer is ready to receive new byte
    if (s1 & UART_S1_TDRE_MASK
            && (FrameState == FRAME_TX || FrameState == FRAME_TX_ENDING)) {
        _sendNextByte();
    }

    // Tx buffer transmitted
    if (s1 & UART_S1_TC_MASK
            && TxBuffer.cur == TxBuffer.size
            && FrameState == FRAME_TX_ENDING) {
        FrameState = FRAME_IDLE;

        // Disable TX interrupts
        UART0->C2 &= ~(UART_C2_TIE_MASK | UART_C2_TCIE_MASK);

        // Disable driver TX direction
        GPIOA->PCOR = PTA1;
    }
}

void SysTick_Handler(void) {
    GPIOA->PTOR = PTC2;
}

}

void _sendNextByte() {
    // Enable driver TX direction
    GPIOA->PSOR = PTA1;

    if (TxBuffer.cur < TxBuffer.size) {
        UART0->D = TxBuffer.buffer[TxBuffer.cur++];
    } else if (FrameState != FRAME_TX_ENDING) {
        FrameState = FRAME_TX_IDLE;
    }
}

void serialQueueBuffer(volatile Buffer_t *buffer) {
    for (uint8_t idx = 0; idx < RxBuffer.size; idx++) {
        serialQueue(buffer->buffer[idx]);
    }
}

void serialQueue(uint8_t byte) {
    if (FrameState == FRAME_IDLE || FrameState == FRAME_TX_IDLE || FrameState == FRAME_TX_ENDING) {
        if (FrameState != FRAME_TX_ENDING) {
            TxBuffer.size = TxBuffer.cur = 0;
        }

        if (FrameState == FRAME_IDLE || FrameState == FRAME_TX_ENDING) {
            TxBuffer.buffer[TxBuffer.size++] = FRAME_MARKER_START;
        }

        FrameState = FRAME_TX;
    }

    if (FrameState == FRAME_TX) {
        if (byte == FRAME_MARKER_START || byte == FRAME_MARKER_ESC || byte == FRAME_MARKER_END) {
            byte ^= FRAME_MARKER_ESC_XOR;
            TxBuffer.buffer[TxBuffer.size++] = FRAME_MARKER_ESC;
        }
        TxBuffer.buffer[TxBuffer.size++] = byte;
    }
}

bool serialSend() {
    if (FrameState != FRAME_TX && FrameState != FRAME_TX_ENDING) {
        return false;
    }

    // Always read UART_S1 before writing to UART_D to allow
    // data to be transmitted.
    if (UART0->S1 & UART_S1_TDRE_MASK) {
        _sendNextByte();

        // Enable TX interrupts
        UART0->C2 |= UART_C2_TIE_MASK | UART_C2_TCIE_MASK;
    }

    return true;
}

void serialQueueEnd() {
    if (FrameState == FRAME_TX || FrameState == FRAME_TX_IDLE) {
        TxBuffer.buffer[TxBuffer.size++] = FRAME_MARKER_END;
        FrameState = FRAME_TX_ENDING;
    }
}
