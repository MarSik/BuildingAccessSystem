#include "mke04z4.h"

/* !!! Check schematic errata
 * = V1 =
 * Real MISO pin is connected to PTB4, but the P105 header's MISO signal
 * is connected to PTB1.
 *
 * Workaround:
 * use TAMPER as MISO and MISO as TAMPER (needs external pull-up)
 *
 * = V2 =
 * TAMPER and MISO mixup sorted out. Software uses PTB4 as MISO.
 *
 */

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
 * - delay 500us
 * < START
 * < 1B status (0x00 = OK, 0xFF = ERROR)
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
    uint8_t buffer[64];
    uint8_t cur = 0;
    uint8_t size = 0;
} Buffer_t;

volatile Buffer_t TxBuffer;
volatile Buffer_t RxBuffer;

static volatile uint8_t spiSent = 0;
static volatile uint8_t spiReceived = 0;

static const uint8_t FRAME_MARKER_START = '{';
static const uint8_t FRAME_MARKER_ESC = 0x7E;
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
static const uint32_t PTB0 = 1 <<  8; // CS

void _sendNextByte();
bool serialSend();
void serialQueueEnd();
void serialQueueBuffer(volatile Buffer_t* buffer);
void serialQueue(uint8_t byte);

extern "C" {

// Create big endian version of the number
// See: https://community.nxp.com/thread/350150?commentID=498593#comment
#define BIG_SHORT_WORD(x) (unsigned short)((x << 8) | (x >> 8))

void LowLevelInit(void) {
    // Disable NMI
    SIM->SOPT &= ~SIM_SOPT_NMIE_MASK;

    // Reconfigure watchdog
    __disable_irq();
    WDOG->CNT = 0x20C5; // unlock configuration
    WDOG->CNT = 0x28D9;
    WDOG->TOVAL = BIG_SHORT_WORD(30000); // 30s reset
    WDOG->WIN = 0;
    WDOG->CS2 = WDOG_CS2_CLK_MASK // setting 1-kHz clock source
                | WDOG_CS2_FLG_MASK; // clear WDOG interrupt flag
    WDOG->CS1 = 0 // WDOG_CS1_EN_MASK // enable counter running
                | WDOG_CS1_DBG_MASK
                | WDOG_CS1_STOP_MASK
                | WDOG_CS1_WAIT_MASK; // enable watchdog in all modes
    __enable_irq();

    SIM->SCGC |= SIM_SCGC_SWD_MASK
            | SIM_SCGC_FLASH_MASK
            ;

    OSC->CR |= OSC_CR_OSCEN_MASK
            | OSC_CR_RANGE_MASK
            | OSC_CR_OSCOS_MASK
            | OSC_CR_OSCSTEN_MASK; // range high, crystal on, auto gain

    while ((OSC->CR & OSC_CR_OSCINIT_MASK) == 0); // waiting until oscillator is ready

    ICS->C1 = (0b011 << ICS_C1_RDIV_SHIFT); // ref divide by 256 -> 40 Mhz main clock
    ICS->C2 = 0;

    //ICS->C1 = (0b000 << ICS_C1_RDIV_SHIFT | ICS_C1_IREFS_MASK); // ref divide by 1, internal

}

void SystemInit(void) { 
    SIM->SCGC |= SIM_SCGC_UART0_MASK
            | SIM_SCGC_SPI0_MASK
            ;
}

void primeSend() {
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk
                    | SysTick_CTRL_ENABLE_Msk
                    | SysTick_CTRL_CLKSOURCE_Msk;
}

int main(void) {
    GPIOA->PIDR |= PTC2 | PTC3 | PTA1 | PTC0 | PTB0; // disable input
    GPIOA->PDDR |= PTC2 | PTC3 | PTA1 | PTC0 | PTB0; // set as output

    FGPIOA->PSOR = PTC2 | PTC3 | PTC0 | PTB0; // set output to 1
    FGPIOA->PCOR = PTA1; // clear output to 0

    // UART on PTA2 / PTA3
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;

    // Configure clock dividers (1 / 2 / 1) to set Bus clock = System clock / 2 -> 20 Mhz
    SIM->CLKDIV = SIM_CLKDIV_OUTDIV2_MASK;

    // UART prescaler
    UART0->BDH = 0;
    UART0->BDL = 11; // Bus clock 20Mhz, 115200 baud

    // TX and RX enable, incl. interrupts
    UART0->C2 |= UART_C2_TE_MASK
                 | UART_C2_RE_MASK
                 | UART_C2_RIE_MASK;

    NVIC_EnableIRQ(UART0_IRQn);

    // Configure SPI0
    // Errata: remove tamper resistor and connect PTB4 with PTB1 in rev.1 of the board
    SPI0->C1 = SPI_C1_SPE_MASK
               | SPI_C1_SPIE_MASK
               | SPI_C1_MSTR_MASK
               | SPI_C1_CPHA_MASK
               | SPI_C1_CPOL_MASK;
    SPI0->C2 = 0;
    SPI0->BR = SPI_BR_SPPR_MASK | 0b0010; // prescalers 8 and 8 -> cca 300kHz SPI clock

    NVIC_EnableIRQ(SPI0_IRQn);

    // Configure SysTick reload value
    // (X-1 see http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0662b/BGBEEJHC.html)
    SysTick->LOAD = 200000 - 1; // 500us (200 000 cycles at 40Mhz)
    SysTick->CTRL = 0;

    GPIOA->PTOR = PTC2 | PTC3; // toggle outputs off

    // Main echo loop
    while (1) {
        /* Reset watchdog clock
        __disable_irq();
        WDOG->CNT = 0x02A6;
        WDOG->CNT = 0x80B4;
        __enable_irq(); */

        if (FrameState == FRAME_DONE) {
            // Process received data - echo it back
            FrameState = FRAME_IDLE;

            // Set pins
            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0x01) {
                if (RxBuffer.buffer[1] & 0b001) {
                    FGPIOA->PSOR = PTB0;
                }

                if (RxBuffer.buffer[1] & 0b010) {
                    FGPIOA->PSOR = PTC2;
                }

                if (RxBuffer.buffer[1] & 0b100) {
                    FGPIOA->PSOR = PTC3;
                }

                if (RxBuffer.buffer[1] & 0b1000) {
                    FGPIOA->PSOR = PTC0;
                }

                serialQueue(0x00); // OK marker
                serialQueueEnd();
                primeSend();
                continue;
            }

            // Clear pins
            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0x02) {
                if (RxBuffer.buffer[1] & 0b001) {
                    FGPIOA->PCOR = PTB0;
                }

                if (RxBuffer.buffer[1] & 0b010) {
                    FGPIOA->PCOR = PTC2;
                }

                if (RxBuffer.buffer[1] & 0b100) {
                    FGPIOA->PCOR = PTC3;
                }

                if (RxBuffer.buffer[1] & 0b1000) {
                    FGPIOA->PCOR = PTC0;
                }

                serialQueue(0x00); // OK marker
                serialQueueEnd();
                primeSend();
                continue;
            }


            if (RxBuffer.size > 1
                && RxBuffer.buffer[0] == 0x00) {
                RxBuffer.cur++; // Skip command byte
                serialQueue(0x00); // OK marker

                // Pass first data byte to SPI and enable interrupts
                spiSent = 1;
                spiReceived = 0;
                while (!(SPI0->S & SPI_S_SPTEF_MASK)) {
                    // Wait for the send buffer to be empty
                    // See section: 30.3.5
                }
                SPI0->D = RxBuffer.buffer[RxBuffer.cur++];
                if (RxBuffer.cur < RxBuffer.size) {
                    SPI0->C1 |= SPI_C1_SPTIE_MASK;
                }
                continue;
            }

            // SPI configuration byte
            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0xE0) {

                SPI0->C1 = RxBuffer.buffer[1]
                           | SPI_C1_SPIE_MASK
                           | SPI_C1_MSTR_MASK;

                serialQueue(0x00); // OK marker
                serialQueueEnd();
                primeSend();
                continue;
            }

            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0xE2) {

                SPI0->C2 = RxBuffer.buffer[1];

                serialQueue(0x00); // OK marker
                serialQueueEnd();
                primeSend();
                continue;
            }

            // SPI baud rate configuration byte
            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0xE1) {

                SPI0->BR = RxBuffer.buffer[1] & 0b01111111;

                serialQueue(0x00); // OK marker
                serialQueueEnd();
                primeSend();
                continue;
            }

            // UART speed
            if (RxBuffer.size == 2
                && RxBuffer.buffer[0] == 0xE3) {

                if (RxBuffer.buffer[1] > 0) {
                    serialQueue(0x00); // OK marker
                } else {
                    serialQueue(0xFF);
                }
                serialQueueEnd();
                primeSend();

                if (RxBuffer.buffer[1] == 0) {
                    continue;
                }

                // finish transmitting before changing speed
                while (FrameState != FRAME_IDLE)
                    ;

                UART0->BDL = RxBuffer.buffer[1];
                continue;
            }

            // About
            if (RxBuffer.size == 1
                && RxBuffer.buffer[0] == 0xFF) {

                serialQueue(0x00); // OK marker

                // Announce itself
                serialQueue('O');
                serialQueue('K');
                serialQueueEnd();
                primeSend();
                continue;
            }

            // Echo data back (DEBUG)
            serialQueue(0xff); // error marker
            serialQueueBuffer(&RxBuffer);
            serialQueueEnd();
            primeSend();
        }

        __WFI();
    }
}

void SPI0_IRQHandler(void) {
    uint8_t status = SPI0->S;

    if (status & SPI_S_SPTEF_MASK) {
        if (RxBuffer.cur < RxBuffer.size) {
            SPI0->D = RxBuffer.buffer[RxBuffer.cur++];
            spiSent++;
            if (RxBuffer.cur == RxBuffer.size) {
                // Last byte queued; disable TX interrupt
                SPI0->C1 &= ~SPI_C1_SPTIE_MASK;
            }
        }
    }

    if (status & SPI_S_SPRF_MASK) {
        const uint8_t data = SPI0->D;
        serialQueue(data);
        spiReceived++;
        if (spiReceived == spiSent && RxBuffer.cur == RxBuffer.size) {
            // Last byte processed, trigger uart
            serialQueueEnd();
            primeSend();
        }
    }
}

void UART0_IRQHandler(void) {
    // Character received
    uint8_t s1 = UART0->S1;
    if (s1 & UART_S1_RDRF_MASK) {
        uint8_t data = UART0->D;

        switch (FrameState) {
            case FRAME_IDLE:
                // Start frame is used for resynchronization
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
                FrameState = FRAME_ACTIVE;
                break;

            case FRAME_ACTIVE:
                switch (data) {
                    case FRAME_MARKER_START:
                        // Start frame is used for resynchronization
                        // and cannot appear in data
                        FrameState = FRAME_ACTIVE;
                        RxBuffer.cur = RxBuffer.size = 0;
                        break;
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
        FGPIOA->PCOR = PTA1;
    }
}

void SysTick_Handler(void) {
    SysTick->CTRL = 0;
    serialSend();
}

void NMI_Handler(void) {
    // Disable NMI
    SIM->SOPT &= ~SIM_SOPT_NMIE_MASK;
}

}

void _sendNextByte() {
    // Enable driver TX direction
    FGPIOA->PSOR = PTA1;

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
