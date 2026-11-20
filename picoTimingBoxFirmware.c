/*
Firmware to control the Raspberry Pi Pico based timing boxes.
These timing boxes, from a software point of view, emulate the type 3 (XMOS) timing boxes currently in use
but with only 4 outputs via BNC's with 2 being capable of driving low impedance (50ohm) loads using dip switches 
accessible from within the timing box unit.

Notable Changes From Previous Version

This Firmware differs from the original TimingBoxPico code to accomodate some questionable design choices.
Namely, that state machine that drives the outputs needed to use consecutive GPIO pins. This requirement wasn't
noticed when designing the PCB layout so there's some less than elegant code to account for this.
*/

#ifdef TIMINGBOX_HOST_TEST
#include "tests/native/pico_shim.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include <pianolaPinDriver.pio.h>
#include <pianola24bitClock.pio.h>

#endif

// ===================================================================================
// Defines for UART Control and GPIOs
// ===================================================================================

#define picoCPUclkRateMHz 150

// UART defines
// bog standard 8n1 @ 115200
#define UART_ID uart0
#define DATA_BITS 8
#define STOP_BITS 1
#define BAUDRATE 115200
#define PARITY UART_PARITY_NONE //no parity bits
#define UART_INTERRUPT UART0_IRQ

// define the function of each GPIO pin we need.
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define ONBOARD_LED_PIN 25 // on board LED
#define STATUS_LED_GREEN 26 // GP26 green channel of status LED
#define STATUS_LED_BLUE 27 // GP27
#define OUTPUT_0_PIN 2
#define OUTPUT_1_PIN 11
#define OUTPUT_2_PIN 14
#define OUTPUT_3_PIN 15
#define IRQ_PIANOLACORE 46

// ===============================================================================
// Set up some pianola attributes
//================================================================================

queue_t responseQ;
queue_t commandQ;
queue_t dumpDebugQ;

#define responseQWidth 32
#define commandQWidth 32
#define commandQDepth 32
#define responseQDepth 32
// Internal queue marker; never transmitted as a protocol response length.
#define RESPONSE_DEBUG_DUMP 0xFF

#define dumpDebugQWidth 32
#define dumpDebugQDepth 32

enum STATUS
{
    INIT = 0,
    READY,
    RUNNING,
    REPEATING,
    STOPPED,
    RESET,
    PANIC
};

uint8_t pianolaRepeatFlag = 0;

uint64_t pianola[256];
uint32_t pianolaCameraClock[8];
int pianolaFinalPosition = 0;
int pianolaPosition = 0;
uint32_t pianolaNumberOfEntries = 1;
uint32_t pianolaTime;

uint8_t pianolaRepeatAddr = 0;
// A scheduled restart can coexist with active playback.
static bool pianolaFirePending;
static uint32_t pianolaFireTime;

// fixed at a 2.56us timebase to mimic the xmos
// clock divisors represents as 24bit Fixed Point Ints - 16 Integral Bit, 8 Frac bits
// If they need to change is relatively straightorward to do this by hand - coding its a bit of a faff
uint32_t pianolaClockDivisorFP = 0x02666; // 38.4X clockdivider (38.3984375 due to rounding.)
uint32_t pinDriverClockDivisorFP = 0x000C00; // 12X clockdivider
// 32 PIO cycles per Pianola tick give a 384X effective divider.

enum STATUS pianolaStatus = INIT;

// Some values to keep the current version of spim-interface happy

// firmware version for type 3 box
int pianolaFirmwareVersion = 6;
int pianolaFirmwareEarliestVersion = 6;

#define PIANOLA_SOURCE_COUNT 8
#define PIN_SOURCE_1HZ 12
#define PIN_SOURCE_PIANOLA_RUNNING 25
#define PIN_SOURCE_PIANOLA_PENDING 26
#define PIN_SOURCE_ON 28
#define PIN_SOURCE_OFF 29
#define ONE_HZ_HALF_PERIOD_US 500000u

// Indexed by pin number in SET_PinSource and GET_PinSource.
static const struct {
    uint gpio;
    uint8_t defaultOverride;
} pianolaOutputs[] = {
    {OUTPUT_0_PIN, 0},
    {OUTPUT_1_PIN, 0},
    {OUTPUT_2_PIN, 0},
    {OUTPUT_3_PIN, 0},
    {ONBOARD_LED_PIN, PIN_SOURCE_PIANOLA_RUNNING},
    {STATUS_LED_BLUE, PIN_SOURCE_OFF},
    {STATUS_LED_GREEN, PIN_SOURCE_OFF},
};

// Core 1 owns routing. The PIO mapping stays current behind an override.
static uint8_t pianolaSources[count_of(pianolaOutputs)];
static uint32_t pianolaGpioMasks[2][16];
static uint32_t pianolaInversionMask;
typedef struct {
    uint8_t source; // Zero means the pin is using its pianola route.
    bool inverted;
} PinOverride;
static PinOverride pinOverrides[count_of(pianolaOutputs)];
static uint32_t clockOutputPins;
static uint64_t nextClockEdgeUs;
static bool clockHigh;
// Snapshot of the last submitted mask, which becomes the held pattern at idle.
static uint8_t lastPianolaMask;
static bool pianolaRoutingDirty;

// piv params
uint8_t pivIdx;
uint32_t pivIntervalLaserPulses;
uint32_t pivDuratonPulsePair;
uint32_t pivIntervalPulseToExp;
uint32_t pivCameraExposure;
uint8_t pivCameraMonitorIdx;
// ===============================================================================
// Set some values for setting the pio block
//================================================================================
PIO pioBlock = pio0;
uint smPinDriver = 0;
uint offsetPinDriver;

uint smPianolaClock = 1;
uint offsetPianolaClock;

// ===============================================================================
// Command Set
//================================================================================

enum CMD_SET 
{
    SET_Pianola             = 0x1,
    SET_PianolaFinalPos     = 0x2,
    SET_PianolaRepeatFrom   = 0x3,
    SET_PianolaRepeating    = 0x4, 
    RUN_Pianola             = 0x5,
    SET_PianolaFireTime     = 0x6,
    IRQ_StopAndReset        = 0x7,
    GET_CurrentPianolaTime  = 0x8,
    SET_PinSource           = 0x9,
    GET_PinSource           = 0xA,
    SET_CameraClk           = 0xB,
    SET_PIVParams           = 0xC,
    ENABLE_PIV              = 0xD,
    GET_FirmwareVersion     = 0xFD, // this is technically different from whats in the communications protocol.rtf (0xD) but the TimingBoxXMOS.cpp file wants 0xFD
    IRQ_DumpLog             = 0xFE,
    IRQ_HARDRESET           = 0xFF,


};

// ===============================================================================
// Helper Functions that need to be declared before the parsers
//================================================================================

uint32_t getPianolaTime()
{
    pio_sm_put_blocking(pioBlock, smPianolaClock, 0xF);
    return 0xFFFFFFu - pio_sm_get_blocking(pioBlock, smPianolaClock);
}

static void setPianolaBitOutput(uint source, uint32_t outputMask, bool enabled)
{
    // Two four-bit lookup tables keep both per-entry and per-command work small.
    uint32_t *masks = pianolaGpioMasks[source / 4];
    uint sourceBit = 1u << (source % 4);
    for (uint i = 0; i < count_of(pianolaGpioMasks[0]); i++)
    {
        if (i & sourceBit)
            masks[i] = enabled ? masks[i] | outputMask : masks[i] & ~outputMask;
    }
}

static uint32_t routePianolaMask(uint8_t mask)
{
    return (pianolaGpioMasks[0][mask & 0x0f] | pianolaGpioMasks[1][mask >> 4]) ^ pianolaInversionMask;
}

static bool pianolaIsRunning(void)
{
    return pianolaStatus == RUNNING || pianolaStatus == REPEATING;
}

static void drivePinOverride(uint pin)
{
    PinOverride route = pinOverrides[pin];
    bool high;
    switch (route.source)
    {
        case PIN_SOURCE_1HZ:             high = clockHigh; break;
        case PIN_SOURCE_PIANOLA_RUNNING: high = pianolaIsRunning(); break;
        case PIN_SOURCE_PIANOLA_PENDING: high = pianolaFirePending; break;
        case PIN_SOURCE_ON:              high = true; break;
        default:                         high = false; break;
    }
    gpio_set_outover(pianolaOutputs[pin].gpio, high != route.inverted ? GPIO_OVERRIDE_HIGH : GPIO_OVERRIDE_LOW);
}

static void resetPinSources(void)
{
    memset(pianolaGpioMasks, 0, sizeof(pianolaGpioMasks));
    for (uint pin = 0; pin < count_of(pianolaOutputs); pin++)
    {
        uint gpio = pianolaOutputs[pin].gpio;
        if (!pianolaOutputs[pin].defaultOverride && pinOverrides[pin].source)
            gpio_set_outover(gpio, GPIO_OVERRIDE_NORMAL);
        pinOverrides[pin] = (PinOverride){.source = pianolaOutputs[pin].defaultOverride};
        if (pinOverrides[pin].source)
            drivePinOverride(pin);
        pianolaSources[pin] = pin;
        setPianolaBitOutput(pin, 1u << (gpio - OUTPUT_0_PIN), true);
    }
    pianolaInversionMask = 0;
    clockOutputPins = 0;
    lastPianolaMask = 0;
    pianolaRoutingDirty = false;
}

static void updatePianolaStatusOutputs(void)
{
    for (uint pin = 0; pin < count_of(pianolaOutputs); pin++)
    {
        uint source = pinOverrides[pin].source;
        if (source == PIN_SOURCE_PIANOLA_RUNNING || source == PIN_SOURCE_PIANOLA_PENDING)
            drivePinOverride(pin);
    }
}

static void updateOneHzClock(void)
{
    if (!clockOutputPins)
        return;
    uint64_t now = time_us_64();
    if (now < nextClockEdgeUs)
        return;

    // Derive phase from elapsed time so a late poll does not shift later edges.
    uint64_t halfPeriods = now / ONE_HZ_HALF_PERIOD_US;
    nextClockEdgeUs = (halfPeriods + 1) * ONE_HZ_HALF_PERIOD_US;
    clockHigh = (halfPeriods & 1u) != 0;
    for (uint pin = 0; pin < count_of(pianolaOutputs); pin++)
    {
        if (clockOutputPins & (1u << pin))
            drivePinOverride(pin);
    }
}

// Core 1 owns all output-SM control. Disabling alone would only pause the
// previous countdown; clear both queued words and in-flight execution state,
// then explicitly reposition the PC. Never reset the independent clock SM.
static void cancelPinDriver(void)
{
    pio_sm_set_enabled(pioBlock, smPinDriver, false);
    pio_sm_clear_fifos(pioBlock, smPinDriver);
    pio_sm_restart(pioBlock, smPinDriver);
    pio_sm_exec_wait_blocking(pioBlock, smPinDriver,
                             pio_encode_jmp(offsetPinDriver + pinDriver_wrap_target));
}

static void queuePianolaEntry(uint8_t position)
{
    uint64_t entry = pianola[position];
    uint32_t duration = (uint32_t)entry;
    // The PIO's zero-count path already takes one tick, including the mask
    // write and the transition to the next entry. Each taken countdown branch
    // adds one tick, so four requested ticks are encoded as three.
    // Encode zero as zero to retain the one-tick minimum without underflow.
    pio_sm_put(pioBlock, smPinDriver, duration ? duration - 1 : 0);
    lastPianolaMask = (uint8_t)(entry >> 32);
    pio_sm_put(pioBlock, smPinDriver, routePianolaMask(lastPianolaMask));
    pianolaRoutingDirty = false;
}

static void startPianola(void)
{
    cancelPinDriver();
    pianolaFirePending = false;
    pianolaPosition = 1;
    pianolaStatus = RUNNING;
    // Preload one complete pair while halted. No old output pattern is applied
    // or cleared between cancellation and the new entry-zero OUT instruction.
    queuePianolaEntry(0);
    pio_sm_set_enabled(pioBlock, smPinDriver, true);
}

// Forced PULL/OUT applies the whole pattern at once while the driver is halted.
static void applyHaltedPinDriverMask(uint32_t pinMask)
{
    pio_sm_put(pioBlock, smPinDriver, pinMask);
    pio_sm_exec_wait_blocking(pioBlock, smPinDriver, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(pioBlock, smPinDriver, pio_encode_out(pio_pins, 32));
    pio_sm_exec_wait_blocking(pioBlock, smPinDriver,
                             pio_encode_jmp(offsetPinDriver + pinDriver_wrap_target));
}

static void refreshIdlePinRouting(void)
{
    if (pianolaRoutingDirty)
    {
        pio_sm_set_enabled(pioBlock, smPinDriver, false);
        applyHaltedPinDriverMask(routePianolaMask(lastPianolaMask));
        pianolaRoutingDirty = false;
    }
}

static void setPinSource(uint pin, uint source, bool inverted)
{
    if (pin >= count_of(pianolaOutputs) ||
        (source >= PIANOLA_SOURCE_COUNT && source != PIN_SOURCE_1HZ &&
         source != PIN_SOURCE_PIANOLA_RUNNING && source != PIN_SOURCE_PIANOLA_PENDING &&
         source != PIN_SOURCE_ON && source != PIN_SOURCE_OFF))
        return;

    uint gpio = pianolaOutputs[pin].gpio;
    uint32_t pinMask = 1u << pin;
    if (source >= PIANOLA_SOURCE_COUNT)
    {
        pinOverrides[pin] = (PinOverride){source, inverted};
        if (source == PIN_SOURCE_1HZ)
        {
            clockOutputPins |= pinMask;
            updateOneHzClock();
        }
        else
        {
            clockOutputPins &= ~pinMask;
        }
        drivePinOverride(pin);
        return;
    }

    uint32_t outputMask = 1u << (gpio - OUTPUT_0_PIN);
    if (pianolaSources[pin] != source)
    {
        setPianolaBitOutput(pianolaSources[pin], outputMask, false);
        setPianolaBitOutput(source, outputMask, true);
        pianolaSources[pin] = source;
        pianolaRoutingDirty = true;
    }
    if (((pianolaInversionMask & outputMask) != 0) != inverted)
    {
        pianolaInversionMask ^= outputMask;
        pianolaRoutingDirty = true;
    }

    // The FIFO keeps masks already submitted by the old routing.
    if (!pianolaIsRunning())
        refreshIdlePinRouting();

    if (pinOverrides[pin].source)
    {
        clockOutputPins &= ~pinMask;
        pinOverrides[pin] = (PinOverride){0};
        gpio_set_outover(gpio, GPIO_OVERRIDE_NORMAL);
    }
}

static void stopPianola(void)
{
    cancelPinDriver();
    pianolaStatus = READY;
    pianolaPosition = 0;
    pianolaRepeatFlag = 0;
    pianolaFirePending = false;
    // Intentional XMOS difference: rewind AND apply the currently programmed
    // entry-zero pattern, ignoring its duration.
    lastPianolaMask = (uint8_t)(pianola[0] >> 32);
    applyHaltedPinDriverMask(routePianolaMask(lastPianolaMask));
    pianolaRoutingDirty = false;
    updatePianolaStatusOutputs();
}

static void hardResetPianola(void)
{
    cancelPinDriver();
    applyHaltedPinDriverMask(0);

    pianolaPosition = 0;
    pianolaFinalPosition = 0;
    pianolaNumberOfEntries = 1;
    pianolaRepeatAddr = 0;
    pianolaRepeatFlag = 0;
    pianolaFirePending = false;
    pianolaFireTime = 0;
    memset(pianola, 0, sizeof(pianola));

    memset(pianolaCameraClock, 0, sizeof(pianolaCameraClock));
    pivIdx = 0;
    pivIntervalLaserPulses = 0;
    pivDuratonPulsePair = 0;
    pivIntervalPulseToExp = 0;
    pivCameraExposure = 0;
    pivCameraMonitorIdx = 0;
    pianolaStatus = INIT;
    resetPinSources();
}

int isTimeInFuture(uint32_t currentTime, uint32_t proposedTime)
{
    // Compare modulo 2^24 using the XMOS half-range convention.
    uint32_t delta = (proposedTime - currentTime) & 0xFFFFFFu;
    return delta != 0 && delta < 0x800000u;
}

// ===============================================================================
// Parsers for Handling Commands and Deciding Input / Output Lengths
//================================================================================
int BytesToReceive(enum CMD_SET command)
{
    uint8_t bytesToReceive;
    // decide bytes we should read per transmission
    // default is one byte i.e just the command opcode 
     switch (command)
     {
        case SET_Pianola:
            bytesToReceive = 6;
            break;
 
        case SET_PianolaFinalPos:
            bytesToReceive = 2;
            break;
 
        case SET_PianolaRepeatFrom:
            bytesToReceive = 2;
            break;

        case SET_PianolaRepeating:
            bytesToReceive = 2;
            break;
 
        case SET_PianolaFireTime:
            bytesToReceive = 4;
            break;
 
        case SET_PinSource:
            bytesToReceive = 4;
            break;
 
        case GET_PinSource:
            bytesToReceive = 2;
            break;
 
        case SET_CameraClk:
            bytesToReceive = 5;
            break;
 
        case SET_PIVParams:
            bytesToReceive = 19;
            break;

        case ENABLE_PIV:
            bytesToReceive = 3;
            break;

        default:
            bytesToReceive = 1;
            break;
    };
    return bytesToReceive;
}
static uint32_t read_be32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24)
         | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)
         | (uint32_t)p[3];
}

void CommandParser(unsigned char *ptrDataReceived, unsigned char *ptrDataResponse)
{
    // Large function that implements the parsing of each opcode that we could be sent, and acting on that command
    enum CMD_SET opcode = ptrDataReceived[0] & 0xFF;
    uint8_t bytesToRespond = 0;
    switch(opcode)
    {
        case SET_Pianola:
            uint32_t pianolaSetAddress = ptrDataReceived[1];
            uint32_t pianolaPinMask = ptrDataReceived[2];
            uint32_t pianolaDuration = ptrDataReceived[3] << 16 | ptrDataReceived[4] << 8 | ptrDataReceived[5] & 0xFF;
            uint64_t pianolaEntry = 0;
            pianolaEntry += pianolaPinMask;
            pianolaEntry <<= 32;
            pianolaEntry += pianolaDuration;
            pianola[pianolaSetAddress] = pianolaEntry;
            break;

        case SET_PianolaFinalPos:
            pianolaFinalPosition = ptrDataReceived[1] & 0xFF;
            pianolaNumberOfEntries = pianolaFinalPosition + 1;
            break;

        case SET_PianolaRepeatFrom:
            pianolaRepeatAddr = ptrDataReceived[1] & 0xFF;
            break;

        case SET_PianolaRepeating:
            pianolaRepeatFlag = ptrDataReceived[1] & 0x1; // flag 
            break;

        case RUN_Pianola:
            bytesToRespond = 3;
            startPianola();
            pianolaTime = getPianolaTime();
            updatePianolaStatusOutputs();
            ptrDataResponse[1] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[2] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[3] = pianolaTime & 0xFF;
            break;

        case SET_PianolaFireTime:
            bytesToRespond = 4;
            uint32_t proposedFireTime = (ptrDataReceived[1] << 16) | (ptrDataReceived[2] << 8) | (ptrDataReceived[3]);

            pianolaTime = getPianolaTime();
            // A rejected request also supersedes any previously armed start.
            pianolaFirePending = isTimeInFuture(pianolaTime, proposedFireTime);
            pianolaFireTime = proposedFireTime;
            updatePianolaStatusOutputs();

            ptrDataResponse[1] = pianolaFirePending;
            ptrDataResponse[2] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[3] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[4] = pianolaTime & 0xFF;
            break;

        case IRQ_StopAndReset:
            stopPianola();
            break;

        case GET_CurrentPianolaTime:
            pianolaTime = getPianolaTime();
            bytesToRespond = 3;
            ptrDataResponse[1] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[2] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[3] = pianolaTime & 0xFF;
            break;

        case SET_PinSource:
            setPinSource(ptrDataReceived[1], ptrDataReceived[2], (ptrDataReceived[3] & 1u) != 0);
            break;

        case GET_PinSource:
        {
            uint pin = ptrDataReceived[1];
            bytesToRespond = 2;
            ptrDataResponse[1] = PIN_SOURCE_OFF;
            ptrDataResponse[2] = 0;
            if (pin < count_of(pianolaOutputs))
            {
                if (pinOverrides[pin].source)
                {
                    ptrDataResponse[1] = pinOverrides[pin].source;
                    ptrDataResponse[2] = pinOverrides[pin].inverted;
                }
                else
                {
                    uint32_t outputMask = 1u << (pianolaOutputs[pin].gpio - OUTPUT_0_PIN);
                    ptrDataResponse[1] = pianolaSources[pin];
                    ptrDataResponse[2] = (pianolaInversionMask & outputMask) != 0;
                }
            }
            break;
        }

        case SET_CameraClk:
            int clockIdx = ptrDataReceived[1];
            if (clockIdx < sizeof(pianolaCameraClock) / sizeof(pianolaCameraClock[0]))
            {
                pianolaCameraClock[clockIdx] = ptrDataReceived[2] << 16 | ptrDataReceived[3] << 8 | ptrDataReceived[4] & 0xFF;
            }
            break;

        case SET_PIVParams:
            pivIdx = ptrDataReceived[1] & 0xFF;
            // Network byte order; durations remain in XMOS 10 ns units.
            pivIntervalLaserPulses = read_be32(ptrDataReceived + 2);
            pivDuratonPulsePair = read_be32(ptrDataReceived + 6);
            pivIntervalPulseToExp = read_be32(ptrDataReceived + 10);
            pivCameraExposure = read_be32(ptrDataReceived + 14);
            // Exposure input bit index, not a bit mask.
            pivCameraMonitorIdx = ptrDataReceived[18] & 0xFF;
            break;

        case ENABLE_PIV:
            // Consume channel and enable parameters to preserve packet framing.
            // PIV pulse generation is not implemented, so this is a no-op.
            break;

        case GET_FirmwareVersion:
            //printf("Boo\n");
            bytesToRespond = 2;
            ptrDataResponse[1] = pianolaFirmwareVersion & 0xFF;
            ptrDataResponse[2] = pianolaFirmwareEarliestVersion & 0xFF;
            break;

        case IRQ_HARDRESET:
            hardResetPianola();
            break;

        case IRQ_DumpLog:
            bytesToRespond = RESPONSE_DEBUG_DUMP;
            break;

        default:
            //printf("Not a valid response\n");
            break;

    };
    ptrDataResponse[0] = bytesToRespond & 0xFF;

    // add response to the debug buffer
    
    unsigned char debugEntry[32] ={0};
    if (bytesToRespond > 0 && bytesToRespond < responseQWidth)
    {
        for (int i = 0; i <= bytesToRespond; i++)
        {
            debugEntry[i] = ptrDataResponse[i];
        }
        if (!queue_is_full(&dumpDebugQ))
        {
            queue_try_add(&dumpDebugQ, &debugEntry);
        }
        
    }

}

// ===============================================================================
// Pianola Manager and UART handler
//================================================================================
// Core 0 is the sole UART owner. No UART IRQ handler is installed. Each call
// does bounded work so RX and TX continue independently, even for partial packets
// or a busy transmitter. Queue backpressure retains complete packets/responses.
static unsigned char uartRxPacket[commandQWidth];
static uint uartRxUsed = 0, uartRxExpected = 0;
static unsigned char uartTxBuffer[dumpDebugQWidth * dumpDebugQDepth];
static uint uartTxUsed = 0, uartTxLength = 0;

void core0Step(void)
{
    // Preserve wire order, including an entire debug dump before later replies.
    if (uartTxUsed == uartTxLength)
    {
        unsigned char response[responseQWidth];
        uartTxUsed = uartTxLength = 0;
        if (queue_try_remove(&responseQ, response))
        {
            if (response[0] == RESPONSE_DEBUG_DUMP)
            {
                // Snapshot the available records and pad complete unused records.
                // The request travelled through responseQ, not a shared flag.
                memset(uartTxBuffer, 0, sizeof(uartTxBuffer));
                uint records = queue_get_level(&dumpDebugQ);
                for (uint i = 0; i < records; i++)
                {
                    if (!queue_try_remove(&dumpDebugQ, uartTxBuffer + i * dumpDebugQWidth))
                        break;
                }
                uartTxLength = sizeof(uartTxBuffer);
            }
            else
            {
                assert(response[0] < responseQWidth);
                uartTxLength = response[0];
                memcpy(uartTxBuffer, response + 1, uartTxLength);
            }
        }
    }
    for (uint budget = 0; budget < 32 && uartTxUsed < uartTxLength; budget++)
    {
        if (!uart_is_writable(UART_ID)) break;
        uart_putc_raw(UART_ID, uartTxBuffer[uartTxUsed++]);
    }

    for (uint budget = 0; budget < 32; budget++)
    {
        if (uartRxExpected && uartRxUsed == uartRxExpected)
        {
            if (!queue_try_add(&commandQ, uartRxPacket)) break;
            uartRxUsed = uartRxExpected = 0;
        }
        if (!uart_is_readable(UART_ID)) break;
        unsigned char byte = uart_getc(UART_ID);
        if (!uartRxUsed)
        {
            memset(uartRxPacket, 0, sizeof(uartRxPacket));
            uartRxExpected = BytesToReceive(byte);
            assert(uartRxExpected > 0 && uartRxExpected <= sizeof(uartRxPacket));
        }
        uartRxPacket[uartRxUsed++] = byte;
    }
    // A packet completed on the last byte of this call's budget is ready now.
    if (uartRxExpected && uartRxUsed == uartRxExpected &&
        queue_try_add(&commandQ, uartRxPacket))
        uartRxUsed = uartRxExpected = 0;
}

// One iteration is shared by the device loop and deterministic native tests.
// Keep all playback decisions here; the test shim substitutes peripherals only.
void pianolaManagerStep(void)
{
    pianolaTime = getPianolaTime();
    // Fire on the target tick or on the next poll if the target was missed.
    if (pianolaFirePending && !isTimeInFuture(pianolaTime, pianolaFireTime))
    {
        // Cancellation happens at firing, not when a future start is armed.
        startPianola();
        updatePianolaStatusOutputs();
    }

    // Never block playback on a full response queue or lose a pending reply.
    // These buffers belong to core 1 only; queue operations provide publication.
    static unsigned char pendingResponse[responseQWidth];
    static bool responsePending = false;
    if (responsePending && queue_try_add(&responseQ, pendingResponse))
        responsePending = false;
    if (!responsePending)
    {
        unsigned char command[commandQWidth];
        if (queue_try_remove(&commandQ, command))
        {
            memset(pendingResponse, 0, sizeof(pendingResponse));
            CommandParser(command, pendingResponse);
            if (pendingResponse[0] && !queue_try_add(&responseQ, pendingResponse))
                responsePending = true;
        }
    }
    //if theres an instruction to execute / we're currently running
    if (pianolaIsRunning() && pio_sm_is_tx_fifo_empty(pioBlock, smPinDriver))
    {
        if (pianolaPosition <= pianolaFinalPosition)
        {
            queuePianolaEntry(pianolaPosition);
            pianolaPosition++;
        }
        else if (pianolaRepeatFlag && pianolaRepeatAddr <= pianolaFinalPosition)
        {
            // Validate at the boundary, not when receiving configuration:
            // uploads may set repeatFrom before final, and live edits can
            // invalidate a previously valid range. Invalid ranges finish the
            // current pass normally, retaining its final pattern and settings.
            pianolaStatus = REPEATING;
            pianolaPosition = pianolaRepeatAddr;
        }
        else if (pio_sm_get_pc(pioBlock, smPinDriver) ==
                 offsetPinDriver + pinDriver_wrap_target)
        {
            // FIFO empty means "consumed", not "finished". Only mark ready
            // once the final entry has returned to the driver's wait loop.
            pianolaPosition = 0;
            pianolaStatus = READY;
            refreshIdlePinRouting();
            updatePianolaStatusOutputs();
        }
    }
    updateOneHzClock();
}

void pianolaManagerCore1(void)
{
    while (true)
    {
        pianolaManagerStep();
    }
}

void stabilisePianolaClock(void)
{
    // the clock statemachine is a bit weird for the first 30-100ms / first few gets
    // just going to gettime a few times and then discard
    for (int i = 0; i < 10; i++)
    {
        pianolaTime = getPianolaTime();
        sleep_us(10000);
    }
}
#ifdef TIMINGBOX_HOST_TEST
#define main firmware_main
#endif
void main(void)
{
    // Core 0 owns UART RX/TX; core 1 owns commands and playback.
    // Only queues cross the core boundary.

    stdio_init_all();

    // create queues to store commands we want to shuffle between cores
    // commandQ stores a single char array of 32 bytes containing data received over UART
    // responseQ stores a single 32byte char array which will contain the reponse to any command.
    queue_init(&commandQ, commandQWidth*sizeof(unsigned char), commandQDepth);
    queue_init(&responseQ, responseQWidth*sizeof(unsigned char), responseQDepth);
    queue_init(&dumpDebugQ, dumpDebugQWidth*sizeof(unsigned char), dumpDebugQDepth);

    // init the uart and gpios
    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_irq_enables(UART_ID, false, false);
    irq_set_enabled(UART_INTERRUPT, false);

    // claim state machines for the pio programs
    //pinDriver
    uint pinSpan = STATUS_LED_BLUE - OUTPUT_0_PIN + 1;
    uint32_t outputGpioMask = 0;
    for (uint pin = 0; pin < count_of(pianolaOutputs); pin++)
        outputGpioMask |= 1u << pianolaOutputs[pin].gpio;
    bool pioTriggerStart = pio_claim_free_sm_and_add_program_for_gpio_range(&pinDriver_program, &pioBlock, &smPinDriver, &offsetPinDriver, OUTPUT_0_PIN, pinSpan, false);
    // pianolaClock
    bool pioClockStart = pio_claim_free_sm_and_add_program(&pianolaClock_program, &pioBlock, &smPianolaClock, &offsetPianolaClock);
    assert(pioTriggerStart & pioClockStart);

    // initialise the state machines but dont start yet
    pinDriver_program_init(pioBlock, smPinDriver, offsetPinDriver, OUTPUT_0_PIN, pinSpan, outputGpioMask, pinDriverClockDivisorFP);
    pianolaClock_program_init(pioBlock, smPianolaClock, offsetPianolaClock, pianolaClockDivisorFP);
    resetPinSources();

    // start both state machines in sync
    pio_enable_sm_mask_in_sync(pioBlock, (1ULL << smPinDriver)|(1ULL << smPianolaClock));

    // launch the other core
    multicore_launch_core1(&pianolaManagerCore1);

    // Poll independently of received bytes, so an isolated query always gets
    // its response serviced. No cross-core status polling or recursive restart.
    while (true)
    {
        core0Step();
        tight_loop_contents();
    }
}
