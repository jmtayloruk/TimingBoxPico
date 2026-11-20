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

// GPIO pin index used for each output pin as indexed in GET_PinSource.
// (n.b. currently the source for each pin is hard-mapped to the equivalent pianola bit)
static const uint pianolaOutputPins[] = {
    OUTPUT_0_PIN, OUTPUT_1_PIN, OUTPUT_2_PIN, OUTPUT_3_PIN, ONBOARD_LED_PIN
};

// ===============================================================================
// Set up some pianola attributes
//================================================================================

queue_t responseQ;
queue_t commandQ;
queue_t dumpDebugQ;
bool dumpDebugFlag=false;

#define responseQWidth 32
#define commandQWidth 32

#define dumpDebugQWidth 32
#define dumpDebugQDepth 32

#define pianolaTimeBaseUS 2.56 // set to match the xmos board
enum STATUS
{
    INIT = 0,
    READY,
    PENDING,
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

// takes 4 cycles (approx. 4*2.56us) of the pio block to go from picking up the next instruction to actually 
// driving pins - if we're within 10us just fire. This makes the future fire command a wee bit more accurate
signed int delT;
const uint32_t pianolaFireTimeThresholdUS = 10; 
int pianolaFireTimeThresholdTicks = (int) (pianolaFireTimeThresholdUS / pianolaTimeBaseUS);
uint8_t pianolaRepeatAddr = 0;
volatile uint32_t pianolaFireTime = 0;

// fixed at a 2.56us timebase to mimic the xmos
// clock divisors represents as 24bit Fixed Point Ints - 16 Integral Bit, 8 Frac bits
// If they need to change is relatively straightorward to do this by hand - coding its a bit of a faff
uint32_t pianolaClockDivisorFP = 0x02666; // 38.4X clockdivider (38.3984375 due to rounding.)
uint32_t pinDriverClockDivisorFP = 0x000C00; // 12X clockdivider
// 12X divider since we also delay each trigger duration by 32 ticks => 384X effective divider giving us the 2.56us tick length.
// trigger durations are zero indexed so remove one tick from each requested trigger.
const int pianolaPinTriggerOffset = 1;

enum STATUS pianolaStatus = INIT;

// Some values to keep the current version of spim-interface happy

// firmware version for type 3 box
int pianolaFirmwareVersion = 6;
int pianolaFirmwareEarliestVersion = 6;

// XMOS protocol source index for constant low
// We report this back in response to a GET_PinSource command for a pin we don't implement.
#define PIN_SOURCE_OFF 29

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
// Helper Functions That need to be decalred before the parsers
//================================================================================

void updateStatusLED(void)
{
    if (pianolaStatus == RUNNING)
    {
        gpio_put(STATUS_LED_GREEN, true);
        gpio_put(STATUS_LED_BLUE, false);
    }
    else if (pianolaStatus == PENDING)
    {
        gpio_put(STATUS_LED_GREEN, false);
        gpio_put(STATUS_LED_BLUE, true);
    }
    else if (pianolaStatus == PANIC | pianolaStatus == RESET)
    {
        gpio_put(STATUS_LED_GREEN, true);
        gpio_put(STATUS_LED_BLUE, true);
    }
    else
    {
        gpio_put(STATUS_LED_GREEN, false);
        gpio_put(STATUS_LED_BLUE, false); 
    }
}

uint32_t getPianolaTime()
{
    pio_sm_put_blocking(pioBlock, smPianolaClock, 0xF);
    uint32_t t = 0xFFFFFF - pio_sm_get_blocking(pioBlock, smPianolaClock);
    pio_sm_drain_tx_fifo(pioBlock, smPianolaClock);
    return t;
}

int isTimeInFuture(uint32_t currentTime, uint32_t proposedTime)
{
    // Compare modulo 2^24 using the XMOS half-range convention.
    uint32_t delta = (proposedTime - currentTime) & 0xFFFFFFu;
    return delta != 0 && delta < 0x800000u;
}
int shouldRunPianolaSoon(uint32_t currentTime, uint32_t proposedTime)
{
    // Include the target tick in the advance window, including across wraparound.
    uint32_t delta = (proposedTime - currentTime) & 0xFFFFFFu;
    return delta <= (uint32_t)pianolaFireTimeThresholdTicks;
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

uint32_t remapPinMask(unsigned char pianolaMask)
{
    // PIO output bits are relative to the first BNC GPIO.
    uint32_t pinMask = 0;
    for (uint bit = 0; bit < count_of(pianolaOutputPins); bit++)
    {
        pinMask |= ((pianolaMask >> bit) & 0x1) << (pianolaOutputPins[bit] - OUTPUT_0_PIN);
    }
    return pinMask;
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
            uint32_t pianolaPinMask = remapPinMask(ptrDataReceived[2]);
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
            pianolaStatus = RUNNING;
            pianolaPosition = 0;
            pianolaTime = getPianolaTime();
            ptrDataResponse[1] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[2] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[3] = pianolaTime & 0xFF;
            break;

        case SET_PianolaFireTime:
            bytesToRespond = 4;
            uint32_t proposedFireTime = (ptrDataReceived[1] << 16) | (ptrDataReceived[2] << 8) | (ptrDataReceived[3]);

            int validFireTime = isTimeInFuture(pianolaTime, proposedFireTime);
            if (validFireTime)
            {
                pianolaFireTime = proposedFireTime;
                pianolaStatus = PENDING;
            }

            ptrDataResponse[1] =  validFireTime & 0xFF;
            ptrDataResponse[2] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[3] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[4] = pianolaTime & 0xFF;
            break;

        case IRQ_StopAndReset:
            pianolaStatus = READY;
            pianolaPosition = 0;  
            pianolaRepeatFlag = 0;
            pianolaFireTime = 0;
            break;

        case GET_CurrentPianolaTime:
            pianolaTime = getPianolaTime();
            bytesToRespond = 3;
            ptrDataResponse[1] = (pianolaTime >> 16) & 0xFF;
            ptrDataResponse[2] = (pianolaTime >> 8) & 0xFF;
            ptrDataResponse[3] = pianolaTime & 0xFF;
            break;

        case SET_PinSource:
            // Accept the XMOS packet, but do not actually do anything with it.
            // The Pico output sources are fixed, and a subsequent call to GET_PinSource
            // will just return our fixed mapping
            break;

        case GET_PinSource:
            // If we are being queried about a pin that is supported by the Pico,
            // report that it is mapped to that numerical pin source (which is one of the pianola bits)
            bytesToRespond = 2;
            ptrDataResponse[1] = ptrDataReceived[1] < count_of(pianolaOutputPins) ? ptrDataReceived[1] : PIN_SOURCE_OFF;
            ptrDataResponse[2] = 0;
            break;

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
            pianolaStatus = INIT;
            pianolaFinalPosition = 0;
            pianolaRepeatAddr = 0;
            pianolaPosition = 0;
            break;

        case IRQ_DumpLog:
            dumpDebugFlag = true;
            break;

        default:
            //printf("Not a valid response\n");
            break;

    };
    ptrDataResponse[0] = bytesToRespond & 0xFF;

    // add response to the debug buffer
    
    unsigned char debugEntry[32] ={0};
    if (bytesToRespond > 0)
    {
        for (int i = 0; i < bytesToRespond; i++)
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
void uartHandlerCore0(void)
{
    // is there data to read
    if(uart_is_readable(UART_ID))
    {
        // get first char and parse to decide how many more bytes to read
        uint8_t opcode = uart_getc(UART_ID);
        unsigned char * inBufferCore0 = malloc(32 * sizeof(unsigned char));
        uint8_t numBytes = BytesToReceive(opcode);
        *(inBufferCore0) = opcode & 0xFF;
        // get the rest of the characters
        for (int i = 1; i < numBytes; i++)
        {
            *(inBufferCore0 + i) = uart_getc(UART_ID);
        }
        // add to the command queue
        queue_try_add(&commandQ, inBufferCore0);
        free(inBufferCore0);
    }
    if (!queue_is_empty(&responseQ))
    {
        // if theres a response waiting, then remove it from the qeueue and send it to the host.
        unsigned char* outBufferCore0 = malloc(32*sizeof(unsigned char));
        queue_try_remove(&responseQ, outBufferCore0);
        int numBytesToWrite = *outBufferCore0;
        uart_write_blocking(UART_ID, (outBufferCore0+1), numBytesToWrite);

        free(outBufferCore0);
    }
    if (dumpDebugFlag)
    {
        // send out all 32 entries of 32 bytes back to the host. 
        // since its a queue we'll just send 0s after we exhaust all the entries

        int numEntries = queue_get_level(&dumpDebugQ);
        int numBytesToWrite = 32;
        unsigned char* outDebugCore0 = malloc(dumpDebugQWidth * dumpDebugQDepth * sizeof(unsigned char));
        for (int i = 0; i < numEntries; i++)
        {
            queue_try_remove(&dumpDebugQ, outDebugCore0+(i* dumpDebugQDepth));
        }
        if (numEntries < dumpDebugQDepth)
        {
            //write zeroes to the remaining slots in the malloced buffer
            for (int i = numEntries; i < dumpDebugQDepth; i++)
            {
                outDebugCore0[i] = 0x00;
            }
        }
        uart_write_blocking(UART_ID, outDebugCore0, 1024);
        free(outDebugCore0);
        dumpDebugFlag = false;      
    }
}

// One iteration is shared by the device loop and deterministic native tests.
// Keep all playback decisions here; the test shim substitutes peripherals only.
void pianolaManagerStep(void)
{
    updateStatusLED();
    // if its time to change pianola state, we should do that now
    pianolaTime = getPianolaTime();
    // if we're not running an pianola and we should be soon then we should get a move on
    if ((pianolaStatus != RUNNING) && (pianolaStatus == PENDING) && (shouldRunPianolaSoon(pianolaTime, pianolaFireTime)))
    {
        //start running from the first pianola entry
        pianolaStatus = RUNNING;
        pianolaPosition = 0;

        // reset pianolaFireTime so we don't fire again when the clock wraps around
        pianolaFireTime = 0;
    }

    // if theres a command sitting waiting to be processed
    if (!queue_is_empty(&commandQ))
    {
        // allocate buffers for data we've just received, and the response
        unsigned char* inBufferCore1 = malloc(32*sizeof(unsigned char));
        unsigned char* outBufferCore1 = malloc(32*sizeof(unsigned char));
        bool removeFlagCore1 = queue_try_remove(&commandQ, inBufferCore1);

        //process the command and form the response
        CommandParser(inBufferCore1, outBufferCore1);

        // if we have a response add it too the queue
        bool addFlagCore1 = false;

        if (*outBufferCore1 > 0)
        {
            addFlagCore1 = queue_try_add(&responseQ, outBufferCore1);
        }

        //free buffers
        free(inBufferCore1);
        free(outBufferCore1);

        //hand over to UART interrupt handler
        uartHandlerCore0();
    }
    //if theres an instruction to execute / we're currently running
    if ((pianolaStatus == RUNNING | pianolaStatus == REPEATING) && pio_sm_is_tx_fifo_empty(pioBlock, smPinDriver))
    {
        if (pianolaPosition <= pianolaFinalPosition)
        {
            uint32_t duration = pianola[pianolaPosition] & 0xFFFFFFFF;
            uint32_t pinMask = pianola[pianolaPosition] >> 32;
            pio_sm_put(pioBlock, smPinDriver, duration);
            pio_sm_put(pioBlock, smPinDriver, pinMask);
            pianolaPosition++;
        }
        // if we get to the end of the pianola
        // stop running if we're not repeating
        else if(pianolaStatus == REPEATING)
        {
            // if we want to repeat then set to the repeat from address
            // this defaults to 0 so its the same as going to the start
            // else if we've set it from a command, repeat from that address till heat death
            pianolaPosition = pianolaRepeatAddr;
        }
        else if (pianolaRepeatFlag)
        {
            // we were running but want to start repeating
            pianolaStatus = REPEATING;
            pianolaPosition = pianolaRepeatAddr;
            pianolaRepeatFlag = 0;
        }
        else // pianolaStatus was running and we should now stop
        {
            pianolaPosition = 0;
            pianolaStatus = READY;
        }
    }
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
    // This is Core0 which handles basic setup of hardware and then runs an interrupt 
    // to handle reading incoming commands and sending out responses
    // Core 1 handles the running of the pianola and some command handling as well.

    stdio_init_all();

    // Red and Green channels of the status LED
    gpio_init(STATUS_LED_GREEN);
    gpio_set_dir(STATUS_LED_GREEN, true);
    gpio_init(STATUS_LED_BLUE);
    gpio_set_dir(STATUS_LED_BLUE, true);

    // create queues to store commands we want to shuffle between cores
    // commandQ stores a single char array of 32 bytes containing data received over UART
    // responseQ stores a single 32byte char array which will contain the reponse to any command.
    queue_init(&commandQ, commandQWidth*sizeof(unsigned char), 1);
    queue_init(&responseQ, responseQWidth*sizeof(unsigned char), 1);
    queue_init(&dumpDebugQ, dumpDebugQWidth*sizeof(unsigned char), dumpDebugQDepth);

    // init the uart and gpios
    uart_init(UART_ID, BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    irq_set_exclusive_handler(UART_INTERRUPT, uartHandlerCore0);
    irq_set_enabled(UART_INTERRUPT, true);
    uart_set_irq_enables(UART_ID, true, false);

    // claim state machines for the pio programs
    //pinDriver
    uint pinSpan = ONBOARD_LED_PIN - OUTPUT_0_PIN + 1;
    uint32_t outputGpioMask = remapPinMask(UINT8_MAX) << OUTPUT_0_PIN;
    bool pioTriggerStart = pio_claim_free_sm_and_add_program_for_gpio_range(&pinDriver_program, &pioBlock, &smPinDriver, &offsetPinDriver, OUTPUT_0_PIN, pinSpan, false);
    // pianolaClock
    bool pioClockStart = pio_claim_free_sm_and_add_program(&pianolaClock_program, &pioBlock, &smPianolaClock, &offsetPianolaClock);
    assert(pioTriggerStart & pioClockStart);

    // initialise the state machines but dont start yet
    pinDriver_program_init(pioBlock, smPinDriver, offsetPinDriver, OUTPUT_0_PIN, pinSpan, outputGpioMask, pinDriverClockDivisorFP);
    pianolaClock_program_init(pioBlock, smPianolaClock, offsetPianolaClock, pianolaClockDivisorFP);

    // start both state machines in sync
    pio_enable_sm_mask_in_sync(pioBlock, (1ULL << smPinDriver)|(1ULL << smPianolaClock));

    // launch the other core
    multicore_launch_core1(&pianolaManagerCore1);

    // enter a tight loop while were not in a reset or panic mode
    while(pianolaStatus < 6) // is pianolaStatus not RESET or PANIC
    {
        tight_loop_contents();
    }
    // after we reset, everything else will be cleared to just auto reboot everything
    main();
}

