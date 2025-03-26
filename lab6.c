// Header Files
#include <ti/devices/msp/msp.h>
#include "lab6_helper.h"
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

// Macro Definitions & Constants
#define MAX_SEQUENCE       5          // Number of correct responses needed to win
#define TIMEOUT_THRESHOLD  3          // Timeout threshold before triggering loss animation

// LED SPI Packet definitions for controlling the LED array
// (Each packet defines the LED colors for the array.)
uint16_t redPacket[] = {
    0x0, 0x0,
    0xE000, 0x0000,
    0xE500, 0x00FF,   // Second LED: bright red
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xFFFF, 0xFFFF
};
uint16_t orangePacket[] = {
    0x0, 0x0,
    0xE500, 0x0FF0,   // First LED: orange
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xFFFF, 0xFFFF
};
uint16_t greenPacket[] = {
    0x0, 0x0,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE500, 0xFF00,   // Third LED: green
    0xE000, 0x0000,
    0xFFFF, 0xFFFF
};
uint16_t yellowPacket[] = {
    0x0, 0x0,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE500, 0xFFFF,   // Fourth LED: yellow
    0xFFFF, 0xFFFF
};
uint16_t emptyPacket[] = { // All LEDs off
    0x0, 0x0,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xE000, 0x0000,
    0xFFFF, 0xFFFF
};

// Song definitions for "Mary Had a Little Lamb"
#define NOTE_E4   330 // E4 note frequency
#define NOTE_D4   294 // D4 note frequency
#define NOTE_C4   262 // C4 note frequency
#define SONG_LENGTH 26 // Number of notes in the song

// Mary Had a Little Lamb song notes and durations
static const uint16_t maryNotes[SONG_LENGTH] = {
    NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4,
    NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4,
    NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_C4,
    NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4
};
static const uint16_t maryDurations[SONG_LENGTH] = {
    500, 500, 500, 500, 500, 500, 1000,
    500, 500, 1000, 500, 500, 1000,
    500, 500, 500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 2000
};

// Tone frequencies for each button/LED
#define BEEP_ORANGE  (392*2)
#define BEEP_RED     (658*2)
#define BEEP_GREEN   (782*2)
#define BEEP_YELLOW  (524*2)
#define BEEP_DURATION_MS  200  // Duration (ms) of button tone

// Global Variables

// SPI transmission variables
uint16_t *txPacket;
int transmissionComplete = 0;
int idx = 0;
int message_len = sizeof(emptyPacket) / sizeof(emptyPacket[0]);

// Simon game state variables
uint8_t simonSequence[MAX_SEQUENCE];
uint8_t sequenceLength = 0;
uint8_t userIndex = 0;

// State machine for game operation
enum current_state_enum {
    INIT_SONG,           // Start by playing a song
    SIMON_SEQUENCE,      // Display the Simon sequence
    SIMON_INPUT,         // Waiting for user input
    LOSS_STATE,          // User made an incorrect move
    WIN_STATE,           // User wins (completed sequence)
    WAIT_FOR_RESTART,    // Waiting for button press to restart game
    LIGHTS_OFF,          // All LEDs off
} currentState;

// Function Prototypes

uint32_t GenerateRandomNumber(void); // TRNG based random number generator

static void playNote(uint16_t freq, uint16_t msDuration); // Play a tone
static void playMaryHadALittleLamb(void); // Play the song
static void playWinAnimation(void); // Win animation
static void startTone(uint16_t freq); // Start a tone
static void stopTone(void); // Stop tone
void sendSPIPacket(uint16_t *packet); // Send LED packet
void displaySimonSequence(void); // Display Simon sequence

void processUserInput(uint8_t buttonPressed); // Process user input

// Function Definitions

// Generates a random number using the TRNG peripheral.
uint32_t GenerateRandomNumber(void)
{
    int randomNumber = 0; // Placeholder for the random number
    // Power on and initialize the TRNG peripheral.
    TRNG->GPRCM.RSTCTL = TRNG_RSTCTL_RESETASSERT_ASSERT | TRNG_RSTCTL_KEY_UNLOCK_W;
    TRNG->GPRCM.PWREN = TRNG_PWREN_KEY_UNLOCK_W | TRNG_PWREN_ENABLE_ENABLE;
    delay_cycles(POWER_STARTUP_DELAY);

    // Set clock division to 2.
    TRNG->CLKDIVIDE = (uint32_t)TRNG_CLKDIVIDE_RATIO_DIV_BY_2;

    // Send "Normal Function" command and wait until it's done.
    update_reg(&TRNG->CTL, (uint32_t)TRNG_CTL_CMD_NORM_FUNC, TRNG_CTL_CMD_MASK);
    while (!(TRNG->CPU_INT.RIS & TRNG_RIS_IRQ_CMD_DONE_MASK))
        ;  // Busy-wait for command completion
    TRNG->CPU_INT.ICLR = TRNG_IMASK_IRQ_CMD_DONE_MASK;  // Clear command interrupt

    // Set clock decimation to 4.
    update_reg(&TRNG->CTL, ((uint32_t)0x3 << TRNG_CTL_DECIM_RATE_OFS), TRNG_CTL_DECIM_RATE_MASK);

    // Start capture and wait until the result is ready.
    while (!(TRNG->CPU_INT.RIS & TRNG_RIS_IRQ_CAPTURED_RDY_MASK))
        ;  // Busy-wait for capture readiness
    TRNG->CPU_INT.ICLR = TRNG_IMASK_IRQ_CAPTURED_RDY_MASK;  // Clear capture interrupt
    randomNumber = TRNG->DATA_CAPTURE;  // Retrieve the captured random number

    // Power off the TRNG peripheral.
    TRNG->GPRCM.PWREN = TRNG_PWREN_KEY_UNLOCK_W | TRNG_PWREN_ENABLE_DISABLE;

    return (uint32_t)randomNumber; // Return the random number
}

// Sound and LED functions

// playNote: Plays a tone at a given frequency for a specified duration (in ms).
static void playNote(uint16_t freq, uint16_t msDuration)
{
    if (freq > 0) { // If frequency is 0, do not play a tone.

        // Set timer parameters for tone generation.
        TIMA1->COUNTERREGS.LOAD = 8000000 / freq;
        TIMA1->COUNTERREGS.CC_01[0] = (TIMA1->COUNTERREGS.LOAD + 1) / 2;
        TIMA1->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    }
    // Delay for the duration of the note.
    delay_cycles((uint32_t)msDuration * 16000);
    TIMA1->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_EN_ENABLED);
    delay_cycles(20000);
}

// playMaryHadALittleLamb: Plays the song while cycling through LED packets.
static void playMaryHadALittleLamb(void)
{
    // Create an array of LED packets (colors) to cycle through (just loops through the 4)
    const uint8_t *ledPackets[] = {orangePacket, redPacket, greenPacket, yellowPacket};

    for (int i = 0; i < SONG_LENGTH; i++) {
        // If any button is pressed, abort the song and begin gameplay.
        if (!(GPIOA->DIN31_0 & SW1) || !(GPIOA->DIN31_0 & SW2) ||
            !(GPIOA->DIN31_0 & SW3) || !(GPIOA->DIN31_0 & SW4)) {
            return;
        }
        // Display a color by sending the LED packet.
        sendSPIPacket((uint16_t *)ledPackets[i % 4]);
        // Play the corresponding note.
        playNote(maryNotes[i], maryDurations[i]);
        // Turn off LEDs.
        sendSPIPacket(emptyPacket);
        // Short pause between notes.
        delay_cycles(5000000);
    }
}

// playWinAnimation: Displays a win animation by cycling through colors with tones.
static void playWinAnimation(void)
{
    for (int i = 0; i < 3; i++) {
        sendSPIPacket(orangePacket);
        playNote(BEEP_ORANGE, BEEP_DURATION_MS);
        sendSPIPacket(redPacket);
        playNote(BEEP_RED, BEEP_DURATION_MS);
        sendSPIPacket(greenPacket);
        playNote(BEEP_GREEN, BEEP_DURATION_MS);
        sendSPIPacket(yellowPacket);
        playNote(BEEP_YELLOW, BEEP_DURATION_MS);
    }
    // Turn off LEDs at the end.
    sendSPIPacket(emptyPacket);
}

// startTone: Begins generating a tone at the specified frequency without blocking.
static void startTone(uint16_t freq)
{
    if (freq > 0) {
        TIMA1->COUNTERREGS.LOAD = 8000000 / freq;
        TIMA1->COUNTERREGS.CC_01[0] = (TIMA1->COUNTERREGS.LOAD + 1) / 2;
        TIMA1->COUNTERREGS.CTRCTL |= GPTIMER_CTRCTL_EN_ENABLED;
    }
}

// stopTone: Stops tone generation.
static void stopTone(void)
{
    TIMA1->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_EN_ENABLED);
    // Short delay to ensure tone stops cleanly.
    delay_cycles(20000);
}

// sendSPIPacket: Sends a packet over SPI to update the LED array.
void sendSPIPacket(uint16_t *packet)
{
    txPacket = packet;
    NVIC_ClearPendingIRQ(SPI0_INT_IRQn);
    NVIC_EnableIRQ(SPI0_INT_IRQn);
    transmissionComplete = 0;
    idx = 1;
    SPI0->TXDATA = txPacket[0];
    while (!transmissionComplete) {
        __WFI();
    }
}

// displaySimonSequence: Iterates over the Simon sequence and displays each move.
void displaySimonSequence(void)
{
    for (int i = 0; i < sequenceLength; i++) {
        switch (simonSequence[i]) {
            case 1: // Orange
                txPacket = orangePacket;
                playNote(BEEP_ORANGE, BEEP_DURATION_MS * 2);
                break;
            case 2: // Red
                txPacket = redPacket;
                playNote(BEEP_RED, BEEP_DURATION_MS * 2);
                break;
            case 3: // Green
                txPacket = greenPacket;
                playNote(BEEP_GREEN, BEEP_DURATION_MS * 2);
                break;
            case 4: // Yellow
                txPacket = yellowPacket;
                playNote(BEEP_YELLOW, BEEP_DURATION_MS * 2);
                break;
        }
        sendSPIPacket(txPacket);
        delay_cycles(2000000);
        sendSPIPacket(emptyPacket);
        delay_cycles(1000000);
    }
}

// processUserInput: Checks the user button press against the Simon sequence.
void processUserInput(uint8_t buttonPressed)
{
    if (buttonPressed == simonSequence[userIndex]) { // Correct input
        userIndex++;
        if (userIndex == sequenceLength) { // Completed sequence
            delay_cycles(1000000);
            if (sequenceLength < MAX_SEQUENCE) { // Continue game if not at max sequence length.
                // Add a new random move to the sequence.
                simonSequence[sequenceLength] = (rand() % 4) + 1; // Random move (1-4)
                sequenceLength++;
                userIndex = 0;
                currentState = SIMON_SEQUENCE;
            } else {
                // Win condition achieved.
                currentState = WIN_STATE;
            }
        }
    } else {
        // Wrong input: transition to loss state.
        currentState = LOSS_STATE;
    }
}

// Initializes hardware, seeds the RNG, and runs the game state machine.
int main(void)
{
    // Initialization routines.
    InitializeProcessor();
    InitializeGPIO();
    InitializeSPI();
    InitializeTimerG0();
    InitializeTimerA1_PWM();

    // Set up timer interrupt with 10 ms period.
    NVIC_EnableIRQ(TIMG0_INT_IRQn);
    TIMG0->COUNTERREGS.LOAD = 160000;  // 10 ms period
    TIMG0->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);

    // Seed the random number generator with a value from the TRNG.
    uint32_t hwSeed = GenerateRandomNumber(); // Get random number from TRNG
    srand((unsigned int)(hwSeed & 0xFFFF));  // Use lower 16 bits

    // Initialize game variables.
    sequenceLength = 1;
    simonSequence[0] = (rand() % 4) + 1;  // First random move (1-4)
    userIndex = 0;
    currentState = INIT_SONG;

    // Main game loop.
    while (1) {
        if (currentState == INIT_SONG) { // Initial song state
            // Play initial song and then display the Simon sequence.
            playMaryHadALittleLamb();
            currentState = SIMON_SEQUENCE;
        }
        else if (currentState == SIMON_SEQUENCE) { // Display Simon sequence
            displaySimonSequence();
            userIndex = 0;
            currentState = SIMON_INPUT;
        }
        else if (currentState == SIMON_INPUT) { // Waiting for user input
            // Process button presses for user input.
            static uint32_t timeoutCounter = 0;
            if (!(GPIOA->DIN31_0 & SW1)) { // SW1 pressed
                timeoutCounter = 0;
                sendSPIPacket(orangePacket);
                startTone(BEEP_ORANGE);

                // Wait until SW1 is released.
                while (!(GPIOA->DIN31_0 & SW1)) { __WFI(); }
                stopTone();
                sendSPIPacket(emptyPacket);
                processUserInput(1);
                delay_cycles(2000000); // Debounce delay
            }
            else if (!(GPIOA->DIN31_0 & SW2)) { // SW2 pressed
                timeoutCounter = 0;
                sendSPIPacket(redPacket);
                startTone(BEEP_RED);
                while (!(GPIOA->DIN31_0 & SW2)) { __WFI(); }
                stopTone();
                sendSPIPacket(emptyPacket);
                processUserInput(2);
                delay_cycles(2000000);
            }
            else if (!(GPIOA->DIN31_0 & SW3)) { // SW3 pressed
                timeoutCounter = 0;
                sendSPIPacket(greenPacket);
                startTone(BEEP_GREEN);
                while (!(GPIOA->DIN31_0 & SW3)) { __WFI(); }
                stopTone();
                sendSPIPacket(emptyPacket);
                processUserInput(3);
                delay_cycles(2000000);
            }
            else if (!(GPIOA->DIN31_0 & SW4)) { // SW4 pressed
                timeoutCounter = 0;
                sendSPIPacket(yellowPacket);
                startTone(BEEP_YELLOW);
                while (!(GPIOA->DIN31_0 & SW4)) { __WFI(); }
                stopTone();
                sendSPIPacket(emptyPacket);
                processUserInput(4);
                delay_cycles(2000000);
            }
            else {
                // If no button is pressed, increment timeout counter.
                timeoutCounter++;
                if (timeoutCounter > TIMEOUT_THRESHOLD) { // Timeout threshold reached
                    currentState = LOSS_STATE; // Transition to loss state
                    timeoutCounter = 0;
                }
            }
        }
        else if (currentState == WIN_STATE) { // User wins
            // Play win animation and then wait for restart.
            playWinAnimation();
            currentState = WAIT_FOR_RESTART;
        }
        else if (currentState == LOSS_STATE) { // User loses
            // Perform loss animation with flashing colors and tones.
            for (int i = 0; i < 3; i++) {
                sendSPIPacket(orangePacket);
                startTone(BEEP_ORANGE);
                delay_cycles(200000);
                stopTone();
                sendSPIPacket(emptyPacket);
                delay_cycles(200000);

                sendSPIPacket(redPacket);
                startTone(BEEP_RED);
                delay_cycles(200000);
                stopTone();
                sendSPIPacket(emptyPacket);
                delay_cycles(200000);

                sendSPIPacket(greenPacket);
                startTone(BEEP_GREEN);
                delay_cycles(200000);
                stopTone();
                sendSPIPacket(emptyPacket);
                delay_cycles(200000);

                sendSPIPacket(yellowPacket);
                startTone(BEEP_YELLOW);
                delay_cycles(200000);
                stopTone();
                sendSPIPacket(emptyPacket);
                delay_cycles(200000);
            }
            // Play a descending tone sequence .
            uint16_t lossMelody[] = {BEEP_RED, BEEP_ORANGE, BEEP_GREEN, BEEP_YELLOW};
            for (int i = 0; i < 4; i++) {
                sendSPIPacket(redPacket);
                playNote(lossMelody[i], 1500);
                sendSPIPacket(emptyPacket);
                delay_cycles(100000);
            }
            currentState = WAIT_FOR_RESTART; // Wait for restart
        }
        else if (currentState == WAIT_FOR_RESTART) {
            // Wait for all buttons to be released.
            while ((GPIOA->DIN31_0 & (SW1 | SW2 | SW3 | SW4)) != (SW1 | SW2 | SW3 | SW4))
                __WFI();
            // Then wait for a button press to restart.
            while ((GPIOA->DIN31_0 & (SW1 | SW2 | SW3 | SW4)) == (SW1 | SW2 | SW3 | SW4))
                __WFI();
            // Reset game variables.
            sequenceLength = 1;
            simonSequence[0] = (rand() % 4) + 1;
            userIndex = 0;
            currentState = INIT_SONG;
        }
        else if (currentState == LIGHTS_OFF) {
            // Turn off all LEDs.
            sendSPIPacket(emptyPacket);
        }
        __WFI(); // Wait for next interrupt
    }
}

// SPI0_IRQHandler: Manages SPI transmission of LED data.
void SPI0_IRQHandler(void)
{
    switch (SPI0->CPU_INT.IIDX) {
        case SPI_CPU_INT_IIDX_STAT_TX_EVT:
            SPI0->TXDATA = txPacket[idx++];
            if (idx == message_len) {
                transmissionComplete = 1;
                NVIC_DisableIRQ(SPI0_INT_IRQn);
            }
            break;
        default:
            break;
    }
}

// TIMG0_IRQHandler: Placeholder for periodic tasks.
void TIMG0_IRQHandler(void)
{
    switch (TIMG0->CPU_INT.IIDX) {
        case GPTIMER_CPU_INT_IIDX_STAT_Z:
            break;
        default:
            break;
    }
}
