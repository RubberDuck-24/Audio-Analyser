#include <Arduino.h>
#include <DMAChannel.h>
#include <math.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════════
// Teensy 4.1 — PCM4222 ADC receiver + PCM5102 DAC transmitter
//
// ADC (receiver, SAI1 RX):
//   Pin 8  = SAI1_RX_DATA0  ← PCM4222 data out
//   Pin 20 = SAI1_RX_SYNC   ← LRCK
//   Pin 21 = SAI1_RX_BCLK   ← BCLK
//
// DAC (transmitter, SAI1 TX):
//   Pin 7  = SAI1_TX_DATA0  → PCM5102 DIN
//   Pin 20 = SAI1_TX_SYNC   ← shared LRCK
//   Pin 21 = SAI1_TX_BCLK   ← shared BCLK
//
// IMPORTANT:
//   This build does NOT decimate.
//   Actual streamed sample rate is SOURCE_FS = 192000 Hz.
//   Therefore Python/debug tools must use FS = 192000 unless you add
//   decimation in firmware.
//
// USB diagnostic stream modes:
//   0 = normal   -> send L,R
//   1 = dup L    -> send L,L
//   2 = dup R    -> send R,R
//   3 = swap     -> send R,L
//
// Start with:
//   #define USB_STREAM_MODE 1
//
// If both Python plots then become identical, the GUI/parser is fine and the
// real problem is upstream of USB (ADC right input / routing / firmware RX).
// ═══════════════════════════════════════════════════════════════════════════

static constexpr uint32_t PACKET_MAGIC = 0x30445541u;

// ── ADC configuration ──────────────────────────────────────────────────────
#define SOURCE_FS       192000
#define BLOCK_PAIRS     512
#define HALF_WORDS      (BLOCK_PAIRS * 2)   // 1024 int32 words = 512 stereo pairs
#define TOTAL_WORDS     (HALF_WORDS * 2)
#define WARMUP_HALVES   24

// ── DAC sine test configuration ────────────────────────────────────────────
#define SINE_FREQ_HZ       1000
#define TARGET_VPP         1.0f
#define PCM5102_PEAK_V     2.9698f
#define DAC_SHIFT_BITS     8
#define USE_BCP            1   // must be 1 in your working DAC build

#define SAMPLES_PER_PERIOD (SOURCE_FS / SINE_FREQ_HZ)
#define SINE_PERIODS       10
#define SINE_BUF_PAIRS     (SAMPLES_PER_PERIOD * SINE_PERIODS)
#define SINE_BUF_WORDS     (SINE_BUF_PAIRS * 2)

#define USB_STREAM_MODE    1   // 0=normal, 1=dup L, 2=dup R, 3=swap

static constexpr float TWO_PI_F = 6.2831853071795864769f;
static const int32_t DAC_FS_24 = 8388607;

static const int32_t SINE_AMPLITUDE_24 =
    (int32_t)((TARGET_VPP / 2.0f / PCM5102_PEAK_V) * (float)DAC_FS_24);

struct PacketHeader {
    uint32_t magic;
    uint32_t words;
    uint32_t sequence;
};

// ── Buffers ────────────────────────────────────────────────────────────────
DMAMEM static int32_t dma_rx_buf[TOTAL_WORDS];
static  int32_t safe_buf[2][HALF_WORDS];
DMAMEM static int32_t sine_buf[SINE_BUF_WORDS];

// USB diagnostic mapping buffer
static int32_t usb_tx_buf[HALF_WORDS];

static DMAChannel dma_rx;
static DMAChannel dma_tx;

volatile uint8_t  ready_mask  = 0;
volatile uint32_t overruns    = 0;
volatile uint32_t halves_seen = 0;
static   uint32_t packet_seq  = 0;


// ───────────────────────────────────────────────────────────────────────────
// Generate DAC sine: 24-bit sample left-justified into 32-bit slot
// ───────────────────────────────────────────────────────────────────────────
static void generate_sine_buf() {
    for (int i = 0; i < SINE_BUF_PAIRS; i++) {
        float phase = TWO_PI_F * (float)i / (float)SAMPLES_PER_PERIOD;
        float s     = sinf(phase);

        int32_t samp24 = (int32_t)((float)SINE_AMPLITUDE_24 * s);

        if (samp24 >  8388607) samp24 =  8388607;
        if (samp24 < -8388608) samp24 = -8388608;

        int32_t word = samp24 << DAC_SHIFT_BITS;

        sine_buf[2 * i + 0] = word;  // L
        sine_buf[2 * i + 1] = word;  // R
    }

    arm_dcache_flush(sine_buf, sizeof(sine_buf));
}


// ───────────────────────────────────────────────────────────────────────────
// ADC RX ISR
// ───────────────────────────────────────────────────────────────────────────
void FASTRUN dma_rx_isr() {
    const uint32_t daddr = (uint32_t)dma_rx.TCD->DADDR;
    dma_rx.clearInterrupt();

    uint8_t completed;
    const int32_t* src;

    if (daddr < ((uint32_t)dma_rx_buf + sizeof(dma_rx_buf) / 2)) {
        completed = 1;
        src       = dma_rx_buf + HALF_WORDS;
    } else {
        completed = 0;
        src       = dma_rx_buf;
    }

    const uint8_t bit = (1u << completed);

    if (ready_mask & bit) {
        overruns++;
    } else {
        arm_dcache_delete((void*)src, HALF_WORDS * sizeof(int32_t));
        memcpy(safe_buf[completed], src, HALF_WORDS * sizeof(int32_t));
        ready_mask |= bit;
    }

    asm volatile("dsb");
}


// ───────────────────────────────────────────────────────────────────────────
// SAI1 configuration
// RX: slave from PCM4222
// TX: synchronous to RX
// ───────────────────────────────────────────────────────────────────────────
static void sai1_config() {
    CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

    // ── RX reset ──────────────────────────────────────────────────────────
    I2S1_RCSR = (1 << 24);
    delayMicroseconds(10);
    I2S1_RCSR = 0;
    delayMicroseconds(10);

    I2S1_RCR1 = I2S_RCR1_RFW(3);
    I2S1_RCR2 = I2S_RCR2_BCP;
    I2S1_RCR3 = I2S_RCR3_RCE;
    I2S1_RCR4 = I2S_RCR4_FSP | I2S_RCR4_FSE | I2S_RCR4_MF
              | I2S_RCR4_SYWD(31) | I2S_RCR4_FRSZ(1);
    I2S1_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

    // ── TX reset ──────────────────────────────────────────────────────────
    I2S1_TCSR = (1 << 24);
    delayMicroseconds(10);
    I2S1_TCSR = 0;
    delayMicroseconds(10);

    I2S1_TCR1 = I2S_TCR1_RFW(3);
    I2S1_TCR2 = I2S_TCR2_SYNC(1) | I2S_TCR2_BCP;
    I2S1_TCR3 = I2S_TCR3_TCE;
    I2S1_TCR4 = I2S_TCR4_FSP | I2S_TCR4_FSE | I2S_TCR4_MF
              | I2S_TCR4_SYWD(31) | I2S_TCR4_FRSZ(1);
    I2S1_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

    // ── Pin mux ───────────────────────────────────────────────────────────
    CORE_PIN8_CONFIG  = 3;   // SAI1_RX_DATA0
    CORE_PIN20_CONFIG = 3;   // SAI1_RX_SYNC / TX_SYNC
    CORE_PIN21_CONFIG = 3;   // SAI1_RX_BCLK / TX_BCLK
    CORE_PIN7_CONFIG  = 3;   // SAI1_TX_DATA0

    IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;
    IOMUXC_SAI1_RX_BCLK_SELECT_INPUT  = 1;
    IOMUXC_SAI1_RX_SYNC_SELECT_INPUT  = 1;
}


// ───────────────────────────────────────────────────────────────────────────
// DMA configuration
// ───────────────────────────────────────────────────────────────────────────
static void dma_config() {
    // ── RX DMA: SAI1_RDR0 → dma_rx_buf ────────────────────────────────────
    dma_rx.begin(true);
    dma_rx.TCD->SADDR    = &I2S1_RDR0;
    dma_rx.TCD->SOFF     = 0;
    dma_rx.TCD->SLAST    = 0;
    dma_rx.TCD->ATTR     = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    dma_rx.TCD->NBYTES   = 4;
    dma_rx.TCD->DADDR    = dma_rx_buf;
    dma_rx.TCD->DOFF     = 4;
    dma_rx.TCD->CITER    = TOTAL_WORDS;
    dma_rx.TCD->BITER    = TOTAL_WORDS;
    dma_rx.TCD->DLASTSGA = -(int32_t)sizeof(dma_rx_buf);
    dma_rx.TCD->CSR      = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
    dma_rx.attachInterrupt(dma_rx_isr);
    dma_rx.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);

    // ── TX DMA: sine_buf → SAI1_TDR0 ──────────────────────────────────────
    dma_tx.begin(true);
    dma_tx.TCD->SADDR    = sine_buf;
    dma_tx.TCD->SOFF     = 4;
    dma_tx.TCD->SLAST    = -(int32_t)sizeof(sine_buf);
    dma_tx.TCD->ATTR     = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    dma_tx.TCD->NBYTES   = 4;
    dma_tx.TCD->DADDR    = &I2S1_TDR0;
    dma_tx.TCD->DOFF     = 0;
    dma_tx.TCD->DLASTSGA = 0;
    dma_tx.TCD->CITER    = SINE_BUF_WORDS;
    dma_tx.TCD->BITER    = SINE_BUF_WORDS;
    dma_tx.TCD->CSR      = 0;
    dma_tx.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX);
}


// ───────────────────────────────────────────────────────────────────────────
// Prepare USB packet payload from captured half-buffer
// src is interleaved stereo: L,R,L,R,...
// ───────────────────────────────────────────────────────────────────────────
static void map_half_for_usb(const int32_t* src, int32_t* dst) {
    if (USB_STREAM_MODE == 0) {
        memcpy(dst, src, HALF_WORDS * sizeof(int32_t));
        return;
    }

    const int pairs = HALF_WORDS / 2;

    for (int i = 0; i < pairs; i++) {
        const int32_t l = src[2 * i + 0];
        const int32_t r = src[2 * i + 1];

        if (USB_STREAM_MODE == 1) {
            dst[2 * i + 0] = l;  // L
            dst[2 * i + 1] = l;  // L
        } else if (USB_STREAM_MODE == 2) {
            dst[2 * i + 0] = r;  // R
            dst[2 * i + 1] = r;  // R
        } else { // USB_STREAM_MODE == 3
            dst[2 * i + 0] = r;  // swap
            dst[2 * i + 1] = l;
        }
    }
}


// ───────────────────────────────────────────────────────────────────────────
// Send one half-buffer over USB
// ───────────────────────────────────────────────────────────────────────────
static void send_half(int half) {
    const int32_t* src = safe_buf[half];
    map_half_for_usb(src, usb_tx_buf);

    PacketHeader hdr;
    hdr.magic    = PACKET_MAGIC;
    hdr.words    = HALF_WORDS;
    hdr.sequence = packet_seq++;

    Serial.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
    Serial.write(reinterpret_cast<const uint8_t*>(usb_tx_buf), HALF_WORDS * sizeof(int32_t));
}


// ───────────────────────────────────────────────────────────────────────────
// Setup
// ───────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 10000) {}
    delay(200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    generate_sine_buf();
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
    memset(safe_buf,   0, sizeof(safe_buf));
    memset(usb_tx_buf, 0, sizeof(usb_tx_buf));

    sai1_config();
    dma_config();

    dma_rx.enable();
    dma_tx.enable();

    I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE;
    I2S1_TCSR |= I2S_TCSR_TE | I2S_TCSR_FRDE;

    Serial.println("=== ADC/DAC USB DIAGNOSTIC ===");
    Serial.printf("SOURCE_FS             = %d Hz\n", SOURCE_FS);
    Serial.printf("BLOCK_PAIRS           = %d\n", BLOCK_PAIRS);
    Serial.printf("HALF_WORDS            = %d\n", HALF_WORDS);
    Serial.printf("USB_STREAM_MODE       = %d\n", USB_STREAM_MODE);
    Serial.printf("Expected packet rate  = %.1f pkt/s\n", (float)SOURCE_FS / (float)BLOCK_PAIRS);
    Serial.printf("DAC sine              = %d Hz\n", SINE_FREQ_HZ);
    Serial.printf("USE_BCP               = %d\n", USE_BCP);
    Serial.println("Modes: 0=normal, 1=dupL, 2=dupR, 3=swap");
    Serial.println("==============================");
}


// ───────────────────────────────────────────────────────────────────────────
// Loop
// ───────────────────────────────────────────────────────────────────────────
void loop() {
    static uint32_t t_led  = 0;
    static uint32_t t_diag = 0;

    // Heartbeat LED
    if (millis() - t_led > 500) {
        t_led = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    // Send ready half-buffers over USB
    if (ready_mask & 1u) {
        noInterrupts();
        ready_mask &= ~1u;
        interrupts();

        if (halves_seen >= WARMUP_HALVES) {
            send_half(0);
        }
        halves_seen++;
    }

    if (ready_mask & 2u) {
        noInterrupts();
        ready_mask &= ~2u;
        interrupts();

        if (halves_seen >= WARMUP_HALVES) {
            send_half(1);
        }
        halves_seen++;
    }

    // Periodic diagnostics
    if (millis() - t_diag > 5000) {
        t_diag = millis();

        Serial.println("=== LIVE DIAG ===");
        Serial.printf("halves_seen          = %lu\n", (unsigned long)halves_seen);
        Serial.printf("packet_seq           = %lu\n", (unsigned long)packet_seq);
        Serial.printf("overruns             = %lu\n", (unsigned long)overruns);
        Serial.printf("USB_STREAM_MODE      = %d\n", USB_STREAM_MODE);
        Serial.println("=================");
    }
}