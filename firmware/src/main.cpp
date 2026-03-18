#include <Arduino.h>
#include <DMAChannel.h>

// ═══════════════════════════════════════════════════════════════════════════
// Teensy 4.1 — PCM4222EVM 24-bit I2S Slave RX
//
// Sends binary packets over USB Serial:
//   header: magic + word_count + sequence
//   payload: interleaved int32_t raw I2S words [L0, R0, L1, R1, ...]
//
// Python side converts from 32-bit left-justified to signed 24-bit by >> 8.
//
// Pin 8  = DATA   ← EVM J6 pin 7
// Pin 20 = LRCK   ← EVM J6 pin 5
// Pin 21 = BCLK   ← EVM J6 pin 3
// ═══════════════════════════════════════════════════════════════════════════

#define BLOCK_PAIRS   2048
#define HALF_WORDS   (BLOCK_PAIRS * 2)        // interleaved L+R words
#define TOTAL_WORDS  (HALF_WORDS * 2)         // ping-pong buffer

static constexpr uint32_t PACKET_MAGIC = 0x30445541u; // 'AUD0' in little-endian

struct PacketHeader {
  uint32_t magic;
  uint32_t words;
  uint32_t sequence;
};

DMAMEM static int32_t dma_buf[TOTAL_WORDS];

static DMAChannel dma;

volatile uint8_t  ready_mask = 0;   // bit0 = first half ready, bit1 = second half ready
volatile uint32_t overruns   = 0;
static uint32_t   packet_seq = 0;

// ─── DMA ISR ───────────────────────────────────────────────────────────────
// Important:
// When INTHALF fires, DADDR is already in the second half,
// so the completed half is the FIRST half.
// When INTMAJOR fires and wraps, DADDR is back near the start,
// so the completed half is the SECOND half.
void FASTRUN dma_isr() {
  const uint32_t daddr = (uint32_t)dma.TCD->DADDR;
  dma.clearInterrupt();

  uint8_t completed_half = 0;

  if (daddr < ((uint32_t)dma_buf + sizeof(dma_buf) / 2)) {
    // DMA wrapped / near start -> second half just finished
    completed_half = 1;
  } else {
    // DMA is in second half -> first half just finished
    completed_half = 0;
  }

  const uint8_t bit = (1u << completed_half);

  if (ready_mask & bit) {
    overruns++;
  } else {
    ready_mask |= bit;
  }

  asm volatile("dsb");
}

// ─── SAI1 config (32-bit I2S slave RX) ─────────────────────────────────────
static void sai1_config_slave_32bit() {
  CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

  I2S1_RCSR = (1 << 24);   // software reset
  delayMicroseconds(10);
  I2S1_RCSR = 0;
  delayMicroseconds(10);

  I2S1_RCR1 = I2S_RCR1_RFW(3);
  I2S1_RCR2 = 0;            // slave: clocks are inputs
  I2S1_RCR3 = I2S_RCR3_RCE;

  I2S1_RCR4 = I2S_RCR4_FSP
            | I2S_RCR4_FSE
            | I2S_RCR4_MF
            | I2S_RCR4_SYWD(31)
            | I2S_RCR4_FRSZ(1);

  I2S1_RCR5 = I2S_RCR5_WNW(31)
            | I2S_RCR5_W0W(31)
            | I2S_RCR5_FBT(31);

  CORE_PIN8_CONFIG  = 3;   // SAI1_RX_DATA0
  CORE_PIN20_CONFIG = 3;   // SAI1_RX_SYNC
  CORE_PIN21_CONFIG = 3;   // SAI1_RX_BCLK

  IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;
  IOMUXC_SAI1_RX_BCLK_SELECT_INPUT  = 1;
  IOMUXC_SAI1_RX_SYNC_SELECT_INPUT  = 1;
}

// ─── DMA setup ─────────────────────────────────────────────────────────────
static void dma_config() {
  dma.begin(true);

  dma.TCD->SADDR    = &I2S1_RDR0;
  dma.TCD->SOFF     = 0;
  dma.TCD->SLAST    = 0;

  dma.TCD->ATTR     = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); // 32-bit
  dma.TCD->NBYTES   = 4;

  dma.TCD->DADDR    = dma_buf;
  dma.TCD->DOFF     = 4;

  dma.TCD->CITER    = TOTAL_WORDS;
  dma.TCD->BITER    = TOTAL_WORDS;

  dma.TCD->DLASTSGA = -(int32_t)sizeof(dma_buf);
  dma.TCD->CSR      = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  dma.attachInterrupt(dma_isr);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);
}

static void send_half(const int32_t* src) {
  PacketHeader hdr;
  hdr.magic    = PACKET_MAGIC;
  hdr.words    = HALF_WORDS;
  hdr.sequence = packet_seq++;

  Serial.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
  Serial.write(reinterpret_cast<const uint8_t*>(src), HALF_WORDS * sizeof(int32_t));
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(2000000);   // USB CDC: mostly a host hint, but keep it high
  while (!Serial && millis() < 3000) {}

  memset(dma_buf, 0, sizeof(dma_buf));

  sai1_config_slave_32bit();
  dma_config();

  dma.enable();
  I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE;
}

// ─── Main loop ─────────────────────────────────────────────────────────────
void loop() {
  int half_to_send = -1;

  noInterrupts();
  if (ready_mask & 0x01) {
    ready_mask &= ~0x01;
    half_to_send = 0;
  } else if (ready_mask & 0x02) {
    ready_mask &= ~0x02;
    half_to_send = 1;
  }
  interrupts();

  if (half_to_send == 0) {
    send_half(dma_buf);
  } else if (half_to_send == 1) {
    send_half(dma_buf + HALF_WORDS);
  }
}