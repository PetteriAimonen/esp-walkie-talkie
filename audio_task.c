#include "audio_task.h"
#include "fast_adc.h"
#include "alaw.h"
#include "sigma_delta.h"
#include <espressif/esp_common.h>
#include <esp/gpio.h>
#include <esp/i2s_regs.h>
#include <i2s_dma/i2s_dma.h>
#include <lwip/pbuf.h>
#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <esplibs/libphy.h>

// 20 milliseconds of audio per packet
#define AUDIO_HDRLEN 8
#define AUDIO_BLOCKSIZE 500
#define AUDIO_SAMPLERATE 12500

// I2S output is run at 1024x the audio samplerate and
// sigma-delta algorithm is used to convert the PCM samples to 1-bit stream.
#define I2S_WORDRATE 200000
#define I2S_BITRATE (I2S_WORDRATE * 32)
#define AUDIO_OUT_OVERSAMPLE (I2S_WORDRATE/AUDIO_SAMPLERATE)

#define ECHO_CANCEL_LEN 12
#define DAC_HISTORY_LEN 16
#define DAC_HISTORY_MASK 15
#define CLICK_REMOVE_LEN 32
#define CLICK_REMOVE_MASK 31

#define UDP_PORT 18294

typedef struct {
  uint16_t peer_port;
  ip_addr_t peer_addr;
  QueueHandle_t received_bufs;
  struct pbuf *current_buf;
  int current_buf_use_count;
  TickType_t last_seen;
} peer_t;

#define MAX_PEERS 4
static peer_t g_peers[MAX_PEERS];

static struct {
  // pbufs allocated by network task, ready to be filled.
  QueueHandle_t free_bufs;
  
  // pbufs filled by interrupt, ready to be sent.
  QueueHandle_t filled_bufs;
  
  // pbuf currently being filled
  struct pbuf *current_buf;
  int bytes_written;
  
  // Estimated DC level of ADC input samples, in Q24.8 fixed point format
  int adc_bias;
  
  // Previous raw ADC value
  int adc_prev;
  
  // Accumulated quantization error (for dithering)
  int quant_acc;
  
  int samplecount;
  
  // Echo cancellation filter
  int echo_cancel[ECHO_CANCEL_LEN];
  
  // Click removal filter
  int click_remove[CLICK_REMOVE_LEN];
  uint8_t click_remove_idx;
} g_audioin;

static struct {
  // Received buffers that have been processed and can now be freed.
  QueueHandle_t free_bufs;
  
  // Current position in input buffers in g_peers.
  int current_pos;
  
  // Previous DAC value, for interpolation
  int dac_prev;
  
  // Error accumulator for sigma-delta
  int error_acc;
  
  // Previous DAC values, for echo cancellation
  int history[DAC_HISTORY_LEN];
  uint8_t history_idx;
} g_audioout;

int get_volume();

static void audio_init()
{
  g_audioin.adc_bias = 4096;
  g_audioin.adc_prev = 4096;
  g_audioin.free_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  g_audioin.filled_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  
  g_audioout.free_bufs = xQueueCreate(MAX_PEERS * 2, sizeof(struct pbuf*));
  
  for (int i = 0; i < MAX_PEERS; i++)
  {
    g_peers[i].received_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  }
}

static inline IRAM void process_audio_in(portBASE_TYPE *task_awoken)
{
  g_audioin.samplecount++;
  
  // Read the results of previous ADC sampling and start a new sampling immediately.
  int raw_adc = adc_read_result();
  adc_sample();
  
  // To improve antialiasing and to filter out high-freq noise, use a sliding window
  // average of 2 samples.
  int filtered = (g_audioin.adc_prev + raw_adc) / 2;
  g_audioin.adc_prev = raw_adc;
  
  // Estimate the DC level of the signal using a 50 Hz low-pass filter.
  int bias = g_audioin.adc_bias / 1024;
  int ac_value = filtered - bias;
  g_audioin.adc_bias += ac_value;
  
  // Perform echo cancellation
  int echo_estimated = 0;
  for (int i = 0; i < ECHO_CANCEL_LEN; i++)
  {
    echo_estimated += (g_audioout.history[(g_audioout.history_idx - i) & DAC_HISTORY_MASK] * (g_audioin.echo_cancel[i] / 1024) + 2048) / 4096;
  }
  
  int blend = g_audioin.echo_cancel[ECHO_CANCEL_LEN - 1];
  for (int i = ECHO_CANCEL_LEN; i < ECHO_CANCEL_LEN + 4; i++)
  {
    blend -= blend / 2;
    echo_estimated += (g_audioout.history[(g_audioout.history_idx - i) & DAC_HISTORY_MASK] * (blend / 1024) + 2048) / 4096;
  }
  
  int echo_delta = ac_value - echo_estimated;
  for (int i = 0; i < ECHO_CANCEL_LEN; i++)
  {
    g_audioin.echo_cancel[i] += (echo_delta * g_audioout.history[(g_audioout.history_idx - i) & DAC_HISTORY_MASK] + 2048) / 4096;
  }
  
  ac_value -= echo_estimated;
  
  // Perform click removal
  int idx = g_audioin.click_remove_idx;
  
  if (g_audioin.click_remove[(idx + 6) & CLICK_REMOVE_MASK] > 3500 ||
      g_audioin.click_remove[(idx + 6) & CLICK_REMOVE_MASK] < -3500)
  {
    int oldest = g_audioin.click_remove[(idx) & CLICK_REMOVE_MASK];
    int newest = g_audioin.click_remove[(idx-1) & CLICK_REMOVE_MASK];
    int delta = oldest - newest;
    
    if (oldest > -1000 && oldest < 1000 && newest > -1000 && newest < 1000 &&
        delta < 1000 && delta > -1000)
    {
      int avg = (oldest + newest) / 2;
      for (int i = 0; i < CLICK_REMOVE_LEN; i++)
      {
        g_audioin.click_remove[i] = avg;
      }
    }
  }
  
  int click_filtered = g_audioin.click_remove[idx & CLICK_REMOVE_MASK];
  g_audioin.click_remove[idx & CLICK_REMOVE_MASK] = ac_value;
  g_audioin.click_remove_idx++;
  
  // Compand using alaw algorithm and write to the packet buffer.
  int to_encode = click_filtered;
  uint8_t companded = alaw_encode(to_encode + g_audioin.quant_acc);
  g_audioin.quant_acc += to_encode - alaw_decode(companded);
  
  // Take new buffers from g_audioin.free_bufs
  if (!g_audioin.current_buf)
  {
    xQueueReceiveFromISR(g_audioin.free_bufs, &g_audioin.current_buf, task_awoken);
    g_audioin.bytes_written = 0;
  }
  
  // Post filled buffers to g_audioin.filled_bufs
  if (g_audioin.current_buf)
  {
    uint8_t *payload_ptr = (uint8_t*)g_audioin.current_buf->payload;
    payload_ptr[AUDIO_HDRLEN + g_audioin.bytes_written] = companded;
    g_audioin.bytes_written += 1;
    
    if (g_audioin.bytes_written >= AUDIO_BLOCKSIZE)
    {
      xQueueSendToBackFromISR(g_audioin.filled_bufs, &g_audioin.current_buf, task_awoken);
      g_audioin.current_buf = NULL;
    }
  }
}

static inline IRAM void process_audio_out(portBASE_TYPE *task_awoken)
{
  if (g_audioout.current_pos == 0)
  {
    // Start of new buffer block, check for new packets of each peer.
    for (int i = 0; i < MAX_PEERS; i++)
    {
      struct pbuf *new_buf = NULL;
      if (xQueueReceiveFromISR(g_peers[i].received_bufs, &new_buf, task_awoken) == pdTRUE)
      {
        // Ok, received new packet, schedule the old one to be freed
        if (g_peers[i].current_buf)
        {
          xQueueSendToBackFromISR(g_audioout.free_bufs, &g_peers[i].current_buf, task_awoken);
        }
        
        g_peers[i].current_buf = new_buf;
        g_peers[i].current_buf_use_count = 0;
      }
      else if (g_peers[i].current_buf)
      {
        // No packet found, repeat the last one for a few blocks and then silence this peer.
        g_peers[i].current_buf_use_count++;
        if (g_peers[i].current_buf_use_count > 4)
        {
          xQueueSendToBackFromISR(g_audioout.free_bufs, &g_peers[i].current_buf, task_awoken);
          g_peers[i].current_buf = NULL;
        }
      }
    }
  }
  
  // Calculate sum of values for all active peers.
  int dac_val = 0;
  for (int i = 0; i < MAX_PEERS; i++)
  {
    if (g_peers[i].current_buf)
    {
      uint8_t sample = ((uint8_t*)g_peers[i].current_buf->payload)[AUDIO_HDRLEN + g_audioout.current_pos];
      int expanded = alaw_decode(sample);
      dac_val += expanded;
    }
  }
  
  // Volume adjustment
  dac_val = (dac_val * get_volume()) / 16;
  
  // Clamp to range 0 to 8192.
  dac_val += 4096;
  if (dac_val < 0) dac_val = 0;
  if (dac_val > 8191) dac_val = 8191;
  
  // Perform sigma-delta modulation
  // Data is written directly to I2S TXFIFO.
  // It has 128 words deep fifo in hardware, so there is no need to use DMA here.
  {
    sigma_delta_encode(g_audioout.dac_prev, dac_val, &I2S.TXFIFO, AUDIO_OUT_OVERSAMPLE, &g_audioout.error_acc);
    g_audioout.dac_prev = dac_val;
  }
  
  g_audioout.history_idx++;
  g_audioout.history[g_audioout.history_idx & DAC_HISTORY_MASK] = dac_val - 4096;
  
  g_audioout.current_pos++;
  if (g_audioout.current_pos >= AUDIO_BLOCKSIZE)
  {
    g_audioout.current_pos = 0;
  }
}

// #define REG sdk_tx_pwctrl_pk_num
// static uint32_t g_foo;

static void IRAM audio_irq(void *arg)
{
  portBASE_TYPE task_awoken = pdFALSE;
  
  gpio_write(13, 1); // For debugging, IRQ processing start

  process_audio_in(&task_awoken);
  process_audio_out(&task_awoken);
  
  gpio_write(13, 0);
  
//   gpio_write(13, REG == g_foo);
  portEND_SWITCHING_ISR(task_awoken);
}

static void remove_old_peers()
{
  for (int i = 0; i < MAX_PEERS; i++)
  {
    if (g_peers[i].peer_port != 0)
    {
      if (xTaskGetTickCount() - g_peers[i].last_seen > configTICK_RATE_HZ * 10)
      {
        g_peers[i].peer_port = 0;
        
        struct pbuf *p;
        while (xQueueReceive(g_peers[i].received_bufs, &p, 0) == pdTRUE)
        {
          pbuf_free(p);
        }
      }
    }
  }
}

static void handle_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  if (ip_addr_cmp(&pcb->local_ip, addr))
  {
    // Our own broadcast packet came back to us for some reason.
    pbuf_free(p);
    return;
  }
  
  int first_free = -1;
  int peer_idx = -1;
  for (int i = 0; i < MAX_PEERS; i++)
  {
    if (g_peers[i].peer_port == port &&
        ip_addr_cmp(&g_peers[i].peer_addr, addr))
    {
      // We already have this peer on our list
      peer_idx = i;
      break;
    }
    else if (g_peers[i].peer_port == 0 && first_free < 0)
    {
      first_free = i;
    }
  }
  
  if (peer_idx < 0 && first_free >= 0)
  {
    // Add a new peer
    peer_idx = first_free;
    g_peers[peer_idx].last_seen = xTaskGetTickCount();
    g_peers[peer_idx].peer_addr = *addr;
    g_peers[peer_idx].peer_port = port;
  }
  
  if (peer_idx >= 0)
  {
    g_peers[peer_idx].last_seen = xTaskGetTickCount();
    
    // The input buffer has associated ESP SDK buffer that we want
    // to free rather soon. Not sure if there is a better way to do
    // it, but this makes a full copy of the buffer to be queued.
    if (p->len == AUDIO_HDRLEN + AUDIO_BLOCKSIZE)
    {
      struct pbuf *p2 = pbuf_clone(PBUF_TRANSPORT, PBUF_RAM, p);
      if (xQueueSendToBack(g_peers[peer_idx].received_bufs, &p2, 0) != pdTRUE)
      {
        pbuf_free(p2);
      }
    }
  }
  
  pbuf_free(p);
}

void audio_task(void *pvParameters)
{
  printf("Audio task starting\n");
  
  audio_init();
  
  // GPIO12 is toggled for measuring IRQ processing length
  gpio_enable(13, GPIO_OUTPUT);
  
  // Enable interrupt handler at 12.5 kHz
  adc_enable();
  _xt_isr_attach(INUM_TIMER_FRC1, audio_irq, NULL);
  timer_set_frequency(FRC1, AUDIO_SAMPLERATE);
  timer_set_interrupts(FRC1, true);
  timer_set_run(FRC1, true);
  
  // Enable I2S for audio output
  {
    i2s_clock_div_t div = i2s_get_clock_div(I2S_BITRATE);
    i2s_pins_t i2s_pins = {.data = true};
    i2s_dma_init(NULL, NULL, div, i2s_pins);
    SET_MASK_BITS(I2S.CONF, I2S_CONF_TX_START);
  }
  
  // Open UDP broadcast port
  LOCK_TCPIP_CORE();
  struct udp_pcb *pcb;
  pcb = udp_new();
  udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
  //ip_set_option(pcb, SOF_BROADCAST);
  udp_recv(pcb, handle_packet, NULL);
  UNLOCK_TCPIP_CORE();
  
  uint8_t msgcount = 0;
  TickType_t last_broadcast = 0;
  
  while (1) {
    if (xTaskGetTickCount() - last_broadcast > configTICK_RATE_HZ)
    {
      // Send broadcast message to notify potential peers
      LOCK_TCPIP_CORE();
      remove_old_peers();
      struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 0, PBUF_RAM);
      udp_sendto(pcb, p, IP_ADDR_BROADCAST, UDP_PORT);
      pbuf_free(p);
      UNLOCK_TCPIP_CORE();
      
      last_broadcast = xTaskGetTickCount();
    }
    
    while (uxQueueSpacesAvailable(g_audioin.free_bufs))
    {
      struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, AUDIO_HDRLEN + AUDIO_BLOCKSIZE, PBUF_RAM);
      xQueueSendToBack(g_audioin.free_bufs, &p, 0);
    }
    
    while (uxQueueMessagesWaiting(g_audioout.free_bufs))
    {
      struct pbuf *p = NULL;
      xQueueReceive(g_audioout.free_bufs, &p, 0);
      pbuf_free(p);
    }
    
    {
      struct pbuf *p = NULL;
      if (xQueueReceive(g_audioin.filled_bufs, &p, 1) == pdTRUE)
      {
//         gpio_write(13, 1);
        
        p->len = p->tot_len = AUDIO_HDRLEN + AUDIO_BLOCKSIZE;
        
        ((uint8_t*)p->payload)[0] = msgcount++;
        ((uint8_t*)p->payload)[1] = 0;
        ((uint8_t*)p->payload)[2] = 0;
        ((uint8_t*)p->payload)[3] = 0;
        ((uint32_t*)p->payload)[1] = g_audioin.samplecount;
        
        int peer_count = 0;
        for (int i = 0; i < MAX_PEERS; i++)
        {
          if (g_peers[i].peer_port > 0)
          {
            LOCK_TCPIP_CORE();
            peer_count++;
            if (peer_count > 1)
            {
              struct pbuf *p_to_send = pbuf_clone(PBUF_TRANSPORT, PBUF_RAM, p);
              udp_sendto(pcb, p_to_send, &g_peers[i].peer_addr, g_peers[i].peer_port);
              pbuf_free(p_to_send);
            }
            else
            {
              udp_sendto(pcb, p, &g_peers[i].peer_addr, g_peers[i].peer_port);
            }
            UNLOCK_TCPIP_CORE();
          }
        }
        
        pbuf_free(p);
        
        printf("ADC val %d bias %d DAC %d pos %d queue %d %d peers %d\n", g_audioin.adc_prev, g_audioin.adc_bias / 256, g_audioout.dac_prev,
               g_audioout.current_pos, (int)uxQueueMessagesWaiting(g_peers[0].received_bufs), !!g_peers[0].current_buf,
              peer_count);
        
        printf("filter = ");
        for (int i = 0; i < ECHO_CANCEL_LEN; i++)
        {
          printf("%4d ", g_audioin.echo_cancel[i]);
        }
        printf("\n");
        
//         printf("%02x\n", REG);
//         g_foo = REG;
        
//         gpio_write(13, 0);
      }
    }
  }
}

void audio_task_start()
{
  xTaskCreate(audio_task, "audio", 512, NULL, 2, NULL);
}
