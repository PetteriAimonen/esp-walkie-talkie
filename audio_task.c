#include "audio_task.h"
#include "fast_adc.h"
#include "alaw.h"
#include <espressif/esp_common.h>
#include <esp/gpio.h>
#include <lwip/pbuf.h>
#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

// 20 milliseconds of audio per packet
#define AUDIO_HDRLEN 8
#define AUDIO_BLOCKSIZE 240

#define UDP_PORT 18294

typedef struct {
  uint16_t peer_port;
  ip_addr_t peer_addr;
  QueueHandle_t received_bufs;
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
} g_audioin;

static void audio_init()
{
  g_audioin.adc_bias = 4096;
  g_audioin.adc_prev = 4096;
  g_audioin.free_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  g_audioin.filled_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  
  for (int i = 0; i < MAX_PEERS; i++)
  {
    g_peers[i].received_bufs = xQueueCreate(4, sizeof(struct pbuf*));
  }
}

static void IRAM audio_irq(void *arg)
{
  portBASE_TYPE task_awoken = pdFALSE;
  
  gpio_write(12, 1); // For debugging, IRQ processing start
  
  g_audioin.samplecount++;
  
  // Read the results of previous ADC sampling and start a new sampling immediately.
  int raw_adc = adc_read_result();
  adc_sample();
  
  // To improve antialiasing and to filter out high-freq noise, use a sliding window
  // average of 2 samples.
  int filtered = (g_audioin.adc_prev + raw_adc) / 2;
  g_audioin.adc_prev = raw_adc;
  
  // Estimate the DC level of the signal using a 50 Hz low-pass filter.
  int bias = g_audioin.adc_bias / 256;
  int ac_value = filtered - bias;
  g_audioin.adc_bias += ac_value;
  
  // Compand using alaw algorithm and write to the packet buffer.
  uint8_t companded = alaw_encode(ac_value + g_audioin.quant_acc);
  g_audioin.quant_acc += ac_value - alaw_decode(companded);
  
  if (!g_audioin.current_buf)
  {
    xQueueReceiveFromISR(g_audioin.free_bufs, &g_audioin.current_buf, &task_awoken);
    g_audioin.bytes_written = 0;
  }
  
  if (g_audioin.current_buf)
  {
    uint8_t *payload_ptr = (uint8_t*)g_audioin.current_buf->payload;
    payload_ptr[AUDIO_HDRLEN + g_audioin.bytes_written] = companded;
    g_audioin.bytes_written += 1;
    
    if (g_audioin.bytes_written >= AUDIO_BLOCKSIZE)
    {
      xQueueSendToBackFromISR(g_audioin.filled_bufs, &g_audioin.current_buf, &task_awoken);
      g_audioin.current_buf = NULL;
    }
  }
  
  gpio_write(12, 0);
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
  if (ip_addr_cmp(pcb->local_ip, addr))
  {
    // Our own broadcast packet..
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
    if (p->len > 0 && xQueueSendToBack(g_peers[peer_idx].received_bufs, &p, 0) == pdTRUE)
    {
      return;
    }
  }
  
  pbuf_free(p);
}

void audio_task(void *pvParameters)
{
  printf("Audio task starting\n");
  
  while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
    vTaskDelay(10);
  }
  
  audio_init();
  
  // GPIO12 is toggled for measuring IRQ processing length
  gpio_enable(12, GPIO_OUTPUT);
  gpio_enable(13, GPIO_OUTPUT);
  
  // Enable interrupt handler at 12 kHz
  adc_enable();
  _xt_isr_attach(INUM_TIMER_FRC1, audio_irq, NULL);
  timer_set_frequency(FRC1, 12000);
  timer_set_interrupts(FRC1, true);
  timer_set_run(FRC1, true);
  
  // Open UDP broadcast port
  LOCK_TCPIP_CORE();
  struct udp_pcb *pcb;
  pcb = udp_new();
  udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
  ip_set_option(pcb, SOF_BROADCAST);
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
    
    {
      struct pbuf *p = NULL;
      if (xQueueReceive(g_audioin.filled_bufs, &p, 1) == pdTRUE)
      {
        gpio_write(13, 1);
        
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
        
        printf("ADC val %d bias %d peers %d\n", g_audioin.adc_prev, g_audioin.adc_bias / 256,
              peer_count);
        
        gpio_write(13, 0);
      }
    }
  }
}

void audio_task_start()
{
  xTaskCreate(audio_task, "audio", 256, NULL, 2, NULL);
}
