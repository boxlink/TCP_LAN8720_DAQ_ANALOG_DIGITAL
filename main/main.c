/*
 * ============================================================
 *  ESP32 – TCP Data Sender
 *  Cuenta entradas digitales por interrupciones + GPTimer
 *  Envía lecturas analógicas ADS1115 cada SAMPLE_PERIOD_MS
 * ============================================================
 *
 *  TRAMA ENVIADA cada SAMPLE_PERIOD_MS:
 *    data;
 *    ai0#<V>;
 *    ai1#<V>;
 *    di0#1;
 *    di1#1;
 *    ci0#<pulsos>;
 *    ci1#<pulsos>;
 *
 *  COMUNICACIÓN ENTRE CONTEXTOS (sin variables globales volátiles)
 *  ┌─────────────────────────────────────────────────────────┐
 *  │  ISR GPIO  ──(g_pulse_queue)──►  task_counter           │
 *  │  GPTimer   ──(g_timer_sem)  ──►  task_counter           │
 *  │  task_ctr  ──(g_count_queue)──►  task_tcp_client        │
 *  │  task_ads  ──(g_adc_queue)  ──►  task_tcp_client        │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  Ajusta SAMPLE_PERIOD_MS para cambiar el periodo de envío.
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"

#include "lwip/sockets.h"

#include "ads111x.h"

/* ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ──
 *  MACRO CONFIGURABLE: periodo de muestreo en milisegundos
 * ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── */
#define SAMPLE_PERIOD_MS        500     

/* ── PINES ────────────────────────────────────────────────── */
#define DIN0                    32
#define DIN1                    35

/* ── ETHERNET ─────────────────────────────────────────────── */
#define ETH_PHY_MDC             23
#define ETH_PHY_MDIO            18
#define SPI_ETHERNETS_NUM       0
#define INTERNAL_ETHERNETS_NUM  1

/* ── TCP ──────────────────────────────────────────────────── */
#define HOST_IP_ADDR            "192.168.18.227"
#define HOST_PORT               5000

/* ── ADS1115 / I2C ────────────────────────────────────────── */
#define I2C_PORT                0
#define SDA_PIN                 4
#define SCL_PIN                 5
#define ADS_GAIN                ADS111X_GAIN_0V256
#define APP_CPU_NUM             PRO_CPU_NUM

/* ── Tamaños de queue ─────────────────────────────────────── */
#define PULSE_QUEUE_LEN         5120 /* capacidad para ráfagas de pulsos  */
#define COUNT_QUEUE_LEN         1    /* siempre el snapshot más reciente  */
#define ADC_QUEUE_LEN           1    /* idem para analógico               */

/* ─────────────────────────────────────────────────────────── */

static const char *TAG     = "ETH_INIT";
static const char *TAG_TCP = "TCP_CLIENT";
static const char *TAG_DIG = "DIG_IN";

/* ══════════════════════════════════════════════════════════
 *  ESTRUCTURAS DE MENSAJES ENTRE TAREAS
 * ══════════════════════════════════════════════════════════ */

/* La ISR solo envía el número de pin que generó el pulso */
typedef struct {
    uint8_t pin;
} pulse_event_t;

/* task_counter → task_tcp: conteos del periodo cerrado */
typedef struct {
    uint32_t ci0;
    uint32_t ci1;
} count_snapshot_t;

/* task_ads → task_tcp: lecturas analógicas en Voltios */
typedef struct {
    float a0;
    float a1;
} adc_snapshot_t;

/* ══════════════════════════════════════════════════════════
 *  HANDLES GLOBALES (solo primitivas FreeRTOS, no datos)
 * ══════════════════════════════════════════════════════════ */
static QueueHandle_t     g_pulse_queue = NULL;
static QueueHandle_t     g_count_queue = NULL;
static QueueHandle_t     g_adc_queue   = NULL;
static SemaphoreHandle_t g_timer_sem   = NULL;

/* ADS1115 descriptor: accedido únicamente por task_ads */
static i2c_dev_t g_adc_dev;

/* ══════════════════════════════════════════════════════════
 *  1. ISR ENTRADAS DIGITALES
 *     Una sola función para ambos pines.
 *     Solo escribe en la queue, sin tocar datos compartidos.
 * ══════════════════════════════════════════════════════════ */
static void IRAM_ATTR isr_gpio(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;

    /* arg contiene el número de GPIO configurado en isr_handler_add */
    pulse_event_t evt = { 
        .pin = (uint8_t)(uint32_t)arg };

    /* Si la queue está llena el pulso se descarta (no bloquea en ISR) */
    xQueueSendFromISR(g_pulse_queue, &evt, &higher_prio_woken);

    portYIELD_FROM_ISR(higher_prio_woken);
}

static void gpio_interrupt_init(void)
{
    gpio_install_isr_service(0);

    gpio_config_t io = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,   /* flanco de subida */
    };

    io.pin_bit_mask = (1ULL << DIN0);
    gpio_config(&io);
    gpio_isr_handler_add(DIN0, isr_gpio, (void *)(uint32_t)DIN0);

    io.pin_bit_mask = (1ULL << DIN1);
    gpio_config(&io);
    gpio_isr_handler_add(DIN1, isr_gpio, (void *)(uint32_t)DIN1);

    ESP_LOGI(TAG_DIG, "ISR DIN0(GPIO%d) y DIN1(GPIO%d) instaladas", DIN0, DIN1);
}

/* ══════════════════════════════════════════════════════════
 *  2. CALLBACK GPTIMER  (ISR de alta precisión)
 *     Solo libera el semáforo binario hacia task_counter.
 * ══════════════════════════════════════════════════════════ */
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_timer_sem, &higher_prio_woken);
    return (higher_prio_woken == pdTRUE);
}

static gptimer_handle_t g_gptimer = NULL;

static void gptimer_init(void)
{
    /* Resolución 1 µs (1 MHz) */
    gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &g_gptimer));

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count                = (uint64_t)SAMPLE_PERIOD_MS * 1000ULL,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(g_gptimer, &alarm_cfg));

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_alarm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(g_gptimer));
    ESP_ERROR_CHECK(gptimer_start(g_gptimer));

    ESP_LOGI(TAG_DIG, "GPTimer iniciado: alarma cada %u ms", SAMPLE_PERIOD_MS);
}

/* ══════════════════════════════════════════════════════════
 *  3. TAREA CONTADORA  (task_counter)
 *     - Espera el semáforo del GPTimer (fin de periodo).
 *     - Drena la pulse_queue acumulando en variables LOCALES.
 *     - Publica el snapshot en count_queue (overwrite).
 *     - Resetea acumuladores para el siguiente periodo.
 *
 *     Prioridad mayor que task_tcp para garantizar que el
 *     snapshot esté listo antes de que TCP lo consuma.
 * ══════════════════════════════════════════════════════════ */
static void task_counter(void *pvParameters)
{
    uint32_t      ci0 = 0, ci1 = 0;
    pulse_event_t evt;

    while (1) {
        /* Bloquea hasta que el GPTimer indique fin de periodo */
        if (xSemaphoreTake(g_timer_sem,
                           pdMS_TO_TICKS(SAMPLE_PERIOD_MS * 3)) != pdTRUE) {
            ESP_LOGW(TAG_DIG, "Timeout en semáforo de timer");
            continue;
        }

        /* Drena TODOS los pulsos acumulados en la queue */
        while (xQueueReceive(g_pulse_queue, &evt, 0) == pdTRUE) {
            if      (evt.pin == DIN0) ci0++;
            else if (evt.pin == DIN1) ci1++;
        }

        /* Publica snapshot (overwrite: siempre el dato más reciente) */
        count_snapshot_t snap = { .ci0 = ci0, .ci1 = ci1 };
        xQueueOverwrite(g_count_queue, &snap);

        ESP_LOGD(TAG_DIG, "Periodo cerrado: ci0=%" PRIu32 "  ci1=%" PRIu32,
                 ci0, ci1);

        /* Resetea para el siguiente periodo */
        ci0 = 0;
        ci1 = 0;
    }
}

/* ══════════════════════════════════════════════════════════
 *  4. TAREA ADS1115  (task_ads1115)
 *     Lee dos canales y publica en adc_queue.
 *     No comparte ningún dato con otras tareas/ISR.
 * ══════════════════════════════════════════════════════════ */
static void task_ads1115(void *pvParameters)
{
    float         gain_val = ads111x_gain_values[ADS_GAIN];
    adc_snapshot_t snap    = {0};

    ESP_ERROR_CHECK(ads111x_init_desc(&g_adc_dev, ADS111X_ADDR_GND,
                                      I2C_PORT, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(ads111x_set_mode     (&g_adc_dev, ADS111X_MODE_SINGLE_SHOT));
    ESP_ERROR_CHECK(ads111x_set_data_rate(&g_adc_dev, ADS111X_DATA_RATE_32));
    ESP_ERROR_CHECK(ads111x_set_gain     (&g_adc_dev, ADS_GAIN));

    while (1) {
        int16_t raw  = 0;
        bool    busy = true;

        /* Canal 0: AIN0 vs GND */
        ads111x_set_input_mux(&g_adc_dev, ADS111X_MUX_0_GND);
        ads111x_start_conversion(&g_adc_dev);
        busy = true;
        while (busy) {
            vTaskDelay(pdMS_TO_TICKS(2));
            ads111x_is_busy(&g_adc_dev, &busy);
        }
        if (ads111x_get_value(&g_adc_dev, &raw) == ESP_OK) {
            snap.a0 = (gain_val / ADS111X_MAX_VALUE) * raw;
        }

        /* Canal 1: AIN3 vs GND */
        ads111x_set_input_mux(&g_adc_dev, ADS111X_MUX_3_GND);
        ads111x_start_conversion(&g_adc_dev);
        busy = true;
        while (busy) {
            vTaskDelay(pdMS_TO_TICKS(2));
            ads111x_is_busy(&g_adc_dev, &busy);
        }
        if (ads111x_get_value(&g_adc_dev, &raw) == ESP_OK) {
            snap.a1 = (gain_val / ADS111X_MAX_VALUE) * raw;
        }

        /* Publica (overwrite: task_tcp siempre lee el valor más fresco) */
        xQueueOverwrite(g_adc_queue, &snap);

        /* Muestrea a la mitad del periodo de envío */
        uint32_t half = SAMPLE_PERIOD_MS / 2;
        vTaskDelay(pdMS_TO_TICKS(half < 50 ? 50 : half));
    }
}

/* ══════════════════════════════════════════════════════════
 *  5. TAREA TCP CLIENT  (task_tcp_client)
 *     Consume count_queue y adc_queue, construye la trama
 *     y la envía al servidor TCP.
 * ══════════════════════════════════════════════════════════ */
static void task_tcp_client(void *pvParameters)
{
    char             tx_buf[256];
    int              sock = -1;
    count_snapshot_t cnt  = {0};
    adc_snapshot_t   adc  = {0};

    while (1) {

        /* ── Abrir conexión si no hay socket ────────────── */
        if (sock < 0) {
            struct sockaddr_in dest = {
                .sin_family = AF_INET,
                .sin_port   = htons(HOST_PORT),
            };
            dest.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);

            sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG_TCP, "socket() falló. Reintentando en 1 s…");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            ESP_LOGI(TAG_TCP, "Conectando a %s:%d …", HOST_IP_ADDR, HOST_PORT);
            if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
                ESP_LOGE(TAG_TCP,
                         "connect() falló (errno %d). Reintentando en 3 s…",
                         errno);
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
            ESP_LOGI(TAG_TCP, "¡Conectado!");
        }

        /* ── Esperar snapshot de contadores ─────────────── */
        if (xQueueReceive(g_count_queue, &cnt,
                          pdMS_TO_TICKS(SAMPLE_PERIOD_MS * 3)) != pdTRUE) {
            ESP_LOGW(TAG_TCP, "Timeout esperando contadores – reconectando…");
            shutdown(sock, 0);
            close(sock);
            sock = -1;
            continue;
        }

        /* ── Leer último valor analógico (no bloquea) ────── */
        xQueuePeek(g_adc_queue, &adc, 0);

        /* ── Construir trama ─────────────────────────────── */
        int len = snprintf(tx_buf, sizeof(tx_buf),
                           "data;\n"
                           "ai0#%f;\n"
                           "ai1#%f;\n"
                           "di0#1;\n"
                           "di1#1;\n"
                           "ci0#%" PRIu32 ";\n"
                           "ci1#%" PRIu32 ";\n",
                           adc.a0, adc.a1,
                           cnt.ci0, cnt.ci1);

        /* ── Enviar ──────────────────────────────────────── */
        if (send(sock, tx_buf, len, 0) < 0) {
            ESP_LOGE(TAG_TCP, "send() falló (errno %d). Cerrando socket…",
                     errno);
            shutdown(sock, 0);
            close(sock);
            sock = -1;
        } else {
            ESP_LOGI(TAG_TCP,
                     "TX → ai0#%.6f ai1#%.6f di0#1 di1#1"
                     " ci0#%" PRIu32 " ci1#%" PRIu32,
                     adc.a0, adc.a1, cnt.ci0, cnt.ci1);
        }
    }
}

/* ══════════════════════════════════════════════════════════
 *  6. INICIALIZACIÓN ETHERNET
 * ══════════════════════════════════════════════════════════ */
static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out,
                                          esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;

    eth_mac_config_t        mac_config  = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t        phy_config  = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr       = 0;
    phy_config.reset_gpio_num = -1;

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num  = ETH_PHY_MDC;
    emac_config.smi_gpio.mdio_num = ETH_PHY_MDIO;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_180_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config     = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK,
                      NULL, err, TAG, "Ethernet driver install failed");

    if (mac_out) *mac_out = mac;
    if (phy_out) *phy_out = phy;
    return eth_handle;

err:
    if (eth_handle) esp_eth_driver_uninstall(eth_handle);
    if (mac)        mac->del(mac);
    if (phy)        phy->del(phy);
    return ret;
}

static esp_err_t example_eth_init(esp_eth_handle_t *eth_handles_out[],
                                  uint8_t *eth_cnt_out)
{
    esp_err_t         ret         = ESP_OK;
    esp_eth_handle_t *eth_handles = NULL;
    uint8_t           eth_cnt     = 0;

    ESP_GOTO_ON_FALSE(eth_handles_out && eth_cnt_out, ESP_ERR_INVALID_ARG,
                      err, TAG, "invalid args");
    eth_handles = calloc(SPI_ETHERNETS_NUM + INTERNAL_ETHERNETS_NUM,
                         sizeof(esp_eth_handle_t));
    ESP_GOTO_ON_FALSE(eth_handles, ESP_ERR_NO_MEM, err, TAG, "no memory");

    eth_handles[eth_cnt] = eth_init_internal(NULL, NULL);
    ESP_GOTO_ON_FALSE(eth_handles[eth_cnt], ESP_FAIL, err, TAG,
                      "internal Ethernet init failed");
    eth_cnt++;

    *eth_handles_out = eth_handles;
    *eth_cnt_out     = eth_cnt;
    return ret;

err:
    free(eth_handles);
    return ret;
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    uint8_t          mac_addr[6] = {0};
    esp_eth_handle_t eth_handle  = *(esp_eth_handle_t *)event_data;
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up  HW: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED: ESP_LOGI(TAG, "Ethernet Link Down"); break;
    case ETHERNET_EVENT_START:        ESP_LOGI(TAG, "Ethernet Started");   break;
    case ETHERNET_EVENT_STOP:         ESP_LOGI(TAG, "Ethernet Stopped");   break;
    default: break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    ip_event_got_ip_t         *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip    = &event->ip_info;
    ESP_LOGI(TAG, "Got IP: " IPSTR "  Mask: " IPSTR "  GW: " IPSTR,
             IP2STR(&ip->ip), IP2STR(&ip->netmask), IP2STR(&ip->gw));
}

/* ══════════════════════════════════════════════════════════
 *  7. app_main
 * ══════════════════════════════════════════════════════════ */
void app_main(void)
{
    /* ── Crear todas las primitivas FreeRTOS ────────────── */
    g_pulse_queue = xQueueCreate(PULSE_QUEUE_LEN, sizeof(pulse_event_t));
    g_count_queue = xQueueCreate(COUNT_QUEUE_LEN, sizeof(count_snapshot_t));
    g_adc_queue   = xQueueCreate(ADC_QUEUE_LEN,   sizeof(adc_snapshot_t));
    g_timer_sem   = xSemaphoreCreateBinary();

    if (!g_pulse_queue || !g_count_queue || !g_adc_queue || !g_timer_sem) {
        ESP_LOGE(TAG, "Error al crear primitivas FreeRTOS – abortando");
        return;
    }

    /* ── Interrupciones digitales ───────────────────────── */
    gpio_interrupt_init();

    /* ── GPTimer de alta precisión ──────────────────────── */
    gptimer_init();

    /* ── Ethernet ───────────────────────────────────────── */
    uint8_t           eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));

    esp_netif_t                 *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t  eth_glues [eth_port_cnt];

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netifs[0] = esp_netif_new(&cfg);
    eth_glues [0] = esp_eth_new_netif_glue(eth_handles[0]);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_glues[0]));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));

    /* ── I2C + ADS1115 ──────────────────────────────────── */
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&g_adc_dev, 0, sizeof(g_adc_dev));

    /* ── Lanzar tareas ──────────────────────────────────── */
    /* Prio 6: counter debe procesar el semáforo antes que TCP lo consuma */
    xTaskCreate(task_counter,    "counter",    2048, NULL, 6, NULL);
    /* Prio 5: ADS en el core de aplicación para no competir con Ethernet */
    xTaskCreatePinnedToCore(task_ads1115, "ads1115",
                            configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL,
                            APP_CPU_NUM);
    /* Prio 5: TCP client */
    xTaskCreate(task_tcp_client, "tcp_client", 4096, NULL, 5, NULL);
}