#include <stdio.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "esp_eth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_check.h"
#include "esp_mac.h"

#define HOST_IP_ADDR "192.168.18.227"
#define PORT 35403

#define SPI_ETHERNETS_NUM           0
#define INTERNAL_ETHERNETS_NUM      1

static const char *TAG = "example_eth_init";
static const char *TAG1 = "TCP_CLIENT";

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "No se pudo crear el socket. Reintentando...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // --- MEJORA: Configurar un timeout de envío ---
        struct timeval timeout;
        timeout.tv_sec = 5; // 5 segundos
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        ESP_LOGI(TAG, "Intentando conectar a %s:%d", host_ip, PORT);
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        
        if (err != 0) {
            ESP_LOGE(TAG, "Conexión fallida (errno %d). ¿Está abierto Packet Sender?", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(3000)); // Espera un poco más antes de reintentar
            continue;
        }

        ESP_LOGI(TAG, "¡Conectado!");

        while (1) {
            const char *payload = "Dato de sensor: 25.4";
            int err = send(sock, payload, strlen(payload), 0);
            
            if (err < 0) {
                ESP_LOGE(TAG, "Error durante el envío. Cerrando socket...");
                break; // Rompe este bucle para volver a conectar afuera
            }

            ESP_LOGI(TAG, "Mensaje enviado");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        // Limpieza elegante
        if (sock != -1) {
            ESP_LOGW(TAG, "Cerrando conexión...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}

static esp_eth_handle_t eth_init_internal(esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;

    // Init common MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    // Update PHY config based on board specific configuration
    phy_config.phy_addr = 0;
    phy_config.reset_gpio_num = -1;
    // Init vendor specific MAC config to default
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    // Update vendor specific MAC config based on board configuration
    esp32_emac_config.smi_gpio.mdc_num = 23;
    esp32_emac_config.smi_gpio.mdio_num = 18;

    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_180_GPIO;

    // Create new ESP32 Ethernet MAC instance
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    // Create new PHY instance based on board configuration

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // Init Ethernet driver to default and install it
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&config, &eth_handle) == ESP_OK, NULL,
                        err, TAG, "Ethernet driver install failed");

    if (mac_out != NULL) {
        *mac_out = mac;
    }
    if (phy_out != NULL) {
        *phy_out = phy;
    }
    return eth_handle;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}

esp_err_t example_eth_init(esp_eth_handle_t *eth_handles_out[], uint8_t *eth_cnt_out)
{
    esp_err_t ret = ESP_OK;
    esp_eth_handle_t *eth_handles = NULL;
    uint8_t eth_cnt = 0;

    ESP_GOTO_ON_FALSE(eth_handles_out != NULL && eth_cnt_out != NULL, ESP_ERR_INVALID_ARG,
                        err, TAG, "invalid arguments: initialized handles array or number of interfaces");
    eth_handles = calloc(SPI_ETHERNETS_NUM + INTERNAL_ETHERNETS_NUM, sizeof(esp_eth_handle_t));
    ESP_GOTO_ON_FALSE(eth_handles != NULL, ESP_ERR_NO_MEM, err, TAG, "no memory");

    eth_handles[eth_cnt] = eth_init_internal(NULL, NULL);
    ESP_GOTO_ON_FALSE(eth_handles[eth_cnt], ESP_FAIL, err, TAG, "internal Ethernet init failed");
    eth_cnt++;





    *eth_handles_out = eth_handles;
    *eth_cnt_out = eth_cnt;

    return ret;

err:
    free(eth_handles);
    return ret;

}

esp_err_t example_eth_deinit(esp_eth_handle_t *eth_handles, uint8_t eth_cnt)
{
    ESP_RETURN_ON_FALSE(eth_handles != NULL, ESP_ERR_INVALID_ARG, TAG, "array of Ethernet handles cannot be NULL");
    for (int i = 0; i < eth_cnt; i++) {
        esp_eth_mac_t *mac = NULL;
        esp_eth_phy_t *phy = NULL;
        if (eth_handles[i] != NULL) {
            esp_eth_get_mac_instance(eth_handles[i], &mac);
            esp_eth_get_phy_instance(eth_handles[i], &phy);
            ESP_RETURN_ON_ERROR(esp_eth_driver_uninstall(eth_handles[i]), TAG, "Ethernet %p uninstall failed", eth_handles[i]);
        }
        if (mac != NULL) {
            mac->del(mac);
        }
        if (phy != NULL) {
            phy->del(phy);
        }
    }

    free(eth_handles);
    return ESP_OK;
}



/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG1, "Ethernet Link Up");
        ESP_LOGI(TAG1, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG1, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG1, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG1, "Ethernet Stopped");
        break;
    default:
        break;
    }
}
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG1, "Ethernet Got IP Address");
    ESP_LOGI(TAG1, "~~~~~~~~~~~");
    ESP_LOGI(TAG1, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG1, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG1, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG1, "~~~~~~~~~~~");
}


void app_main(void)
{
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_eth_init(&eth_handles,&eth_port_cnt));

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netifs[0] = esp_netif_new(&cfg);
    eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

}
