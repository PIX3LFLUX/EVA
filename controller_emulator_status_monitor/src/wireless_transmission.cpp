#include "wireless_transmission.h"

void init_wifi() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]); /* overwrite board mac address with known value to make it work on any ESP32*/

  #if DEBUG
    Serial.print("ESP32 Board MAC Address: "); Serial.println(WiFi.macAddress());
  #endif

  /* Init ESP-NOW */
  if (esp_now_init() != ESP_OK) {
    #if DEBUG
      Serial.println("Error initializing ESP-NOW"); 
    #endif
    return;
  }
  
  /* Register peer */
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); /* init peerInfo with 0, else there are errors */
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  /* Add peer */
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    #if DEBUG
      Serial.println("Failed to add peer");
    #endif
    return;
  }
  /* function that gets called every time data is received */
  esp_now_register_recv_cb(on_data_receive);
}

IRAM_ATTR void send_data_wirelessly(uint8_t data_to_transmit)
{
  for (uint8_t ii = WIRELESS_TRANSMISSION_TRIES; ii > 0; ii--)
  {
    esp_now_send(broadcastAddress, (uint8_t *) &data_to_transmit, 1);
    #if DEBUG
      Serial.printf("Transmitting data: %d\n", data_to_transmit);
    #endif
  }
}

IRAM_ATTR void on_data_receive(const uint8_t * mac, const uint8_t *incoming_data, int len)
{
  /* update DAC with new value, if DAC is accessible. If not, discard the value. */
  if ( xSemaphoreTake(dac_access_semaphore, 0) == pdTRUE )
  {
    #if DEBUG
      Serial.printf("Updating DAC with new value %d which was received wirelessly.\n", *incoming_data);
    #endif
    dac_output_voltage(DAC_CHANNEL_1, *incoming_data);
    xSemaphoreGive(dac_access_semaphore);
  }
  send_data_wirelessly(RECEIVED_VALUE_WIRELESSLY);
}