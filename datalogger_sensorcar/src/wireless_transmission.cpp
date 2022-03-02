#include "wireless_transmission.h"
uint8_t race_status           = NO_RACE_GOING; /* For states, look at declaration of initialization value */

void init_wifi() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  Serial.print("ESP32 Board MAC Address: "); Serial.println(WiFi.macAddress());

  /* Init ESP-NOW */
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW"); 
    return;
  }
  
  /* Register peer */
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); /* init peerInfo with 0 */
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  /* Add peer */
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(on_data_receive);
}

IRAM_ATTR void send_data_wirelessly(uint8_t data_to_transmit)
{
  for (uint8_t ii = WIRELESS_TRANSMISSION_TRIES; ii > 0; ii--)
  {
    esp_now_send(broadcastAddress, (uint8_t *) &data_to_transmit, 1);
    #if MEASURE_RTT
      tic();
    #endif    
    #if DEBUG
    Serial.printf("Transmitting data: %d\n", data_to_transmit);
    #endif
  }
}

/* this function handles some sensorcar_state switches. */
IRAM_ATTR void on_data_receive(const uint8_t * mac, const uint8_t *incoming_data, int len)
{
  race_status = *incoming_data;
  #if DEBUG
    Serial.printf("New race status received: %d\n", race_status);
  #endif

  switch (race_status)
  {
    case NO_RACE_GOING: /* race is stopped. */
      #if (OPERATION_MODE==RACING_MODE)
        sensorcar_state = SENSORCAR_IDLE_STATE;
      #endif
      break;
    case RACE_GOING: /* race has just started. */
      #if (OPERATION_MODE==RACING_MODE) /* normal operation mode */
        if (track_mapped_out_flag)
        {
          sensorcar_state = SENSORCAR_RACING_STATE;
        }
        else
        {
          sensorcar_state = SENSORCAR_TRACK_MAPPING_STATE;
        }
      #else /* measuring mode */
        sensorcar_state = SENSORCAR_MEASUREMENT_STATE;
      #endif
      break;
    case CAR_NO_0_PASSED_FINISH_LINE:
      xSemaphoreGive(finish_line_passed_semaphore);
      break;
    case RECEIVED_VALUE_WIRELESSLY:
      #if MEASURE_RTT    
        toc(); /* part of two functions to calculate wireless round trip time (RTT) */
      #endif
      break;
  }

}