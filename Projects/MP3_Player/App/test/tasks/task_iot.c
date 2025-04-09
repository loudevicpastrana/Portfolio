/*
 * University of Washington
 * Certificate in Embedded and Real-Time Systems
 *
 * (c) 2021 Cristian Pop
 */

#include "main.h"
#include "log.h"
#include "es_wifi.h"
#include "MQTTPacket.h"
#include "mp3_control.h"


#define WIFI_SSID "x"
#define WIFI_PASS "x"
#define WIFI_TYPE ES_WIFI_SEC_WPA2 // WIFI Type

// Look for this in AWS Console IoT / Settings
static const char* endpoint = "x";

// Defined in AWS Console IoT / Secure / Policies
static char* client_id = "x"; // iot:Connect
static char* pub_topic = "x";     // iot:Publish
static char* sub_topic = "x";     // iot:Subscribe; iot:Receive

#define WIFI_BUFFER_LEN 1024
static unsigned char wifi_buffer[WIFI_BUFFER_LEN];

static Mp3Command_t cmd;



OS_FLAG_GRP *dataFlag; 
OS_FLAGS flags_iot;

// Obtained when creating a new thing in AWS Console / Manage / Things:

// Credentials
// Amazon Root CA 1
static const char* ca_certificate_pem = 
"x";

static const char* device_certificate_pem = 
"x";

static const char* device_key_pem =
"x";

int32_t wifi_probe(void **ll_drv_context);

ES_WIFIObject_t  *pEsWifiObj;

void PrintWiFiMAC()
{
  uint8_t mac[6] = { 0 };
  ES_WIFI_GetMACAddress(pEsWifiObj, mac);
  log_debug("%x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void PrintIP(uint8_t addr[4])
{
  log_debug("%d.%d.%d.%d\n", addr[0], addr[1], addr[2], addr[3]);
}

void PrintIfConfig()
{
    log_debug("IFCONFIG:\n");

    log_debug("WiFi Device Product: %s\n", (char*)pEsWifiObj->Product_Name);
    log_debug(" ID: %s\n", (char*)pEsWifiObj->Product_ID);
    log_debug("WiFi Device Firmware Rev: %s\n", (char*)pEsWifiObj->FW_Rev);
    log_debug("WiFi Device API Rev: %s\n", (char*)pEsWifiObj->API_Rev);
    log_debug("WiFi Device Stack Rev: %s\n", (char*)pEsWifiObj->Stack_Rev);

    log_debug("WiFi Device RTOS Rev: %s\n", (char*)pEsWifiObj->RTOS_Rev);
    log_debug("  MAC: ");
    PrintWiFiMAC();

    log_debug("  IP : ");
    PrintIP(pEsWifiObj->NetSettings.IP_Addr);

    log_debug("  Netmask : ");
    PrintIP(pEsWifiObj->NetSettings.IP_Mask);

    log_debug("  GW : ");
    PrintIP(pEsWifiObj->NetSettings.Gateway_Addr);

    log_debug("  DNS: ");
    PrintIP(pEsWifiObj->NetSettings.DNS1);
    PrintIP(pEsWifiObj->NetSettings.DNS2);
}

// Can be 0,1,2,3
#define CONN_NUMBER  0
// Can be 0,1,2,3 (ES_WIFI_TLS_MULTIPLE_WRITE_SLOT),??
#define CRED_SET ES_WIFI_TLS_MULTIPLE_WRITE_SLOT
#define IO_TIMEOUT_MILLISECONDS 10000

int transport_sendPacketBuffer(unsigned char* buf, int buflen)
{
  ES_WIFI_Status_t ret = 0;
  int index = 0;
  
  while((!ret) && (buflen - index > 0))
  {
    uint16_t wrote = 0;
    ret = ES_WIFI_SendData(pEsWifiObj, CONN_NUMBER, buf + index, buflen - index, &wrote, IO_TIMEOUT_MILLISECONDS);
    index += wrote;
  }

  return ret;
}

/* must return -1 for error, 0 for call again, or the number of bytes read */
int transport_getdata(unsigned char* buf, int count)
{
  ES_WIFI_Status_t ret;
  uint16_t read = 0;
  ret = ES_WIFI_ReceiveData(pEsWifiObj, CONN_NUMBER, buf, count, &read, IO_TIMEOUT_MILLISECONDS);

  if (ret == ES_WIFI_STATUS_OK || ret == ES_WIFI_STATUS_TIMEOUT)
  {
    return read;
  }

  return -1;
}

static void WifiInit()
{
  // Bind WiFi driver to SPI driver.
  if(wifi_probe((void**)&pEsWifiObj)) 
  {
    log_error("Failed to bind WiFi to SPI.\n");
    ErrorHandler();
  }
      
  log_debug("WiFi Reset to Factory Defaults...\n");
  if(ES_WIFI_Init(pEsWifiObj)) ErrorHandler();
  if(ES_WIFI_ResetToFactoryDefault(pEsWifiObj)) ErrorHandler();
  if(ES_WIFI_ResetModule(pEsWifiObj)) ErrorHandler();
  //if(ES_WIFI_HardResetModule(pEsWifiObj)) ErrorHandler();

  if(ES_WIFI_Init(pEsWifiObj)) ErrorHandler();
}

static void WifiConnect()
{
  ES_WIFI_Status_t ret;

  if(ES_WIFI_Connect(pEsWifiObj, WIFI_SSID, WIFI_PASS, WIFI_TYPE))
  {
    ErrorHandler();
  }

  log_debug("WiFi connected\n");

  if(ES_WIFI_GetNetworkSettings(pEsWifiObj))
  {
    ErrorHandler();
  }

  PrintIfConfig();

  // Not supported by the WiFi driver (see documentation for details).
  log_debug("Syncrhonizing NTP. Unix Time=");
  if((ret = ES_WIFI_AtCommand(pEsWifiObj,"GT\r", (char*)pEsWifiObj->CmdData)))
  {
    ErrorHandler();
  }
  log_debug((char*)pEsWifiObj->CmdData);
  log_debug("\n");
  // Retry if time appears to be time from device start "1234\r\nOK\r\n" instead of UnixTime "1636299132\r\n\OK\r\n"
  if(strlen((char*)pEsWifiObj->CmdData) < 15)
  {
    log_error("Failed to sync real-time clock to NTP.\n");
    // Not critical for TLS communications with AWS.
    // This can be used to initialize the on-board RTC.
  }
}

static void MQTTConfigureCredentials()
{
  ES_WIFI_Status_t ret;

  if((ret = ES_WIFI_StoreCreds(
      pEsWifiObj,
      ES_WIFI_FUNCTION_TLS,
      CRED_SET,
      (uint8_t*)ca_certificate_pem,
      strlen(ca_certificate_pem) + 1,
      (uint8_t*)device_certificate_pem,
      strlen(device_certificate_pem) + 1,
      (uint8_t*)device_key_pem,
      strlen(device_key_pem) + 1)))
  {
    ErrorHandler();
  }
}

void TLSConnect()
{
  ES_WIFI_Status_t ret;

  ES_WIFI_Conn_t conn =
  {
    ES_WIFI_TCP_SSL_CONNECTION,
    ES_WIFI_TLS_CHECK_DEVICE_CERTS,   // ES_WIFI_TLS_CHECK_DEVICE_CERTS: mutual auth, ES_WIFI_TLS_CHECK_ROOTCA: server auth.
    CONN_NUMBER,
    8883,
    0,
    { 0 },
    NULL,
    0
  };

  // http://www.inventeksys.com/iwin/getting-started-guide/#connect and http://www.inventeksys.com/iwin/wp-content/uploads/IWIN_Command_Set_Users_Manual.pdf
  // D0 (DNS query) will auto-update IP.

  // Note - storing the IP in the struct has no use or effect.
  log_debug("Looking up IP\n");
  if ((ret = ES_WIFI_DNS_LookUp(pEsWifiObj, endpoint, &conn.RemoteIP[0])))
  {
    ErrorHandler();
  }
  log_debug("%s resolved to ", endpoint);
  PrintIP(conn.RemoteIP);

  log_debug("Connecting to TLS %s :%d.\n", endpoint, conn.RemotePort);
  ret = ES_WIFI_StartClientConnection(pEsWifiObj, &conn);
  if (ret)
  {
    log_debug("Failed to connect: %d\n", ret);
    ErrorHandler();
  }
}

void MQTTConnect()
{
  ES_WIFI_Status_t ret;
  int buffer_len;

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  
  // Note: Only ClientIDs allowed by the device policy are allowed.
  data.clientID.cstring = client_id; //sdk-java
  data.cleansession = 1;    // Maintain the previous session, if available.
  
  buffer_len = sizeof(wifi_buffer);
  buffer_len = MQTTSerialize_connect(wifi_buffer, buffer_len, &data);

  if((ret = transport_sendPacketBuffer(wifi_buffer, buffer_len)))
  {
    log_debug("Failed to send MQTT packet: %d.\n", ret);
    ErrorHandler();
  }

  unsigned char sessionPresent;

  buffer_len = sizeof(wifi_buffer);
  if (MQTTPacket_read(wifi_buffer, buffer_len, transport_getdata) == CONNACK)
  {
    unsigned char connack_rc;

    if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, wifi_buffer, buffer_len) != 1 || connack_rc != 0)
    {
      log_debug("MQTT Unable to connect. CONNACK returned %d\n", connack_rc);
      ErrorHandler();
    }
  }
  else
  {
    log_debug("MQTT: invalid server response.\n");
    ErrorHandler();
  }

  log_debug("Connected [%s]\n", sessionPresent ? "Session:true " : "Session:false ");
}

void MQTTSubscribe()
{
  MQTTString topic = MQTTString_initializer;
  int msgid = 1;
  int req_qos = 0;
  int buffer_len;

  /* subscribe */
  // Note: Only topics allowed by the device policy are allowed.
  // Use https://console.aws.amazon.com/iot/home#/test to listen for messages from this topic.
  topic.cstring = sub_topic;
  buffer_len = sizeof(wifi_buffer);
  buffer_len = MQTTSerialize_subscribe(wifi_buffer, buffer_len, 0, msgid, 1, &topic, &req_qos);

  if(transport_sendPacketBuffer(wifi_buffer, buffer_len))
  {
    ErrorHandler();
  }

  buffer_len = sizeof(wifi_buffer);
  if (MQTTPacket_read(wifi_buffer, buffer_len, transport_getdata) == SUBACK) 	/* wait for suback */
  {
    unsigned short submsgid;
    int subcount;
    int granted_qos;

    MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, wifi_buffer, buffer_len);
    if (granted_qos != 0)
    {
      log_debug("MQTT Invalid SUBACK qos.\n");
      ErrorHandler();
    }
    else
    {
      log_debug("SUBACK\n");
    }
  }
  else
  {
    ErrorHandler();
  }
}

void MQTTRecv()
{
  INT8U err;
  int buffer_len;
  buffer_len = sizeof(wifi_buffer);
  int type = MQTTPacket_read(wifi_buffer, buffer_len, transport_getdata);
  log_debug("MQTT RECV Type: %d\n", type);
  if (type == PUBLISH)
  {
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short msgid;
    int payloadlen;
    unsigned char* payload;
    MQTTString receivedTopic;

    MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
        &payload, &payloadlen, wifi_buffer, buffer_len);

    // Convert payload to a C-string
    char payload_buffer[payloadlen + 1];
    memcpy(payload_buffer, payload, payloadlen);
    payload_buffer[payloadlen] = '\0'; // Null terminate

    if (strstr(payload_buffer, "PLAY") != NULL)
    {
        log_debug("Detected 'PLAY' in the message!\n");
        // Execute relevant action for "PLAY" command
        cmd = MP3_CMD_PLAY;
        OSQPost(mp3CmdQueue, (void *)&cmd);
        p_msg = &cmd;
        OSFlagPost(dataFlag, FLAG_NEW_DATA, OS_FLAG_SET, &err);  // Set the flag
    }

    else if (strstr(payload_buffer, "STOP") != NULL)
    {
        log_debug("Detected 'STOP' in the message!\n");
        // Execute relevant action for "PLAY" command
        cmd = MP3_CMD_STOP;
        OSQPost(mp3CmdQueue, (void *)&cmd);
        OSFlagPost(dataFlag, FLAG_NEW_DATA, OS_FLAG_SET, &err);  // Set the flag
    }

    else if (strstr(payload_buffer, "NEXT") != NULL)
    {
        log_debug("Detected 'NEXT' in the message!\n");
       
        cmd = MP3_CMD_NEXT;
        OSQPost(mp3CmdQueue, (void *)&cmd);
        OSFlagPost(dataFlag, FLAG_NEW_DATA, OS_FLAG_SET, &err);  // Set the flag
        currentSongIndex++;
    }
    else if (strstr(payload_buffer, "PREV") != NULL)
    {
        log_debug("Detected 'PREV' in the message!\n");
        // Execute relevant action for "PLAY" command
        cmd = MP3_CMD_PREV;
        OSQPost(mp3CmdQueue, (void *)&cmd);
        OSFlagPost(dataFlag, FLAG_NEW_DATA, OS_FLAG_SET, &err);  // Set the flag
        currentSongIndex--;
        
    }
    
    // Note that MQTTString.cstring in this case is NULL.
    log_debug(
      "MQTT RECEIVED: topic='%.*s'\n", 
      receivedTopic.lenstring.len, receivedTopic.lenstring.data);

    log_debug("Payload [%d bytes]: [%.*s]\n", payloadlen, payloadlen, payload);
  }
  else if (type == PUBACK)
  {
    log_debug("PUBACK\n");
  }
}

void MQTTSend()
{
  ES_WIFI_Status_t ret;
  int buffer_len;
  MQTTString topic = MQTTString_initializer;

  topic.cstring = pub_topic;
  buffer_len = sizeof(wifi_buffer);

  // JSON Formatted payloads are accepted by most IoT providers:
 // char message[] = "{\r\n  \"message\": \"Hello from device!\"\r\n}";
  buffer_len = MQTTSerialize_publish(wifi_buffer, buffer_len, 0, 0, 0, 0, 
                topic, (unsigned char*)mp3Files[currentSongIndex], sizeof(mp3Files[currentSongIndex]));
  
  if((ret = transport_sendPacketBuffer(wifi_buffer, buffer_len)))
  {
    log_debug("Failed to send MQTT packet: %d.\n", ret);
    ErrorHandler();
  }
}

void MQTTDisconnect()
{
  ES_WIFI_Status_t ret;
  int buffer_len;

  buffer_len = sizeof(wifi_buffer);
  buffer_len = MQTTSerialize_disconnect(wifi_buffer, buffer_len);
  if((ret = transport_sendPacketBuffer(wifi_buffer, buffer_len)))
  {
    log_debug("Failed to send MQTT disconnect: %d\n", ret);
    ErrorHandler();
  }

  ret = ES_WIFI_StopClientConnection(pEsWifiObj, CONN_NUMBER);
  if (ret)
  {
    log_debug("Failed to disconnect client socket: %d", ret);
  }
}

void IoTTask(void* pdata)
{
  INT8U err;
	log_debug("IoTTask: starting\n");

  // UW_TODO: Copy/paste code from test_iot.c.
  // Code within test_iot() will run from within this task.
  //UW_SOLUTION

  printf("-- TEST: IoT\n");

  /* Layers:
   *  1. WiFi (802.11)
   *  2. IP
   *  3. TCP
   *  4. TLS           ^ All above handled by the WiFi module ^
   *  5. MQTT            Handled by the application (Paho MQTTPacket).
   */

  dataFlag = OSFlagCreate(0, &err);  // Create a flag group with initial value 0 

  WifiInit();

  printf("Connecting to WiFi...\n");
  WifiConnect();

  printf("Configuring X509 certificates\n");
  MQTTConfigureCredentials();

  printf("Connecting TLS/TCP socket\n");
  TLSConnect();

  printf("Connecting to MQTT server\n");
  MQTTConnect();

  printf("Subscribing to topic %s\n", sub_topic);
  MQTTSubscribe();

  // UW_TODO: In order to limit cost in case the device is left running, please limit traffic to 10 
  // messages instead of an infinite Send/Receive loop.
  //UW_SOLUTION
  //for (int i = 0; i < 10; i++)
  //{
    // Assuming read timeout.
    
  //}

  //printf("Disconnecting.\n");
  //MQTTDisconnect();

  //return 0;



	log_debug("IoTTask: START.\n");

  while(1)
  {
    MQTTRecv();

    printf("Publishing to topic %s\n", pub_topic);
    MQTTSend();

    OSTimeDly(1);
  }
}
