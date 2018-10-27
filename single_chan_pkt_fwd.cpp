/* Enable debug print */
// #define ENABLE_DEBUG_PRINT

/* Enable ISR to get DIO0 signal, we need as root */
#define ENABLE_DIO0_ISR
#define DIO0_ISR_TIMEOUT_MS (1000)  /* wait timeout in milli-sec */

/* Enable quirk for LoRa driver adding waste 4 bytes at head */
#define QUIRK_LORA_PACKET_SIZE

 /******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/

// Raspberry PI pin mapping
// Pin number in this global_conf.json are Wiring Pi number (wPi colunm)
// issue a `gpio readall` on PI command line to see mapping
// +-----+-----+---------+--B Plus--+---------+-----+-----+
// | BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
// +-----+-----+---------+----++----+---------+-----+-----+
// |     |     |    3.3v |  1 || 2  | 5v      |     |     |
// |   2 |   8 |   SDA.1 |  3 || 4  | 5V      |     |     |
// |   3 |   9 |   SCL.1 |  5 || 6  | 0v      |     |     |
// |   4 |   7 | GPIO. 7 |  7 || 8  | TxD     | 15  | 14  |
// |     |     |      0v |  9 || 10 | RxD     | 16  | 15  |
// |  17 |   0 | GPIO. 0 | 11 || 12 | GPIO. 1 | 1   | 18  |
// |  27 |   2 | GPIO. 2 | 13 || 14 | 0v      |     |     |
// |  22 |   3 | GPIO. 3 | 15 || 16 | GPIO. 4 | 4   | 23  |
// |     |     |    3.3v | 17 || 18 | GPIO. 5 | 5   | 24  |
// |  10 |  12 |    MOSI | 19 || 20 | 0v      |     |     |
// |   9 |  13 |    MISO | 21 || 22 | GPIO. 6 | 6   | 25  |
// |  11 |  14 |    SCLK | 23 || 24 | CE0     | 10  | 8   |
// |     |     |      0v | 25 || 26 | CE1     | 11  | 7   |
// |   0 |  30 |   SDA.0 | 27 || 28 | SCL.0   | 31  | 1   |
// |   5 |  21 | GPIO.21 | 29 || 30 | 0v      |     |     |
// |   6 |  22 | GPIO.22 | 31 || 32 | GPIO.26 | 26  | 12  |
// |  13 |  23 | GPIO.23 | 33 || 34 | 0v      |     |     |
// |  19 |  24 | GPIO.24 | 35 || 36 | GPIO.27 | 27  | 16  |
// |  26 |  25 | GPIO.25 | 37 || 38 | GPIO.28 | 28  | 20  |
// |     |     |      0v | 39 || 40 | GPIO.29 | 29  | 21  |
// +-----+-----+---------+----++----+---------+-----+-----+
// | BCM | wPi |   Name  | Physical | Name    | wPi | BCM |
// +-----+-----+---------+--B Plus--+---------+-----+-----+

// global_conf.json For Dragino RPI Lora HAT
// http://wiki.dragino.com/index.php?title=Lora/GPS_HAT
//    "pin_nss": 6,
//    "pin_dio0": 7,
//    "pin_rst": 0
//
// For LoRasPi
// https://github.com/hallard/LoRasPI
//    "pin_nss": 8,
//    "pin_dio0": 6,
//    "pin_rst": 3,
//    "pin_led1":4


#include "base64.h"

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netdb.h>
#include <signal.h>

#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

using namespace rapidjson;

#define BASE64_MAX_LENGTH 341

static const int SPI_CHANNEL = 0;

bool g_is_sx1272 = true;

int g_socket;
#if 0 //YSK//
struct ifreq g_ifr;
#endif //YSK//

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_ok_tot;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

typedef enum SpreadingFactors
{
    SF7 = 7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12
} SpreadingFactor_t;

typedef struct Server
{
    string address;
    uint16_t port;
    bool enabled;
} Server_t;

/*******************************************************************************
 *
 * Default values, configure them in global_conf.json
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
// Put them in global_conf.json
int g_ssPin    = 0xff;
int g_dio0Pin  = 0xff;
int g_resetPin = 0xff;
int g_led1Pin  = 0xff;
int g_led2Pin  = 0xff;      /* life led */

#if defined(ENABLE_DIO0_ISR)
static char g_dio0Path[256];
#endif /* defined(ENABLE_DIO0_ISR) */

// Set location in global_conf.json
float g_lat =  0.0;
float g_lon =  0.0;
int   g_alt =  0;

/* Informal status fields */
char g_platform[24] ;       /* platform definition */
char g_email[40] ;          /* used for contact email */
char g_client_desc[64] ;    /* used for free form description */
uint8_t g_client_id;        /* udp client id */

// Set spreading factor (SF7 - SF12), &nd  center frequency
// Overwritten by the ones set in global_conf.json
SpreadingFactor_t g_sf = SF7;
uint16_t g_bw = 125;
uint32_t g_freq = 868100000; // in Mhz! (868.1)
uint8_t g_sync_word = 0x34;  // public LoRaWAN as default


// Servers
vector<Server_t> g_servers;

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB        0x1F
#define REG_PKT_SNR_VALUE           0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH      0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD               0x39
#define REG_VERSION                 0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81


#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08

#define FRF_MSB                  0xD9 // 868.1 Mhz
#define FRF_MID                  0x06
#define FRF_LSB                  0x66

#define PROTOCOL_VERSION  0x12
#define PKT_PUSH_DATA     0
//YSK// #define PKT_PUSH_ACK  1
//YSK// #define PKT_PULL_DATA 2
//YSK// #define PKT_PULL_RESP 3
//YSK// #define PKT_PULL_ACK  4

//YSK// #define BUFLEN 2048  //Max length of buffer
//YSK// #define TX_BUFF_SIZE    2048
//YSK// #define STATUS_SIZE     1024

#define MAX_SMARTHIVE_PAYLOAD_SIZE 127
#define SMARTHIVE_PAYLOAD_SIZE_MASK 0x7fU
#define BUFSIZE 256



void LoadConfiguration(string filename);
void PrintConfiguration();

//Flag for Ctrl-C
volatile sig_atomic_t g_force_exit = 0;

void sig_handler(int sig)
{
  g_force_exit=true;
}

void Die(const char *s)
{
  perror(s);
  exit(1);
}

void SelectReceiver()
{
  digitalWrite(g_ssPin, LOW);
}

void UnselectReceiver()
{
  digitalWrite(g_ssPin, HIGH);
}

uint8_t ReadRegister(uint8_t addr)
{
  uint8_t spibuf[2];
  spibuf[0] = addr & 0x7F;
  spibuf[1] = 0x00;

  SelectReceiver();
  wiringPiSPIDataRW(SPI_CHANNEL, spibuf, 2);
  UnselectReceiver();

  return spibuf[1];
}

void WriteRegister(uint8_t addr, uint8_t value)
{
  uint8_t spibuf[2];
  spibuf[0] = addr | 0x80;
  spibuf[1] = value;

  SelectReceiver();
  wiringPiSPIDataRW(SPI_CHANNEL, spibuf, 2);
  UnselectReceiver();
}

bool ReceivePkt(uint8_t *payload, uint8_t* p_length)
{
  // clear rxDone
  WriteRegister(REG_IRQ_FLAGS, 0x40);

  int irqflags = ReadRegister(REG_IRQ_FLAGS);

  cp_nb_rx_rcv++;

  //  payload crc: 0x20
  if((irqflags & 0x20) == 0x20) {
    printf("CRC error\n");
    WriteRegister(REG_IRQ_FLAGS, 0x20);
    return false;

  } else {
    cp_nb_rx_ok++;
    cp_nb_rx_ok_tot++;

    uint8_t currentAddr = ReadRegister(REG_FIFO_RX_CURRENT_ADDR);
    uint8_t receivedCount = ReadRegister(REG_RX_NB_BYTES);
    *p_length = receivedCount;

    WriteRegister(REG_FIFO_ADDR_PTR, currentAddr);

    for(int i = 0; i < receivedCount; i++) {
      payload[i] = ReadRegister(REG_FIFO);
    }
  }
  return true;
}

char * PinName(int pin, char * buff) {
  strcpy(buff, "unused");
  if (pin != 0xff) {
    sprintf(buff, "%d", pin);
  }
  return buff;
}

void SetupLoRa()
{
  char buff[16];

  printf("Trying to detect module with ");
  printf("NSS=%s "  , PinName(g_ssPin, buff));
  printf("DIO0=%s " , PinName(g_dio0Pin, buff));
  printf("Reset=%s ", PinName(g_resetPin, buff));
  printf("Led1=%s ",  PinName(g_led1Pin, buff));
  printf("Led2=%s ",  PinName(g_led2Pin, buff));
  printf("SyncWord=0x%02X\n", g_sync_word);

  // check basic
  if (g_ssPin == 0xff || g_dio0Pin == 0xff) {
    Die("Bad pin configuration ssPin and dio0 need at least to be defined");
  }

  digitalWrite(g_resetPin, HIGH);
  delay(100);
  digitalWrite(g_resetPin, LOW);
  delay(100);

  uint8_t version = ReadRegister(REG_VERSION);

  if (version == 0x22) {
    // sx1272
    printf("SX1272 detected, starting.\n");
    g_is_sx1272 = true;
  } else {
    // sx1276?
    digitalWrite(g_resetPin, LOW);
    delay(100);
    digitalWrite(g_resetPin, HIGH);
    delay(100);
    version = ReadRegister(REG_VERSION);
    if (version == 0x12) {
      // sx1276
      printf("SX1276 detected, starting.\n");
      g_is_sx1272 = false;
    } else {
      printf("Transceiver version 0x%02X\n", version);
      Die("Unrecognized transceiver");
    }
  }

  WriteRegister(REG_OPMODE, SX72_MODE_SLEEP);

  // set frequency
  uint64_t frf = ((uint64_t)g_freq << 19) / 32000000;
  WriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16) );
  WriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8) );
  WriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0) );

  // set sync word (18=0x12:private LoRa, 52=0x34:public LoRaWAN)
  WriteRegister(REG_SYNC_WORD, g_sync_word);

  if (g_is_sx1272) {
    if (g_sf == SF11 || g_sf == SF12) {
      WriteRegister(REG_MODEM_CONFIG, 0x0B);
    } else {
      WriteRegister(REG_MODEM_CONFIG, 0x0A);
    }
    WriteRegister(REG_MODEM_CONFIG2, (g_sf << 4) | 0x04);
  } else {
    if (g_sf == SF11 || g_sf == SF12) {
      WriteRegister(REG_MODEM_CONFIG3, 0x0C);
    } else {
      WriteRegister(REG_MODEM_CONFIG3, 0x04);
    }
    WriteRegister(REG_MODEM_CONFIG, 0x72);
    WriteRegister(REG_MODEM_CONFIG2, (g_sf << 4) | 0x04);
  }

  if (g_sf == SF10 || g_sf == SF11 || g_sf == SF12) {
    WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x05);
  } else {
    WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x08);
  }
  WriteRegister(REG_MAX_PAYLOAD_LENGTH, 0x80);
  WriteRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
  WriteRegister(REG_HOP_PERIOD, 0xFF);
  WriteRegister(REG_FIFO_ADDR_PTR, ReadRegister(REG_FIFO_RX_BASE_AD));

  // Set Continous Receive Mode
  WriteRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
  WriteRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

#if defined(ENABLE_DIO0_ISR)
  /*
   * https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
   */
  do {
    int brcmDio0Pin = wpiPinToGpio(g_dio0Pin);
    FILE *pFileExport = NULL;
    FILE *pFileDirection = NULL;
    FILE *pFileEdge = NULL;
    char path[256];

#if defined(ENABLE_DEBUG_PRINT)
    printf("Dio0 brcm pin#: %d\n", brcmDio0Pin);
#endif /* defined(ENABLE_DEBUG_PRINT) */

    /* export DIO0 */
    snprintf(g_dio0Path, sizeof(g_dio0Path),
        "/sys/class/gpio/gpio%d", brcmDio0Pin);
    g_dio0Path[sizeof(g_dio0Path) - 1] = '\0';

    pFileExport = fopen("/sys/class/gpio/export", "r+b");
    if (!pFileExport)
#if defined(ENABLE_DEBUG_PRINT)
      printf("Failed to open /sys/class/gpio/export.\n");
#endif /* defined(ENABLE_DEBUG_PRINT) */
      break;
    }
    fprintf(pFileExport, "%d\n", brcmDio0Pin);
    fclose(pFileExport); pFileExport = 0;
#if defined(ENABLE_DEBUG_PRINT)
    printf("Wrote '%d' into /sys/class/gpio/export.\n", brcmDio0Pin0);
#endif /* defined(ENABLE_DEBUG_PRINT) */

    /* set DIO0 to input */
    snprintf(path, sizeof(path), "%s/direction", g_dio0Path);
    path[sizeof(path) - 1] = '\0';

    pFileDirection = fopen(path, "r+b");
    if (!pFileDirection) {
#if defined(ENABLE_DEBUG_PRINT)
      printf("Failed to open %s.\n", path);
#endif /* defined(ENABLE_DEBUG_PRINT) */
      break;
    }
    fprintf(pFileDirection, "in\n");
    fclose(pFileDirection); pFileDirection = NULL;
#if defined(ENABLE_DEBUG_PRINT)
    printf("Wrote 'in' into %s.\n", path);
#endif /* defined(ENABLE_DEBUG_PRINT) */

    /* set DIO0 interrupt to getting rising-edge */
    snprintf(path, sizeof(path), "%s/edge", g_dio0Path);
    path[sizeof(path) - 1] = '\0';

    pFileEdge = fopen(path, "r+b");
    if (!pFileEdge) {
#if defined(ENABLE_DEBUG_PRINT)
      printf("Failed to open %s.\n", path);
#endif /* defined(ENABLE_DEBUG_PRINT) */
      break;
    }
    fprintf(pFileEdge, "rising\n");
    fclose(pFileEdge); pFileEdge = NULL;
#if defined(ENABLE_DEBUG_PRINT)
    printf("Wrote 'rising' into %s.\n", path);
#endif /* defined(ENABLE_DEBUG_PRINT) */
  } while (0);
#endif /* defined(ENABLE_DIO0_ISR) */
}

void SolveHostname(const char* p_hostname, uint16_t port, struct sockaddr_in* p_sin)
{
  struct addrinfo hints;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  char service[6] = { '\0' };
  snprintf(service, 6, "%hu", port);

  struct addrinfo* p_result = NULL;

  // Resolve the domain name into a list of addresses
  int error = getaddrinfo(p_hostname, service, &hints, &p_result);
  if (error != 0) {
      fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
      exit(EXIT_FAILURE);
  }

  // Loop over all returned results
  for (struct addrinfo* p_rp = p_result; p_rp != NULL; p_rp = p_rp->ai_next) {
    struct sockaddr_in* p_saddr = (struct sockaddr_in*)p_rp->ai_addr;
    //printf("%s solved to %s\n", p_hostname, inet_ntoa(p_saddr->sin_addr));
    p_sin->sin_addr = p_saddr->sin_addr;
  }

  freeaddrinfo(p_result);
}

void SendUdp(uint8_t *msg, size_t length)
{
    struct sockaddr_in sa = { .sin_family = AF_INET };

    for (vector<Server_t>::iterator it = g_servers.begin(); it != g_servers.end(); ++it) {
        if (it->enabled) {
            sa.sin_port = htons(it->port);
            SolveHostname(it->address.c_str(), it->port, &sa);
            if (sendto(g_socket, msg, length, 0 , (struct sockaddr *) &sa, sizeof(sa))==-1) {
                perror("sendto");
            }
        }
    }
}

void SendStat()
{
#if 0 //YSK//
  static char status_report[STATUS_SIZE]; /* status report as a JSON object */
  char stat_timestamp[24];

  int stat_index = 0;

  /* pre-fill the data buffer with fixed fields */
  status_report[0] = PROTOCOL_VERSION;
  status_report[3] = PKT_PUSH_DATA;

  status_report[4] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[0];
  status_report[5] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[1];
  status_report[6] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[2];
  status_report[7] = 0xFF;
  status_report[8] = 0xFF;
  status_report[9] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[3];
  status_report[10] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[4];
  status_report[11] = (unsigned char)g_ifr.ifr_hwaddr.sa_data[5];

  /* start composing datagram with the header */
  uint8_t token_h = (uint8_t)rand(); /* random token */
  uint8_t token_l = (uint8_t)rand(); /* random token */
  status_report[1] = token_h;
  status_report[2] = token_l;
  stat_index = 12; /* 12-byte header */

  /* get timestamp for statistics */
  time_t t = time(NULL);
  strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

  // Build JSON object.
  StringBuffer sb;
  Writer<StringBuffer> writer(sb);
  writer.StartObject();
  writer.String("stat");
  writer.StartObject();
  writer.String("time");
  writer.String(stat_timestamp);
  writer.String("lati");
  writer.Double(g_lat);
  writer.String("long");
  writer.Double(g_lon);
  writer.String("alti");
  writer.Int(g_alt);
  writer.String("rxnb");
  writer.Uint(cp_nb_rx_rcv);
  writer.String("rxok");
  writer.Uint(cp_nb_rx_ok);
  writer.String("rxfw");
  writer.Uint(cp_up_pkt_fwd);
  writer.String("ackr");
  writer.Double(0);
  writer.String("dwnb");
  writer.Uint(0);
  writer.String("txnb");
  writer.Uint(0);
  writer.String("pfrm");
  writer.String(g_platform);
  writer.String("mail");
  writer.String(g_email);
  writer.String("desc");
  writer.String(g_client_desc);
  writer.EndObject();
  writer.EndObject();

  string json = sb.GetString();
  //printf("stat update: %s\n", json.c_str());
  printf("stat update: %s", stat_timestamp);
  if (cp_nb_rx_ok_tot==0) {
    printf(" no packet received yet\n");
  } else {
    printf(" %u packet%sreceived\n", cp_nb_rx_ok_tot, cp_nb_rx_ok_tot>1?"s ":" ");
  }

  // Build and send message.
  memcpy(status_report + 12, json.c_str(), json.size());
  SendUdp(status_report, stat_index + json.size());
#endif //YSK//
}

#if defined(ENABLE_DIO0_ISR)
static bool WaitForDio0_(void)
{
  int fd = -1;
  int ret = 0;
  struct pollfd fds = { 0 };

  fd = open(g_dio0Path, O_RDONLY);
  if (fd < 0) {
#if defined(ENABLE_DEBUG_PRINT)
    printf("Failed to open %s.\n", g_dio0Path);
#endif /* defined(ENABLE_DEBUG_PRINT) */
    return false;
  }

  /* dummy read */
  {
    int nr;
    if (!ioctl(fd, FIONREAD, &nr)) {
      int i;
      char c;
#if defined(ENABLE_DEBUG_PRINT)
      printf("Need dummy read %d bytes.\n", nr);
#endif /* defined(ENABLE_DEBUG_PRINT) */
      for (i = 0 ; i < nr ; ++i) {
        read(fd, &c, sizeof(c));
      }
    }
  }

  /* polling gpio */
  fds.events = (POLLPRI | POLLERR) ;
  fds.revents = 0;
  fds.fd = fd;

  ret = poll(&fds, 1, DIO0_ISR_TIMEOUT_MS);
  close(fd), fd = -1;

#if defined(ENABLE_DEBUG_PRINT)
  printf("Polling %s returns %d.\n", g_dio0Path, ret);
#endif /* defined(ENABLE_DEBUG_PRINT) */
  return 1 == ret ? true : false;
}
#else /* defined(ENABLE_DIO0_ISR) */
# define WaitForDio0_() {}
#endif /* defined(ENABLE_DIO0_ISR) */

bool Receivepacket()
{
    static uint8_t udp_buf[BUFSIZE];
    static uint8_t lora_buf[BUFSIZE];

    uint8_t len = 0;
    uint8_t lora_len = 0;

    WaitForDio0_();

    if (digitalRead(g_dio0Pin) != 1) {
        return false;
    }
#if defined(QUIRK_LORA_PACKET_SIZE)
    if (!ReceivePkt(lora_buf, &len) || len < 4) {
#else /* defined(QUIRK_LORA_PACKET_SIZE) */
    if (!ReceivePkt(lora_buf, &len) || 0 == len) {
#endif /* defined(QUIRK_LORA_PACKET_SIZE) */
        return false;
    }
    // OK got one

#if defined(QUIRK_LORA_PACKET_SIZE)
    lora_len = len - 4;
#else /* defined(QUIRK_LORA_PACKET_SIZE) */
    lora_len = len;
#endif /* defined(QUIRK_LORA_PACKET_SIZE) */
    lora_len &= SMARTHIVE_PAYLOAD_SIZE_MASK;

    /*
     * |<----------- A bytes ----------->|
     * |               |<-- A-4 bytes -->|
     * +---+---+---+---+-----.......-----+
     * | A | B | C | D |        E        |
     * +---+---+---+---+-----.......-----+
     *
     *  * A: Protocol version, 1byte
     *  * B: Length (from A to D), 1byte
     *  * C: UDP client ID, 1byte
     *  * D: Packet type, 1byte
     *  * E: LoRa data, A-4bytes (max 127 bytes)
     */

    udp_buf[0] = PROTOCOL_VERSION;
    udp_buf[1] = 4 + lora_len;
    udp_buf[2] = g_client_id;
    udp_buf[3] = PKT_PUSH_DATA;
#if defined(QUIRK_LORA_PACKET_SIZE)
    memcpy(&udp_buf[4], lora_buf + 4, lora_len);
#else /* defined(QUIRK_LORA_PACKET_SIZE) */
    memcpy(&udp_buf[4], lora_buf, lora_len);
#endif /* defined(QUIRK_LORA_PACKET_SIZE) */

#if defined(ENABLE_DEBUG_PRINT)
    printf("Send UDP, header: %02x %02x %02x %02x\n",
            udp_buf[0], udp_buf[1], udp_buf[2], udp_buf[3]);
#endif /* defined(ENABLE_DEBUG_PRINT) */
    SendUdp(udp_buf, udp_buf[1]);

#if defined(ENABLE_DEBUG_PRINT)
    {
        struct timeval now;
        uint8_t v8_tmp = 0;

        uint32_t tmst = 0;
        long int snr = 0;
        int rssicorr = 0;

        // TODO: tmst can jump is time is (re)set, not good.
        gettimeofday(&now, NULL);
        tmst = (uint32_t)(now.tv_sec * 1000000 + now.tv_usec);

        v8_tmp = ReadRegister(REG_PKT_SNR_VALUE);
        if (v8_tmp & 0x80) { // The snr sign bit is 1
            // Invert and divide by 4
            v8_tmp = ((~v8_tmp + 1) & 0xFF) >> 2;
            snr = -v8_tmp;
        } else {
            // Divide by 4
            snr = ( v8_tmp & 0xFF ) >> 2;
        }

        rssicorr = g_is_sx1272 ? 139 : 157;

        printf(
            "[rxpk] "
            "tmst: %u, "
            "g_freq: %lf, "
            "chan: 0, rfch: 0, stat: 1, "
            "modu LORA datr: SF%hhuBW%hu, "
            "codr: 4/5\n"
            "Packet RSSI: %d, "
            "RSSI: %d, "
            "LSNR: %li, "
            "Length: %hhu, Buf:'"
            , tmst
            , (double)g_freq / 1000000
            , g_sf, g_bw
            , ReadRegister(0x1A) - rssicorr
            , ReadRegister(0x1B) - rssicorr
            , snr
            , len
            );
        for (int i=0; i<len; i++) {
            printf(" %02x", lora_buf[i]);
        }
        printf("'\n");
    }
    fflush(stdout);

#endif /* defined(ENABLE_DEBUG_PRINT) */

  return true;
}

int main()
{
  struct timeval nowtime;
  uint32_t lasttime;
  unsigned int led1_timer;

  // caught CTRL-C to do clean-up
  signal(SIGINT, sig_handler);

  LoadConfiguration("global_conf.json");
  PrintConfiguration();

  // Init WiringPI
  wiringPiSetup() ;
  pinMode(g_ssPin, OUTPUT);
  pinMode(g_dio0Pin, INPUT);
  pinMode(g_resetPin, OUTPUT);

  // Init SPI
  wiringPiSPISetup(SPI_CHANNEL, 500000);

  // Setup LORA
  SetupLoRa();

  // LED ?
  if (g_led1Pin != 0xff) {
    pinMode(g_led1Pin, OUTPUT);

    // Blink to indicate startup
    for (uint8_t i=0; i<5 ; i++) {
      digitalWrite(g_led1Pin, 1);
      delay(50);
      digitalWrite(g_led1Pin, 0);
      delay(50);
    }
  }
  if (g_led2Pin != 0xff) {
    pinMode(g_led2Pin, OUTPUT);
    digitalWrite(g_led2Pin, 1);
  }

  // Prepare Socket connection
  if ((g_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    Die("socket");
  }

#if 0 //YSK//
  g_ifr.ifr_addr.sa_family = AF_INET;
  strncpy(g_ifr.ifr_name, "eth0", IFNAMSIZ-1);  // can we rely on eth0?
  ioctl(g_socket, SIOCGIFHWADDR, &g_ifr);

  // ID based on MAC Adddress of eth0
  printf( "Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[0],
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[1],
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[2],
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[3],
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[4],
              (uint8_t)g_ifr.ifr_hwaddr.sa_data[5]
  );
#endif //YSK//

  printf("Listening at SF%i on %.6lf Mhz.\n", g_sf,(double)g_freq/1000000);
  printf("-----------------------------------\n");

  while (!g_force_exit) {

    // Packet received ?
    if (Receivepacket()) {
      // Led ON
      if (g_led1Pin != 0xff) {
        digitalWrite(g_led1Pin, 1);
      }

      // start our Led blink timer, LED as been lit in Receivepacket
      led1_timer=millis();
    }

    gettimeofday(&nowtime, NULL);
    uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
    if (nowseconds - lasttime >= 30) {
      lasttime = nowseconds;
      SendStat();
      cp_nb_rx_rcv = 0;
      cp_nb_rx_ok = 0;
      cp_up_pkt_fwd = 0;
    }

    // Led timer in progress ?
    if (led1_timer) {
      // Led timer expiration, Blink duration is 250ms
      if (millis() - led1_timer >= 250) {
        // Stop Led timer
        led1_timer = 0;

        // Led OFF
        if (g_led1Pin != 0xff) {
          digitalWrite(g_led1Pin, 0);
        }
      }
    }

    // Let some time to the OS
    delay(1);
  }
  printf("\nBreak received, exiting!\n");

  // All module LEDs off
  if (g_led1Pin != 0xff) {
    digitalWrite(g_led1Pin, 0);
  }
  if (g_led2Pin != 0xff) {
    digitalWrite(g_led2Pin, 0);
  }


  // CS Ping as output and set to 1
  // to avoid any problem with SPI sharing
  UnselectReceiver();
  delay(150);

  // Reset
  digitalWrite(g_resetPin, LOW);
  delay(150);

  return (0);
}

void LoadConfiguration(string configurationFile)
{
  FILE* p_file = fopen(configurationFile.c_str(), "r");
  char buffer[65536];
  FileReadStream fs(p_file, buffer, sizeof(buffer));

  Document document;
  document.ParseStream(fs);

  for (Value::ConstMemberIterator fileIt = document.MemberBegin(); fileIt != document.MemberEnd(); ++fileIt) {
    string objectType(fileIt->name.GetString());
    if (objectType.compare("SX127x_conf") == 0) {
      const Value& sx127x_conf = fileIt->value;
      if (sx127x_conf.IsObject()) {
        for (Value::ConstMemberIterator confIt = sx127x_conf.MemberBegin(); confIt != sx127x_conf.MemberEnd(); ++confIt) {
          string key(confIt->name.GetString());
          if (key.compare("freq") == 0) {
            g_freq = confIt->value.GetUint();
          } else if (key.compare("spread_factor") == 0) {
            g_sf = (SpreadingFactor_t)confIt->value.GetUint();
          } else if (key.compare("sync_word") == 0) {
            g_sync_word = (uint8_t)(0xffU & confIt->value.GetUint());
          } else if (key.compare("pin_nss") == 0) {
            g_ssPin = confIt->value.GetUint();
          } else if (key.compare("pin_dio0") == 0) {
            g_dio0Pin = confIt->value.GetUint();
          } else if (key.compare("pin_rst") == 0) {
            g_resetPin = confIt->value.GetUint();
          } else if (key.compare("pin_led1") == 0) {
            g_led1Pin = confIt->value.GetUint();
          } else if (key.compare("pin_led2") == 0) {
            g_led2Pin = confIt->value.GetUint();
          }
        }
      }

    } else if (objectType.compare("gateway_conf") == 0) {
      const Value& gateway_conf = fileIt->value;
      if (gateway_conf.IsObject()) {
        for (Value::ConstMemberIterator confIt = gateway_conf.MemberBegin(); confIt != gateway_conf.MemberEnd(); ++confIt) {
          string memberType(confIt->name.GetString());
          if (memberType.compare("ref_latitude") == 0) {
            g_lat = confIt->value.GetDouble();
          } else if (memberType.compare("ref_longitude") == 0) {
            g_lon = confIt->value.GetDouble();
          } else if (memberType.compare("ref_altitude") == 0) {
            g_alt = confIt->value.GetUint();

          } else if (memberType.compare("name") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(g_platform, str.length()<=24 ? str.c_str() : "name too long");
          } else if (memberType.compare("email") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(g_email, str.length()<=40 ? str.c_str() : "email too long");
          } else if (memberType.compare("desc") == 0 && confIt->value.IsString()) {
            string str = confIt->value.GetString();
            strcpy(g_client_desc, str.length()<=64 ? str.c_str() : "description is too long");
          } else if (memberType.compare("client_id") == 0 && confIt->value.IsUint()) {
            g_client_id = 0xffU & confIt->value.GetUint();
          } else if (memberType.compare("servers") == 0) {
            const Value& serverConf = confIt->value;
            if (serverConf.IsObject()) {
              const Value& serverValue = serverConf;
              Server_t server;
              for (Value::ConstMemberIterator srvIt = serverValue.MemberBegin(); srvIt != serverValue.MemberEnd(); ++srvIt) {
                string key(srvIt->name.GetString());
                if (key.compare("address") == 0 && srvIt->value.IsString()) {
                  server.address = srvIt->value.GetString();
                } else if (key.compare("port") == 0 && srvIt->value.IsUint()) {
                  server.port = srvIt->value.GetUint();
                } else if (key.compare("enabled") == 0 && srvIt->value.IsBool()) {
                  server.enabled = srvIt->value.GetBool();
                }
              }
              g_servers.push_back(server);
            }
            else if (serverConf.IsArray()) {
              for (SizeType i = 0; i < serverConf.Size(); i++) {
                const Value& serverValue = serverConf[i];
                Server_t server;
                for (Value::ConstMemberIterator srvIt = serverValue.MemberBegin(); srvIt != serverValue.MemberEnd(); ++srvIt) {
                  string key(srvIt->name.GetString());
                  if (key.compare("address") == 0 && srvIt->value.IsString()) {
                    server.address = srvIt->value.GetString();
                  } else if (key.compare("port") == 0 && srvIt->value.IsUint()) {
                    server.port = srvIt->value.GetUint();
                  } else if (key.compare("enabled") == 0 && srvIt->value.IsBool()) {
                    server.enabled = srvIt->value.GetBool();
                  }
                }
                g_servers.push_back(server);
              }
            }
          }
        }
      }
    }
  }
}

void PrintConfiguration()
{
  for (vector<Server_t>::iterator it = g_servers.begin(); it != g_servers.end(); ++it) {
    printf("server: .address = %s; .port = %hu; .enable = %d\n", it->address.c_str(), it->port, it->enabled);
  }
  printf("Gateway Configuration\n");
  printf("  %s (%s)\n  %s\n", g_platform, g_email, g_client_desc);
  printf("  Latitude=%.8f\n  Longitude=%.8f\n  Altitude=%d\n", g_lat,g_lon,g_alt);

}



/* vim: set ts=4 sts=4 sw=4 expandtab autoindent : */
