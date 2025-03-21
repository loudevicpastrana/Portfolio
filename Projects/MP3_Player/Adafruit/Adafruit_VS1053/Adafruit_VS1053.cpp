/*!
 * @file Adafruit_VS1053.cpp
 *
 * @mainpage Adafruit VS1053 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit VS1053 Codec Breakout
 *
 * Designed specifically to work with the Adafruit VS1053 Codec Breakout
 * ----> https://www.adafruit.com/products/1381
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include <Adafruit_VS1053.h>
#include <stdio.h>

#define byte char

#ifndef _BV
#define _BV(x) (1 << (x)) //!< Macro that returns the "value" of a bit
#endif

// UW interrupt management
#define noInterrupts()
#define interrupts()

// UW reset not implemented
#define reset()

volatile boolean feedBufferLock = false; //!< Locks feeding the buffer

// Important: VS1053 requires variable SPI transfer speeds:
#define VS1053_CONTROL_SPI_SPEED_BPS  250000  // 250kbps
#define VS1053_DATA_SPI_SETTING_BPS  8000000  // 8Mbps
//#define VS1053_DATA_SPI_SETTING_BPS  2000000

/* VS1053 'low level' interface */
Adafruit_VS1053::Adafruit_VS1053() {
}

uint16_t Adafruit_VS1053::loadPlugin(char *plugname) {

  File plugin = SD.open(plugname);
  if (!plugin) {
    printf("Couldn't open the plugin file %s \n", plugin.name());
    return 0xFFFF;
  }

  if ((plugin.read() != 'P') || (plugin.read() != '&') ||
      (plugin.read() != 'H'))
    return 0xFFFF;

  int type;

  // Serial.print("Patch size: "); printf(patchsize);
  while ((type = plugin.read()) >= 0) {
    uint16_t offsets[] = {0x8000UL, 0x0, 0x4000UL};
    uint16_t addr, len;

    // Serial.print("type: "); printf(type, HEX);

    if (type >= 4) {
      plugin.close();
      return 0xFFFF;
    }

    len = plugin.read();
    len <<= 8;
    len |= plugin.read() & ~1;
    addr = plugin.read();
    addr <<= 8;
    addr |= plugin.read();
    // Serial.print("len: "); Serial.print(len);
    // Serial.print(" addr: $"); printf(addr, HEX);

    if (type == 3) {
      // execute rec!
      plugin.close();
      return addr;
    }

    // set address
    sciWrite(VS1053_REG_WRAMADDR, addr + offsets[type]);
    // write data
    do {
      uint16_t data;
      data = plugin.read();
      data <<= 8;
      data |= plugin.read();
      sciWrite(VS1053_REG_WRAM, data);
    } while ((len -= 2));
  }

  plugin.close();
  return 0xFFFF;
}

boolean Adafruit_VS1053::readyForData(void) 
{ 
  return DREQRead();
}

void Adafruit_VS1053::playData(uint8_t *buffer, uint8_t buffsiz) {
  SPISetDataSpeed();
  DCSAssert();
  spiwrite(buffer, buffsiz);

  DCSDeassert();
  SPIResetSpeed();
}

void Adafruit_VS1053::setVolume(uint8_t left, uint8_t right) {
  // accepts values between 0 and 255 for left and right.
  uint16_t v;
  v = left;
  v <<= 8;
  v |= right;

  noInterrupts(); // cli();
  sciWrite(VS1053_REG_VOLUME, v);
  interrupts(); // sei();
}

uint16_t Adafruit_VS1053::decodeTime() {
  noInterrupts(); // cli();
  uint16_t t = sciRead(VS1053_REG_DECODETIME);
  interrupts(); // sei();
  return t;
}

void Adafruit_VS1053::softReset(void) {
  sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_SDINEW | VS1053_MODE_SM_RESET);
  HAL_Delay(100);
}

uint8_t Adafruit_VS1053::begin(void) {
  Init();

  HAL_Delay(100);
  softReset();
  HAL_Delay(100);

  sciWrite(VS1053_REG_CLOCKF, 0x6000);

  setVolume(40, 40);
  return (sciRead(VS1053_REG_STATUS) >> 4) & 0x0F;
}

void Adafruit_VS1053::dumpRegs(void) {
  printf("Mode = 0x%x\n", sciRead(VS1053_REG_MODE));
  printf("Stat = 0x%x\n", sciRead(VS1053_REG_STATUS));
  printf("ClkF = 0x%x\n", sciRead(VS1053_REG_CLOCKF));
  printf("Vol. = 0x%x\n", sciRead(VS1053_REG_VOLUME));
}

uint16_t Adafruit_VS1053::recordedWordsWaiting(void) {
  return sciRead(VS1053_REG_HDAT1);
}

uint16_t Adafruit_VS1053::recordedReadWord(void) {
  return sciRead(VS1053_REG_HDAT0);
}

boolean Adafruit_VS1053::prepareRecordOgg(char *plugname) {
  sciWrite(VS1053_REG_CLOCKF, 0xC000); // set max clock
  HAL_Delay(1);
  while (!readyForData())
    ;

  sciWrite(VS1053_REG_BASS, 0); // clear Bass

  softReset();
  HAL_Delay(1);
  while (!readyForData())
    ;

  sciWrite(VS1053_SCI_AIADDR, 0);
  // disable all interrupts except SCI
  sciWrite(VS1053_REG_WRAMADDR, VS1053_INT_ENABLE);
  sciWrite(VS1053_REG_WRAM, 0x02);

  int pluginStartAddr = loadPlugin(plugname);
  if (pluginStartAddr == 0xFFFF)
    return false;
  printf("Plugin at $%d\n", pluginStartAddr);
  if (pluginStartAddr != 0x34)
    return false;

  return true;
}

void Adafruit_VS1053::stopRecordOgg(void) { sciWrite(VS1053_SCI_AICTRL3, 1); }

void Adafruit_VS1053::startRecordOgg(boolean mic) {
  /* Set VS1053 mode bits as instructed in the VS1053b Ogg Vorbis Encoder
     manual. Note: for microphone input, leave SMF_LINE1 unset! */
  if (mic) {
    sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_ADPCM | VS1053_MODE_SM_SDINEW);
  } else {
    sciWrite(VS1053_REG_MODE, VS1053_MODE_SM_LINE1 | VS1053_MODE_SM_ADPCM |
                                  VS1053_MODE_SM_SDINEW);
  }
  sciWrite(VS1053_SCI_AICTRL0, 1024);
  /* Rec level: 1024 = 1. If 0, use AGC */
  sciWrite(VS1053_SCI_AICTRL1, 1024);
  /* Maximum AGC level: 1024 = 1. Only used if SCI_AICTRL1 is set to 0. */
  sciWrite(VS1053_SCI_AICTRL2, 0);
  /* Miscellaneous bits that also must be set before recording. */
  sciWrite(VS1053_SCI_AICTRL3, 0);

  sciWrite(VS1053_SCI_AIADDR, 0x34);
  HAL_Delay(1);
  while (!readyForData())
    ;
}

uint16_t Adafruit_VS1053::sciRead(uint8_t addr) {
  uint16_t data;

  SPISetControlSpeed();
  CSAssert();
  spiwrite(VS1053_SCI_READ);
  spiwrite(addr);
  HAL_Delay(1);  // UW: DelayMicroseconds(10)
  data = spiread();
  data <<= 8;
  data |= spiread();
  CSDeassert();
  SPIResetSpeed();

  return data;
}

void Adafruit_VS1053::sciWrite(uint8_t addr, uint16_t data) {
  
  SPISetControlSpeed();
  CSAssert();
  spiwrite(VS1053_SCI_WRITE);
  spiwrite(addr);
  spiwrite(data >> 8);
  spiwrite(data & 0xFF);
  CSDeassert();
  SPIResetSpeed();
}

uint8_t Adafruit_VS1053::spiread(void) {
  uint8_t x = 0;
  SPIRead(&x, sizeof(x));
  return x;
}

void Adafruit_VS1053::spiwrite(uint8_t c) {

  uint8_t x __attribute__((aligned(32))) = c;
  spiwrite(&x, 1);
}

void Adafruit_VS1053::spiwrite(uint8_t *c, uint16_t num) {
  // MSB first, clock low when inactive (CPOL 0), data valid on leading edge
  // (CPHA 0) Make sure clock starts low
  SPIWrite(c, num);
}

void Adafruit_VS1053::sineTest(uint8_t n, uint16_t ms) {
  reset();

  uint16_t mode = sciRead(VS1053_REG_MODE);
  mode |= 0x0020;
  sciWrite(VS1053_REG_MODE, mode);

  while (!DREQRead())
    ;
    //  HAL_Delay(10);

  SPISetDataSpeed();
  DCSAssert();
  spiwrite(0x53);
  spiwrite(0xEF);
  spiwrite(0x6E);
  spiwrite(n);
  spiwrite(0x00);
  spiwrite(0x00);
  spiwrite(0x00);
  spiwrite(0x00);
  DCSDeassert();
  SPIResetSpeed();

  HAL_Delay(ms);

  SPISetDataSpeed();
  DCSAssert();
  spiwrite(0x45);
  spiwrite(0x78);
  spiwrite(0x69);
  spiwrite(0x74);
  spiwrite(0x00);
  spiwrite(0x00);
  spiwrite(0x00);
  spiwrite(0x00);
  DCSDeassert();
  SPIResetSpeed();
}

#define MP3_VS1053_MCS_GPIO               GPIOA
#define MP3_VS1053_MCS_GPIO_Pin           GPIO_PIN_4

#define MP3_VS1053_DCS_GPIO               GPIOB
#define MP3_VS1053_DCS_GPIO_Pin           GPIO_PIN_1

#define MP3_VS1053_DREQ_GPIO              GPIOB
#define MP3_VS1053_DREQ_GPIO_Pin          GPIO_PIN_0

void Adafruit_VS1053::Init()
{
  GPIO_InitTypeDef GPIO_Init;

  /* Enable the GPIOA and GPIOB Clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* configure the CS pin */
  GPIO_Init.Pin   = MP3_VS1053_MCS_GPIO_Pin;
  GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull  = GPIO_PULLUP;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MP3_VS1053_MCS_GPIO, &GPIO_Init);

  CSDeassert();

  /* configure the DCS (Data Chip Select) pin */
  GPIO_Init.Pin   = MP3_VS1053_DCS_GPIO_Pin;
  GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull  = GPIO_PULLUP;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MP3_VS1053_DCS_GPIO, &GPIO_Init);

  /* configure the DREQ pin */
  GPIO_Init.Pin   = MP3_VS1053_DREQ_GPIO_Pin;
  GPIO_Init.Mode  = GPIO_MODE_INPUT;
  GPIO_Init.Pull  = GPIO_PULLDOWN;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MP3_VS1053_DREQ_GPIO, &GPIO_Init);

  DCSDeassert();

  BSP_SPI1_Init();
}

uint16_t Adafruit_VS1053::DREQRead()
{
  return (uint16_t)HAL_GPIO_ReadPin(MP3_VS1053_DREQ_GPIO, MP3_VS1053_DREQ_GPIO_Pin);
}

void Adafruit_VS1053::CSAssert()
{
  HAL_GPIO_WritePin(MP3_VS1053_MCS_GPIO, MP3_VS1053_MCS_GPIO_Pin, GPIO_PIN_RESET);
}

void Adafruit_VS1053::CSDeassert()
{
  HAL_GPIO_WritePin(MP3_VS1053_MCS_GPIO, MP3_VS1053_MCS_GPIO_Pin, GPIO_PIN_SET);
}

void Adafruit_VS1053::DCSAssert()
{
  HAL_GPIO_WritePin(MP3_VS1053_DCS_GPIO, MP3_VS1053_DCS_GPIO_Pin, GPIO_PIN_RESET);
}

void Adafruit_VS1053::DCSDeassert()
{
  HAL_GPIO_WritePin(MP3_VS1053_DCS_GPIO, MP3_VS1053_DCS_GPIO_Pin, GPIO_PIN_SET);
}

int32_t Adafruit_VS1053::SPIWrite(uint8_t *data, uint16_t size)
{
  return BSP_SPI1_Send(data, size);
}

int32_t Adafruit_VS1053::SPIRead(uint8_t *data, uint16_t size)
{
  return BSP_SPI1_Recv(data, size);
}

int32_t Adafruit_VS1053::SPISetControlSpeed()
{
  return BSP_SPI1_SetSpeed(VS1053_CONTROL_SPI_SPEED_BPS);
}

int32_t Adafruit_VS1053::SPISetDataSpeed()
{
  return BSP_SPI1_SetSpeed(VS1053_DATA_SPI_SETTING_BPS);
}

int32_t Adafruit_VS1053::SPIResetSpeed()
{
  return BSP_SPI1_SetSpeed(BUS_SPI1_BAUDRATE);
}
