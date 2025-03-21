/*!
 * @file Adafruit_VS1053.h
 */

#ifndef ADAFRUIT_VS1053_H
#define ADAFRUIT_VS1053_H

#include <SD.h>
#include <stm32l475e_iot01_bus.h>

#define VS1053_SCI_READ 0x03  //!< Serial read address
#define VS1053_SCI_WRITE 0x02 //!< Serial write address

#define VS1053_REG_MODE 0x00       //!< Mode control
#define VS1053_REG_STATUS 0x01     //!< Status of VS1053b
#define VS1053_REG_BASS 0x02       //!< Built-in bass/treble control
#define VS1053_REG_CLOCKF 0x03     //!< Clock frequency + multiplier
#define VS1053_REG_DECODETIME 0x04 //!< Decode time in seconds
#define VS1053_REG_AUDATA 0x05     //!< Misc. audio data
#define VS1053_REG_WRAM 0x06       //!< RAM write/read
#define VS1053_REG_WRAMADDR 0x07   //!< Base address for RAM write/read
#define VS1053_REG_HDAT0 0x08      //!< Stream header data 0
#define VS1053_REG_HDAT1 0x09      //!< Stream header data 1
#define VS1053_REG_VOLUME 0x0B     //!< Volume control

#define VS1053_GPIO_DDR 0xC017   //!< Direction
#define VS1053_GPIO_IDATA 0xC018 //!< Values read from pins
#define VS1053_GPIO_ODATA 0xC019 //!< Values set to the pins

#define VS1053_INT_ENABLE 0xC01A //!< Interrupt enable

#define VS1053_MODE_SM_DIFF                                                    \
  0x0001 //!< Differential, 0: normal in-phase audio, 1: left channel inverted
#define VS1053_MODE_SM_LAYER12 0x0002  //!< Allow MPEG layers I & II
#define VS1053_MODE_SM_RESET 0x0004    //!< Soft reset
#define VS1053_MODE_SM_CANCEL 0x0008   //!< Cancel decoding current file
#define VS1053_MODE_SM_EARSPKLO 0x0010 //!< EarSpeaker low setting
#define VS1053_MODE_SM_TESTS 0x0020    //!< Allow SDI tests
#define VS1053_MODE_SM_STREAM 0x0040   //!< Stream mode
#define VS1053_MODE_SM_SDINEW 0x0800   //!< VS1002 native SPI modes
#define VS1053_MODE_SM_ADPCM 0x1000    //!< PCM/ADPCM recording active
#define VS1053_MODE_SM_LINE1 0x4000 //!< MIC/LINE1 selector, 0: MICP, 1: LINE1
#define VS1053_MODE_SM_CLKRANGE                                                \
  0x8000 //!< Input clock range, 0: 12..13 MHz, 1: 24..26 MHz

#define VS1053_SCI_AIADDR                                                      \
  0x0A //!< Indicates the start address of the application code written earlier
       //!< with SCI_WRAMADDR and SCI_WRAM registers.
#define VS1053_SCI_AICTRL0                                                     \
  0x0C //!< SCI_AICTRL register 0. Used to access the user's application program
#define VS1053_SCI_AICTRL1                                                     \
  0x0D //!< SCI_AICTRL register 1. Used to access the user's application program
#define VS1053_SCI_AICTRL2                                                     \
  0x0E //!< SCI_AICTRL register 2. Used to access the user's application program
#define VS1053_SCI_AICTRL3                                                     \
  0x0F //!< SCI_AICTRL register 3. Used to access the user's application program

#define VS1053_DATABUFFERLEN 32 //!< Length of the data buffer

/*!
 * Driver for the Adafruit VS1053
 */
class Adafruit_VS1053 {
public:
  Adafruit_VS1053();
  /*!
   * @brief Initialize communication and (hard) reset the chip.
   * @return Returns true if a VS1053 is found
   */
  uint8_t begin(void);
  /*!
   * @brief Attempts a soft reset of the chip
   */
  void softReset(void);
  /*!
   * @brief Reads from the specified register on the chip
   * @param addr Register address to read from
   * @return Retuns the 16-bit data corresponding to the received address
   */
  uint16_t sciRead(uint8_t addr);
  /*!
   * @brief Writes to the specified register on the chip
   * @param addr Register address to write to
   * @param data Data to write
   */
  void sciWrite(uint8_t addr, uint16_t data);
  /*!
   * @brief Generate a sine-wave test signal
   * @param n Defines the sine test to use
   * @param ms Delay (in ms)
   */
  void sineTest(uint8_t n, uint16_t ms);
  /*!
   * @brief Low-level SPI write operation
   * @param d What to write
   */
  void spiwrite(uint8_t d);
  /*!
   * @brief Low-level SPI write operation
   * @param c Pointer to a buffer containing the data to send
   * @param num How many elements in the buffer should be sent
   */
  void spiwrite(uint8_t *c, uint16_t num);
  /*!
   * @brief Low-level SPI read operation
   * @return Returns a byte read from SPI
   */
  uint8_t spiread(void);

  /*!
   * @brief Reads the DECODETIME register from the chip
   * @return Returns the decode time as an unsigned 16-bit integer
   */
  uint16_t decodeTime(void);
  /*!
   * @brief Set the output volume for the chip
   * @param left Desired left channel volume
   * @param right Desired right channel volume
   */
  void setVolume(uint8_t left, uint8_t right);
  /*!
   * @brief Prints the contents of the MODE, STATUS, CLOCKF and VOLUME registers
   */
  void dumpRegs(void);

  /*!
   * @brief Decode and play the contents of the supplied buffer
   * @param buffer Buffer to decode and play
   * @param buffsiz Size to decode and play
   */
  void playData(uint8_t *buffer, uint8_t buffsiz);
  /*!
   * @brief Test if ready for more data
   * @return Returns true if it is ready for data
   */
  boolean readyForData(void);
  
  /*!
   * @brief Load the specified plug-in
   * @details See http://www.vlsi.fi/en/support/software/vs10xxplugins.html for plugin examples.
   * @param fn Plug-in to load
   * @return Either returns 0xFFFF if there is an error, or the address of the
   * plugin that was loaded
   */
  uint16_t loadPlugin(char *fn);

  /*!
   * @brief Initialize chip for OGG recording
   * @param plugin Binary file of the plugin to use
   * @return Returns true if the device is ready to record
   */
  boolean prepareRecordOgg(char *plugin);
  /*!
   * @brief Start recording
   * @param mic mic=true for microphone input
   */
  void startRecordOgg(boolean mic);
  /*!
   * @brief Stop the recording
   */
  void stopRecordOgg(void);
  /*!
   * @brief Returns the number of words recorded
   * @return 2-byte unsigned int with the number of words
   */
  uint16_t recordedWordsWaiting(void);
  /*!
   * @brief Reads the next word from the buffer of recorded words
   * @return Returns the 16-bit data corresponding to the received address
   */
  uint16_t recordedReadWord(void);

  uint8_t mp3buffer[VS1053_DATABUFFERLEN]; //!< mp3 buffer that gets sent to the
                                           //!< device

private:
  // STM32 HAL dependency
  void Init();
  void CSAssert();
  void CSDeassert();
  void DCSAssert();
  void DCSDeassert();
  uint16_t DREQRead();

  int32_t SPISetControlSpeed();
  int32_t SPISetDataSpeed();
  int32_t SPIResetSpeed();
  int32_t SPIRead(uint8_t *, uint16_t);
  int32_t SPIWrite(uint8_t *, uint16_t);

};

#endif // ADAFRUIT_VS1053_H
