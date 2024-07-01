/******************************************************************************************************************************************
  EEPROM_CRC.ino
  For STM32F1 using Flash emulated-EEPROM

  The FlashStorage_STM32F1 library aims to provide a convenient way to store and retrieve user's data using the non-volatile flash memory
  of STM32F1. It now supports writing and reading the whole object, not just byte-and-byte.

  Based on and modified from Cristian Maglie's FlashStorage (https://github.com/cmaglie/FlashStorage)

  Built by Khoi Hoang https://github.com/khoih-prog/FlashStorage_STM32F1
  Licensed under LGPLv3 license

  Orginally written by A. Christian

  Copyright (c) 2015-2016 Arduino LLC.  All right reserved.
  Copyright (c) 2021 Khoi Hoang.

  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
  as published bythe Free Software Foundation, either version 3 of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public License along with this library.
  If not, see (https://www.gnu.org/licenses/)
 ******************************************************************************************************************************************/
/***
  Written by Christopher Andrews.
  CRC algorithm generated by pycrc, MIT licence ( https://github.com/tpircher/pycrc )

  A CRC is a simple way of checking whether data has changed or become corrupted.
  This example calculates a CRC value directly on the EEPROM values.
  The purpose of this example is to highlight how the EEPROM object can be used just like an array.
***/

// Use 0-2. Larger for more debugging messages
#define FLASH_DEBUG       2

// You can select another sector. Be careful not larger than (REGISTERED_NUMBER_FLASH_SECTORS - 1) and large enough not to overwrite your program
// Default is (REGISTERED_NUMBER_FLASH_SECTORS - 1) if you don't specify here
#define USING_FLASH_SECTOR_NUMBER           (REGISTERED_NUMBER_FLASH_SECTORS - 2)

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <FlashStorage_STM32F1.h>

unsigned long eeprom_crc() 
{
  const unsigned long crc_table[16] =
  {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < EEPROM.length()  ; ++index)
  {
    crc = crc_table[(crc ^ EEPROM.read(index)) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM.read(index) >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }

  return crc;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print(F("\nStart EEPROM_CRC on ")); Serial.println(BOARD_NAME);
  Serial.println(FLASH_STORAGE_STM32F1_VERSION);

  //Print length of data to run CRC on.
  Serial.print("EEPROM length: ");
  Serial.println(EEPROM.length());

  EEPROM.init();

  //Print the result of calling eeprom_crc()
  Serial.print("CRC32 of EEPROM data: 0x");
  Serial.println(eeprom_crc(), HEX);
  Serial.print("Done!");
}

void loop() 
{
  /* Empty loop */
}