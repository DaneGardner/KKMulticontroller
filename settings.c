#include "settings.h"

#include <avr/eeprom.h>
#include "gyros.h"
#include "receiver.h"

struct config Config;

void eeprom_write_byte_changed(uint8_t *addr, uint8_t value)
{
  if(eeprom_read_byte(addr) != value)
    eeprom_write_byte(addr, value);
}

void eeprom_write_block_changes(const uint8_t *src, void *dest, size_t size)
{
  size_t len;

  for(len = 0;len < size;len++) {
    eeprom_write_byte_changed(dest, *src);
    src++;
    dest++;
  }
}

void Save_Config_to_EEPROM()
{
  eeprom_write_block_changes(
    (const void *)&Config, (void *)EEPROM_DATA_START_POS,
    sizeof(struct config));
}

void Set_EEPROM_Default_Config()
{
  Config.RollGyroDirection  = GYRO_REVERSED;
  Config.PitchGyroDirection  = GYRO_REVERSED;
  Config.YawGyroDirection    = GYRO_NORMAL;
  Config.CenterRollValue     = DEFAULT_CENTER;
  Config.CenterPitchValue    = DEFAULT_CENTER;
  Config.CenterCollValue     = DEFAULT_CENTER;
  Config.CenterYawValue      = DEFAULT_CENTER;
}

void Initial_EEPROM_Config_Load()
{
  // load up last settings from EEPROM
  if(eeprom_read_byte((uint8_t *)EEPROM_DATA_START_POS) != CONFIG_VERSION) {
    Config.setup = CONFIG_VERSION;
    Set_EEPROM_Default_Config();
    // write to eeProm
    Save_Config_to_EEPROM();
  } else {
    // read eeprom
    eeprom_read_block(&Config, (void *)EEPROM_DATA_START_POS, sizeof(struct config));
  }
}

void settingsSetup()
{
  Initial_EEPROM_Config_Load();
}

void settingsClearAll()
{
  for(uint8_t i = 0;i < 5;i++) {
    LED = 1;
    _delay_ms(25);
    LED = 0;
    _delay_ms(25);
  }

  Set_EEPROM_Default_Config();
  while(1)
          ;
}
