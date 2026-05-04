
/*
 * Flash memory page start address
 */
//uint32_t FlashAddress = 0x0803F800;
//uint16_t FlashSize = 2000;
#define FlashAddress 0x0803F800
//#define FlashAddress 0x0801D000
#define FlashSize 2000


extern UART_HandleTypeDef huart2;



/*
 * function declarations
 */
void EEPROM_Write(uint16_t address, char data);

void EEPROM_Write_uint32(uint16_t address, int32_t  data);

void EEPROM_Write_String(uint16_t address,  char* data);

void EEPROM_Read_String(uint16_t address, char* buffer, uint16_t length);

int8_t  EEPROM_Read(uint16_t address);

int32_t  EEPROM_Read_uint32(uint16_t address);

uint32_t GetPage(uint32_t Addr);

void Flash_Read_Data(uint32_t StartPageAddress, char *RxBuf,uint16_t numberofwords);

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint64_t *Data,uint16_t numberofwords);

uint32_t Flash_Erash(uint32_t StartPageAddress);

void EEPROM_Write_Array(uint16_t address, uint8_t *data, size_t length);

void EEPROM_Read_Array(uint16_t address, uint8_t *buffer, size_t length);
