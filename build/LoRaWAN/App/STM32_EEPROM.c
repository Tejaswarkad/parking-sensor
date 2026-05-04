#include "main.h"
#include "STM32_EEPROM.h"
#include "string.h"

/*
 * Write data to flash memory
 */

void EEPROM_Write(uint16_t address, char data) {
	char savedData[FlashSize];
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	savedData[address] = data;
	Flash_Write_Data(FlashAddress, (uint64_t*) savedData, numberofwords);
}
void EEPROM_Write_String(uint16_t address, char *data) {
	char savedData[FlashSize];
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	strcpy(savedData + address, data);
	Flash_Write_Data(FlashAddress, (uint64_t*) savedData, numberofwords);
}

void EEPROM_Read_String(uint16_t address, char *buffer, uint16_t length) {
	char savedData[FlashSize];
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	strncpy(buffer, savedData + address, length);
	buffer[length] = '\0'; // Null-terminate the string
}

void EEPROM_Write_uint32(uint16_t address, int32_t data) {
	char savedData[FlashSize];
	uint8_t byteArray[4];
	memcpy(byteArray, &data, sizeof(data));
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	memcpy(savedData + address, &byteArray, sizeof(byteArray));
	Flash_Write_Data(FlashAddress, (uint64_t*) savedData, numberofwords);
}

int8_t EEPROM_Read(uint16_t address) {
	char savedData[FlashSize];
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	return savedData[address];
}

// Function to write an array (e.g., join_eui or app_key) into EEPROM
void EEPROM_Write_Array(uint16_t address, uint8_t *data, size_t length) {
    char savedData[FlashSize];
    uint16_t numberofwords = FlashSize / 8;
    Flash_Read_Data(FlashAddress, savedData, numberofwords);
    memcpy(savedData + address, data, length);
    Flash_Write_Data(FlashAddress, (uint64_t*)savedData, numberofwords);
}

// Function to read an array from EEPROM
void EEPROM_Read_Array(uint16_t address, uint8_t *buffer, size_t length) {
    char savedData[FlashSize];
    uint16_t numberofwords = FlashSize / 8;
    Flash_Read_Data(FlashAddress, savedData, numberofwords);
    memcpy(buffer, savedData + address, length);
}

int32_t EEPROM_Read_uint32(uint16_t address) {
	char savedData[FlashSize];
	int32_t retrievedValue;
	uint16_t numberofwords = FlashSize / 8;
	Flash_Read_Data(FlashAddress, savedData, numberofwords);
	memcpy(&retrievedValue, savedData + address, sizeof(retrievedValue));
	return retrievedValue;
}

// Write data to flash
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint64_t *Data,
		uint16_t numberofwords) {

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	/* Erase the user Flash area*/

	uint32_t StartPage = GetPage(StartPageAddress);
	uint32_t EndPageAdress = StartPageAddress + (numberofwords * 8);
	uint32_t EndPage = GetPage(EndPageAdress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = StartPage;
	EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError();
	}

	/* Program the user Flash area word by word*/

	while (sofar < numberofwords) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress,
				(uint64_t) Data[sofar]) == HAL_OK) {
			StartPageAddress += 8; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

// reads data from flash
void Flash_Read_Data(uint32_t StartPageAddress, char *RxBuf,
		uint16_t numberofwords) {
	numberofwords = numberofwords * 2;
	while (numberofwords > 0) {
		uint32_t data = *(__IO uint32_t*) StartPageAddress;
		memcpy(RxBuf, &data, sizeof(uint32_t));
		StartPageAddress += 4;
		RxBuf += sizeof(uint32_t);
		numberofwords--;
	}
}

uint32_t Flash_Erash(uint32_t StartPageAddress) {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	/* Erase the user Flash area*/

	uint32_t StartPage = GetPage(StartPageAddress);
	uint32_t EndPageAdress = StartPageAddress;
	uint32_t EndPage = GetPage(EndPageAdress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = StartPage;
	EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError();
	}
	return HAL_OK;
}
uint32_t GetPage(uint32_t Addr) {
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}
