/*
 * SD_lib.h
 *
 *  Created on: Jan 29, 2024
 *      Author: Felipe
 */

#include "fatfs.h"
#include "main.h"

#ifndef INC_SD_LIB_H_
#define INC_SD_LIB_H_

char* Mount_SD();
char* CreateFile_SD(char* name);
char* WriteToFile_SD(char* filename, char* content);
char* ClearFile_SD(char* filename);
char* CloseFile_SD();


extern FATFS FAT;
extern FIL file;
extern FILINFO fili;
extern UINT testByte;
FRESULT out;

char* Mount_SD(){
	if(f_mount(&FAT, SDPath, 1) == FR_OK){
		return "SD MOUNTED/UNMOUNTED SUCCESSFULY\n";
	}
	return "SD NOT MOUNTED\n";
}

char* CreateFile_SD(char* name){
	if(f_open(&file, name, FA_WRITE | FA_CREATE_ALWAYS ) == FR_OK){
		f_close(&file);
		return "FILE CREATED SUCCESSFULY\n";
	}
	return "ERROR ON FILE CREATION\n";
}


char* WriteToFile_SD(char* filename, char* content){
	if(f_open(&file, filename, FA_OPEN_APPEND | FA_READ | FA_WRITE  ) == FR_OK){
		f_lseek(&file, file.fptr);
		f_puts(content, &file);
//		f_write(&file, content, size, &testByte);
		f_close(&file);
		return "CONTENT WAS WRITTEN IN FILE";
	}
	return "NO CONTENT WRITTEN";
}

//char* ReadFileLine(){
//
//}

char* ClearFile_SD(char* filename){
	if(f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE ) == FR_OK){
		f_write(&file, " ", 1, &testByte);
		f_close(&file);
		return "CONTENT OF FILE WAS DELETED";
	}
	return "NO CONTENT WAS DELETED";
}

char* CloseFile_SD(){
	f_close(&file);
	return "FILE CLOSED";
}

#endif /* INC_SD_LIB_H_ */
