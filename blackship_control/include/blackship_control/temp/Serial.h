#pragma once

#include <windows.h>

// CSerial
class CSerial{

public:
	CSerial();
	~CSerial();
	bool InitSerial(char *comport, int baudrate);
	bool CloseSerial(void);
	bool Write(BYTE data);
	bool Write2(void* data, int size);
	BYTE Read(void);
	DWORD Read2(void* data, int size);
	DWORD CheckSerialData(void);
	void ClearRXbuffer(void);

protected:
	HANDLE hCom;
	DCB dcb;
	BOOL fRetVal ;
	BYTE bSet ;
	COMMTIMEOUTS ctmo;

};


