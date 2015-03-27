// Myserial.cpp : 実装ファイル
//
#include "Serial.h"

// CSerial
CSerial::CSerial()
{
}

CSerial::~CSerial()
{
}

// 初期化関数
bool CSerial::InitSerial(char *comport, int baudrate)
{
	//RS232C制御用 RS232C の初期化
	hCom = CreateFile(comport, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hCom == INVALID_HANDLE_VALUE) {
		return false;
	}

	dcb.DCBlength = sizeof(DCB) ;

	GetCommState(hCom, &dcb) ;

	//シリアル通信基本設定
	dcb.BaudRate = baudrate;	// 通信速度
	dcb.ByteSize = 8;			// データ長
	dcb.Parity = NOPARITY;		// パリティビット：EVENPARITY,MARKPARITY,NOPARITY,ODDPARITY
	dcb.StopBits = ONESTOPBIT;  // ストップビット：ONESTOPBIT,ONE5STOPBITS,TWOSTOPBITS
	dcb.fOutxCtsFlow = FALSE;	// 送信時に、CTS を監視するかどうか
	dcb.fOutxDsrFlow = FALSE;	// 送信時に、DSR を監視するかどうか
	dcb.fDsrSensitivity = FALSE;// DSR がOFFの間は受信データを無視するか

	bool flag;
	////追加設定
	////ボーレートの設定

	if(SetCommState(hCom, &dcb)){
		flag = true;
	}else{
		flag = false;
	}

	if(flag==true)
	{
		GetCommTimeouts(hCom,&ctmo);
		ctmo.ReadIntervalTimeout = 0;		    // 受信データ間のタイムアウト時間[msec]
		ctmo.ReadTotalTimeoutMultiplier = 10;	// 受信タイムアウトトータル時間　＝（ ReadTotalTimeoutMultiplier ＊　受信予定バイト数　）
		ctmo.ReadTotalTimeoutConstant = 50;		// 　＋　ReadTotalTimeoutConstant　[mSec]
		ctmo.WriteTotalTimeoutMultiplier = 10;	// 受信タイムアウトトータル時間　＝（ WriteTotalTimeoutMultiplier ＊　受信予定バイト数　）
		ctmo.WriteTotalTimeoutConstant = 50;	// 　＋　WriteTotalTimeoutConstant　[mSec]
		if(!SetCommTimeouts(hCom,&ctmo)) return false;
	}

	////通信バッファをすべてクリア
	PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	return flag;	
}

// 終了関数
bool CSerial::CloseSerial(void)
{
	if(CloseHandle(hCom)){
		return true;
	}else{
		return false;
	}
}

// １バイト送信
bool CSerial::Write(BYTE data)
{
	unsigned long byte;
	
	// 1バイト送信
	if(WriteFile(hCom, &data, sizeof(BYTE), &byte, NULL)){
		return true;
	}else{
		return false;
	}
}

// 文字列送信
bool CSerial::Write2(void* data, int size)
{
	unsigned long byte;
	
	//文字列送信
	if(WriteFile(hCom, data, size, &byte, NULL)){
		return true;
	}else{
		return false;
	}
}

// 一文字受信
BYTE CSerial::Read(void)
{
	unsigned long byte;
	BYTE data = 0x00;

	//１バイト受信
	ReadFile(hCom, &data, 1, &byte, NULL);

	return data;
}

// 文字列受信
DWORD CSerial::Read2(void* data, int size)
{
	DWORD byte;

	//文字列受信
	ReadFile(hCom, data, size, &byte, NULL);

	return byte;

}

// 受信データのバイト数を調べて値を返す
DWORD CSerial::CheckSerialData(void)
{

	DWORD	dwErrors;  // エラー情報
	COMSTAT	ComStat; // デバイスの状態
	DWORD	dwCount;   // 受信データのバイト数

	ClearCommError(hCom, &dwErrors, &ComStat);
	dwCount = ComStat.cbInQue;

    return dwCount;
}

void CSerial::ClearRXbuffer(void)
{
	//受信バッファをすべてクリア
	PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR);
}