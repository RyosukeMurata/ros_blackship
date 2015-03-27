// Myserial.cpp : �����t�@�C��
//
#include "Serial.h"

// CSerial
CSerial::CSerial()
{
}

CSerial::~CSerial()
{
}

// �������֐�
bool CSerial::InitSerial(char *comport, int baudrate)
{
	//RS232C����p RS232C �̏�����
	hCom = CreateFile(comport, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hCom == INVALID_HANDLE_VALUE) {
		return false;
	}

	dcb.DCBlength = sizeof(DCB) ;

	GetCommState(hCom, &dcb) ;

	//�V���A���ʐM��{�ݒ�
	dcb.BaudRate = baudrate;	// �ʐM���x
	dcb.ByteSize = 8;			// �f�[�^��
	dcb.Parity = NOPARITY;		// �p���e�B�r�b�g�FEVENPARITY,MARKPARITY,NOPARITY,ODDPARITY
	dcb.StopBits = ONESTOPBIT;  // �X�g�b�v�r�b�g�FONESTOPBIT,ONE5STOPBITS,TWOSTOPBITS
	dcb.fOutxCtsFlow = FALSE;	// ���M���ɁACTS ���Ď����邩�ǂ���
	dcb.fOutxDsrFlow = FALSE;	// ���M���ɁADSR ���Ď����邩�ǂ���
	dcb.fDsrSensitivity = FALSE;// DSR ��OFF�̊Ԃ͎�M�f�[�^�𖳎����邩

	bool flag;
	////�ǉ��ݒ�
	////�{�[���[�g�̐ݒ�

	if(SetCommState(hCom, &dcb)){
		flag = true;
	}else{
		flag = false;
	}

	if(flag==true)
	{
		GetCommTimeouts(hCom,&ctmo);
		ctmo.ReadIntervalTimeout = 0;		    // ��M�f�[�^�Ԃ̃^�C���A�E�g����[msec]
		ctmo.ReadTotalTimeoutMultiplier = 10;	// ��M�^�C���A�E�g�g�[�^�����ԁ@���i ReadTotalTimeoutMultiplier ���@��M�\��o�C�g���@�j
		ctmo.ReadTotalTimeoutConstant = 50;		// �@�{�@ReadTotalTimeoutConstant�@[mSec]
		ctmo.WriteTotalTimeoutMultiplier = 10;	// ��M�^�C���A�E�g�g�[�^�����ԁ@���i WriteTotalTimeoutMultiplier ���@��M�\��o�C�g���@�j
		ctmo.WriteTotalTimeoutConstant = 50;	// �@�{�@WriteTotalTimeoutConstant�@[mSec]
		if(!SetCommTimeouts(hCom,&ctmo)) return false;
	}

	////�ʐM�o�b�t�@�����ׂăN���A
	PurgeComm(hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	return flag;	
}

// �I���֐�
bool CSerial::CloseSerial(void)
{
	if(CloseHandle(hCom)){
		return true;
	}else{
		return false;
	}
}

// �P�o�C�g���M
bool CSerial::Write(BYTE data)
{
	unsigned long byte;
	
	// 1�o�C�g���M
	if(WriteFile(hCom, &data, sizeof(BYTE), &byte, NULL)){
		return true;
	}else{
		return false;
	}
}

// �����񑗐M
bool CSerial::Write2(void* data, int size)
{
	unsigned long byte;
	
	//�����񑗐M
	if(WriteFile(hCom, data, size, &byte, NULL)){
		return true;
	}else{
		return false;
	}
}

// �ꕶ����M
BYTE CSerial::Read(void)
{
	unsigned long byte;
	BYTE data = 0x00;

	//�P�o�C�g��M
	ReadFile(hCom, &data, 1, &byte, NULL);

	return data;
}

// �������M
DWORD CSerial::Read2(void* data, int size)
{
	DWORD byte;

	//�������M
	ReadFile(hCom, data, size, &byte, NULL);

	return byte;

}

// ��M�f�[�^�̃o�C�g���𒲂ׂĒl��Ԃ�
DWORD CSerial::CheckSerialData(void)
{

	DWORD	dwErrors;  // �G���[���
	COMSTAT	ComStat; // �f�o�C�X�̏��
	DWORD	dwCount;   // ��M�f�[�^�̃o�C�g��

	ClearCommError(hCom, &dwErrors, &ComStat);
	dwCount = ComStat.cbInQue;

    return dwCount;
}

void CSerial::ClearRXbuffer(void)
{
	//��M�o�b�t�@�����ׂăN���A
	PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR);
}