//#ifndef _MY_DATA_H_
//#define _MY_DATA_H_

class MyData {
public:
	int                     antennaId;
	char		        	epc[128];			     //��ǩEPC
	double		        	phase;				     //��λ
	double                  rssi;                                // �ź�ǿ��
	unsigned long long		timestamp;		             //firstSeenTimestampUTC
	double      			timestamp_pc;			     //��¼����ȡ����ǩ��Ϣ��ʱ���
	double					robotX;			             //����������-X cm
	double					robotY;			             //����������-Y cm
	double		        	robotTh;			     //�����˳���ǣ����ȣ�-Th
	int                 	channelIndex;                        // frequecy point
};
//#endif
