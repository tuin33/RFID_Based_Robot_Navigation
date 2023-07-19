#include "ltkcpp.h"
#include "impinj_ltkcpp.h"
//#include "TagData.h"
#include <stdio.h>
#include <vector>
#include "MyData.h"
#include <ros/ros.h>
#include "RFIDPubFormat.h"
#include <RFID_Based_Robot_Navigation/RFIDdata.h>
#include <RFID_Based_Robot_Navigation/sendEnding.h>


using namespace LLRP;

/*
** Sorry, we use this linux safe method
** to print buffers.  WIndows has the same
** method, but by a different name
*/
#if (WIN32)
#define snprintf _snprintf
#endif

void getKeyboard(int *flag);

class CMyApplication
{
private:
	unsigned int m_messageID;
	ros::NodeHandle n;
    ros::Publisher pub1;
	ros::Publisher pub2;

	
public:

	/** Verbose level, incremented by each -v on command line */
	int                         m_Verbose;
	//TagData                     leftdataArray[1500];
	//int                         leftdataIndex = 0;
	//TagData                     rightdataArray[1500];
	//int                         rightdataIndex = 0;
	std::vector<MyData>  antDataVec1;
	std::vector<MyData>  antDataVec2;
    std::vector<MyData>  antDataVec3;
	std::vector<MyData>  antDataVec4;
	std::vector<double>  timeVec1;
	std::vector<double>  timeVec2;
	std::vector<double>  timeVec3;
	std::vector<double>  timeVec4;

	unsigned int m_channelIndex; // frequency point (1-16), default to 1


	/** Connection to the LLRP reader */
	CConnection *               m_pConnectionToReader;


	inline
		CMyApplication(void)
		: m_Verbose(0), m_pConnectionToReader(NULL)
	{
		m_messageID = 0;
		m_channelIndex = 1;
		pub1 = n.advertise<RFID_Based_Robot_Navigation::RFIDdata>("/Data_RFID",10);
		pub2 = n.advertise<RFID_Based_Robot_Navigation::sendEnding>("/Ending_msg",10);
	}


	int
		run(
		char 					 *pReaderHostName);

	int
		checkConnectionStatus(void);

	int
		enableImpinjExtensions(void);

	int
		resetConfigurationToFactoryDefaults(void);

	int
		getReaderCapabilities(void);

	int
		setImpinjReaderConfig();

	int
		addROSpec(void);

	int
		enableROSpec(void);

	int
		startROSpec(void);

	int
		stopROSpec(void);

	int
		awaitAndPrintReport(int timeoutSec);

	int numi;

	void
		printTagReportData(
		CRO_ACCESS_REPORT *       pRO_ACCESS_REPORT);

	void
		printOneTagReportData(
		CTagReportData *          pTagReportData);

	void
		formatOneEPC(
		CParameter *          pEpcParameter,
		char *                buf,
		int                   buflen);

	int getOnePhaseAngle(
		CImpinjRFPhaseAngle  *pRfPhase,
		double               *out);

	int
		getOnePeakRSSI(
		CImpinjPeakRSSI      *pPeakRSSI,
		double               *out);


	int
		getOneTimestamp(
		CParameter           *pTimestamp,
		unsigned long long   *out);

	int
		getOneAntenna(
		CAntennaID           *pAntenna,
		unsigned short       *out);

	int
		getOneChannelIndex(
		CChannelIndex        *pChannelIndex,
		unsigned short       *out);

	int
		estimateVelocity(
		char *                epcStr,
		double                rssi,
		double                phase,
		unsigned short        channelIndex,
		unsigned short        antenna,
		unsigned long long    time,
		double                *outVelocity);

	void
		handleReaderEventNotification(
		CReaderEventNotificationData *pNtfData);

	void
		handleAntennaEvent(
		CAntennaEvent *           pAntennaEvent);

	void
		handleReaderExceptionEvent(
		CReaderExceptionEvent *   pReaderExceptionEvent);

	int
		checkLLRPStatus(
		CLLRPStatus *             pLLRPStatus,
		char *                    pWhatStr);

	CMessage *
		transact(
		CMessage *                pSendMsg);

	CMessage *
		recvMessage(
		int                       nMaxMS);

	int
		sendMessage(
		CMessage *                pSendMsg);

	void
		printXMLMessage(
		CMessage *                pMessage);
};