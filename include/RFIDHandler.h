#include "ltkcpp.h"
#include "impinj_ltkcpp.h"
//#include "TagData.h"
#include <stdio.h>
#include <vector>
#include "MyData.h"
#include <ros/ros.h>
#include "RFIDPubFormat.h"
#include <csignal>
#include <cctype>
#include <termios.h>


using namespace LLRP;

/*
** Sorry, we use this linux safe method
** to print buffers.  WIndows has the same
** method, but by a different name
*/
#if (WIN32)
#define snprintf _snprintf
#endif

typedef struct _tagReportData{
    char		        	    epc[128];	
    std::vector<int>      antennaId;
    std::vector<double>		phase;	
    std::vector<double>   rssi;     
    std::vector<double>	  timestamp_ros;
    std::vector<double>		timestamp_pc;
    std::vector<double>   channelIndex;
}RFIDData;

// void getKeyboard(int *flag);

class CMyApplication
{
private:
	unsigned int m_messageID;
	// ros::NodeHandle n;
    // ros::Publisher pub1;
	// ros::Publisher pub2;

	// Reader ip
	char *pReaderHostName;

	

	int kfd = 0;
	

	struct termios cooked, raw;

	
	
public:

	bool isRunning = true;

	/** Verbose level, incremented by each -v on command line */
	int                         m_Verbose;
	//TagData                     leftdataArray[1500];
	//int                         leftdataIndex = 0;
	//TagData                     rightdataArray[1500];
	//int                         rightdataIndex = 0;
	// std::vector<MyData>  antDataVec1;
	// std::vector<MyData>  antDataVec2;
    // std::vector<MyData>  antDataVec3;
	// std::vector<MyData>  antDataVec4;
	// std::vector<double>  timeVec1;
	// std::vector<double>  timeVec2;
	// std::vector<double>  timeVec3;
	// std::vector<double>  timeVec4;

	std::vector<RFIDData>  dataVec;

	unsigned int m_channelIndex; // frequency point (1-16), default to 1

	// static CMyApplication instance;


	/** Connection to the LLRP rstatic void SignalHandler(int signal) {eader */
	CConnection *               m_pConnectionToReader;


	CMyApplication(const char *pReaderHostName_t);
	

	~CMyApplication();

	// void SignalHandler();

	// static void SignalHandlerWrapper(int signal);

	void keyboardThread();

	void init_keyboard();


	void restore_keyboard();
	
	int
		run(
		char 					 *pReaderHostName);

	int
    	start ();

	int 
		stop();

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

	void saveTag2Buff(void);
};

