
#include <vector>
#include <stdio.h>
#include <stdlib.h>

struct Antenna
{
    int antennaId;
    std::vector<double> phase;
    std::vector<double> rssi;
    std::vector<double> timestamp;
};


struct Reader
{
    int readerID;
    Antenna ant1;
    Antenna ant2;
};


struct EPC
{
    char   epc[128];
	Reader reader;        
};

