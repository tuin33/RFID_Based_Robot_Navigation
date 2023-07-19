#include "RFIDHandler.h"

int main(int argc, char** argv){
    CMyApplication app("192.168.0.105");
    app.m_Verbose = 1;
    app.stop();
    return 0;
}