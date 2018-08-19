#include <serial/serial.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>

namespace serial { class Serial; }

class ublox_parser {
public : 

	typedef enum {
        PARSING_PVT_SUCCESS_ = 0,
        PARSING_NED_SUCCESS_ = 1,
		PARSING_FAIL_ = -1
	} PARSING_STATUS;

	typedef struct {

		uint64_t now;

		double 		iTOW;
		uint16_t 	year;
		uint8_t 	month;
		uint8_t 	day;
		uint8_t 	hour;
		uint8_t 	min;
		uint8_t 	sec;
		uint8_t 	valid;
		double 		tAcc;
		double 		nano;
		uint8_t 	fixType;
		uint8_t 	flags;
		uint8_t 	flags2;
		uint8_t 	numSV;
		double 		lon;
		double 		lat;
		double 		height;
		double 		hMSL;
		double 		hAcc;
		double 		vAcc;
		double 		velN;
		double 		velE;
		double 		velD;
		double 		gSpeed;
		double 		headMot;
		double 		sAcc;
		double 		headAcc;
		double 		pDOP;
		double 		headVeh;

	} PARSING_TYPEDEF_UBX_M8P_PVT;
    typedef struct{
       double relpos_N;
       double relpos_E;
       double relpos_D;
    } PARSING_TYPEDEF_UBX_M8P_NED;

public : 
    void run();
    void run_thread();
    static void* run_(void *);
    bool start();
	PARSING_STATUS valid();
	void copyNED(PARSING_TYPEDEF_UBX_M8P_NED *pDst);
	void copyPVT(PARSING_TYPEDEF_UBX_M8P_PVT *pDst);
	ublox_parser(const std::string port_name, int baudrate);
	~ublox_parser();		
	bool isInit;

protected:
    PARSING_TYPEDEF_UBX_M8P_PVT mPVT;
    PARSING_TYPEDEF_UBX_M8P_NED mNED;
    PARSING_STATUS mValid;

private:
	uint64_t little64(uint8_t *c);
	uint32_t little32(uint8_t *c);
	uint16_t little16(uint8_t *c);
    uint32_t s_little32(uint8_t * c);

	PARSING_STATUS c94_m8p_parser(const uint8_t ch);

    serial::Serial *serial_;

    pthread_t pThread;
};

