#include "parser_vercpp.hpp"
extern pthread_mutex_t mutex;

void ublox_parser::run_thread()
{
    while(isInit){
        uint8_t ch = 0x00;
        if (serial_->read(&ch, 1) == 1)
        {
            mValid = c94_m8p_parser(ch);

        }
    }
}
void ublox_parser::run()
{
    uint8_t ch = 0x00;
    if (serial_->read(&ch, 1) == 1)
    {
        mValid = c94_m8p_parser(ch);
    }
}

void *ublox_parser::run_(void *pthis){
    ublox_parser *ubx = (ublox_parser*)pthis;
    ubx->run_thread();
    pthread_exit(NULL);
}

bool ublox_parser::start()
{
	if (isInit)
	{
        if (serial_->isOpen())
		{
            pthread_create(&pThread, NULL, ublox_parser::run_, (void*)this);
            return true;
		}
        else{
            return false;
        }
	}
    else{
        return false;
    }
}

ublox_parser::PARSING_STATUS ublox_parser::valid()
{
	return mValid;
}
void ublox_parser::copyNED(PARSING_TYPEDEF_UBX_M8P_NED *pDst){
    (*pDst) = mNED;
}
void ublox_parser::copyPVT(PARSING_TYPEDEF_UBX_M8P_PVT *pDst){
    (*pDst) = mPVT;
}


ublox_parser::ublox_parser(const std::string port_name, int baudrate) : 
	mValid(PARSING_FAIL_),
	isInit(false)
{

    serial_ = new serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(1000));
	

    if (!serial_->isOpen())
	{
		std::cout << "Connection Failed" << std::endl;
		return;
	}

	isInit = true;
}

ublox_parser::~ublox_parser()
{
	
	//pThread->join();

    if (serial_->isOpen())
	{
        serial_->close();
	}


}

uint64_t ublox_parser::little64(uint8_t * c)
{
	uint8_t i = 0;
	uint64_t temp = 0x00;

	for (i = 7; i >= 0; i--)
	{
		temp |= c[i];

		if (i == 0) break;
		else temp = temp << 8;

	}

	return temp;
}

uint32_t ublox_parser::little32(uint8_t * c)
{
	uint8_t i = 0;
	uint32_t temp = 0x00;

	for (i = 3; i >= 0; i--)
	{
		temp |= c[i];

		if (i == 0) break;
		else temp = temp << 8;

	}

	return temp;
}

uint32_t ublox_parser::s_little32(uint8_t * c)
{
    uint8_t i = 0;
    int32_t temp = 0x00;

    for (i = 3; i >= 0; i--)
    {
        temp |= c[i];

        if (i == 0) break;
        else temp = temp << 8;

    }
    return temp;
}


uint16_t ublox_parser::little16(uint8_t * c)
{
	uint8_t i = 0;
	uint16_t temp = 0x00;

	for (i = 1; i >= 0; i--)
	{
		temp |= c[i];

		if (i == 0) break;
		else temp = temp << 8;

	}

	return temp;
}

ublox_parser::PARSING_STATUS ublox_parser::c94_m8p_parser(const uint8_t ch)
{

	static volatile 	  uint8_t	flag		= 0x00;

	static const volatile uint8_t	_SYNC1		= 0xB5;
	static const volatile uint8_t	_SYNC2		= 0x62;

	static const volatile uint8_t	_CLASS_HPP	= 0x01;
	static const volatile uint8_t	_ID_HPP		= 0x14;
	static const volatile uint8_t	_CLASS_PVT	= 0x01;
	static const volatile uint8_t	_ID_PVT		= 0x07;
    static const volatile uint8_t	_ID_NED		= 0x3C;

    static uint8_t 					payload_pvt[100] = { 0 };
    static uint8_t 					payload_ned[46] = { 0 };
	static volatile uint8_t 		count		= 0;

	if (flag == 0x00) flag = 0x01;

	/** @Parsing */
	switch (flag)
	{
	case 0x01:

		if (ch == _SYNC1) { flag += 0x01; }
		else { flag = 0x00; }
		break;

	case 0x02:

		if (ch == _SYNC2) { flag += 0x01; }
		else { flag = 0x00; count = 0; }
		break;

	case 0x03: 	/** @CompareCLASS */

        if (ch == _CLASS_PVT){
            flag += 0x01;
            payload_pvt[count] = ch;
            payload_ned[count++] = ch;
       }
		else { flag = 0x00; count = 0; }
		break;

	case 0x04:		/** @CompareID */

        if (ch == _ID_PVT) {
            flag = 0x07;
            payload_pvt[count++] = ch;
        }
        else if(ch == _ID_NED){
            flag = 0x3C; payload_ned[count++] = ch;
        }
		else { flag = 0x00; count = 0; }
		break;

	case 0x07:		/** @PVT */
        payload_pvt[count++] = ch;

        if (count == 98)
		{
			count = 0;
			flag = 0x00;

			uint8_t i = 0x00;
			uint8_t CK_A = 0x00, CK_B = 0x00;

			for (i = 0; i < 96; i++)
			{
                CK_A = CK_A + payload_pvt[i];
				CK_B = CK_B + CK_A;
			}

            i = 4;
            if ((CK_A == payload_pvt[96]) && (CK_B == payload_pvt[97]))
			{
				/* To Parse the payload on format for use */
				PARSING_TYPEDEF_UBX_M8P_PVT toRet;

				/* Parsing code */
                toRet.iTOW = little32(&payload_pvt[0 + i]) / 1000.0;                // second
                toRet.year = little16(&payload_pvt[4 + i]);                         // year
                toRet.month = payload_pvt[6 + i];       							// month
                toRet.day = payload_pvt[7 + i];                         			// day
                toRet.hour = payload_pvt[8 + i];                                	// hour
                toRet.min = payload_pvt[9 + i];             						// minutes
                toRet.sec = payload_pvt[10 + i];                                    // seconds
                toRet.valid = payload_pvt[11 + i];                                   // bit field
                toRet.tAcc = little32(&payload_pvt[12 + i]) / 1000000000.0;	// seconds
                toRet.nano = little32(&payload_pvt[16 + i]) / 1000000000.0;	// seconds
                toRet.fixType = payload_pvt[20 + i];                        // unsigned number
                toRet.flags = payload_pvt[21 + i];							// bit field
                toRet.flags2 = payload_pvt[22 + i];							// bit field
                toRet.numSV = payload_pvt[23 + i];							// Unsigned number
                toRet.lon = little32(&payload_pvt[24 + i]) / 10000000.0;		// degree
                toRet.lat = little32(&payload_pvt[28 + i]) / 10000000.0;		// degree
                toRet.height = little32(&payload_pvt[32 + i]) / 1000.0;				// height above ellipsoid (meter)
                toRet.hMSL = little32(&payload_pvt[36 + i]) / 1000.0;				// height above mean sea level (meter)
                toRet.hAcc = little32(&payload_pvt[40 + i]) / 1000.0;				// Horizontal Accuracy Estimate (meter)
                toRet.vAcc = little32(&payload_pvt[44 + i]) / 1000.0;				// Vertical Accuracy Estimate (meter)
                toRet.velN = little32(&payload_pvt[48 + i]) / 1000.0;				// NED North Velocity (m/s)
                toRet.velE = little32(&payload_pvt[52 + i]) / 1000.0;				// NED East Velocity (m/s)
                toRet.velD = little32(&payload_pvt[56 + i]) / 1000.0;				// NED Down Velocity (m/s)
                toRet.gSpeed = little32(&payload_pvt[60 + i]) / 1000.0;				// Ground Speed (2D) (m/s)
                toRet.headMot = little32(&payload_pvt[64 + i]) / 100000.0;			// Heading of Motion (2D) (degree)
                toRet.sAcc = little32(&payload_pvt[68 + i]) / 1000.0;				// Speed Accuracy Estimation (m/s)
                toRet.headAcc = little32(&payload_pvt[72 + i]) / 100000.0;			// Heading Accuracy Estimate [Both Motion & Vehicle] (degree)
                toRet.pDOP = little16(&payload_pvt[76 + i]) / 100.0;				// Position DOP
                toRet.headVeh = little32(&payload_pvt[84 + i]) / 100000.0;			// Heading of vehicle (2D) (degree)

                mPVT = toRet;
                return PARSING_PVT_SUCCESS_;

				break;


			} /** @EndOfIf if ( (CK_A == payload[96]) && (CK_B == payload[97]) ) */

		} /** @EndOfIf if ( count == 98 ) */

		break;
    case 0x3C:
        payload_ned[count++] = ch;
        if(count == 46){
            count = 0;
            flag = 0x00;

            uint8_t i = 0x00;
            uint8_t CK_A = 0x00, CK_B = 0x00;
            for(int i=0; i<44; i++){
                CK_A = CK_A + payload_ned[i];
                CK_B = CK_B + CK_A;
            }
            int offset = 4;
            if ((CK_A == payload_ned[44]) && (CK_B == payload_ned[45]))
            {
                PARSING_TYPEDEF_UBX_M8P_NED tmp;
                int N = s_little32(&payload_ned[offset + 8]);
                tmp.relpos_N = (double)N/100.0;    //Unit is cm so convert to unit m -> divide 100
                int E = s_little32(&payload_ned[offset + 12]);
                tmp.relpos_E = (double)E/ 100.0;
                int D = s_little32(&payload_ned[offset + 16]);
                tmp.relpos_D = (double)D/ 100.0;
                mNED = tmp;
                return PARSING_NED_SUCCESS_;
            }
            break;
        }
        break;

    default:
		break;

	} /** @EndOfSwitch switch(flag) */

	return PARSING_FAIL_;
	/** @EndOfParsing */

	//	__HAL_UART_CLEAR_PEFLAG(&huart3);
}
