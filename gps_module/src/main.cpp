#include <iostream>
#include <parser_vercpp.hpp>
#include <string>
#include <mutex>

const std::string port_name = "/dev/ttyACM0";
const int baudrate = 19200;

int main(int argc, char *argv[])
{
    ublox_parser ublox(port_name, baudrate);
    if(!ublox.isInit){
        std::cout<<"Init Error\n";
        return -1;
    }
    ublox_parser::PARSING_TYPEDEF_UBX_M8P_NED ned;
    ublox_parser::PARSING_TYPEDEF_UBX_M8P_PVT pvt;

    bool init_state = ublox.start();

    while(1){
        ublox.copyNED(&ned);
        printf("%f %f\n",ned.relpos_N, ned.relpos_E);
        ublox.copyPVT(&pvt);
        printf("%f %f\n",pvt.lon, pvt.lat);
    }


    return 0;
}
