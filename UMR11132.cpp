#include "UMR11132.hpp"


Sensor::CAN_UMR11::CAN_UMR11(Sensor::toolBox& tb):tb(tb){
    opMode.byte = CANMODE_DEFAULT;
    bitrate.index = BAUDRATE;
    std::cout << CKvaserCAN::GetVersion() << std::endl;
    if ((retVal = myDriver.InitializeChannel(CHANNEL, opMode)) != CCanApi::NoError) {
        std::cerr << "+++ error: interface could not be initialized" << std::endl;
        exit(retVal);
    }
    if ((retVal = myDriver.StartController(bitrate)) != CCanApi::NoError) {
        std::cerr << "+++ error: interface could not be started" << std::endl;
        stop();
    }
}

Sensor::CAN_UMR11::parser Sensor::CAN_UMR11::Parse(CANAPI_Message_t& message){
    parser p;
    p.id = message.id;
    for (uint8_t i = 0; i < CCanApi::Dlc2Len(message.dlc); i++){
        auto val = message.data[i];
        switch (i){
            case 0:
                p.byte0 = val;
                break;
            case 1:
                p.byte1 = val;
                break;
            case 2:
                p.byte2 = val;
                break;
            case 3:
                p.byte3 = val;
                break;
            case 4:
                p.byte4 = val;
                break;
            case 5:
                p.byte5 = val;
                break;
            case 6:
                p.byte6 = val;
                break;
            case 7:
                p.byte7 = val;
                break;
            default:
                break;
        } 
    }
    p.data = (static_cast<uint64_t>(p.byte7) << 56) | (static_cast<uint64_t>(p.byte6) << 48) | (static_cast<uint64_t>(p.byte5) << 40) | (static_cast<uint64_t>(p.byte4) << 32) | (static_cast<uint64_t>(p.byte3) << 24) | (static_cast<uint64_t>(p.byte2) << 16) | (static_cast<uint64_t>(p.byte1) << 8) | static_cast<uint64_t>(p.byte0);
    return p;

}

std::vector<Sensor::radarPoint>& Sensor::CAN_UMR11::getFrame(){

    frame.clear();
    while (true)
    {
       if ((retVal = myDriver.ReadMessage(message)) == CCanApi::NoError) parsed = Parse(message);
        if (parsed.id == 0x400){
            uint64_t modeBit = 0;
            BM::copyBits(modeBit, parsed.data, 62, 63);
            if (modeBit == 0){
                BM::copyBits(numberOfTargets, parsed.data, 47, 54);
                BM::copyBits(CycleCount, parsed.data, 7, 46);
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 11);
                CycleDuration = (tmp * 0.000064);
                
            }
            else if (modeBit == 1){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp = tmp * 1.0;

            }
            else if (modeBit == 2){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp += (tmp * 0.00000000023283064365386962890625);
                break;
            }
            else continue;
        }
    }
    auto targetCounter = numberOfTargets;
    while(targetCounter > 0){
        if ((retVal = myDriver.ReadMessage(message)) == CCanApi::NoError)
        {
            parsed = Parse(message);
            if (parsed.id > 0x400 && parsed.id < 0x4FF)
            {
                if(BM::getBit(parsed.data, 0) == 0) //DataFrame 0
                {
                    uint64_t tmp;
                    BM::copyBits(tmp, parsed.data, 1, 13);
                    point.R = (tmp * 0.04); //meter
                    BM::copyBits(tmp, parsed.data, 39, 50);
                    point.RR = (tmp * 0.04); //m/s
                    BM::copyBits(tmp, parsed.data, 22, 31);
                    point.Azimuth = tmp * 0.16; //degree
                    point.ObjID = parsed.id - 0x400;
                  
                }
                if(BM::getBit(parsed.data, 0) == 1) //DataFrame 1
                {
                    uint64_t tmp;
                    BM::copyBits(tmp, parsed.data, 37, 46);
                    point.Elevation = tmp * 0.04; //degree
                    frame.push_back(point);
                    targetCounter--;
                }
            }
        }
    }
    frameCounter++;
    return frame;
}

void Sensor::CAN_UMR11::stop() {
    myDriver.SignalChannel();
    if ((retVal = myDriver.TeardownChannel()) != CCanApi::NoError)
        std::cerr << "+++ error: interface could not be shutdown" << std::endl;
    std::cout << "Radar Stopped" << std::endl;

}

int Sensor::CAN_UMR11::getFrameCount() const{
    return frameCounter;
}

double Sensor::CAN_UMR11::getCycleDuration() const{
    return CycleDuration;
}

uint64_t Sensor::CAN_UMR11::getCycleCount() const{
    return CycleCount;
}

double Sensor::CAN_UMR11::getTimeStamp() const{
    return TimeStamp;
}

uint64_t Sensor::CAN_UMR11::getnumberOfTargets() const{
    return numberOfTargets;
}


//----------------------------------------------------------------------------
//Offline_UMR11-------------------------------------------------------
    Sensor::Offline_UMR11::Offline_UMR11(Sensor::toolBox& tb, std::string filename):tb(tb){
    file = std::ifstream(filename);
        if (!file.is_open()) {
        std::cerr << "Failed to open log.csv" << std::endl;
        exit(7);
    }
    std::getline(file, line);  // Skip the first line for compatibility issues.
}

Sensor::Offline_UMR11::parser Sensor::Offline_UMR11::Parse(std::string line){
    std::stringstream ss(line);
    std::string value;
    parser p;

    // Read each value separated by semicolon
    std::getline(ss, value, ';');
    p.id = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte7 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte6 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte5 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte4 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte3 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte2 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte1 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte0 = std::stoi(value, nullptr, 10);

    p.data = (static_cast<uint64_t>(p.byte7) << 56) | (static_cast<uint64_t>(p.byte6) << 48) | (static_cast<uint64_t>(p.byte5) << 40) | (static_cast<uint64_t>(p.byte4) << 32) | (static_cast<uint64_t>(p.byte3) << 24) | (static_cast<uint64_t>(p.byte2) << 16) | (static_cast<uint64_t>(p.byte1) << 8) | static_cast<uint64_t>(p.byte0);

    return p;
}

std::vector<Sensor::radarPoint>& Sensor::Offline_UMR11::getFrame(){
    frame.clear();

    while (std::getline(file, line))
    {
        parsed = Parse(line);
        if (parsed.id == 0x400)
        {
            uint64_t modeBit = 0;
            BM::copyBits(modeBit, parsed.data, 62, 63);
            if (modeBit == 0){
                BM::copyBits(numberOfTargets, parsed.data, 47, 54);
                BM::copyBits(CycleCount, parsed.data, 7, 46);
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 11);
                CycleDuration = (tmp * 0.000064);
                
            }
            else if (modeBit == 1){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp = tmp * 1.0;

            }
            else if (modeBit == 2){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp += (tmp * 0.00000000023283064365386962890625);
                break;
            }
            else continue;
        }
    }
    
    auto targetCounter = numberOfTargets;
    while(targetCounter > 0){
        if (std::getline(file, line))
        {
            parsed = Parse(line);
            if (parsed.id > 0x400 && parsed.id < 0x4FF)
            {
                if(BM::getBit(parsed.data, 0) == 0) //DataFrame 0
                {
                    uint64_t tmp=0;
                    BM::copyBits(tmp, parsed.data, 1, 13);
                    point.R = (tmp * 0.04); //meter
                    tmp=0;
                    BM::copyBits(tmp, parsed.data, 39, 50);
                    point.RR = (tmp * 0.04); //m/s
                    tmp=0;
                    BM::copyBits(tmp, parsed.data, 22, 31);
                    int64_t tmps = 0;
                    tmps=static_cast<int64_t>(tmp);
                    tmps-=511;
                    point.Azimuth = tmps * 0.16; //degree
                    point.ObjID = parsed.id - 0x400;
                  
                }
                if(BM::getBit(parsed.data, 0) == 1) //DataFrame 1
                {
                    uint64_t tmp=0;
                    BM::copyBits(tmp, parsed.data, 37, 46);
                    int64_t tmps = 0;
                    tmps = static_cast<int64>(tmp);
                    tmps-=511;
                    point.Elevation = tmps * 0.04; //degree
                    frame.push_back(point);
                    targetCounter--;
                }
            }

        }
    }
    frameCounter++;
    return frame;
}

void Sensor::Offline_UMR11::stop() {
    file.close();
}

int Sensor::Offline_UMR11::getFrameCount() const{
    return frameCounter;
}

double Sensor::Offline_UMR11::getCycleDuration() const{
    return CycleDuration;
}

uint64_t Sensor::Offline_UMR11::getCycleCount() const{
    return CycleCount;
}

double Sensor::Offline_UMR11::getTimeStamp() const{
    return TimeStamp;
}

uint64_t Sensor::Offline_UMR11::getnumberOfTargets() const{
    return numberOfTargets;
}
