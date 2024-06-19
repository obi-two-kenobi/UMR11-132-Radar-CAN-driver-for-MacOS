#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgcodecs.hpp>


#include <iostream>
#include <vector>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>


#include <thread>
#include <algorithm>
#include <arpa/inet.h>
#include <pcap.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <unistd.h>
#include "Includes/KvaserCAN.h"
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <sys/types.h>
#include <signal.h>

#include <ostream>
#include <chrono>


#define uchar unsigned char
#define toRad(x) (x * 3.14159265358979323846 / 180.00000)
#define toDeg(x) (x * 180.00000 / 3.14159265358979323846)
using namespace std;

namespace Sensor{
    
    enum timeUnit{
        year,month,day,hour,minute,second,us
    };

    struct toolBox{
        struct box{
            int NXlim=-1150/*+850*/,
                NYlim=-475/*-230*/,
                NZlim=-110,
                PXlim=780/*+850*/,
                PYlim=430/*-230*/,
                PZlim=105;
        }box;
        
        struct cluster{
            int minPts = 30;
            double eps = 125;
        }cluster;
        
        struct view{
            bool showIgnored = false;
            bool showIDs = true;
        }view;
    };
    
    struct radarPoint{
        int ObjID;
        double R,RR,Azimuth,Elevation;
    };


    class CAN_UMR11{
    private:
        #define CHANNEL  0
        #define BAUDRATE  CANBTR_INDEX_500K

        CKvaserCAN myDriver = CKvaserCAN();
        CANAPI_OpMode_t opMode = {};
        CANAPI_Bitrate_t bitrate = {};
        CANAPI_Message_t message = {};
        CANAPI_Return_t retVal = 0;
        double CycleDuration; // in seconds
        uint64_t CycleCount;
        double TimeStamp; //in seconds
        uint64_t numberOfTargets = 0;
        int frameCounter = 0;
        Sensor::toolBox& tb;
        volatile bool running = true;
        Sensor::radarPoint point;
        std::vector<Sensor::radarPoint> frame;
        class BM {
        public:
            // Get the value of a specific bit at a given position
            static bool getBit(uint64_t value, int position) {
                return (value & (1ULL << position)) != 0;
            }

            // Set a specific bit at a given position to 1
            static void setBit(uint64_t& value, int position) {
                value |= (1ULL << position);
            }

            // Set a specific bit at a given position to 0
            static void clearBit(uint64_t& value, int position) {
                value &= ~(1ULL << position);
            }

            // Toggle a specific bit at a given position
            static void toggleBit(uint64_t& value, int position) {
                value ^= (1ULL << position);
            }

            // Copy a range of bits from source to destination
            static void copyBits(uint64_t& destination, const uint64_t& source, int start, int end) {
                uint64_t mask = ((1ULL << (end - start + 1)) - 1) << start;
                destination &= ~mask;  // Clear the bits in the destination range
                destination |= ((source >> start) & ((1ULL << (end - start + 1)) - 1)) << start;
                destination = destination >> start;
            }
        };
        struct parser{
        uint16_t id;
        uint8_t     byte7; // 1-byte variable
        uint8_t     byte6; // 1-byte variable
        uint8_t     byte5; // 1-byte variable
        uint8_t     byte4; // 1-byte variable
        uint8_t     byte3; // 1-byte variable
        uint8_t     byte2; // 1-byte variable
        uint8_t     byte1; // 1-byte variable
        uint8_t     byte0; // 1-byte variable
        uint64_t    data;

    }parsed;
        parser Parse(CANAPI_Message_t& message);
        
    public:
        CAN_UMR11(Sensor::toolBox& tb);
        std::vector<Sensor::radarPoint>& getFrame();
        int getFrameCount() const;
        double getCycleDuration() const;
        uint64_t getCycleCount() const;
        double getTimeStamp() const;
        uint64_t getnumberOfTargets() const;
        void stop();
      };
    
    class Offline_UMR11{
        private:
            double CycleDuration; // in seconds
            uint64_t CycleCount;
            double TimeStamp; //in seconds
            uint64_t numberOfTargets = 0;
            int frameCounter = 0; //frame counter
        Sensor::radarPoint point; //{ int ObjID; double R,RR,Azimuth,Elevation;}
            std::vector<Sensor::radarPoint> frame;
        Sensor::toolBox& tb;
        
            class BM {
            public:
                // Get the value of a specific bit at a given position
                static bool getBit(uint64_t value, int position) {
                    return (value & (1ULL << position)) != 0;
                }

                // Set a specific bit at a given position to 1
                static void setBit(uint64_t& value, int position) {
                    value |= (1ULL << position);
                }

                // Set a specific bit at a given position to 0
                static void clearBit(uint64_t& value, int position) {
                    value &= ~(1ULL << position);
                }

                // Toggle a specific bit at a given position
                static void toggleBit(uint64_t& value, int position) {
                    value ^= (1ULL << position);
                }

                // Copy a range of bits from source to destination
                static void copyBits(uint64_t& destination, const uint64_t& source, int start, int end) {
                    uint64_t mask = ((1ULL << (end - start + 1)) - 1) << start;
                    destination &= ~mask;  // Clear the bits in the destination range
                    destination |= ((source >> start) & ((1ULL << (end - start + 1)) - 1)) << start;
                    destination = destination >> start;
                }
            }; //Bit Manipulator static class
            struct parser{
                uint16_t id;
                uint8_t     byte7; // 1-byte variable
                uint8_t     byte6; // 1-byte variable
                uint8_t     byte5; // 1-byte variable
                uint8_t     byte4; // 1-byte variable
                uint8_t     byte3; // 1-byte variable
                uint8_t     byte2; // 1-byte variable
                uint8_t     byte1; // 1-byte variable
                uint8_t     byte0; // 1-byte variable
                uint64_t    data;

            }parsed; //parsed can message
        
            
            std::ifstream file;
            std::string line;
            parser Parse(std::string line);

        public:
            Offline_UMR11(toolBox& tb, std::string filename);
            std::vector<Sensor::radarPoint>& getFrame();
            int getFrameCount() const;
            double getCycleDuration() const;
            uint64_t getCycleCount() const;
            double getTimeStamp() const;
            uint64_t getnumberOfTargets() const;
            void stop();
        };


}
