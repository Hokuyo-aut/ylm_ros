// Copyright (C) 2021, 2022 Lumotive
// Copyright 2023 HOKUYO AUTOMATIC CO.,LTD.

// YLM TCP Stream Input Example

// Usage:
// Call getDepthBuffer(portNum) to get a full depth buffer.
//
// Note the first port is by default 10940 (first sensor head, first FOV). Each subsequent
// FOV is on consecutively increasing port numbers (10941, 10942, ...), dependent on system
// configuration.
//
// It is intended that getDepthBuffer() be called in a loop. It will try to [re-]establish a
// TCP connection as needed.
//
// Return values <0 indicate failure. It is suggested to sleep before calling getDepthBuffer()
// again after a failure. This prevents needless resource consumption (connection floods,
// spinning CPU) when an FOV is not yet available.
//
// Return values >=0 indicate success. Data of type depthBuffer_t (for now uint16_t) is
// returned in depthBuffer[], with dimensions (depthBuffer_dimStare, depthBuffer_dimSteer).
// A total of depthBuffer_numPoints will be available. depthBuffer is valid from the time
// getDepthBuffer() returns success, till the next time getDepthBuffer() is called. I.e.
// it is intended to be accessed between calls to getDepthBuffer(), but only if
// getDepthBuffer() returns success.
//
// PLEASE NOTE: The sensor must be configured, and the scan started, using either the REST
// interface or the web UI. This example code only receives the output stream, and does *not*
// configure or start the sensor
//
// See main() for an example.

#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <memory>
#include <boost/asio.hpp>
#include "../include/lumotive_ros/ylm_packet_structs.h"
#include "../include/lumotive_ros/ylm_data_types.h"
#include "../include/lumotive_ros/ylm_client.h"

using boost::asio::ip::tcp;
using namespace std;

static boost::asio::io_service ioService;
static tcp::socket* tcpSocket = nullptr;

#define PACKET_SIZE_MAX 2048
static char packetBuffer[PACKET_SIZE_MAX];
size_t packetBufferUsed = 0;

static ostream *errS = new ostream(nullptr);
static ostream *errV = new ostream(nullptr);
static bool verboseTimestamps = false;

// Keep track of frame sequence bounds
// Only returning full frames for now
static int64_t currentSceneStartSeq = -1;
static int64_t currentSceneEndSeq = -1;

typedef struct {
    float theta;
    float phi;
} mapping_table_entry_t;

static const double PI = 3.14159;
static const double arcSecToRadian_ = PI / 648000.0;
static mapping_table_entry_t *mappingTable = (mapping_table_entry_t *) nullptr;
static size_t mappingTable_numPoints = 0;
static size_t mappingTable_dimStare = 0;
static size_t mappingTable_dimSteer = 0;

int handleTypeD(TypeDHeader_t *typeDHeader, uint32_t seq);
int handleTypeC(TypeCHeader_t *typeCHeader, uint32_t seq);

// ---------------------------------------------------------------------------------
// Outputs
std::shared_ptr<metadataFrame> mFrame(new metadataFrame(0, 0, 0));
std::shared_ptr<rangeFrame> rFrame(new rangeFrame(0));
std::shared_ptr<intensityFrame> iFrame(new intensityFrame(0));

std::shared_ptr<metadataFrame> get_metadata_frame() {
    return mFrame;
}

std::shared_ptr<rangeFrame> get_range_frame() {
    return rFrame;
}

std::shared_ptr<intensityFrame> get_intensity_frame() {
    return iFrame;
}

int ensureConnected(std::string IP, int port) {
    boost::system::error_code ec;

    // Do we already have a good connection open?
    if(tcpSocket) {
        if(!tcpSocket->is_open()) {
            tcpSocket->close();
            delete tcpSocket;
            tcpSocket = nullptr;
        }
        else {
            // Looks like we're already in a good state
            return 0;
        }
    }

    // Try to [re-]establish connection
    if(tcpSocket)
        delete tcpSocket;

    // TCP Connect
    tcp::endpoint ep(boost::asio::ip::address::from_string(IP), port);
    *errS << "Trying to connect to sensor stream at " << ep << "." << endl;
    
    tcp::socket* freshTcpSocket = new tcp::socket(ioService);
    freshTcpSocket->connect(ep, ec);
    
    if(ec) {
        *errS << "Failed to connect to sensor stream: " << ec.message() << endl;
        freshTcpSocket->close(ec);
        delete freshTcpSocket;
        return -1;
    }

    // Success
    tcpSocket = freshTcpSocket;
    *errS << "Connected to stream at " << ep << "." << endl;
    return 0;
}

int readPacket() {
        boost::system::error_code ec;

        // Read framing header, then entire packet
        char framingHeaderBuffer[TCPFramingHeader_t::read_len];

        boost::asio::read(*tcpSocket, boost::asio::buffer(framingHeaderBuffer, TCPFramingHeader_t::read_len), ec);
        if(ec) {
            *errS << ec.message() << endl;
            tcpSocket->close();
            return -5;
        }

        TCPFramingHeader_t *TCPFramingHeader = (TCPFramingHeader_t*) framingHeaderBuffer;

        size_t packetLen = TCPFramingHeader->len;
        if(packetLen > PACKET_SIZE_MAX) {
            tcpSocket->close();
            *errS << "Framed packet too large: " << packetLen << " bytes." << endl;
            return -3;   
        }

        packetBufferUsed = boost::asio::read(*tcpSocket, boost::asio::buffer(packetBuffer, TCPFramingHeader->len), ec);  
        if(ec) {
            *errS << ec.message() << endl;
            tcpSocket->close();
            return -4;
        }

        // Check Magic
        GlobalHeader_t *globalHeader = (GlobalHeader_t *) packetBuffer;
        if(memcmp("BCDA", globalHeader->magic, 4) != 0)
        {
            *errS << "Bad magic. Invalid stream?" << endl;
            tcpSocket->close();
            return -2;
        }

        // Success
        return 0;
}

int pollNetworkForFrame(std::string IP, int port){
    // Read till we have a full frame
    // Reallocate buffer if applicable
    #ifndef _UNIT_TESTS_ // Test mode is off
    while (true) {
        // Try to read a packet
        int ecRet = ensureConnected(IP, port);
        if (ecRet < 0)
            return ecRet;
        int rpRet = readPacket();
        if (rpRet < 0) 
            return rpRet;

        // Space out verbose output
        *errV << endl;
        
        GlobalHeader_t *globalHeader = (GlobalHeader_t *) packetBuffer;
        if(globalHeader->extract_version() != 1) {
            stringstream readableVer;
            readableVer << hex << uppercase << (uint16_t) (globalHeader->extract_version());
            *errS << "Got packet with version " << readableVer.str() << ". Skipping." << endl;
            continue;
        }
 
        int tshRet = 0;
        // Handle Type-D packets
        if(globalHeader->extract_type() == 0xD) {
            tshRet = handleTypeD(globalHeader->get_nextHeader_TypeD(), globalHeader->seq);
            if(tshRet == 1) {
                return 1;
            }
        }

        // Handle Type-C packets        
        else if(globalHeader->extract_type() == 0xC) {
            tshRet = handleTypeC(globalHeader->get_nextHeader_TypeC(), globalHeader->seq);
        }
        
        
        else {
            stringstream readableType;
            readableType << hex << uppercase << (uint16_t) (globalHeader->extract_type());
            *errS << "Got packet with type " << readableType.str() << ". Skipping." << endl;
            continue;
        }

        if(tshRet < 0)
            return tshRet;
    }
    #else // Test mode is on
    // Prepare the frame
    size_t startStare = 0;
    size_t startSteer = 0;
    size_t endStare = 1278 - 1;
    size_t endSteer = 10 - 1;
    size_t stepStare = 1;
    size_t stepSteer = 1;
    size_t frameNumPoints = (endStare + 1) * (endSteer + 1);

    mFrame.reset(); // delete this instance of shared_ptr (ownership has been transfered to top layer)
    mFrame = std::make_shared<metadataFrame>(frameNumPoints, endStare + 1, endSteer + 1);

    mFrame->range = true;
    mFrame->signal = true;
    mFrame->noise = true;
    mFrame->SNR = true;
    
    rFrame.reset();
    rFrame = std::make_shared<rangeFrame>(frameNumPoints);

    iFrame.reset();
    iFrame = std::make_shared<intensityFrame>(frameNumPoints);

    // rawFrame.reset();
    // rawFrame = std::make_shared<rawDataFrame>(frameNumPoints);

    for (size_t px = 0; px < (endStare + 1)*(endSteer + 1); px++)
    {

        rFrame->range[px] = 1234.5;
        rFrame->x[px] = 6.0;
        rFrame->y[px] = 7.0;
        rFrame->z[px] = 8.0;

        iFrame->signal[px] = 678;
        iFrame->noise[px] = 901;
        iFrame->SNR[px] = 23.0;

    }

    return 42; // Frame is ready! (the test mode returns a frame (always the same one) at each call)

    #endif

    return -8;
}

int handleTypeD(TypeDHeader_t *typeDHeader, uint32_t seq) {
        
        static size_t dimStare = 0;
        static size_t dimSteer = 0;

        // Did we start a new frame -- Clean out current buffer if so
        if (typeDHeader->isSet_currentSceneBeginSequenceValid() && (currentSceneStartSeq != typeDHeader->ao_cs_start))
        {
            *errS << "New Frame Detected." << endl;
            // Re-allocate frame buffer if needed
            // TODO -- make dimensions match maximum of mapping table and image --
            // if we've seen the mapping table already.
            dimStare = max(max(((size_t)typeDHeader->complete_size_stare_dim - 1) * typeDHeader->payload_step_u + typeDHeader->payload_start_u + 1,
                                           dimStare), mappingTable_dimStare);
            dimSteer = max(max(((size_t)typeDHeader->complete_size_steer_dim - 1) * typeDHeader->payload_step_v + typeDHeader->payload_start_v + 1,
                                           dimSteer), mappingTable_dimSteer);
            size_t frameNumPoints = (size_t) typeDHeader->complete_size_steer_dim * (size_t) typeDHeader->complete_size_stare_dim;
            if(frameNumPoints > mFrame->frameNumPoints) { // Reset buffer because dims have changed
                mFrame->resize(frameNumPoints, (size_t) typeDHeader->complete_size_stare_dim, (size_t) typeDHeader->complete_size_steer_dim);
                rFrame->resize(frameNumPoints);
                iFrame->resize(frameNumPoints);
            }

            mFrame.reset(); // delete this instance of shared_ptr (ownership has been transfered to top layer)
            mFrame = std::make_shared<metadataFrame>(frameNumPoints, (size_t) typeDHeader->complete_size_stare_dim, (size_t) typeDHeader->complete_size_steer_dim);
        
            rFrame.reset();
            rFrame = std::make_shared<rangeFrame>(frameNumPoints);

            iFrame.reset();
            iFrame = std::make_shared<intensityFrame>(frameNumPoints);
        }

        // Update Framing
        if(typeDHeader->isSet_currentSceneBeginSequenceValid())
            currentSceneStartSeq = typeDHeader->ao_cs_start;
        if(typeDHeader->isSet_currentSceneEndSequenceValid())
            currentSceneEndSeq = typeDHeader->ao_cs_end;

        // Copy Range Data into Buffer
        // U, V space (for mapping table)
        size_t u = typeDHeader->payload_start_u;
        size_t v = typeDHeader->payload_start_v;

        // Mapping-table agnostic dense indicies
        size_t i = typeDHeader->payload_stareorder_offset;
        size_t j = typeDHeader->payload_steerorder_offset;
        size_t j_inv;

        // Add timestamp
        PTPTimestamp_t ptpTS((uint8_t *) &typeDHeader->timestamp);
        mFrame->timestamps.push_back(ptpTS.asTimespec());

        if (verboseTimestamps) {
            char timeString[52];
            snprintf(timeString, sizeof(timeString), "%jd.%09ld", (intmax_t)mFrame->timestamps.front().tv_sec, mFrame->timestamps.front().tv_nsec);
            std::cout << "typeDHeader->timestamp: " << timeString << std::endl;
        }

        // Get payload pointer, and the offset+stride to extract just range
        uint8_t* payload = (uint8_t *) typeDHeader->get_payloads_TypeDReturns();
        uint8_t *payloadEnd = (uint8_t *) (typeDHeader->get_payload_TypeDReturn(typeDHeader->payload_count_TypeDReturns - 1) + 1);
        payloadEnd = min(payloadEnd, (uint8_t *)packetBuffer + packetBufferUsed);

        int stride = TypeDReturn_t::read_len;

        for(uint8_t *curr = payload; curr < payloadEnd; curr += stride) {
            TypeDReturn_t *currReturn = (TypeDReturn_t*)curr;
            boost::endian::big_uint16_t *currRange = (boost::endian::big_uint16_t *)(&(currReturn->range));
            boost::endian::big_uint16_t *currSignal = (boost::endian::big_uint16_t *)(&(currReturn->intensity));
            boost::endian::big_uint16_t *currSNR = (boost::endian::big_uint16_t *)(&(currReturn->snr));
            boost::endian::big_uint16_t *currNoise = (boost::endian::big_uint16_t *)(&(currReturn->background));

            if (currReturn->isSet_returnPresentAndValid())
            {
                // Range checks (note that we won't check/fail on points without valid flag)

                // Drop all (i, j) outside of ([0, complete_size_stare_dim), [0, complete_size_stare))
                if (i >= typeDHeader->complete_size_stare_dim)
                {
                    *errV << "return data at/over end-of-line bound (i)" << endl;
                    break;
                }
                // Note that we never need to wrap i and increment j ourselves -- a fresh packet will come with a fresh j
                if (j >= typeDHeader->complete_size_steer_dim)
                {
                    *errV << "return data at/over last line bound (j)" << endl;
                    break;
                }

                if (u >= dimStare)
                {
                    *errV << "return data at/over line bounds end-of-line bound (u)" << endl;
                    break;
                }
                if (v >= dimSteer)
                {
                    *errV << "return data at/over last line bound (v)" << endl;
                    break;
                }

                // Only output debug spew on present points
                *errV << "(u=" << u << ", v=" << v << ")";

                if (mappingTable && (u < mappingTable_dimStare) && (v < mappingTable_dimSteer))
                    *errV << "=>(" << mappingTable[u + v * mappingTable_dimStare].theta
                          << "," << mappingTable[u + v * mappingTable_dimStare].phi << ")";
                else
                    *errV << "[Unmapped]";

                *errV << " = {";

                j_inv = typeDHeader->complete_size_steer_dim - j - 1; // flipud

                size_t mappingTableIndex = u + v * mappingTable_dimStare;
                size_t frameNumPoints = mappingTable_dimStare * mappingTable_dimSteer;
                if (currReturn->isSet_rangePresentAndValid() && mappingTableIndex < frameNumPoints) {
                    float range = ((float) *currRange) / 1024.0;
                    rFrame->range[i + j_inv * typeDHeader->complete_size_stare_dim] = range;
                    rFrame->x[i + j_inv * typeDHeader->complete_size_stare_dim] = range * std::cos(mappingTable[mappingTableIndex].phi) * std::sin(mappingTable[mappingTableIndex].theta);
                    rFrame->y[i + j_inv * typeDHeader->complete_size_stare_dim] = range * std::sin(mappingTable[mappingTableIndex].phi) * std::sin(mappingTable[mappingTableIndex].theta);
                    rFrame->z[i + j_inv * typeDHeader->complete_size_stare_dim] = range * std::cos(mappingTable[mappingTableIndex].theta);
                    mFrame->range = true;
                }   
                else
                {
                    rFrame->range[i + j_inv * typeDHeader->complete_size_stare_dim] = 0.0;
                    rFrame->x[i + j_inv * typeDHeader->complete_size_stare_dim] = 0.0;
                    rFrame->y[i + j_inv * typeDHeader->complete_size_stare_dim] = 0.0;
                    rFrame->z[i + j_inv * typeDHeader->complete_size_stare_dim] = 0.0;
                }
                
                if (currReturn->isSet_intensityPresentAndValid()) {
                    iFrame->signal[i + j_inv * typeDHeader->complete_size_stare_dim] = *currSignal;
                    mFrame->signal = true;
                }
                else
                    iFrame->signal[i + j_inv * typeDHeader->complete_size_stare_dim] = 0;

                if (currReturn->isSet_snrPresentAndValid()) {
                    iFrame->SNR[i + j_inv * typeDHeader->complete_size_stare_dim] = *currSNR;
                    mFrame->SNR = true;
                }
                else
                    iFrame->SNR[i + j_inv * typeDHeader->complete_size_stare_dim] = 0;

                if (currReturn->isSet_backgroundPresentAndValid()) {
                    iFrame->noise[i + j_inv * typeDHeader->complete_size_stare_dim] = *currNoise;
                    mFrame->noise = true;
                }
                else
                    iFrame->noise[i + j_inv * typeDHeader->complete_size_stare_dim] = 0;

                *errV <<  "<END>}" << endl;
            }

            // Dont' wrap (u, v). Will complain on next time through loop if out of bounds.
            u += typeDHeader->payload_step_u;

            // Dont' wrap (i, j), but drop all (i, j) outside of ([0, complete_size_stare_dim), [0, complete_size_stare))
            // during next run through loop
            i++;

            // Note that we never need to wrap i and increment j ourselves -- a fresh packet will come with a fresh j

        }

        // Is this the last packet in the frame?
        if(seq == currentSceneEndSeq) {
            *errS << "Frame complete." << endl;
            return 1;
        }

        // Success, but frame not yet done
        return 0;
}


int handleTypeC(TypeCHeader_t *typeCHeader, uint32_t seq) {
        // Re-allocate mapping table if needed
        mappingTable_dimStare = max((size_t) typeCHeader->image_end_u + 1, mappingTable_dimStare);
        mappingTable_dimSteer = max((size_t) typeCHeader->image_end_v + 1, mappingTable_dimSteer);
        size_t frameNumPoints =  mappingTable_dimStare * mappingTable_dimSteer;
        if(frameNumPoints > mappingTable_numPoints) {
            //*errS << "Type-C (Mapping Table Update) triggered buffer resize to " << frameNumPoints * sizeof(depthBuffer_t)
            //      << " bytes ";
            *errS << "(" << mappingTable_dimStare << "x" << mappingTable_dimSteer << ")" << endl;
            if(mappingTable)
                delete mappingTable;
            mappingTable = new mapping_table_entry_t[frameNumPoints];
            mappingTable_numPoints = frameNumPoints;
        }

        // For now, only support a single parameter type for the mapping table
        uint8_t paramType =  typeCHeader->parameter_type;
        if(paramType != (int) TypeCHeader_t::TypeCParameterTypeCode::coordinateMap32ASTheta32ASPhi) {
                *errV << "Unsupported Type-C (Mapping Table Update) parameter type, ignoring." << endl;
                return 0;
        }
        
        // Get payload pointer and stride for single parameter
        uint8_t* payload = (uint8_t *) typeCHeader->get_nextSection_TypeCParameters()->raw_data;
        uint8_t* payloadEnd = (uint8_t *) packetBuffer + packetBufferUsed;
        int stride = sizeof(TypeCParameter_CM32T32P_t); 

        // Copy Mapping Table Data into Buffer
        size_t u = typeCHeader->payload_start_u;
        size_t v = typeCHeader->payload_start_v;

        *errV << "typeCHeader->payload_start_u: " << typeCHeader->payload_start_u << endl;
        *errV << "typeCHeader->payload_start_v: " << typeCHeader->payload_start_v << endl;
        *errV << "typeCHeader->image_end_u: " << typeCHeader->image_end_u << endl;
        *errV << "typeCHeader->image_end_v: " << typeCHeader->image_end_v << endl;

        for (uint8_t *curr = payload; curr < payloadEnd; curr += stride)
        {
            // Note that Type-C packets *never wrap* (as Type-4 would), but don't have a valid flag.
            // We simply discard any portion of the packet that extends beyond valid indicies.
            if ((u >= mappingTable_dimStare) || (v >= mappingTable_dimSteer))
                break;

            *errV << "(u=" << u << ",v=" << v << ") => ";
            TypeCParameter_CM32T32P_t *entry = (TypeCParameter_CM32T32P_t *)curr;
            mappingTable[u + v * mappingTable_dimStare].theta = arcSecToRadian_*float(entry->theta);
            mappingTable[u + v * mappingTable_dimStare].phi = arcSecToRadian_*float(entry->phi);

            *errV << "(" << entry->theta << ", " << entry->phi << ")" << endl;

            // Never wrap (u, v) for Type-3
            u++;
        }

        // Success
        return 0;
}
