// Copyright (C) 2021, 2022 Lumotive
// Copyright 2023 HOKUYO AUTOMATIC CO.,LTD.

#ifndef YLM_DATA_TYPES_H
#define YLM_DATA_TYPES_H

#include <vector>

struct metadataFrame
{
    // these fields should be used to reference the mapping table
    size_t frameNumPoints;
    size_t imgStareSize;
    size_t imgSteerSize;
    std::vector<struct timespec> timestamps;
    bool range;
    bool signal;
    bool noise;
    bool SNR;

    metadataFrame(size_t frameNumPoints, size_t imgStareSize, size_t imgSteerSize):
    frameNumPoints(frameNumPoints),
    imgStareSize(imgStareSize),
    imgSteerSize(imgSteerSize)
    {
        range = false;
        signal = false;
        noise = false;
        SNR = false;
        // rawdata = false;
    }

    inline void resize(size_t frameNumPoints, size_t imgStareSize, size_t imgSteerSize)
    {
        frameNumPoints = frameNumPoints;
        imgStareSize = imgStareSize;
        imgSteerSize = imgSteerSize;

        range = false;
        signal = false;
        noise = false;
        SNR = false;
        // rawdata = false;
    }

    inline size_t get_image_stare_size(void) const { return imgStareSize; }
    inline size_t get_image_steer_size(void) const { return imgSteerSize; }
};

struct rangeFrame
{
    std::vector<float> range;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    rangeFrame(size_t frameNumPoints) { resize(frameNumPoints); }
    inline void resize(size_t frameNumPoints) 
    {
        range.resize(frameNumPoints);
        x.resize(frameNumPoints);
        y.resize(frameNumPoints); 
        z.resize(frameNumPoints); 
    }
};

struct intensityFrame
{
    std::vector<uint16_t> signal;
    std::vector<uint16_t> noise;
    std::vector<float> SNR;
    intensityFrame(size_t frameNumPoints) { resize(frameNumPoints); }
    inline void resize(size_t frameNumPoints) 
    { 
        signal.resize(frameNumPoints);
        noise.resize(frameNumPoints); 
        SNR.resize(frameNumPoints); 
    }
};

// Output accessors
std::shared_ptr<metadataFrame> get_metadata_frame();
std::shared_ptr<rangeFrame> get_range_frame();
std::shared_ptr<intensityFrame> get_intensity_frame();


#endif