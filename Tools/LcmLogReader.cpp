/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "LcmLogReader.h"

#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>


LcmLogReader::LcmLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    assert(pangolin::FileExists(file.c_str()));


    logFile = new lcm::LogFile(file, "r");

    fp = logFile->getFilePtr();

    currentFrame = 0;

    numFrames = 10000;
    next_exists = true;

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[numPixels * 2];
    decompressionBufferImage = new Bytef[numPixels * 3];
}

LcmLogReader::~LcmLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    delete logFile;
}

void LcmLogReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore();
}

void LcmLogReader::getNext()
{
    filePointers.push(ftell(fp));

    getCore();
}

bool LcmLogReader::safeGetNext()
{
    getNext();
    return next_exists;
}

void LcmLogReader::getCore()
{
    std::string channel = "";
    const lcm::LogEvent* event = 0;

    while (channel != "OPENNI_FRAME") {
        event = logFile->readNextEvent();
        if (event==NULL) {
            std::cout << "LcmLogReader found the end of the file." << std::endl;
            next_exists = false;
            return;
        }
        channel = event->channel;
        //std::cout << "read event on channel: " << channel << std::endl;
    }

    bot_core::images_t message;
    message.decode(event->data, 0, event->datalen);
    timestamp = message.utime;

    // std::cout << "timestamp: " << timestamp << std::endl;
    // std::cout << "frame: " << currentFrame << std::endl;

    bot_core::image_t colorImage = message.images[0];
    bot_core::image_t depthImage = message.images[1];


    bool isZlibCompressed = false;

    if (depthImage.pixelformat == bot_core::image_t::PIXEL_FORMAT_INVALID)
    {
      isZlibCompressed = true;
    }

    depthSize = depthImage.size;
    imageSize = colorImage.size;


/*
    auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
    assert(tmp);
    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);
    if(imageSize > 0)
    {
        tmp = fread(imageReadBuffer,imageSize,1,fp);
        assert(tmp);
    }
*/

    if(depthSize == numPixels * 2)
    {
       // printf("copying depth image\n");
        memcpy(&decompressionBufferDepth[0], depthImage.data.data(), numPixels * 2);
    }
    else
    {
      // printf("uncompress depth image\n");
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthImage.data.data(), depthSize);
    }

    if(imageSize == numPixels * 3)
    {
      //  printf("copy color image\n");
        memcpy(&decompressionBufferImage[0], colorImage.data.data(), numPixels * 3);
    }
    else if(imageSize > 0)
    {
      //  printf("jpeg read color image\n");

        jpeg.readData(colorImage.data.data(), imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)&decompressionBufferImage[0];

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
    //return true;
}

void LcmLogReader::fastForward(int frame)
{
  printf("LcmLogReader::fastForward not implemented\n");
/*
    while(currentFrame < frame && hasMore())
    {
        filePointers.push(ftell(fp));
        auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
        assert(tmp);
        tmp = fread(&depthSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(&imageSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(depthReadBuffer,depthSize,1,fp);
        assert(tmp);
        if(imageSize > 0)
        {
            tmp = fread(imageReadBuffer,imageSize,1,fp);
            assert(tmp);
        }
        currentFrame++;
    }
*/
}

int LcmLogReader::getNumFrames()
{
    return numFrames;
}

bool LcmLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void LcmLogReader::rewind()
{
    if (filePointers.size() != 0)
    {
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }
/*
    fclose(fp);
    fp = fopen(file.c_str(), "rb");
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    assert(tmp);
    currentFrame = 0;
*/

    printf("LcmLogReader::rewind\n");
    /*
    delete logFile;
    logFile = new lcm::LogFile(file, "r");
    fp = logFile->getFilePtr();
    */

    currentFrame = 0;
}

bool LcmLogReader::rewound()
{
    return filePointers.size() == 0;
}

const std::string LcmLogReader::getFile()
{
    return file;
}

void LcmLogReader::setAuto(bool value)
{

}
