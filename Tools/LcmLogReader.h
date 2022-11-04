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

#ifndef LcmLogReader_H_
#define LcmLogReader_H_

#include "../Core/Utils/Resolution.h"
#include "../Core/Utils/Stopwatch.h"
#include <pangolin/utils/file_utils.h>

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>

namespace lcm {
  class LogFile;
}


class LcmLogReader : public LogReader
{
    public:
        LcmLogReader(std::string file, bool flipColors);

        virtual ~LcmLogReader();

        void getNext();

        bool safeGetNext();

        void getBack();

        int getNumFrames();

        bool hasMore();

        bool rewound();

        void rewind();

        void fastForward(int frame);

        const std::string getFile();

        void setAuto(bool value);

        std::stack<int> filePointers;

    protected:
        lcm::LogFile* logFile;

    private:
        void getCore();
};

#endif /* LcmLogReader_H_ */
