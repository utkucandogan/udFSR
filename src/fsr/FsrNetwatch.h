//
// udFSR : Fisheye State Routing (FSR) on OmNet++
//
// Copyright (C) 2025 Utkucan Doğan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#ifndef FSR_FSRNETWATCH_H_
#define FSR_FSRNETWATCH_H_

#include <omnetpp.h>

namespace udFSR {
namespace fsr {

class FsrNetwatch : public omnetpp::cSimpleModule
{
protected:
    omnetpp::simtime_t updateInterval;

    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void handleAverageDegree();

    omnetpp::cMessage* updateTimer;

    int numOfNodes = 0;
    int totalDegree = 0;
    int averageDegree = 0;

    omnetpp::simsignal_t averageDegreeSignal;

public:
    FsrNetwatch();
    virtual ~FsrNetwatch();
};

} /* namespace fsr */
} /* namespace udFSR */

#endif /* FSR_FSRNETWATCH_H_ */
