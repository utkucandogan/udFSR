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

#include "FsrNetwatch.h"
#include "Fsr.h"

namespace udFSR {
namespace fsr {

Define_Module(FsrNetwatch);

void FsrNetwatch::initialize()
{
    updateInterval = par("updateInterval");

    updateTimer = new omnetpp::cMessage("updateTimer");
    scheduleAfter(updateInterval, updateTimer);

    for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
        auto submodule = *it;
        if (strcmp(submodule->getNedTypeName(), "udFSR.fsr.FsrHost") == 0) {
            ++numOfNodes;
        }
    }

    averageDegreeSignal = registerSignal("averageDegree");
    recordScalar("numOfNodes", numOfNodes);
}

void FsrNetwatch::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == updateTimer) {
        handleAverageDegree();
        scheduleAfter(updateInterval, updateTimer);
    }
}

void FsrNetwatch::handleAverageDegree()
{
    totalDegree = 0;
    for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
        auto submodule = *it;
        if (strcmp(submodule->getNedTypeName(), "udFSR.fsr.FsrHost") == 0) {
            auto fsr = omnetpp::check_and_cast<Fsr*>(submodule->getSubmodule("fsr"));
            totalDegree += fsr->topology[fsr->getSelfIPAddress()].neighbors.size();
        }
    }

    averageDegree = totalDegree / numOfNodes;
    emit(averageDegreeSignal, averageDegree);
}

FsrNetwatch::FsrNetwatch()
{

}

FsrNetwatch::~FsrNetwatch()
{
    cancelEvent(updateTimer);
    delete updateTimer;
}

} /* namespace fsr */
} /* namespace udFSR */
