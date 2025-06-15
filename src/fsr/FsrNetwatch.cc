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

#include <inet/networklayer/common/L3AddressResolver.h>

namespace udFSR {
namespace fsr {

Define_Module(FsrNetwatch);

void FsrNetwatch::initialize()
{
    updateInterval = par("updateInterval");
    drawNetwork = par("drawNetwork");

    canvas = getParentModule()->getCanvas();

    updateTimer = new omnetpp::cMessage("updateTimer");
    scheduleAfter(updateInterval, updateTimer);

    for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
        auto submodule = *it;
        if (strcmp(submodule->getNedTypeName(), "udFSR.fsr.FsrHost") == 0) {
            ++numOfNodes;
        }
    }

    averageDegreeSignal = registerSignal("averageDegree");
    averageControlPacketCountSignal = registerSignal("averageControlPacketCount");
    averageControlBitsCountSignal = registerSignal("averageControlBitsCount");
    recordScalar("numOfNodes", numOfNodes);
}

void FsrNetwatch::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == updateTimer) {
        handleAverageDegree();
        if (drawNetwork) handleDrawNetwork();
        handleAverageContol();
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

void FsrNetwatch::handleDrawNetwork()
{
    clearConnections();

    inet::L3AddressResolver resolver;

    for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
        auto submodule = *it;
        if (strcmp(submodule->getNedTypeName(), "udFSR.fsr.FsrHost") == 0) {
            auto fsr = omnetpp::check_and_cast<Fsr*>(submodule->getSubmodule("fsr"));
            for (const auto& addr : fsr->topology[fsr->getSelfIPAddress()].neighbors) {
                drawConnection(submodule, resolver.findHostWithAddress(addr));
            }
        }
    }
}

void FsrNetwatch::handleAverageContol()
{
    averageControlPacketCount = 0;
    averageControlBitsCount = 0;

    for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
        auto submodule = *it;
        if (strcmp(submodule->getNedTypeName(), "udFSR.fsr.FsrHost") == 0) {
            auto fsr = omnetpp::check_and_cast<Fsr*>(submodule->getSubmodule("fsr"));
            averageControlPacketCount += fsr->controlPacketCount;
            averageControlBitsCount += fsr->controlBitsCount;
        }
    }

    averageControlPacketCount /= numOfNodes;
    averageControlBitsCount /= numOfNodes;

    emit(averageControlPacketCountSignal, averageControlPacketCount);
    emit(averageControlBitsCountSignal, averageControlBitsCount);
}

void FsrNetwatch::drawConnection(cModule* first, cModule* second)
{
    // Get this node's display position in the canvas
    auto firstX = first->getDisplayString().getTagArg("p", 0);
    auto firstY = first->getDisplayString().getTagArg("p", 1);
    if (!firstX || !firstY)
        return;

    double x1 = atof(firstX);
    double y1 = atof(firstY);

    // Get this node's display position in the canvas
    auto secondX = second->getDisplayString().getTagArg("p", 0);
    auto secondY = second->getDisplayString().getTagArg("p", 1);
    if (!secondX || !secondY)
        return;

    double x2 = atof(secondX);
    double y2 = atof(secondY);

    auto line = new omnetpp::cLineFigure("fsrConn");
    line->setStart(omnetpp::cFigure::Point(x1, y1));
    line->setEnd(omnetpp::cFigure::Point(x2, y2));
    line->setLineColor("gray");
    line->setLineWidth(2);

    canvas->addFigure(line);
    connectionLines.push_back(line);

    line->lowerToBottom();
}

void FsrNetwatch::clearConnections()
{
    for (auto line : connectionLines) {
        canvas->removeFigure(line);
        delete line;
    }
    connectionLines.clear();
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
