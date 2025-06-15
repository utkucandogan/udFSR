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

#ifndef FSR_FSR_H_
#define FSR_FSR_H_

#include <map>
#include <set>
#include <vector>

#include <inet/routing/base/RoutingProtocolBase.h>
#include <inet/networklayer/common/L3Address.h>
#include <inet/networklayer/contract/IRoutingTable.h>
#include <inet/networklayer/contract/IInterfaceTable.h>
#include "inet/transportlayer/contract/udp/UdpSocket.h"

#include "FsrPackets_m.h"

namespace udFSR {
namespace fsr {

class Fsr : public inet::RoutingProtocolBase, public inet::UdpSocket::ICallback
{
protected:
    friend class FsrNetwatch;

    inet::UdpSocket socket;
    unsigned int udpPort = 0;
    inet::L3Address selfIPAddr;

    inet::ModuleRefByPar<inet::IRoutingTable> routingTable;
    inet::ModuleRefByPar<inet::IInterfaceTable> interfaceTable;

    std::vector<int> scopes;
    omnetpp::simtime_t helloInterval;
    omnetpp::simtime_t updateInterval;

    struct TopologyInfo
    {
        unsigned int seqNum;
        omnetpp::simtime_t lastHeard;
        std::set<inet::L3Address> neighbors;
    };

    std::map<inet::L3Address, TopologyInfo> topology;
    size_t scopeIndex = 0;

    void initialize(int stage) override;
    void handleMessageWhenUp(omnetpp::cMessage *msg) override;

    /* Control Packet handlers */
    //void handleHello(inet::L3Address addr, );

    /* FSR packet handlers */
    void sendHello();
    void sendUpdate();
    void receiveHello(const inet::Ptr<Hello>& hello);
    void receiveUpdate(const inet::Ptr<Update>& update, int timeToLive);

    /* Routing functions */
    void clearRoutes();
    void updateRoutes();

    /* Helper functions */
    inet::L3Address getSelfIPAddress() const;
    void clearState();
    bool staleHello();
    bool staleUpdate();
    void sendPacket(const inet::Ptr<FsrPacket>& fsrPacket, int timeToLive);

    /* self messages */
    omnetpp::cMessage* helloTimer  = nullptr;
    omnetpp::cMessage* updateTimer = nullptr;

    /* UDP callback interface */
    void socketDataArrived(inet::UdpSocket *socket, inet::Packet *packet) override;
    void socketErrorArrived(inet::UdpSocket *socket, inet::Indication *indication) override;
    void socketClosed(inet::UdpSocket *socket) override;

    /* Lifecycle */
    void handleStartOperation(inet::LifecycleOperation *operation) override;
    void handleStopOperation(inet::LifecycleOperation *operation) override;
    void handleCrashOperation(inet::LifecycleOperation *operation) override;

public:
    static constexpr inet::IRoute::SourceType FSR_ROUTE_TYPE = inet::IRoute::SourceType::MANUAL;

    Fsr();
    virtual ~Fsr();
};

} /* namespace fsr */
} /* namespace udFSR */

#endif /* FSR_FSR_H_ */
