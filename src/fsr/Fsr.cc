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

#include "Fsr.h"

#include <string>
#include <queue>

#include <inet/common/Units.h>
#include <inet/linklayer/common/InterfaceTag_m.h>
#include <inet/networklayer/common/L3AddressResolver.h>
#include <inet/networklayer/contract/IL3AddressType.h>
#include <inet/networklayer/common/HopLimitTag_m.h>
#include <inet/networklayer/common/L3AddressTag_m.h>
#include <inet/transportlayer/common/L4PortTag_m.h>

namespace udFSR {
namespace fsr {

Define_Module(Fsr);

void Fsr::initialize(int stage)
{
    inet::RoutingProtocolBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        routingTable.reference(this, "routingTableModule", true);
        interfaceTable.reference(this, "interfaceTableModule", true);

        // Read parameters from the .ini / NED file
        helloInterval = par("helloInterval");
        updateInterval = par("updateInterval");
        udpPort = par("udpPort");

        controlPacketCountSignal = registerSignal("controlPacketCount");
        controlBitsCountSignal = registerSignal("controlBitsCount");

        auto scopeStr = std::istringstream(par("scopes").stdstringValue());

        for (std::string token ; std::getline(scopeStr, token, ' ') ;) {
            int value = std::stoi(token);
            scopes.push_back(value);
        }

        helloTimer  = new omnetpp::cMessage("HelloTimer");
        updateTimer = new omnetpp::cMessage("UpdateTimer");
    }
}

void Fsr::handleMessageWhenUp(omnetpp::cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg == helloTimer) {
            sendHello();
        } else if (msg == updateTimer) {
            sendUpdate();
        }
    } else {
        socket.processMessage(msg);
    }
}

/* Self message handlers */
void Fsr::sendHello()
{
    // Stale the connections every time we send a hello packet
    if (staleHello()) {
        updateRoutes();
    }

    auto& seqNum = topology[getSelfIPAddress()].seqNum;

    auto hello = inet::makeShared<Hello>();
    hello->setPacketType(FsrPacketType::HELLO);
    hello->setOrigin(getSelfIPAddress());
    hello->setSeqNum(++seqNum);

    auto usingIpv6 = (getSelfIPAddress().getType() == inet::L3Address::IPv6);
    hello->setChunkLength(usingIpv6 ? inet::B(24) : inet::B(12));

    sendPacket(hello, 1);

    scheduleAfter(helloInterval, helloTimer);
}

void Fsr::sendUpdate()
{
    // Stale the connections every time we send a update packet
    if (staleUpdate()) {
        updateRoutes();
    }

    auto& seqNum = topology[getSelfIPAddress()].seqNum;

    auto update = inet::makeShared<Update>();
    update->setPacketType(FsrPacketType::UPDATE);
    update->setOrigin(getSelfIPAddress());
    update->setSeqNum(++seqNum);

    auto& neighbors = topology[getSelfIPAddress()].neighbors;

    update->setNeighborsArraySize(neighbors.size());
    size_t i = 0;
    for (auto addr : neighbors) {
        update->setNeighbors(i++, addr);
    }

    auto usingIpv6 = (getSelfIPAddress().getType() == inet::L3Address::IPv6);
    update->setChunkLength(usingIpv6 ? inet::B(24 + 16 * neighbors.size()) : inet::B(12 + 4 * neighbors.size()));

    sendPacket(update, scopes[scopeIndex]);

    scheduleAfter(updateInterval, updateTimer);

    scopeIndex = (scopeIndex + 1) % scopes.size();
}

void Fsr::receiveHello(const inet::Ptr<Hello>& hello)
{
    auto addr = hello->getOrigin();

    auto& neighbors = topology[getSelfIPAddress()].neighbors;

    bool updated = false;
    auto result1 = neighbors.insert(addr);
    updated |= result1.second;

    auto result2 = topology.try_emplace(addr);
    updated |= result2.second;

    if (updated) updateRoutes();
}

void Fsr::receiveUpdate(const inet::Ptr<Update>& update, int timeToLive)
{
    auto addr = update->getOrigin();

    bool updated = false;
    std::set<inet::L3Address> newNeighbors;
    for (size_t i = 0 ; i < update->getNeighborsArraySize() ; ++i) {
        auto n = update->getNeighbors(i);
        newNeighbors.insert(n);

        auto result = topology.try_emplace(n);
        updated |= result.second;
    }

    if (topology[addr].neighbors != newNeighbors) {
        topology[addr].neighbors = std::move(newNeighbors);
        updated |= true;
    }

    if (updated) updateRoutes();

    if (timeToLive != 0) {
        sendPacket(update, timeToLive);
    }
}

/* Routing functions */
void Fsr::clearRoutes()
{
    std::vector<inet::IRoute*> toDelete;
    for (size_t i = 0 ; i < routingTable->getNumRoutes() ; ++i) {
        auto route = routingTable->getRoute(i);
        if (route->getSourceType() == FSR_ROUTE_TYPE)
            toDelete.push_back(route);
    }
    for (auto route : toDelete)
        routingTable->removeRoute(route);
}

void Fsr::updateRoutes()
{
    // Clear the routes at the start
    clearRoutes();

    auto selfAddr = getSelfIPAddress();
    auto addrType = selfAddr.getAddressType();

    // First run Dijkstra
    std::map<inet::L3Address, int> distance;
    std::map<inet::L3Address, inet::L3Address> previous;
    std::set<inet::L3Address> visited;

    auto comp = [&distance](const inet::L3Address& a, const inet::L3Address& b){
        return distance[a] > distance[b];
    };

    std::priority_queue<inet::L3Address, std::vector<inet::L3Address>, decltype(comp)> vertices(comp);

    // Initialize
    for (const auto& [addr, _] : topology) {
        distance[addr] = (addr == selfAddr) ? 0 : std::numeric_limits<int>::max();
    }

    vertices.push(selfAddr);

    // Main Dijkstra algorithm
   while (!vertices.empty()) {
       auto u = vertices.top();
       vertices.pop();

       if (visited.count(u)) continue;

       visited.insert(u);

       for (const auto& v : topology[u].neighbors) {
           int alt = distance[u] + 1;  // weight is hop count
           if (alt < distance[v]) {
               distance[v] = alt;
               previous[v] = u;
               vertices.push(v);
           }
       }
   }

   // Add routes
   for (const auto& [addr, dist] : distance) {
       if (addr == selfAddr || dist == std::numeric_limits<int>::max())
           continue;

       auto nextHop = addr;
       while (previous.count(nextHop) && previous[nextHop] != selfAddr)
           nextHop = previous[nextHop];

       // Create a route object
       auto route = routingTable->createRoute();
       route->setDestination(addr);
       route->setNextHop(nextHop);
       route->setPrefixLength(addrType->getMaxPrefixLength());
       route->setMetric(dist);
       auto ifEntry = interfaceTable->findInterfaceByName(par("interface"));
       if (ifEntry) route->setInterface(ifEntry);
       route->setSourceType(FSR_ROUTE_TYPE);
       route->setSource(this);

       routingTable->addRoute(route);
   }

   /* Print routes */
   inet::L3AddressResolver resolver;
   auto myName = getParentModule()->getFullName();

   for (size_t i = 0 ; i < routingTable->getNumRoutes() ; ++i) {
       auto route = routingTable->getRoute(i);
       if (route->getSourceType() != FSR_ROUTE_TYPE) continue;

       auto dest = resolver.findHostWithAddress(route->getDestinationAsGeneric())->getFullName();
       auto hop  = resolver.findHostWithAddress(route->getNextHopAsGeneric())->getFullName();

       EV_INFO << "[" << myName << "] " << "Route to: \"" << dest << "\" via \"" << hop << "\" metric: " << route->getMetric() << std::endl;
   }
}

/* Helper functions */
inet::L3Address Fsr::getSelfIPAddress() const
{
    return routingTable->getRouterIdAsGeneric();
}

void Fsr::clearState()
{
    cancelEvent(helloTimer);
    cancelEvent(updateTimer);
}

bool Fsr::staleHello()
{
    std::vector<inet::L3Address> toRemove;
    for (auto addr : topology[getSelfIPAddress()].neighbors) {
        // Times four is arbitrary
        if ((inet::simTime() - topology[addr].lastHeard) > helloInterval * 4) {
            // Connection is stale, remove from the neighbors
            toRemove.push_back(addr);
        }
    }

    for (auto& addr : toRemove) {
        topology[getSelfIPAddress()].neighbors.erase(addr);
        topology[addr].neighbors.erase(getSelfIPAddress());
    }

    return !toRemove.empty();
}

bool Fsr::staleUpdate()
{
    std::vector<inet::L3Address> toRemove;
    for (auto& [addr, info] : topology) {
        if (addr == getSelfIPAddress()) continue;

        // Times four is arbitrary
        if ((inet::simTime() - info.lastHeard) > updateInterval * scopes.size() * 4) {
            // Connection is stale, remove from the neighbors
            toRemove.push_back(addr);
        }
    }

    for (auto& addr : toRemove) {
        topology.erase(addr);
        for (auto& [addr, info] : topology) {
            info.neighbors.erase(addr);
        }
    }

    return !toRemove.empty();
}

const char* FsrPacketName(FsrPacketType type)
{
    switch (type) {
        case FsrPacketType::HELLO: return "FsrHello";
        case FsrPacketType::UPDATE: return "FsrUpdate";
        default: return "FsrUnknown";
    }
}

void Fsr::sendPacket(const inet::Ptr<FsrPacket>& fsrPacket, int timeToLive)
{
    ++controlPacketCount;
    controlBitsCount += fsrPacket->getChunkLength().get() * 8;

    emit(controlPacketCountSignal, controlPacketCount);
    emit(controlBitsCountSignal, controlBitsCount);

    auto packet = new inet::Packet(FsrPacketName(fsrPacket->getPacketType()), fsrPacket);
    int interfaceId = CHK(interfaceTable->findInterfaceByName(par("interface")))->getInterfaceId();

    packet->addTag<inet::InterfaceReq>()->setInterfaceId(interfaceId);
    packet->addTag<inet::HopLimitReq>()->setHopLimit(timeToLive);
    packet->addTag<inet::L3AddressReq>()->setDestAddress(getSelfIPAddress().getAddressType()->getBroadcastAddress());
    packet->addTag<inet::L4PortReq>()->setDestPort(udpPort);

    socket.send(packet);
}

/* UDP callback interface */
void Fsr::socketDataArrived(inet::UdpSocket *socket, inet::Packet *packet)
{
    auto arrivalTime = packet->getArrivalTime();
    auto arrivalTtl  = packet->getTag<inet::HopLimitInd>()->getHopLimit() - 1;
    const auto& fsrPacket = packet->popAtFront<FsrPacket>();

    auto addr = fsrPacket->getOrigin();
    auto seqNum = fsrPacket->getSeqNum();

    if (seqNum > topology[addr].seqNum) {
        topology[addr].seqNum = seqNum;
        topology[addr].lastHeard = arrivalTime;

        switch (fsrPacket->getPacketType()) {
            case FsrPacketType::HELLO:
                receiveHello(CHK(inet::dynamicPtrCast<Hello>(fsrPacket->dupShared())));
                break;
            case FsrPacketType::UPDATE:
                receiveUpdate(CHK(inet::dynamicPtrCast<Update>(fsrPacket->dupShared())), arrivalTtl);
                break;
        }
    }
    delete packet;
}

void Fsr::socketErrorArrived(inet::UdpSocket *socket, inet::Indication *indication)
{
    EV_WARN << "Ignoring UDP error report " << indication->getName() << std::endl;
    delete indication;
}

void Fsr::socketClosed(inet::UdpSocket *socket)
{
    // Currently does nothing
}

/* Lifecycle */
void Fsr::handleStartOperation(inet::LifecycleOperation *operation)
{
    socket.setOutputGate(gate("socketOut"));
    socket.bind(udpPort);
    socket.setCallback(this);
    socket.setBroadcast(true);

    omnetpp::simtime_t clockOffset = par("clockOffset");

    scheduleAfter(clockOffset + helloInterval, helloTimer);
    scheduleAfter(clockOffset + updateInterval, updateTimer);

    auto myName = getParentModule()->getFullName();
    auto myIP = getSelfIPAddress().str();
    EV_INFO << "[" << myName << "] " << "My IP is: \"" << myIP << "\"" << std::endl;
}

void Fsr::handleStopOperation(inet::LifecycleOperation *operation)
{
    socket.close();
    clearState();
}

void Fsr::handleCrashOperation(inet::LifecycleOperation *operation)
{
    socket.destroy();
    clearState();
}

/* Constructor & Destructor */
Fsr::Fsr()
{

}

Fsr::~Fsr()
{
    clearState();
    delete helloTimer;
    delete updateTimer;
}

} /* namespace fsr */
} /* namespace udFSR */
