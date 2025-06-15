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

package udFSR.fsr;

simple Fsr
{
    parameters:
        @class("fsr::Fsr");
        @display("i=block/routing");
        
        @signal[controlPacketCount](type=int);
        @signal[controlBitsCount](type=int);
        
        string routingTableModule = default("^.ipv4.routingTable");
        string interfaceTableModule = default("^.interfaceTable");
        
        string scopes = default("1 2 -1");
        double helloInterval @unit(s)  = default(1s);
        double updateInterval @unit(s) = default(5s);
        int udpPort = default(654);
        string interface = default("wlan0");
        double clockOffset @unit(s)  = default(uniform(0s, helloInterval));
        
    gates:
        input socketIn;
        output socketOut;
}
