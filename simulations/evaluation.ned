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

package udFSR.simulations;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.contract.INetworkNode;
import inet.physicallayer.wireless.common.contract.packetlevel.IRadioMedium;
import inet.visualizer.contract.IIntegratedVisualizer;
import inet.environment.common.PhysicalEnvironment;

import udFSR.fsr.FsrHost;
import udFSR.fsr.FsrNetwatch;

network Evaluation
{
    parameters:
        @display("bgb=1000,1000;bgg=100,1,grey95");
        @figure[title](type=label; pos=0,-1; anchor=sw; color=darkblue);

        @figure[rcvdPkText](type=indicatorText; pos=500,50; anchor=w; font=,36; textFormat="packets received: %g"; initialValue=0);
        @statistic[packetReceived](source=host[1].app[0].packetReceived; record=figure(count); targetFigure=rcvdPkText);
        
        int numOfHosts = default(10);

    submodules:
        visualizer: <default(firstAvailableOrEmpty("IntegratedCanvasVisualizer"))> like IIntegratedVisualizer if typename != "" {
            @display("p=-150,100");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=-150,300");
        }
        radioMedium: <default("UnitDiskRadioMedium")> like IRadioMedium {
            @display("p=-150,500");
        }
        fsrNetwatch: FsrNetwatch {
            @display("p=-150,700");
        }
        physicalEnvironment: PhysicalEnvironment {
            @display("p=-150,900");
        }
        host[numOfHosts]: FsrHost;
}
