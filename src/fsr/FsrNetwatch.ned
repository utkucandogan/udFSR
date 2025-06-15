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

simple FsrNetwatch
{
    parameters:
        @class("fsr::FsrNetwatch");
        @display("i=block/table");
        double updateInterval @unit(s) = default(1s);
        bool drawNetwork = default(false);
        
        @signal[averageDegree](type=int);
        @statistic[averageDegree](title="Average Degree"; source=averageDegree; record=vector);
        
        @signal[averageControlPacketCount](type=double);
        @statistic[averageControlPacketCount](title="Average Control Packet Count"; source=averageControlPacketCount; record=vector);
        
        @signal[averageControlBitsCount](type=double);
        @statistic[averageControlBitsCount](title="Average Control Bits Count"; source=averageControlBitsCount; record=vector);
}
