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

import inet.common.packet.chunk.Chunk;
import inet.networklayer.common.L3Address;


namespace udFSR::fsr;

enum FsrPacketType
{
    HELLO = 1;
    UPDATE = 2;
};

class FsrPacket extends inet::FieldsChunk
{
    FsrPacketType packetType = static_cast<FsrPacketType>(-1);
    inet::L3Address origin;
    unsigned int seqNum;
}


class Hello extends FsrPacket {

}

class Update extends FsrPacket {
    inet::L3Address neighbors[];
}
