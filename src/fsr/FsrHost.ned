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

import udFSR.fsr.Fsr;
import inet.node.inet.AdhocHost;

module FsrHost extends AdhocHost
{
    submodules:
        fsr: Fsr {
            @display("p=250,100");
        }
        
	connections:
        fsr.socketOut --> at.in++;
        fsr.socketIn <-- at.out++;
}
