﻿/* ARDrone Control .NET - An application for flying the Parrot AR drone in Windows.
 * Copyright (C) 2010, 2011 Thomas Endres
 * 
 * This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using ARDrone.Control.Data;

namespace ARDrone.Control.Commands
{
    public class SetControlModeCommand : Command
    {
        private DroneControlMode mode;

        public SetControlModeCommand(DroneControlMode mode)
            : base()
        {
            this.mode = mode;
        }

        public override String CreateCommand(SupportedFirmwareVersion firmwareVersion)
        {
            CheckSequenceNumber();
            return String.Format("AT*CTRL={0},{1},0\r", sequenceNumber, (int)mode);
        }
    }

    public enum DroneControlMode
    {
        IdleMode = 0,
        SoftwareUpdateReceptionMode,
        PicSoftwareUpdateReceptionMode,
        ControlMode,
        LogControlMode,
    }
}
