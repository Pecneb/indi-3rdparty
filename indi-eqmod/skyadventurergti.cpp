/*
    Copyright(c) 2022 Jasem Mutlaq. All rights reserved.

    Sky Adventurer GTi

    INDI driver is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    INDI driver is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Skywatcher Protocol INDI driver.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "skyadventurergtibase.h"
#include <memory>

static std::unique_ptr<SkyAdventurerGTIBase> azgti(new SkyAdventurerGTIBase());
