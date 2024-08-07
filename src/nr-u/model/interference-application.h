/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef INTERFERENCEAPPLICATION_H
#define INTERFERENCEAPPLICATION_H

#include "application-scenario.h"
#include <ns3/node-container.h>

namespace ns3 {

class InterferenceApplication : public ApplicationScenario
{
public:
  InterferenceApplication (const NodeContainer &firstSet, const NodeContainer &secondSet,
                           double firstStart, double secondStart, double firstEnd,
                           double secondEnd);
};

} // namespace ns3
#endif // INTERFERENCEAPPLICATION_H
