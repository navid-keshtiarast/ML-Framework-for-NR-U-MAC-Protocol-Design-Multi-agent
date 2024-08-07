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
#ifndef IN_FRONT_NODE_SCENARIO_H
#define IN_FRONT_NODE_SCENARIO_H

#include "node-distribution-scenario.h"
#include <ns3/vector.h>

namespace ns3 {

class InFrontNodeDistribution : public NodeDistributionScenario
{
public:

  InFrontNodeDistribution (const Vector& gnbReferencePos, double distanceBetweenGnb,
                           double distanceBetweenUeAndGnb);
  /**
    * \brief destructor
    */
  ~InFrontNodeDistribution ();
};

} // namespace ns3

#endif // IN_FRONT_NODE_SCENARIO_H
