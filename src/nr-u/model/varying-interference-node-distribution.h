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
#ifndef VARYING_INTERFERENCE_NODE_DISTRIBUTION_H
#define VARYING_INTERFERENCE_NODE_DISTRIBUTION_H

#include "node-distribution-scenario.h"
#include <ns3/vector.h>

namespace ns3 {

/**
 * \brief Raying interference scenario (limited to 2 GNB and UE)
 *
 * Please note that this scenario considers one UE per each GNB.
 */
class VaryingInterferenceNodeDistribution : public NodeDistributionScenario
{
public:
  /**
   * \brief RayingInterferenceScenario constructor: initialize the node position
   * \param
   * \param gnbReferencePos reference position for the first GNB (other position
   * will be derived from this information)
   * \param ueY Distance between GNB and UE in meters
   */
  VaryingInterferenceNodeDistribution (const Vector& gnbReferencePos, double ueY);
  /**
    * \brief destructor
    */
  ~VaryingInterferenceNodeDistribution ();
};

} // namespace ns3

#endif // VARYING_INTERFERENCE_NODE_DISTRIBUTION_H
