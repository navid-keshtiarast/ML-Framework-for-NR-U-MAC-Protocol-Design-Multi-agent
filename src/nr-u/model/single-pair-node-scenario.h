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
#ifndef SINGLE_PAIR_NODE_SCENARIO_H
#define SINGLE_PAIR_NODE_SCENARIO_H

#include "node-distribution-scenario.h"
#include <ns3/vector.h>

namespace ns3 {
/**
 * \brief The most easy node scenario: just one UE with one access point.
 *
 *
 * \verbatim

                  d = ueY
          |----------------------|
    GNB_i                          UE_i
 \endverbatim
 *
 * The distance between each GNB is for the x value \f$ GNB_{i+1} = GNB_{i} \f$ ,
 * for z \f$ GNB_{i+1} = GNB_{i} \f$ and for y \f$ GNB_{i+1} = GNB_{i} + 0.5 \f$
 */
class SinglePairNodeScenario : public NodeDistributionScenario
{
public:
  /**
   * \brief SimpleInterferenceScenario constructor: initialize the node position
   * \param gnbNum Number of gnb (or base stations)
   * \param gnbReferencePos reference position for the first GNB (other position
   * will be derived from this information)
   * \param ueY Distance between GNB and UE in meters
   */
  SinglePairNodeScenario (uint32_t gnbNum, const Vector& gnbReferencePos, double ueY,double uex);
  /**
    * \brief destructor
    */
  ~SinglePairNodeScenario ();
};

} // namespace ns3

#endif // SINGLE_PAIR_NODE_SCENARIO_H
