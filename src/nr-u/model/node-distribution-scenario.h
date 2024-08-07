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
#ifndef NODE_DISTRIBUTION_SCENARIO_H
#define NODE_DISTRIBUTION_SCENARIO_H

#include <ns3/node-container.h>

namespace ns3 {

/**
 * \brief Represents a scenario with gnb (or base stations) and UEs (or users).
 *
 * Please build the scenario in the constructor of the class, and create nodes
 * in m_gNb and m_ue. Please initialize also a mobility model...
 */
class NodeDistributionScenario
{
public:
  /**
   * \brief ~Scenario
   */
  virtual ~NodeDistributionScenario ();
  /**
   * \brief Get the list of gnb/base station nodes
   * \return A NodeContainer with all the Gnb (or base stations)
   */
  const NodeContainer & GetGnbs () const;
  /**
   * \brief Get the list of user nodes
   * \return A NodeContainer with all the users
   */
  const NodeContainer & GetUes () const;

protected:
  NodeContainer m_gNb; //!< GNB (or base stations)
  NodeContainer m_ue;  //!< users

  double m_ueHeight {1.5}; //!< Height of UE nodes
  double m_gnbHeight {1.5}; //!< Height of gNB nodes
};

} // namespace ns3

#endif // NODE_DISTRIBUTION_SCENARIO_H
