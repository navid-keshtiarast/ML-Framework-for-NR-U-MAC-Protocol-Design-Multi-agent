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
#include "in-front-node-distribution.h"
#include <ns3/mobility-helper.h>

namespace ns3 {

InFrontNodeDistribution::InFrontNodeDistribution (const Vector& gnbReferencePos,
                                                  double distanceBetweenGnb,
                                                  double distanceBetweenUeAndGnb)
{
  // create base stations and mobile terminals
  static MobilityHelper mobility;

  m_gNb.Create (2);
  m_ue.Create (2);

  Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator> ();

  // GNB positions:
  {
    gnbPos->Add (gnbReferencePos);
    Vector pos (gnbReferencePos);
    pos.x += distanceBetweenGnb;
    gnbPos->Add (pos);
    std::cout << "gnb0 pos " << gnbReferencePos << " and gnb1 pos " << pos << std::endl;
  }

  // UE positions:
  {
    Vector pos (gnbReferencePos);
    pos.x += distanceBetweenUeAndGnb;
    uePos->Add (pos);
    Vector pos2 (pos);
    pos2.y += 0.5;
    uePos->Add (pos2);
    std::cout << "ue0 pos " << pos << " and ue1 pos " << pos2 << std::endl;
  }

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (gnbPos);
  mobility.Install (m_gNb);

  mobility.SetPositionAllocator (uePos);
  mobility.Install (m_ue);
}

InFrontNodeDistribution::~InFrontNodeDistribution ()
{

}

} // namespace ns3
