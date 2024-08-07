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
#include "single-pair-node-scenario.h"
#include <ns3/mobility-helper.h>

namespace ns3 {

SinglePairNodeScenario::SinglePairNodeScenario (uint32_t gnbNum,
                                                const Vector& gnbReferencePos,
                                                double ueX,double uey )
{
  // create base stations and mobile terminals
  static MobilityHelper mobility;

  m_gNb.Create (gnbNum);
  m_ue.Create (gnbNum);

  Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator> ();

  // GNB positions:
  {
    double delta = 0.0;
    for (uint32_t i = 0; i < gnbNum; ++i)
      {
        Vector pos (gnbReferencePos);
        pos.y = pos.y + delta;
        delta += uey;
        std::cout << "gnb " << i << " pos " << pos << std::endl;
        gnbPos->Add (pos);
      }
  }

  // UE positions:
  {
    double delta = 0.0;
    for (uint32_t i = 0; i < gnbNum; ++i)
      {
        Vector pos (gnbReferencePos.x + ueX, gnbReferencePos.y + delta, 1.5);
        delta += uey;
        std::cout << "ue " << i << " pos " << pos << std::endl;
        uePos->Add (pos);
      }
  }

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (gnbPos);
  mobility.Install (m_gNb);

  mobility.SetPositionAllocator (uePos);
  mobility.Install (m_ue);
}

SinglePairNodeScenario::~SinglePairNodeScenario ()
{

}

} // namespace ns3
