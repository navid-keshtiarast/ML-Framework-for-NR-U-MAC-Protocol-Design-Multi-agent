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
#include "no-interference-node-distribution.h"
#include <ns3/mobility-helper.h>
#include <ns3/names.h>
#include <math.h>

namespace ns3 {

NoInterferenceNodeDistribution::NoInterferenceNodeDistribution (const Vector& gnbReferencePos,
                                                                double ueX)
{
  // create base stations and mobile terminals
  static MobilityHelper mobility;

  uint32_t gnbNum = 2;

  m_gNb.Create (gnbNum);
  m_ue.Create (gnbNum);

  for (uint32_t i = 0; i < gnbNum; ++i)
    {
      std::stringstream ssGnb, ssUe;
      ssGnb << "gNb" << m_gNb.Get (i)->GetId ();
      ssUe << "UE" << m_ue.Get (i)->GetId ();

      Names::Add (ssGnb.str (), m_gNb.Get (i));
      Names::Add (ssUe.str (), m_ue.Get (i));

      std::cout << "GNB ID " << m_gNb.Get (i)->GetId () << std::endl;
      std::cout << "UE ID " << m_ue.Get (i)->GetId () << std::endl;
    }

  Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator> ();

  // GNB positions:
  {
    double delta = 0.0;
    for (uint32_t i = 0; i < gnbNum; ++i)
      {
        Vector pos (gnbReferencePos);
        pos.y = pos.y + delta;
        delta += 0.5;
        std::cout << "gnb " << i << " pos " << pos << std::endl;
        gnbPos->Add (pos);
      }
  }

  // UE positions:
  {
    Vector ue1Pos = Vector (gnbReferencePos.x + ueX, gnbReferencePos.y, 1.5);
    Vector ue2Pos = Vector (gnbReferencePos.x + sqrt (0.5) * ueX,
                            gnbReferencePos.y + sqrt (0.5) * ueX, 1.5);

    uePos->Add (ue1Pos);
    std::cout << "ue0 pos " << ue1Pos << std::endl;
    uePos->Add (ue2Pos);
    std::cout << "ue1 pos " << ue2Pos << std::endl;
  }

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (gnbPos);
  mobility.Install (m_gNb);

  mobility.SetPositionAllocator (uePos);
  mobility.Install (m_ue);
}


NoInterferenceNodeDistribution::~NoInterferenceNodeDistribution ()
{

}

} // namespace ns3
