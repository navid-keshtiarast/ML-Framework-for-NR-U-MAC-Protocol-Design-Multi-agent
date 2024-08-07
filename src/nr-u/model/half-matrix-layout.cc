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
#include "half-matrix-layout.h"
#include <ns3/mobility-helper.h>
#include <ns3/mobility-model.h>
#include <ns3/names.h>
#include <ns3/double.h>
#include <cmath>

namespace ns3 {

class ThreeGppRandomBoxPositionAllocator : public RandomBoxPositionAllocator
{
public:
  ThreeGppRandomBoxPositionAllocator () : RandomBoxPositionAllocator () { }
  ~ThreeGppRandomBoxPositionAllocator () override { }

  void SetMinDistance (double minDistance) { m_minDistance = minDistance; }
  void SetGnbContainer (const NodeContainer *gnb) { m_gnb = gnb; }
  Vector GetNext () const override;

private:
  double m_minDistance {0};
  const NodeContainer *m_gnb;
};

Vector
ThreeGppRandomBoxPositionAllocator::GetNext () const
{
  Vector uePos = RandomBoxPositionAllocator::GetNext ();

  for (auto itGnb = m_gnb->Begin(); itGnb != m_gnb->End(); itGnb++)
    {
      Ptr<MobilityModel> gnbMm = (*itGnb)->GetObject<MobilityModel>();
      Vector gnbPos = gnbMm->GetPosition ();
      double x = uePos.x - gnbPos.x;
      double y = uePos.y - gnbPos.y;
      double distance = std::sqrt ((x * x) + (y * y));
      if (distance < m_minDistance)
        {
          // Not valid; try another time
          return GetNext ();
        }
    }

  return uePos;
}

HalfMatrixLayout::HalfMatrixLayout (uint32_t nRow, uint32_t nColumn, uint32_t nUe, Vector start,
                                    Vector end, StartPosition startP, double gnbHeight, double ueHeight)
{
  m_gNb.Create (nRow * nColumn / 2);
  m_ue.Create (nUe);

  m_ueHeight = ueHeight;
  m_gnbHeight = gnbHeight;

  Ptr<ListPositionAllocator> gnbPos = GetGnbPosition(nRow, nColumn, start, end, startP);

  // create base stations and mobile terminals
  MobilityHelper mobility;

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (gnbPos);
  mobility.Install (m_gNb);

  Ptr<RandomBoxPositionAllocator> ueRandomRectPosAlloc = GetUePosition (start, end);

  mobility.SetPositionAllocator (ueRandomRectPosAlloc);
  mobility.Install (m_ue);
}

HalfMatrixLayout::~HalfMatrixLayout ()
{

}

void
HalfMatrixLayout::PrintGnbPosToFile (const std::string &filename) const
{
  std::ofstream outPositionsFile;

  outPositionsFile.open (filename.c_str ());
  outPositionsFile.setf (std::ios_base::fixed);

  for (auto itGnb = m_gNb.Begin(); itGnb != m_gNb.End(); itGnb++)
    {
      Vector v = (*itGnb)->GetObject<MobilityModel>()->GetPosition();
      outPositionsFile << (*itGnb)->GetId() << " " << v.x << " " << v.y << " " << v.z << std::endl;
    }
  outPositionsFile.close ();
}

void
HalfMatrixLayout::PrintUePosToFile (const std::string &filename) const
{
  std::ofstream outPositionsFile;

  outPositionsFile.open (filename.c_str ());
  outPositionsFile.setf (std::ios_base::fixed);

  for (auto itUe = m_ue.Begin(); itUe != m_ue.End(); itUe++)
    {
      Vector v = (*itUe)->GetObject<MobilityModel>()->GetPosition();
      outPositionsFile << (*itUe)->GetId() << " " << v.x << " " << v.y << " " << v.z << std::endl;
    }
  outPositionsFile.close ();
}

Ptr<ListPositionAllocator>
HalfMatrixLayout::GetGnbPosition (uint32_t nRow, uint32_t nColumn, Vector start, Vector end,
                                  StartPosition startP) const
{
  NS_ASSERT (nColumn > nRow);
  // At the center of that block there will be a GNB
  double blockWidth = end.x / nColumn;
  double blockHeight = end.y / nRow;

  // The pattern is ENABLE - DISABLE - ENABLE - ENABLE - DISABLE - DISABLE ...
  bool enableNode = startP == TOP ? true : false;
  bool previousItEnabled = enableNode;

  Ptr<ListPositionAllocator> gnbPos = CreateObject <ListPositionAllocator> ();

  // We start from the block at top left, then go down till the bottom left,
  // and then increase x, starting again at the top
  for (double x = start.x; x < end.x; x += blockWidth)
    {
      for (double y = end.y; y > start.y; y -= blockHeight)
        {
          // Get the point at the diagonal's end
          double xDiag = x + blockWidth;
          double yDiag = y - blockHeight;

          double xCenter = (x + xDiag) / 2;
          double yCenter = (y + yDiag) / 2;

          if (enableNode)
            {
              gnbPos->Add (Vector (xCenter, yCenter, m_gnbHeight));
            }

          bool temp = enableNode;
          if (previousItEnabled == enableNode)
            {
              enableNode = !enableNode;
            }
          previousItEnabled = temp;
        }
    }

  NS_ASSERT(gnbPos->GetSize() == nRow * nColumn / 2);

  return gnbPos;
}

Ptr<RandomBoxPositionAllocator>
HalfMatrixLayout::GetUePosition (Vector start, Vector end) const
{
  Ptr<ThreeGppRandomBoxPositionAllocator> ueRandomRectPosAlloc = CreateObject<ThreeGppRandomBoxPositionAllocator> ();
  ueRandomRectPosAlloc->SetMinDistance (0.0);
  ueRandomRectPosAlloc->SetGnbContainer (&m_gNb);

  Ptr<UniformRandomVariable> ueRandomVarX = CreateObject<UniformRandomVariable>();
  ueRandomVarX->SetAttribute ("Min", DoubleValue (start.x));
  ueRandomVarX->SetAttribute ("Max", DoubleValue (end.x));
  ueRandomRectPosAlloc->SetX(ueRandomVarX);
  Ptr<UniformRandomVariable> ueRandomVarY = CreateObject<UniformRandomVariable>();
  ueRandomVarY->SetAttribute ("Min", DoubleValue (start.y));
  ueRandomVarY->SetAttribute ("Max", DoubleValue (end.y));
  ueRandomRectPosAlloc->SetY(ueRandomVarY);
  Ptr<ConstantRandomVariable> ueRandomVarZ = CreateObject<ConstantRandomVariable>();
  ueRandomVarZ->SetAttribute("Constant", DoubleValue (m_ueHeight));
  ueRandomRectPosAlloc->SetZ(ueRandomVarZ);

  return (ueRandomRectPosAlloc);
}

} // namespace ns3
