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
#ifndef HALF_MATRIX_LAYOUT_H
#define HALF_MATRIX_LAYOUT_H

#include "node-distribution-scenario.h"
#include <ns3/vector.h>
#include <ns3/position-allocator.h>

namespace ns3 {

/**
 * \brief
 *
 * NOTE: That works only for horizontal placement of gnb.
 */
class HalfMatrixLayout : public NodeDistributionScenario
{
public:
  enum StartPosition
  {
    TOP,
    BOTTOM
  };

  HalfMatrixLayout (uint32_t nRow, uint32_t nColumn, uint32_t m_nUe,
                    Vector start, Vector end,
                    StartPosition startP, double gnbHeight, double ueHeight);
  /**
    * \brief destructor
    */
  ~HalfMatrixLayout ();

  void PrintGnbPosToFile (const std::string &filename) const;
  void PrintUePosToFile (const std::string &filename) const;

private:
  Ptr<ListPositionAllocator> GetGnbPosition (uint32_t nRow, uint32_t nColumn, Vector start, Vector end,
                                             StartPosition startP) const;
  Ptr<RandomBoxPositionAllocator> GetUePosition (Vector start, Vector end) const;

};


} // namespace ns3

#endif // HALF_MATRIX_LAYOUT_H
