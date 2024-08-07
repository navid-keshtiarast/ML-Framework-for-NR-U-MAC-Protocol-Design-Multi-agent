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
#include "l2-setup.h"

namespace ns3 {

L2Setup::L2Setup (const NodeContainer &gnbNodes, const NodeContainer &ueNodes)
  : m_gnbNodes (gnbNodes),
  m_ueNodes (ueNodes)
{
}

L2Setup::~L2Setup ()
{
}

const NetDeviceContainer &
L2Setup::GetUeDev () const
{
  return m_ueDev;
}

const NetDeviceContainer &
L2Setup::GetGnbDev () const
{
  return m_gnbDev;
}

const NodeContainer &
L2Setup::GetGnbNodes () const
{
  return m_gnbNodes;
}

const NodeContainer &
L2Setup::GetUeNodes () const
{
  return m_ueNodes;
}

void
L2Setup::SetSinrCallback (const SinrCb &cb)
{
  m_sinrCb = cb;
}

void L2Setup::SetChannelOccupancyCallback (const L2Setup::ChannelOccupancyTimeCb &cb)
{
  m_channelOccupancyTimeCb = cb;
}

void L2Setup::SetMacTxDataFailedCb (const L2Setup::MacTxDataFailedCb &cb)
{
  m_macTxDataFailedCb = cb;
}

} // namespace ns3
