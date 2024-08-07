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
#include "file-output-manager.h"
#include <cmath>

namespace ns3 {

FileOutputManager::FileOutputManager (const std::string &prefix)
{
  m_outSinrFile.open ((prefix + "-sinr.txt").c_str (), std::ios::trunc);
}

FileOutputManager::~FileOutputManager ()
{
  m_outSinrFile.close ();
}

void
FileOutputManager::SinrStore (uint32_t nodeId, double sinr)
{
  m_outSinrFile << nodeId
                << " " << 10 * log (sinr) / log (10) << std::endl;
}

void FileOutputManager::MacDataTxFailed (uint32_t nodeId, uint32_t bytes)
{

}

void FileOutputManager::StoreChannelOccupancyRateFor (const std::string &technology, double value)
{

}

void FileOutputManager::StoreE2EStatsFor (const std::string &technology, double throughput, uint32_t txBytes, uint32_t rxBytes, double meanDelay, double meanJitter, const std::string &addr)
{

}

void FileOutputManager::UidIsTxing (uint32_t uid)
{

}

void FileOutputManager::SimultaneousTxOtherTechnology(uint32_t nodeId)
{

}

void FileOutputManager::SimultaneousTxSameTechnology(uint32_t nodeId)
{

}

void FileOutputManager::ChannelRequestTime(uint32_t nodeId, Time value)
{

}

void FileOutputManager::Close ()
{

}

} // namespace ns3
