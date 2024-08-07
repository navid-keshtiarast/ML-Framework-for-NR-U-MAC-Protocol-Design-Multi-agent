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
#ifndef FILE_OUTPUT_MANAGER_H
#define FILE_OUTPUT_MANAGER_H

#include <cstdint>
#include "output-manager.h"
#include <string>
#include <fstream>

namespace ns3 {

/**
 * \ingroup output-managers
 * \brief Store the values in files
 *
 * The class is not ready for general use; it is here only for reference.
 * Patches welcome.
 */
class FileOutputManager : public OutputManager
{
public:
  /**
   * \brief FileOutputManager constructor
   * \param prefix File prefix
   */
  FileOutputManager (const std::string &prefix);
  virtual ~FileOutputManager () override;

  virtual void SinrStore (uint32_t nodeId, double sinr) override;
  virtual void MacDataTxFailed (uint32_t nodeId, uint32_t bytes) override;
  virtual void StoreChannelOccupancyRateFor (const std::string &technology, double value) override;
  virtual void StoreE2EStatsFor (const std::string &technology, double throughput,
                                 uint32_t txBytes, uint32_t rxBytes, double meanDelay,
                                 double meanJitter, const std::string &addr) override;
  virtual void UidIsTxing (uint32_t uid) override;

  virtual void SimultaneousTxOtherTechnology (uint32_t nodeId) override;

  virtual void SimultaneousTxSameTechnology (uint32_t nodeId) override;
  virtual void ChannelRequestTime (uint32_t nodeId, Time value) override;
  virtual void Close () override;

private:
  std::string m_prefix;          //!< File prefix
  std::ofstream m_outSinrFile;   //!< SINR file
  std::ofstream m_outSnrFile;    //!< SNR file
  std::ofstream m_outRssiFile;   //!< RSSI file
};


} // namespace ns3

#endif // FILE_OUTPUT_MANAGER_H
