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
#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include <ns3/nstime.h>

namespace ns3 {

/**
 * \ingroup output-managers
 * \brief The OutputManager interface for storing the simulation data
 *
 * The class is born with the objective of providing an unified interface
 * to an output device. The class is optimized for the need of the examples
 * provided in the nr-u module, but it can be extended to support more traces.
 *
 * The idea here is that the methods here should be connected to the right
 * trace sources, so that every time it fires, a value is written in the output
 * device. We support functions to write the SINR of a packet, the number of
 * bytes which failed at MAC level, the occupancy rate, and the e2e statistics.
 * Then, we also support the number of times that two technologies occupied the
 * channel at the same time.
 *
 * Please remember to correctly close the output manager by calling Close().
 */
class OutputManager
{
public:
  /**
   * \brief OutputManager destructor
   */
  virtual ~OutputManager ()
  {
  }

  /**
   * \brief Store a SINR value
   * \param nodeId Node id
   * \param sinr Value of SINR in dB
   */
  virtual void SinrStore (uint32_t nodeId, double sinr) = 0;

  /**
   * \brief Store the transmission failed at MAC level
   * \param nodeId Node id
   * \param bytes Bytes that failed at MAC level
   */
  virtual void MacDataTxFailed (uint32_t nodeId, uint32_t bytes) = 0;

  /**
   * \brief Store the Technology occupancy rate
   * \param technology name of the technology
   * \param value Channel occupancy rate (between 0 and 1)
   */
  virtual void StoreChannelOccupancyRateFor (const std::string &technology, double value) = 0;

  virtual void UidIsTxing (uint32_t uid) = 0;

  /**
   * \brief Store the E2E stats
   * \param technology Technology name
   * \param throughput throughput
   * \param txBytes Transmitted bytes
   * \param rxBytes Received bytes
   * \param meanDelay Average delay (per packet)
   * \param meanJitter Average jitter (per packet)
   * \param addr Destination address
   */
  virtual void StoreE2EStatsFor (const std::string &technology, double throughput,
                                 uint32_t txBytes, uint32_t rxBytes, double meanDelay,
                                 double meanJitter, const std::string &addr) = 0;

  /**
   * \brief Indicates that there is a simultaneuous transmission with another technology
   *
   * Each call to this method will be counted as one simultaneous transmission.
   *
   * \param nodeId ID of the node
   */
  virtual void SimultaneousTxOtherTechnology (uint32_t nodeId) = 0;

  virtual void SimultaneousTxSameTechnology (uint32_t nodeId) = 0;

  virtual void ChannelRequestTime (uint32_t nodeId, Time value) = 0;

  /**
   * \brief Correctly close the output manager
   */
  virtual void Close () = 0;
};

} // namespace ns3

#endif // OUTPUT_MANAGER_H
