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
#ifndef SQLITE_OUTPUT_MANAGER_H
#define SQLITE_OUTPUT_MANAGER_H

#include "output-manager.h"
#include <string>
#include <map>
#include <ns3/sqlite-output.h>

namespace ns3 {

/**
 * \ingroup output-managers
 *
 * \brief Output manager that stores data in a database
 *
 * This output manager is done to save the output of the examples
 * cttc-coexistence-indoor-scenario.cc and cttc-nr-wigig-interference.cc.
 *
 * The class create a SQLite database which contains the following tables:
 *
 * * sinr_results, which contains the SINR of every packet in the network
 * * mac_data_tx_failed which contains the count of MAC data transmission failed
 * * channel_occupancy which contains the time of channel occupancy divided per technology
 * * collision, which despite the name contains the number of simultaneous transmission per technology
 * * e2e, which contains the e2e IP statistics of the flow.
 *
 * The data is saved through a call to each virtual method, that saves the inputs
 * in the database. For an usage example, please look into the provided examples.
 */
class SqliteOutputManager : public OutputManager
{
public:
  /**
   * \brief SqliteOutputManager constructor
   * \param dbName the database file name
   * \param dbLockName the lock file name
   * \param ueX distance of the UEs from the gnb (only for nr-wigig-interference)
   * \param seed seed of the simulation
   * \param run run id of the simulation
   */
  SqliteOutputManager (std::string &Namout,uint32_t num,const std::string &dbName, const std::string &dbLockName,
                      uint32_t seed,uint32_t num2, uint32_t run);

  virtual ~SqliteOutputManager () override;

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
  void DeleteWhere (uint32_t seed, uint32_t run, const std::string &table);
private:
  SQLiteOutput m_dbOutput;                       //!< Instance of the db
  std::string m_dbName {""};                     //!< DB Name
  std::string m_sinrTableName {""};              //!< Table for SINR
  std::string m_macDataTxFailedTableName {""};   //!< Table for mac tx failed
  std::string m_channelOccupancyTableName {""};  //!< Table for channel occupancy
  std::string m_simultaneousTxTableName {""};         //!< Table for collisions
  std::string m_e2eStatsTableName {""};          //!< Table for IP e2e stats
  std::string m_channelRequestTimeTableName {""}; //!< Table for channel request time
  uint32_t m_seed {0};                           //!< Seed
  uint32_t m_run  {0};                           //!< Run id
  double numb  {0};
  double numb2  {0};

  std::string m_Namout {""}; 

  std::map<uint32_t, std::pair<uint32_t,uint32_t>> m_dataTxFailed; //!< Storage for tx data failed
  std::map<uint32_t, uint32_t> m_simultaneousTx;      //!< Storage for collisions
  std::map<uint32_t, uint32_t> m_simultaneousTxSameTech;  //!< Storage for collisions
  std::map<uint32_t, uint32_t> m_tx; //!< Storage for uid txing
};

} // namespace ns3

#endif // SQLITE_OUTPUT_MANAGER_H
