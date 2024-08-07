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
#include "sqlite-output-manager.h"
#include <ns3/abort.h>
#include <ns3/object.h>
#include <cmath>
#include <fstream>
#include <iostream>

namespace ns3 {

SqliteOutputManager::SqliteOutputManager (std::string &Namout,uint32_t num,const std::string &dbName, const std::string &dbLockName,
                                          uint32_t seed, uint32_t num2,uint32_t run)
  : m_dbOutput (dbName, dbLockName),
  m_dbName (dbName),
  m_seed (seed),
  m_run (run),
  numb(num),
  numb2(num2),
  m_Namout(Namout)
{
  std::ofstream myfile_Sinrstore;
  std::stringstream sinrou;
  sinrou<<m_Namout<<"Sinrstore_"<<numb<<"_"<<numb2<<".txt";
  myfile_Sinrstore.open (sinrou.str());
  myfile_Sinrstore<<"nodeId"<<"\t,"<<"sinr (db)"<<"\t,"<<"m_seed"<<"\t,"<<"run"<<"\n";
  std::ofstream myfile_StoreChannelOccupancyRateFor;
  myfile_StoreChannelOccupancyRateFor.open ("StoreChannelOccupancyRateFor.txt");
  std::stringstream ss;

  m_sinrTableName = "sinr_results_" + ss.str ();
  m_macDataTxFailedTableName = "mac_data_tx_failed_" + ss.str ();
  m_channelOccupancyTableName = "channel_occupancy_" + ss.str ();
  m_simultaneousTxTableName = "simultaneous_tx_" + ss.str ();
  m_e2eStatsTableName = "e2e_" + ss.str ();
  m_channelRequestTimeTableName = "channel_request_time_" + ss.str ();

  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_channelRequestTimeTableName + "\" "
                       "(UID                     INT    NOT NULL, "
                       "VALUE_US                 DOUBLE NOT NULL, "
                       "SEED                     INT    NOT NULL, "
                       "RUN                      INT    NOT NULL"
                       ");");
  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_sinrTableName + "\" "
                       "(UID                     INT    NOT NULL, "
                       "SINR                    DOUBLE NOT NULL, "
                       "SEED                    INT    NOT NULL, "
                       "RUN                     INT    NOT NULL"
                       ");");
  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_macDataTxFailedTableName + "\" "
                       "(UID                     INT    NOT NULL, "
                       "NUMBER                    INT NOT NULL, "
                       "BYTES                    INT NOT NULL, "
                       "SEED                    INT    NOT NULL, "
                       "RUN                     INT    NOT NULL"
                       ");");
  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_channelOccupancyTableName + "\" "
                       "(TECHNOLOGY             STRING   NOT NULL, "
                       "VALUE                    DOUBLE NOT NULL, "
                       "SEED                    INT    NOT NULL, "
                       "RUN                     INT    NOT NULL"
                       ");");
  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_simultaneousTxTableName + "\" "
                       "(UID                    INT    NOT NULL, "
                       "SIMULTANEOUS_TX_SAME_TECH INT  NOT NULL, "
                       "SIMULTANEOUS_TX_OTHER_TECH INT NOT NULL, "
                       "TOTALTX                 INT    NOT NULL, "
                       "SEED                    INT    NOT NULL, "
                       "RUN                     INT    NOT NULL"
                       ");");
  m_dbOutput.WaitExec ("CREATE TABLE IF NOT EXISTS \"" + m_e2eStatsTableName + "\" "
                       "(TECHNOLOGY             STRING   NOT NULL, "
                       "THROUGHPUT_MBPS         DOUBLE NOT NULL, "
                       "TXBYTES                 INT NOT NULL, "
                       "RXBYTES                 INT NOT NULL, "
                       "LATENCY_US              DOUBLE NOT NULL, "
                       "JITTER_US               DOUBLE NOT NULL, "
                       "ADDR                    STRING   NOT NULL, "
                       "SEED                    INT    NOT NULL, "
                       "RUN                     INT    NOT NULL"
                       ");");

  DeleteWhere (seed, run, m_sinrTableName);
  DeleteWhere (seed, run, m_macDataTxFailedTableName);
  DeleteWhere (seed, run, m_channelOccupancyTableName);
  DeleteWhere (seed, run, m_simultaneousTxTableName);
  DeleteWhere (seed, run, m_e2eStatsTableName);
}

SqliteOutputManager::~SqliteOutputManager ()
{
}

void
SqliteOutputManager::DeleteWhere (uint32_t seed, uint32_t run, const std::string &table)
{
  // bool ret;
  // sqlite3_stmt *stmt;
  // ret = m_dbOutput.WaitPrepare (&stmt, "DELETE FROM \"" + table + "\" WHERE SEED = ? AND RUN = ?;");
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 1, seed);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 2, run);

  // ret = m_dbOutput.WaitExec (stmt);
  // NS_ABORT_IF (ret == false);
}

void
SqliteOutputManager::SinrStore (uint32_t nodeId, double sinr)
{
  std::stringstream sinrou;
  sinrou<<m_Namout<<"Sinrstore_"<<numb<<"_"<<numb2<<".txt";
  std::ofstream myfile_Sinrstore;
  myfile_Sinrstore.open (sinrou.str(),std::ios::app);
  // bool ret;
  // sqlite3_stmt *stmt;
  // ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_sinrTableName + " VALUES (?,?,?,?);");
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 1, nodeId);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 2, 10 * log (sinr) / log (10));
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 3, m_seed);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 4, m_run);
  // NS_ABORT_IF (ret == false);
  myfile_Sinrstore<<nodeId<<"\t,"<<10 * log (sinr) / log (10)<<"\t,"<<m_seed<<"\t,"<<m_run<<"\n";
  // ret = m_dbOutput.WaitExec (stmt);
  // NS_ABORT_IF (ret == false);
}

void SqliteOutputManager::MacDataTxFailed (uint32_t nodeId, uint32_t bytes)
{
  // auto it = m_dataTxFailed.find (nodeId);
  // if (it == m_dataTxFailed.end ())
  //   {
  //     m_dataTxFailed.insert (std::make_pair (nodeId, std::make_pair (1, bytes)));
  //   }
  // else
  //   {
  //     it->second.first += 1;
  //     it->second.second += bytes;
  //   }
}

void SqliteOutputManager::StoreChannelOccupancyRateFor (const std::string &technology, double value)
{
  // std::ofstream myfile_StoreChannelOccupancyRateFor;
  // myfile_StoreChannelOccupancyRateFor.open ("StoreChannelOccupancyRateFor.txt",std::ios::app);
  // bool ret;
  // sqlite3_stmt *stmt;
  // ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_channelOccupancyTableName + " VALUES (?,?,?,?);");
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 1, technology);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 2, value);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 3, m_seed);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 4, m_run);
  // NS_ABORT_IF (ret == false);
  // myfile_StoreChannelOccupancyRateFor<<technology<<"\t,"<<value<<"\t,"<<m_seed<<"\t,"<<m_run<<"\n";
  // ret = m_dbOutput.WaitExec (stmt);
  // NS_ABORT_IF (ret == false);
}

void
SqliteOutputManager::StoreE2EStatsFor (const std::string &technology, double throughput,
                                       uint32_t txBytes, uint32_t rxBytes,
                                       double meanDelay, double meanJitter,
                                       const std::string &addr)
{
  // bool ret;
  // sqlite3_stmt *stmt;
  // ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_e2eStatsTableName + " VALUES (?,?,?,?,?,?,?,?,?);");
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 1, technology);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 2, throughput);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 3, txBytes);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 4, rxBytes);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 5, meanDelay);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 6, meanJitter);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 7, addr);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 8, m_seed);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 9, m_run);
  // NS_ABORT_IF (ret == false);

  // ret = m_dbOutput.WaitExec (stmt);
  // NS_ABORT_IF (ret == false);
}

void
SqliteOutputManager::UidIsTxing (uint32_t uid)
{
  // auto it = m_tx.find (uid);
  // if (it == m_tx.end ())
  //   {
  //     m_tx.insert (std::make_pair (uid, 1));
  //   }
  // else
  //   {
  //     (it->second) = it->second + 1;
  //   }
}

void
SqliteOutputManager::SimultaneousTxSameTechnology (uint32_t nodeId)
{
  // auto it = m_simultaneousTxSameTech.find (nodeId);
  // if (it == m_simultaneousTxSameTech.end ())
  //   {
  //     m_simultaneousTxSameTech.insert (std::make_pair (nodeId, 1));
  //   }
  // else
  //   {
  //     (it->second) = it->second + 1;
  //   }
}

void
SqliteOutputManager::ChannelRequestTime (uint32_t nodeId, Time value)
{
  // bool ret;
  // sqlite3_stmt *stmt;
  // ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_channelRequestTimeTableName + " VALUES (?,?,?,?);");
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 1, nodeId);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 2, static_cast<uint32_t> (value.GetMicroSeconds()));
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 3, m_seed);
  // NS_ABORT_IF (ret == false);
  // ret = m_dbOutput.Bind (stmt, 4, m_run);
  // NS_ABORT_IF (ret == false);

  // ret = m_dbOutput.WaitExec (stmt);
  // NS_ABORT_IF (ret == false);
}

void
SqliteOutputManager::SimultaneousTxOtherTechnology (uint32_t nodeId)
{
  // auto it = m_simultaneousTx.find (nodeId);
  // if (it == m_simultaneousTx.end ())
  //   {
  //     m_simultaneousTx.insert (std::make_pair (nodeId, 1));
  //   }
  // else
  //   {
  //     (it->second) = it->second + 1;
  //   }
}

void SqliteOutputManager::Close ()
{

  // for (const auto & v : m_dataTxFailed)
  //   {
  //     bool ret;
  //     sqlite3_stmt *stmt;
  //     ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_macDataTxFailedTableName + " VALUES (?,?,?,?,?);");
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 1, v.first);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 2, v.second.first);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 3, v.second.second);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 4, m_seed);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 5, m_run);
  //     NS_ABORT_IF (ret == false);

  //     ret = m_dbOutput.WaitExec (stmt);
  //     NS_ABORT_IF (ret == false);
  //   }

  // for (const auto & v : m_simultaneousTx)
  //   {
  //     bool ret;
  //     sqlite3_stmt *stmt;

  //     auto it = m_simultaneousTxSameTech.find (v.first);
  //     ret = m_dbOutput.WaitPrepare (&stmt, "INSERT INTO " + m_simultaneousTxTableName + " VALUES (?,?,?,?,?,?);");
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 1, v.first);
  //     NS_ABORT_IF (ret == false);
  //     if (it != m_simultaneousTxSameTech.end())
  //       {
  //         ret = m_dbOutput.Bind (stmt, 2, it->second);
  //       }
  //     else
  //       {
  //         ret = m_dbOutput.Bind (stmt, 2, 0);
  //       }
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 3, v.second);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 4, m_tx.find(v.first)->second);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 5, m_seed);
  //     NS_ABORT_IF (ret == false);
  //     ret = m_dbOutput.Bind (stmt, 6, m_run);
  //     NS_ABORT_IF (ret == false);

  //     ret = m_dbOutput.WaitExec (stmt);
  //     NS_ABORT_IF (ret == false);
  //   }
}

}
