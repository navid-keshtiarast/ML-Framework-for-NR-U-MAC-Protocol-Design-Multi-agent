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

#include <ns3/core-module.h>
#include <ns3/network-module.h>

#ifndef NRU_TRACE_HELPER_H
#define NRU_TRACE_HELPER_H


namespace ns3 {

struct RetriesTraces
{
  Time m_time;
  uint32_t m_nodeId;
  Mac48Address m_dest;
};


class NruTraceHelper: public Object
{

public:

  /**
   * \brief Constructor
   */
  NruTraceHelper ();

  /**
   * \brief Destructor
   */
  virtual ~NruTraceHelper ();

  // inherited from Object
  /**
   *  \brief Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId (void);

  /**
   * \brief DoDispose
   */
  virtual void DoDispose ();

  /**
   * \brief Connect traces that are enabled through attributes of this class
   */
  void ConnectTraces ();

  /**
    * \brief Disconnect traces that are enabled through attributes of this class
    */
  void DisconnectTraces ();

  /**
   * \brief Save traces to the corresponding files
   */
  void SaveTracesToFile ();

private:

  /**
   * \brief Connect wigig retry traces to the callback function that will save traces to the vector
   */
  void EnableWigigRetriesTraces ();

  /**
   * \brief Retries callback that should be called every time that the retransmission occurs
   * @param context the context of the node which has triggered this callback
   * @param dest the MAC address of the destination node
   */
  void RetriesCb (std::string context, Mac48Address dest);
  /**
   * \brief Save retry traces to the file
   * @param filename the name of traces file
   * @param entries the vector containing traces
   */
  void SaveRetriesTraces (std::string filename);

  bool m_enableRetriesTraces; //!< whether to enable retries traces
  std::vector<RetriesTraces> m_retriesTraces;  //!< vector that saves all the retransmission logs
  std::string m_outputFileName; //!< output file name for traces, different traces will append different sufix to this output file name
};

} // end of ns3 namespace


#endif // NRU_TRACE_HELPER_H
