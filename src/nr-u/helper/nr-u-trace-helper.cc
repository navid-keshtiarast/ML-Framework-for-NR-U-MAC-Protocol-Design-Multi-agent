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

#include "nr-u-trace-helper.h"
#include <ns3/log.h>
#include <ns3/boolean.h>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NruTraceHelper");
NS_OBJECT_ENSURE_REGISTERED (NruTraceHelper);

// Parse context strings of the form "/NodeList/3/DeviceList/1/Mac/Assoc"
// to extract the NodeId
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);  // skip "/NodeList/"
  uint32_t pos = sub.find ("/Device");
  NS_LOG_DEBUG ("Found NodeId " << atoi (sub.substr (0, pos).c_str ()));
  return atoi (sub.substr (0,pos).c_str ());
}

// Generate a 'fake' context id that can be parsed by ContextToNodeId ()
// For use in direct calls to "TraceConnect()" for LAA code, so that
// uniform ContextToNodeId() can be used in Wifi and LAA callbacks
std::string
NodeIdToContext (uint32_t nodeId)
{
  std::stringstream ss;
  ss << "/NodeList/";
  ss << nodeId;
  ss << "/DeviceList/";
  return ss.str ();
}

NruTraceHelper::NruTraceHelper ():Object(),
    m_enableRetriesTraces (false)
{
  NS_LOG_FUNCTION (this);
}

NruTraceHelper::~NruTraceHelper ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
NruTraceHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NruTraceHelper")
    .SetParent<Object> ()
    .SetGroupName("nr-u")
    .AddAttribute ("EnableRetriesTraces",
                   "If true retries traces will be enabled, "
                   "if false they will be disabled",
                   BooleanValue (false),
                   MakeBooleanAccessor (&NruTraceHelper::m_enableRetriesTraces),
                   MakeBooleanChecker ())
    .AddAttribute ("OutputFileName",
                   "Traces output file name",
                    StringValue ("nru-traces"),
                    MakeStringAccessor (&NruTraceHelper::m_outputFileName),
                    MakeStringChecker ())
    ;
  return tid;
}


void
NruTraceHelper::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  Object::DoDispose ();
}

void
NruTraceHelper::ConnectTraces()
{
  NS_LOG_FUNCTION (this);

  if (m_enableRetriesTraces)
    {
      EnableWigigRetriesTraces ();
      //EnableNrRetriesTraces ();
    }
}

void
NruTraceHelper::DisconnectTraces()
{
  NS_LOG_FUNCTION (this);
}

void
NruTraceHelper::SaveTracesToFile ()
{
  NS_LOG_FUNCTION (this);

  if (m_enableRetriesTraces)
    {
       SaveRetriesTraces (m_outputFileName + "_retries_log");
    }
}

void
NruTraceHelper::EnableWigigRetriesTraces ()
{
  NS_LOG_FUNCTION (this);
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/MacTxDataFailed",
                   MakeCallback (&NruTraceHelper::RetriesCb, this));
}

void
NruTraceHelper::RetriesCb (std::string context, Mac48Address dest)
{
  NS_LOG_FUNCTION (this);

  RetriesTraces entry;
  entry.m_time = Simulator::Now ();
  entry.m_nodeId = ContextToNodeId (context);
  entry.m_dest = dest;
  m_retriesTraces.push_back (entry);
}

void
NruTraceHelper::SaveRetriesTraces (std::string filename)
{
  NS_LOG_FUNCTION (this);

  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ofstream::out | std::ofstream::app);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  outFile.setf (std::ios_base::fixed);
  outFile << "#time(s) nodeId dest" << std::endl;

  for (std::vector<RetriesTraces>::size_type i = 0; i != m_retriesTraces.size (); i++)
    {
        {
          outFile << std::setprecision (9) << std::fixed << m_retriesTraces[i].m_time.GetSeconds () <<  " ";
          outFile << m_retriesTraces[i].m_nodeId << " ";
          outFile << m_retriesTraces[i].m_dest << std::endl;
        }
    }
}

}
