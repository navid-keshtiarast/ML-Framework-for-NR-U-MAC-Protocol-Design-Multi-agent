/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2024 RWTH Aachen University
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
 *   Modify: Oliver Renaldi <oliver.renaldi@rwth-aachen.de> 
 */
#include "nr-on-off-access-manager.h"
#include <ns3/log.h>
#include <ns3/random-variable-stream.h>
#include <ns3/simulator.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrOnOffAccessManager");
NS_OBJECT_ENSURE_REGISTERED (NrOnOffAccessManager);

TypeId
NrOnOffAccessManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrOnOffAccessManager")
    .SetParent<NrChAccessManager> ()
    .SetGroupName ("nr-u")
    .AddAttribute ("OnOffChangeTime",
                   "The default fixed value of the change time in OnOffAccessManager is used.",
                   TimeValue (MilliSeconds (10)),
                   MakeTimeAccessor (&NrOnOffAccessManager::SetChangeTime,
                                     &NrOnOffAccessManager::GetChangeTime),
                   MakeTimeChecker ())
    .AddConstructor <NrOnOffAccessManager> ()
  ;
  return tid;
}

NrOnOffAccessManager::NrOnOffAccessManager () : NrChAccessManager ()
{
  NS_LOG_FUNCTION (this);
  Time schedTime = MilliSeconds (0);
  m_changeStateEvent = Simulator::Schedule (schedTime, &NrOnOffAccessManager::Start, this);
  m_changeTime = MilliSeconds (10);
  NS_LOG_DEBUG ("Scheduled ON period at " << schedTime);
}

NrOnOffAccessManager::~NrOnOffAccessManager ()
{
  NS_LOG_FUNCTION (this);
}

void
NrOnOffAccessManager::SetChangeTime (const Time& changeTime)
{
  m_changeTime = changeTime;
}

const Time&
NrOnOffAccessManager::GetChangeTime () const
{
  return m_changeTime;
}

void
NrOnOffAccessManager::Start ()
{
  NS_LOG_FUNCTION (this);
  m_grant = true;
  if (!m_changeStateEvent.IsRunning ())
    {
      m_changeStateEvent = Simulator::Schedule (m_changeTime, &NrOnOffAccessManager::ShutDown, this);
      m_changeStateTime = Simulator::Now () + m_changeTime;
      NS_LOG_DEBUG ("START! Scheduled OFF period at " << Simulator::Now () + m_changeTime);
    }

  if (m_call)
    {
      for (const auto & cb : m_accessGrantedCb)
        {
          cb (m_changeStateTime - Simulator::Now ());
        }
    }
  m_call = false;
}

void
NrOnOffAccessManager::ShutDown ()
{
  NS_LOG_FUNCTION (this);
  m_grant = false;
  m_changeStateEvent.Cancel ();
  m_changeStateEvent = Simulator::Schedule (m_changeTime, &NrOnOffAccessManager::Start, this);
  m_changeStateTime = Simulator::Now () + m_changeTime;
  NS_LOG_DEBUG ("STOP! Scheduled ON period at " << Simulator::Now () + m_changeTime);
}

void
NrOnOffAccessManager::RequestAccess ()
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_call == false);
  m_call = true;

  if (m_grant)
    {
      Start ();
    }
  else
    {
      NS_LOG_DEBUG ("Time to ON state: " << Time (m_changeStateEvent.GetTs ()));
    }
}

void
NrOnOffAccessManager::SetAccessGrantedCallback (const AccessGrantedCallback &cb)
{
  NS_LOG_FUNCTION (this);
  m_accessGrantedCb.push_back (cb);
}

void
NrOnOffAccessManager::SetAccessDeniedCallback (const NrChAccessManager::AccessDeniedCallback &cb)
{
  NS_LOG_FUNCTION (this);
  // Do nothing, we'll end up calling Grant() sooner or later
}


void
NrOnOffAccessManager::Cancel ()
{
  NS_LOG_FUNCTION (this);
  m_call = false;
}

}
