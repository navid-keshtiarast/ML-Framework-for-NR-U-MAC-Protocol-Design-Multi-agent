/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
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

#include "nr-lbt-access-manager.h"
#include "ns3/assert.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrLbtAccessManager");

NS_OBJECT_ENSURE_REGISTERED (NrCat2LbtAccessManager);
NS_OBJECT_ENSURE_REGISTERED (NrCat3LbtAccessManager);
NS_OBJECT_ENSURE_REGISTERED (NrCat4LbtAccessManager);


TypeId
NrLbtAccessManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrLbtAccessManager")
    .SetParent<NrChAccessManager> ()
    .SetGroupName ("nr-u")
    .AddAttribute ("EnergyDetectionThreshold",
                   "CCA-ED threshold for channel sensing",
                   DoubleValue (-79.0),
                   MakeDoubleAccessor (&NrLbtAccessManager::SetEdThreshold,
                                       &NrLbtAccessManager::GetEdThreshold),
                   MakeDoubleChecker<double> (-100.0, 0.0))
    .AddAttribute ("Slot", "The duration of a Slot.",
                   TimeValue (MicroSeconds (9)),
                   MakeTimeAccessor (&NrLbtAccessManager::SetSlotTime,
                                     &NrLbtAccessManager::GetSlotTime),
                   MakeTimeChecker ())
    .AddAttribute ("DeferTime", "TimeInterval to defer during CCA",
                   TimeValue (MicroSeconds (16)),
                   MakeTimeAccessor (&NrLbtAccessManager::SetDeferTime,
                                     &NrLbtAccessManager::GetDeferTime),
                   MakeTimeChecker ())
    .AddAttribute ("Mcot",
                   "Duration of channel access grant.",
                   TimeValue (MilliSeconds (5)),
                   MakeTimeAccessor (&NrLbtAccessManager::SetMcot,
                                     &NrLbtAccessManager::GetMcot),
                   MakeTimeChecker (MilliSeconds (0), MilliSeconds (20)))
  ;
  return tid;
}


NrLbtAccessManager::NrLbtAccessManager ()
  : NrChAccessManager ()
{
  NS_LOG_FUNCTION (this);
  m_rng = CreateObject<UniformRandomVariable> ();
}

NrLbtAccessManager::~NrLbtAccessManager ()
{
  NS_LOG_FUNCTION (this);
  // TODO Auto-generated destructor stub
  if (m_waitForDeferEventId.IsRunning ())
    {
      m_waitForDeferEventId.Cancel ();
    }
  if (m_waitForBackoffEventId.IsRunning ())
    {
      m_waitForBackoffEventId.Cancel ();
    }
  if (m_waitForBusyEventId.IsRunning ())
    {
      m_waitForBusyEventId.Cancel ();
    }
}

void
NrLbtAccessManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
}

void
NrLbtAccessManager::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
}

int64_t
NrLbtAccessManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rng->SetStream (stream);
  return 1;
}


void
NrLbtAccessManager::SetNrSpectrumPhy (Ptr<NrSpectrumPhy> spectrumPhy)
{
  NS_LOG_FUNCTION (this << spectrumPhy);
  NrChAccessManager::SetNrSpectrumPhy (spectrumPhy);
  std::cout<<"spectrum phy"<<spectrumPhy;
  std::cout<<"spectrum channell"<<spectrumPhy->GetSpectrumChannel();
  std::cout<<"spectrum"<<spectrumPhy<<"keke"<<GetNrSpectrumPhy ();
  // Configure the NrSpectrumPhy to treat each incoming signal as a foreign signal
  // (energy detection only)
  
  GetNrSpectrumPhy ()->SetAttribute ("CcaMode1Threshold", DoubleValue (GetEdThreshold ()));
  GetNrSpectrumPhy ()->SetAttribute ("UnlicensedMode", BooleanValue (true));
  GetNrSpectrumPhy ()->TraceConnectWithoutContext ("ChannelOccupied", MakeCallback (&ns3::NrLbtAccessManager::TransitionToBusy, this));
  //TraceConnectWithoutContext("kk",MakeCallback (&ns3::NrLbtAccessManager::malmalak, this));
}

void
NrLbtAccessManager::SetNrGnbMac (Ptr<NrGnbMac> mac)
{
  NS_LOG_FUNCTION (this << mac);
  NrChAccessManager::SetNrGnbMac (mac);
}

NrLbtAccessManager::LbtState
NrLbtAccessManager::GetLbtState () const
{
  NS_LOG_FUNCTION (this);
  return m_state;
}

void
NrLbtAccessManager::SetLbtState (NrLbtAccessManager::LbtState state)
{
  NS_LOG_FUNCTION (this << state);
  m_state = state;
}

void
NrLbtAccessManager::RequestAccess ()
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (GetNrSpectrumPhy (), "NrLbtAccessManager not connected to a NrSpectrumPhy");

  if (!this->IsInitialized ())
    {
      DoInitialize ();
    }

  if (m_grantRequested == true)
    {
      NS_LOG_LOGIC ("Already waiting to grant access; ignoring request");
      return;
    }

  if (Simulator::Now () - m_lastBusyTime >= m_deferTime)
    {
      // may grant access immediately according to ETSI BRAN flowchart
      NS_LOG_LOGIC ("Grant access immediately, channel is free more than defer time");
      SetGrant ();
      return;
    }
  m_grantRequested = true;

  m_backoffCount = GetBackoffSlots ();   // Draw new backoff value upon entry to the access granting process

  m_currentBackoffSlots = m_backoffCount; // update traced value with the new value of the backoff initial value

  NS_LOG_DEBUG ("New backoff counter: " << m_backoffCount);

  if (GetLbtState () == IDLE)
    {
      // Continue to wait until defer time has expired
      Time deferRemaining = m_deferTime - (Simulator::Now () - m_lastBusyTime);
      NS_LOG_LOGIC ("Must wait " << deferRemaining.GetSeconds () << " defer period");
      m_waitForDeferEventId = Simulator::Schedule (deferRemaining, &NrLbtAccessManager::RequestAccessAfterDefer, this);
      SetLbtState (WAIT_FOR_DEFER);
    }
  else if (m_lastBusyTime > Simulator::Now ())
    {
      // Access has come in while channel is already busy
      NS_ASSERT_MSG (Simulator::Now () < m_lastBusyTime, "Channel is busy but m_lastBusyTime in the past");
      Time busyRemaining = m_lastBusyTime - Simulator::Now ();
      NS_LOG_LOGIC ("Must wait " << busyRemaining.GetSeconds () << " sec busy period");
      TransitionToBusy (busyRemaining);
    }
}

void
NrLbtAccessManager::TransitionFromBusy ()
{
  NS_LOG_FUNCTION (this);
  if (m_lastBusyTime > Simulator::Now ())
    {
      Time busyRemaining = m_lastBusyTime - Simulator::Now ();
      NS_LOG_LOGIC ("Must wait additional " << busyRemaining.GetSeconds () << " busy period");
      m_waitForBusyEventId = Simulator::Schedule (busyRemaining, &NrLbtAccessManager::TransitionFromBusy, this);
      SetLbtState (BUSY);
    }
  else if (m_grantRequested == true)
    {
      NS_LOG_DEBUG ("Scheduling defer time to expire " << m_deferTime.GetMicroSeconds () + Simulator::Now ().GetMicroSeconds ());
      m_waitForDeferEventId = Simulator::Schedule (m_deferTime, &NrLbtAccessManager::RequestAccessAfterDefer, this);
      SetLbtState (WAIT_FOR_DEFER);
    }
  else
    {
      SetLbtState (IDLE);
    }
}

void
NrLbtAccessManager::TransitionToBusy (Time duration)
{
  NS_LOG_FUNCTION (this);
  switch (GetLbtState ())
    {
    case IDLE:
      {
        NS_ASSERT_MSG (m_backoffCount == 0, "was idle but m_backoff nonzero");
        m_lastBusyTime = Simulator::Now () + duration;
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrLbtAccessManager::TransitionFromBusy, this);
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
        break;
      }
    case TXOP_GRANTED:
      {
        m_lastBusyTime = Simulator::Now () + duration;
        //std::cout<<"TXop_granted"<<duration;
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrLbtAccessManager::TransitionFromBusy, this);
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
        break;
      }
    case BUSY:
      {
        // Update last busy time
        if (m_lastBusyTime < (Simulator::Now () + duration))
          {
            NS_LOG_DEBUG ("Update last busy time to " << m_lastBusyTime.GetMicroSeconds ());
            m_lastBusyTime = Simulator::Now () + duration;
          }
        // Since we went busy, a request for access may have come in
        if (!m_waitForBusyEventId.IsRunning () && m_grantRequested > 0)
          {
            Time busyRemaining = m_lastBusyTime - Simulator::Now ();
            m_waitForBusyEventId = Simulator::Schedule (busyRemaining, &NrLbtAccessManager::TransitionFromBusy, this);
            NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
            SetLbtState (BUSY);
          }
      }
      break;
    case WAIT_FOR_DEFER:
      {
        NS_LOG_DEBUG ("TransitionToBusy from WAIT_FOR_DEFER");
        if (m_waitForDeferEventId.IsRunning ())
          {
            NS_LOG_DEBUG ("Cancelling Defer");
            m_waitForDeferEventId.Cancel ();
          }
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrLbtAccessManager::TransitionFromBusy, this);
        m_lastBusyTime = Simulator::Now () + duration;
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
      }
      break;
    case WAIT_FOR_BACKOFF:
      {
        // std::cout<<"WAIT_FOR_BACKOFF";
        NS_LOG_DEBUG ("TransitionToBusy from WAIT_FOR_BACKOFF");
        if (m_waitForBackoffEventId.IsRunning ())
          {
            NS_LOG_DEBUG ("Cancelling Backoff");
            m_waitForBackoffEventId.Cancel ();
          }
        Time timeSinceBackoffStart = Simulator::Now () - m_backoffStartTime;
        //NS_ASSERT (timeSinceBackoffStart < m_backoffCount * m_slotTime);
        // Decrement backoff count for every full and fractional m_slot time
        while (timeSinceBackoffStart > Seconds (0) && m_backoffCount > 0)
          {
            m_backoffCount--;
            timeSinceBackoffStart -= m_slotTime;
            NS_LOG_DEBUG("kiri"<<m_backoffCount);
          }
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrLbtAccessManager::TransitionFromBusy, this);
        m_lastBusyTime = Simulator::Now () + duration;
        NS_LOG_DEBUG ("Suspend backoff, go busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
      }
      break;
    default:
      NS_FATAL_ERROR ("Should be unreachable " << GetLbtState ());
    }
}

void
NrLbtAccessManager::RequestAccessAfterDefer ()
{
  NS_LOG_FUNCTION (this);
  // if channel has remained idle so far, wait for a number of backoff
  // slots to see if it is remains idle
  if (GetLbtState () == WAIT_FOR_DEFER)
    {
      if (m_backoffCount == 0)
        {
          NS_LOG_DEBUG ("Defer succeeded, backoff count already zero");
          SetGrant ();
        }
      else
        {
          NS_LOG_DEBUG ("Defer succeeded, scheduling for " << m_backoffCount << " backoffSlots");
          m_backoffStartTime = Simulator::Now ();
          Time timeForBackoff = m_slotTime * m_backoffCount;
          m_waitForBackoffEventId = Simulator::Schedule (timeForBackoff, &NrLbtAccessManager::RequestAccessAfterBackoff, this);
          
          SetLbtState (WAIT_FOR_BACKOFF);
          Simulator::Schedule (m_slotTime, &NrLbtAccessManager::force_sensing_navid, this);///add by navid
          //std::cout<<"kiri is on";
        }
    }
  else
    {
      NS_LOG_DEBUG ("Was not deferring, scheduling for " << m_deferTime.GetMicroSeconds () << " microseconds");
      m_waitForDeferEventId = Simulator::Schedule (m_deferTime, &NrLbtAccessManager::RequestAccessAfterDefer, this);
      SetLbtState (WAIT_FOR_DEFER);
    }
}


// void
// NrLbtAccessManager::malmalak(uint16_t abbb)
// {
//  std::cout<<"malmalllllllllllllllkkkk"; 
// }
void
NrLbtAccessManager::navid_change_st_after()
{
//std::cout<<"navidkhare gave mane"<<GetNrSpectrumPhy()->GetSpectrumChannel();
GetNrSpectrumPhy()->GetSpectrumChannel()->navid_changer_status(0);

}

void
NrLbtAccessManager::force_sensing_navid()
{
  //uint32_t aaa=1;
//m_backoffCount--;

NS_LOG_FUNCTION (this);
  switch (GetLbtState ())
    {
    case IDLE:
      {
       
        break;
      }
    case TXOP_GRANTED:
      {
       
        break;
      }
    case BUSY:
      {
        
      }
      break;
    case WAIT_FOR_DEFER:
      {
        
      }
      break;
    case WAIT_FOR_BACKOFF:
      {
        //GetNrSpectrumPhy()->fuckoff();
        //GetNrSpectrumPhy()->MaybeCcaBusy();
        
        // if (kalekiri!=GetNrSpectrumPhy()->GetCellId())
        // {
        //   SetLbtState (BUSY);
        //   TransitionToBusy(m_slotTime);
        //   //Simulator::Schedule(m_slotTime, &NrLbtAccessManager::Kiroff, this);
        //   //kalekiri=0;
        // }
        //GetNrSpectrumPhy()->MaybeCcaBusy();
        //Simulator::Schedule (m_slotTime, &NrLbtAccessManager::force_sensing_navid, this);///add by navid
        //NS_LOG_DEBUG ("TransitionToBusy from WAIT_FOR_BACKOFF");
        // if (m_waitForBackoffEventId.IsRunning ())
        //   {
        //     NS_LOG_DEBUG ("Cancelling Backoff");
        //     m_waitForBackoffEventId.Cancel ();
        //   }
        GetNrSpectrumPhy()->MaybeCcaBusy();
        //Time timeSinceBackoffStart = Simulator::Now () - m_backoffStartTime;
        //m_backoffCount--;
        //////navid jadid
        // if(!GetNrSpectrumPhy()->GetSpectrumChannel()->Navid_checker())
        // {
        //   m_backoffCount++;
        //   //m_waitForBackoffEventId.Cancel ();
        //   //TransitionToBusy(MilliSeconds(1));
        //   if (m_waitForBackoffEventId.IsRunning ())
        //   {
        //     NS_LOG_DEBUG ("Cancelling Backoff");
        //     m_waitForBackoffEventId.Cancel ();
        //     Time timeSinceBackoffStart = Simulator::Now () - m_backoffStartTime;
        //     //NS_ASSERT (timeSinceBackoffStart < m_backoffCount * m_slotTime);
        //     // Decrement backoff count for every full and fractional m_slot time
        //     while (timeSinceBackoffStart > Seconds (0) && m_backoffCount > 0)
        //       {
        //         m_backoffCount--;
        //         timeSinceBackoffStart -= m_slotTime;
        //         NS_LOG_DEBUG("kiri"<<m_backoffCount);
        //       }
        //     m_waitForBusyEventId = Simulator::Schedule (MilliSeconds(1), &NrLbtAccessManager::TransitionFromBusy, this);
        //     //m_lastBusyTime = Simulator::Now () + duration;
        //     //NS_LOG_DEBUG ("Suspend backoff, go busy until " << m_lastBusyTime.GetMicroSeconds ());
        //     SetLbtState (BUSY);
        //     //SetLbtState(BUSY);
        //     //TransitionToBusy(MilliSeconds(1));
            
        //   }
        //   //Simulator::Schedule (MilliSeconds(1), &NrLbtAccessManager::force_sensing_navid, this);///add by navid
          


        // }
        // if(GetNrSpectrumPhy()->GetSpectrumChannel()->Navid_checker())
        // {
          //navid/
          Simulator::Schedule (m_slotTime, &NrLbtAccessManager::force_sensing_navid, this);///add by navid
          


        //}
        

        // NS_ASSERT (timeSinceBackoffStart < m_backoffCount * m_slotTime);
        // // Decrement backoff count for every full and fractional m_slot time
        // while (timeSinceBackoffStart > Seconds (0) && m_backoffCount > 0)
        //   {
        //     m_backoffCount--;
        //     timeSinceBackoffStart -= m_slotTime;
        //     NS_LOG_DEBUG("kiri"<<m_backoffCount);
        //   }
        // m_waitForBusyEventId = Simulator::Schedule (duration, &NrLbtAccessManager::TransitionFromBusy, this);
        // m_lastBusyTime = Simulator::Now () + duration;
        // NS_LOG_DEBUG ("Suspend backoff, go busy until " << m_lastBusyTime.GetMicroSeconds ());
        // SetLbtState (BUSY);
      }
      break;
    default:
      NS_FATAL_ERROR ("Should be unreachable " << GetLbtState ());
    }


}

void
NrLbtAccessManager::RequestAccessAfterBackoff ()
{
  NS_LOG_FUNCTION (this);
  // if the channel has remained idle, grant access now for a TXOP
  if (GetLbtState () == WAIT_FOR_BACKOFF)
    {
      SetGrant ();
      
      m_backoffCount = 0;
      
    }
  else
    {
      NS_FATAL_ERROR ("Unreachable?");
    }
}

void
NrLbtAccessManager::SetGrant ()
{
  NS_LOG_DEBUG ("Granting access through ChannelAccessManager at time " << Simulator::Now ().GetMicroSeconds ());
  //////////GetNrSpectrumPhy()->GetSpectrumChannel()->navid_changer_status(1);
  //////////Simulator::Schedule(MilliSeconds(1),&NrLbtAccessManager::navid_change_st_after,this);
  //std::cout<<"celllll"<<GetNrSpectrumPhy()->GetCellId();
  //kalekiri=GetNrSpectrumPhy()->GetCellId();
  
  //notify of new grant
  SetGrantDuration (m_mcot);
  NotifyAccessGranted ();
  // reset state
  SetLbtState (TXOP_GRANTED);
  m_grantRequested = false;
  //GetNrSpectrumPhy()->fuckoff();
  m_lastTxopStartTime = Simulator::Now ();
  
}

uint32_t
NrLbtAccessManager::GetCurrentBackoffCount (void) const
{
  NS_LOG_FUNCTION (this);
  return m_backoffCount;
}

Time
NrLbtAccessManager::GetLastTxopStartTime () const
{
  NS_LOG_FUNCTION (this);
  return m_lastTxopStartTime;
}

void
NrLbtAccessManager::NotifyAccessGranted ()
{
  NS_LOG_FUNCTION (this);
  for (const auto & cb : m_accessGrantedCb)
    {
      cb (GetGrantDuration ());
    }
}

void
NrLbtAccessManager::Cancel ()
{
  NS_LOG_FUNCTION (this);
  m_grantRequested = false;
}

void
NrLbtAccessManager::SetAccessGrantedCallback (const AccessGrantedCallback &cb)
{
  NS_LOG_FUNCTION (this);
  m_accessGrantedCb.push_back (cb);
}

void
NrLbtAccessManager::SetAccessDeniedCallback(const NrChAccessManager::AccessDeniedCallback &cb)
{
  NS_LOG_FUNCTION (this);
  m_accessDeniedCb.push_back (cb);
}

void
NrLbtAccessManager::SetEdThreshold (double edThreshold)
{
  NS_LOG_FUNCTION (this);
  m_edThreshold = edThreshold;
}

double
NrLbtAccessManager::GetEdThreshold () const
{
  NS_LOG_FUNCTION (this);
  return m_edThreshold;
}

void
NrLbtAccessManager::SetDeferTime (const Time& deferTime)
{
  m_deferTime = deferTime;
}

const Time&
NrLbtAccessManager::GetDeferTime () const
{
  return m_deferTime;
}

const Time&
NrLbtAccessManager::GetSlotTime () const
{
  return m_slotTime;
}

void
NrLbtAccessManager::SetSlotTime (const Time& slotTime)
{
  m_slotTime = slotTime;
}

const Time&
NrLbtAccessManager::GetMcot () const
{
  return m_mcot;
}

void
NrLbtAccessManager::SetMcot (const Time& mcot)
{
  m_mcot = mcot;
}


/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/

void
NrCat2LbtAccessManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
}

void
NrCat2LbtAccessManager::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
}

TypeId
NrCat2LbtAccessManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrCat2LbtAccessManager")
    .SetParent<NrLbtAccessManager> ()
    .SetGroupName ("nr-u")
    .AddAttribute ("Cat2DeferTime", "TimeInterval to defer during CCA",
                   TimeValue (MicroSeconds (25)),
                   MakeTimeAccessor (&NrLbtAccessManager::SetDeferTime,
                                     &NrLbtAccessManager::GetDeferTime),
                   MakeTimeChecker ())
    .AddAttribute ("Cat2EDThreshold",
                   "CCA-ED threshold for channel sensing",
                   DoubleValue (-69.0),
                   MakeDoubleAccessor (&NrLbtAccessManager::SetEdThreshold,
                                       &NrLbtAccessManager::GetEdThreshold),
                   MakeDoubleChecker<double> (-100.0, 0.0))
    .AddConstructor<NrCat2LbtAccessManager> ();
  return tid;
}


void
NrCat2LbtAccessManager::RequestAccess ()
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (GetNrSpectrumPhy (), "NrCat2LbtAccessManager not connected to a NrSpectrumPhy");

  if (!this->IsInitialized ())
    {
      DoInitialize ();
    }

  if (m_grantRequested == true)
    {
      NS_LOG_LOGIC ("Cat2: Already waiting to grant access; ignoring request");
      return;
    }

  if (Simulator::Now () - m_lastBusyTime >= m_deferTime)
    {
      // may grant access immediately according to ETSI BRAN flowchart
      NS_LOG_LOGIC ("Cat2: Grant access immediately, channel is free more than defer time");
      SetGrant ();
      return;
    }
  m_grantRequested = true;

  if (GetLbtState () == IDLE)
    {
      // Continue to wait until defer time has expired
      Time deferRemaining = m_deferTime - (Simulator::Now () - m_lastBusyTime);
      NS_LOG_LOGIC ("Cat2: Must wait " << deferRemaining.GetSeconds () << " seconds of defer period");
      m_waitForDeferEventId = Simulator::Schedule (deferRemaining, &NrCat2LbtAccessManager::RequestAccessAfterDefer, this);
      SetLbtState (WAIT_FOR_DEFER);
    }
  else
    {
      NS_LOG_INFO ("Cat 2: Channel busy, LBT CAT2 failure.");
      NotifyAccessDenied ();
    }
}

uint32_t
NrCat2LbtAccessManager::GetBackoffSlots ()
{
  NS_ABORT_MSG ("Should not be called with Cat2 algortihm.");
  // cat2 does not enter into backoff thus this function returns 0
  return 0;
}


void
NrCat2LbtAccessManager::RequestAccessAfterDefer ()
{
  NS_LOG_FUNCTION (this);
  if (GetLbtState () == WAIT_FOR_DEFER)
    {
      NS_LOG_DEBUG ("Cat2 LBT defer succeeded!");
      SetGrant ();
     }
  else
    {
      NS_ABORT_MSG ("In CAT 2 algorithm, once that defer is finished, the LBT state machine can be only in WAIT_FOR_DEFER state.");
    }
}

void
NrCat2LbtAccessManager::TransitionToBusy (Time duration)
{
  NS_LOG_FUNCTION (this);
  switch (GetLbtState ())
    {
    case IDLE:
      {
        NS_ASSERT_MSG (m_backoffCount == 0, "was idle but m_backoff nonzero");
        m_lastBusyTime = Simulator::Now () + duration;
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrCat2LbtAccessManager::TransitionFromBusy, this);
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
        break;
      }
    case TXOP_GRANTED:
      {
        m_lastBusyTime = Simulator::Now () + duration;
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrCat2LbtAccessManager::TransitionFromBusy, this);
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
        break;
      }
    case BUSY:
      {
        // Update last busy time
        if (m_lastBusyTime < (Simulator::Now () + duration))
          {
            NS_LOG_DEBUG ("Update last busy time to " << m_lastBusyTime.GetMicroSeconds ());
            m_lastBusyTime = Simulator::Now () + duration;
          }
        // Since we went busy, a request for access may have come in
        if (!m_waitForBusyEventId.IsRunning () && m_grantRequested > 0)
          {
            Time busyRemaining = m_lastBusyTime - Simulator::Now ();
            m_waitForBusyEventId = Simulator::Schedule (busyRemaining, &NrCat2LbtAccessManager::TransitionFromBusy, this);
            NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
            SetLbtState (BUSY);
          }
      }
      break;
    case WAIT_FOR_DEFER:
      {
        NS_LOG_DEBUG ("Cat2: In defer, but going busy until " << m_lastBusyTime.GetMicroSeconds ());
        if (m_waitForDeferEventId.IsRunning ())
          {
            NS_LOG_DEBUG ("Cat2: Cancelling Defer because the channel is not idle for deterministic defer time.");
            m_waitForDeferEventId.Cancel ();
          }

        // deny this request because channel is not idle during a deterministic defer time - we consider
        // uninterupted defer time, we do not allow in Cat2 as in Cat3 and Cat4 to got to busy and then try again the defer
        NotifyAccessDenied ();
        // we have to deny this request but still we need to schedule transition from busy to return state machine to IDLE
        m_waitForBusyEventId = Simulator::Schedule (duration, &NrCat2LbtAccessManager::TransitionFromBusy, this);
        m_lastBusyTime = Simulator::Now () + duration;
        NS_LOG_DEBUG ("Going busy until " << m_lastBusyTime.GetMicroSeconds ());
        SetLbtState (BUSY);
      }
      break;
    case WAIT_FOR_BACKOFF:
      {
        NS_ABORT_MSG ("Cat 2 Lbt should not go into backoff state.");
      }
      break;
    default:
      NS_FATAL_ERROR ("Should be unreachable " << GetLbtState ());
    }
}

void
NrCat2LbtAccessManager::NotifyAccessDenied ()
{
  NS_LOG_FUNCTION (this);
  m_grantRequested = false;
  for (const auto & cb : m_accessDeniedCb)
    {
      cb ();
    }
}


/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/


void
NrCat3LbtAccessManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
}

void
NrCat3LbtAccessManager::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
}

TypeId
NrCat3LbtAccessManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrCat3LbtAccessManager")
    .SetParent<NrLbtAccessManager> ()
    .SetGroupName ("nr-u")
    .AddAttribute ("Cat3ContentionWindow",
                   "The default fixed value of the CW in the case that Cat.3 LBT. is used.",
                   UintegerValue (15),
                   MakeUintegerAccessor (&NrCat3LbtAccessManager::SetCat3ContentionWindow,
                                         &NrCat3LbtAccessManager::GetCat3ContentionWindow),
                   MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("Backoff",
                     "Backoff change trace source; logs any new backoff",
                     MakeTraceSourceAccessor (&NrCat3LbtAccessManager::m_currentBackoffSlots),
                     "ns3::TracedValue::Uint32Callback")
    .AddConstructor<NrCat3LbtAccessManager> ()
  ;
  return tid;
}

void
NrCat3LbtAccessManager::SetCat3ContentionWindow (const uint32_t& contentionWindow)
{
  m_cat3ContentionWindow = contentionWindow;
}


const uint32_t&
NrCat3LbtAccessManager::GetCat3ContentionWindow () const
{
  return m_cat3ContentionWindow;
}


uint32_t
NrCat3LbtAccessManager::GetBackoffSlots ()
{
  NS_LOG_FUNCTION (this);
  return m_rng->GetInteger (0, m_cat3ContentionWindow);
}

/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/
TypeId
NrCat4LbtAccessManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NrCat4LbtAccessManager")
    .SetParent<NrLbtAccessManager> ()
    .SetGroupName ("nr-u")
    .AddAttribute ("RetryLimit", "How many times to try to retransmit with the "
                   "current CW before reseting it to the MinCw value.",
                   UintegerValue (5),
                   MakeUintegerAccessor (&NrCat4LbtAccessManager::m_nRetry),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("Cat4MinCw", "The minimum value of the contention window.",
                   UintegerValue (15),
                   MakeUintegerAccessor (&NrCat4LbtAccessManager::SetCat4MinCw,
                                         &NrCat4LbtAccessManager::GetCat4MinCw),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Cat4MaxCw", "The maximum value of the contention window. "
                   "For the priority class 3 this value is set to 63, and for "
                   "priority class 4 it is 1023.",
                   UintegerValue (1023),
                   MakeUintegerAccessor (&NrCat4LbtAccessManager::SetCat4MaxCw,
                                         &NrCat4LbtAccessManager::GetCat4MaxCw),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Cat4CwUpdateRule",
                   "Rule according which CW will be updated",
                   EnumValue (NrCat4LbtAccessManager::ANY_NACK),
                   MakeEnumAccessor (&NrCat4LbtAccessManager::m_cat4CwUpdateRule),
                   MakeEnumChecker (NrCat4LbtAccessManager::ALL_NACKS, "ALL_NACKS",
                                    NrCat4LbtAccessManager::ANY_NACK, "ANY_NACK",
                                    NrCat4LbtAccessManager::NACKS_10_PERCENT, "NACKS_10_PERCENT",
                                    NrCat4LbtAccessManager::NACKS_80_PERCENT, "NACKS_80_PERCENT"))
    .AddAttribute ("Cat4LinearBackoff", "The increment value for linear backoff algorithm. ",
                   UintegerValue (4),
                   MakeUintegerAccessor (&NrCat4LbtAccessManager::SetCat4LinBackoff,
                                         &NrCat4LbtAccessManager::GetCat4LinBackoff),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BackOffType", "The type of backoff algorithm used for the contention window.",
                   UintegerValue (3),
                   MakeUintegerAccessor (&NrCat4LbtAccessManager::SetCat4BackoffType,
                                         &NrCat4LbtAccessManager::GetCat4BackoffType),
                   MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("Cw",
                     "CW change trace source; logs changes to CW",
                     MakeTraceSourceAccessor (&NrCat4LbtAccessManager::m_cat4ContentionWindowTracedValue),
                     "ns3::TracedValue::Uint32Callback")
    .AddTraceSource ("Backoff",
                     "Backoff change trace source; logs any new backoff",
                     MakeTraceSourceAccessor (&NrCat4LbtAccessManager::m_currentBackoffSlots),
                     "ns3::TracedValue::Uint32Callback")
    // .AddTraceSource ("kk",
    //                  "kk change trace source",
    //                  MakeTraceSourceAccessor (&NrCat4LbtAccessManager::kalekiri),
    //                  "ns3::TracedValue::Uint16Callback")
    .AddConstructor<NrCat4LbtAccessManager> ()
  ;
  return tid;
}

void
NrCat4LbtAccessManager::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  NrLbtAccessManager::DoInitialize ();
  m_cat4ContentionWindowTracedValue = m_cat4CwMin;
}

void
NrCat4LbtAccessManager::SetCat4MinCw (const uint32_t& contentionWindow)
{
  std::cout << "Testing bug minCW " << contentionWindow << std::endl;
  m_cat4CwMin = contentionWindow;
}

void
NrCat4LbtAccessManager::SetCat4MaxCw (const uint32_t& contentionWindow)
{
  std::cout << "Testing bug maxCW " << contentionWindow << std::endl;
  m_cat4CwMax = contentionWindow;
}

void
NrCat4LbtAccessManager::SetCat4LinBackoff (const uint32_t& backoffConst)
{
  std::cout << "Testing bug BackOff Constant : " << backoffConst << std::endl;
  m_linBackoff = backoffConst;
}

void
NrCat4LbtAccessManager::SetCat4BackoffType (const uint32_t& backoffType)
{
  std::cout << "Testing bug BackOffType : " << backoffType << std::endl;
  m_backoffType = backoffType;
}

const uint32_t&
NrCat4LbtAccessManager::GetCat4MinCw () const
{
  return m_cat4CwMin;
}

const uint32_t&
NrCat4LbtAccessManager::GetCat4MaxCw () const
{
  return m_cat4CwMax;
}

const uint32_t&
NrCat4LbtAccessManager::GetCat4LinBackoff () const
{
  return m_linBackoff;
}

const uint32_t&
NrCat4LbtAccessManager::GetCat4BackoffType () const
{
  return m_backoffType;
}

uint32_t
NrCat4LbtAccessManager::GetBackoffSlots ()
{
  NS_LOG_FUNCTION (this);

  if (m_nacks == 0 && m_acks == 0)
    {
      return m_rng->GetInteger (0, m_cat4ContentionWindowTracedValue.Get ());
    }

  bool updateFailedCw = false;

  switch (m_cat4CwUpdateRule)
    {
    case ALL_NACKS:
      {
        if (m_acks == 0)
          {
            updateFailedCw = true;
          }
      }
      break;
    case ANY_NACK:
      {
        if (m_nacks > 0)
          {
            updateFailedCw  = true;
          }
      }
      break;
    case NACKS_80_PERCENT:
      {
        if (static_cast<double> (m_nacks) / (m_acks + m_nacks) >= 0.8)
          {
            updateFailedCw = true;
            NS_LOG_INFO ("CW will be updated, since NACKs are greater than 80%");
          }
      }
      break;
    case NACKS_10_PERCENT:
      {
        if (static_cast<double> (m_nacks) / (m_acks + m_nacks) >= 0.1)
          {
            updateFailedCw = true;
          }
      }
      break;
    }

  m_acks = m_nacks = 0;

  if (updateFailedCw)
    {
      if(m_backoffType == 0) //Linear Algorithm
      {
        UpdateFailedCwLinear ();
      }
      else if (m_backoffType == 1) //EIED Algorithm
      {
        UpdateFailedCw ();
      }
      else if (m_backoffType == 2) //Constant Algorithm
      {
        UpdateFailedCwConstant ();
      }
      else if (m_backoffType == 3)//BEB Algorithm 
      {
        UpdateFailedCw ();
      }
    }
  else
    {
      if(m_backoffType == 0) //Linear Algorithm
      {
        ResetCw ();
      }
      else if (m_backoffType == 1) //EIED Algorithm
      {
        ResetCwExp ();
      }
      else if (m_backoffType == 2) //Constant Algorithm
      {
        ResetCw ();
      }
      else if (m_backoffType == 3) //BEB Algorithm 
      {
        ResetCw ();
      }
    }

  return m_rng->GetInteger (0, m_cat4ContentionWindowTracedValue.Get ());
}

void
NrCat4LbtAccessManager::SetNrGnbMac (Ptr<NrGnbMac> mac)
{
  NS_LOG_FUNCTION (this << mac);
  NrLbtAccessManager::SetNrGnbMac (mac);
  GetNrGnbMac ()->TraceConnectWithoutContext ("DlHarqFeedback", MakeCallback (&ns3::NrCat4LbtAccessManager::UpdateCwBasedOnHarq,this));
}

void
NrCat4LbtAccessManager::ResetCw (void)
{
  NS_LOG_FUNCTION (this);
  if (m_numberOfAttempts < m_nRetry)
    {
      m_numberOfAttempts++;
    }
  else
    {
      NS_LOG_DEBUG ("CW reset from " << m_cat4ContentionWindowTracedValue  << " to " << m_cat4CwMin);
      m_cat4ContentionWindowTracedValue = m_cat4CwMin;
      m_numberOfAttempts = 0;
    }
}

void
NrCat4LbtAccessManager::ResetCwExp (void)
{
  NS_LOG_FUNCTION (this);
  if (m_numberOfAttempts < m_nRetry)
    {
      m_numberOfAttempts++;
    }
  else
    {
      uint32_t tempValue = m_cat4ContentionWindowTracedValue/2 + 1;
      NS_LOG_DEBUG ("CW reset from " << m_cat4ContentionWindowTracedValue  << " to " << tempValue);
      m_cat4ContentionWindowTracedValue = tempValue;
      m_numberOfAttempts = 0;
    }
}

void
NrCat4LbtAccessManager::UpdateFailedCw (void)
{
  NS_LOG_FUNCTION (this);
  uint32_t oldValue = m_cat4ContentionWindowTracedValue.Get ();
  m_cat4ContentionWindowTracedValue = std::min ( 2 * (m_cat4ContentionWindowTracedValue.Get () + 1) - 1, m_cat4CwMax);
  NS_LOG_DEBUG ("CW updated from " << oldValue << " to " << m_cat4ContentionWindowTracedValue);
}

void
NrCat4LbtAccessManager::UpdateFailedCwConstant (void)
{
  NS_LOG_FUNCTION (this);
  uint32_t oldValue = m_cat4ContentionWindowTracedValue.Get ();
  NS_LOG_DEBUG ("CW updated from " << oldValue << " to " << m_cat4ContentionWindowTracedValue);
}

void
NrCat4LbtAccessManager::UpdateFailedCwLinear (void)
{
  NS_LOG_FUNCTION (this);
  uint32_t oldValue = m_cat4ContentionWindowTracedValue.Get ();
  m_cat4ContentionWindowTracedValue = std::min (m_linBackoff+ m_cat4ContentionWindowTracedValue.Get (), m_cat4CwMax);
  NS_LOG_DEBUG ("CW updated from " << oldValue << " to " << m_cat4ContentionWindowTracedValue);
}

void
NrCat4LbtAccessManager::UpdateCwBasedOnHarq (const DlHarqInfo &dlInfoListReceived)
{
  NS_LOG_FUNCTION (this);

  // If we wanna check only the first subframe/slot, we just have to set a boolean to true
  // after we get a RequestAccess() call, and then here set it to false:
  // if (!m_check)
  //   {
  //     return;
  //   }
  // m_check = false;
  if (dlInfoListReceived.IsReceivedOk ())
    {
      m_acks++;
    }
  else
    {
      m_nacks++;
    }

}



} // ns3 namespace

