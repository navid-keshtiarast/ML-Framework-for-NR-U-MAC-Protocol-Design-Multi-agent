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
 *           Navid Keshtiarast <navid.keshtiarast@mcc.rwth-aachen.de>
 */

#include <ns3/object.h>
#include <ns3/nstime.h>
#include <ns3/callback.h>
#include "ns3/traced-value.h"

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include "ns3/random-variable-stream.h"
#include "ns3/ff-mac-common.h"

#include <ns3/nr-ch-access-manager.h>

#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-spectrum-phy.h"


#ifndef NRLBTACCESSMANAGER_H_
#define NRLBTACCESSMANAGER_H_

namespace ns3 {

class NrSpectrumPhy;
class LbtPhyListener;

/**
 * \ingroup cam-managers
 * \brief Class that implements the generic LBT channel access manager algorithm.
 *
 * Subclasses of NrLbtAccessManager implement different categories of the LBT
 * algorithm. The categories of LBT algorithm according to 3GPP TR 38.889 V1.0.0
 * (2018-11), Section 8.2. are 1, 2, 3 and 4. Category 1 represents
 * the switching gap from the reception to transmission,
 * so it is not considered as a specific sub-type of LBT CAM algorithm.
 * Cat. 2 means that LBT executes without a random back-off, i.e. the
 * LBT procedure is done for a deterministic amount of time, e.g. 25 us.
 * Cat. 3 means that there is a random back-off but the contention window (CW)
 * is fixed; and Cat. 4 means that there is a random back-off and the CW is
 * variable.
 *
 */
class NrLbtAccessManager : public NrChAccessManager
{
public:
  enum LbtState
  {
    IDLE = 0,
    BUSY,
    WAIT_FOR_DEFER,
    WAIT_FOR_BACKOFF,
    TXOP_GRANTED
  };

  static TypeId GetTypeId (void);

  NrLbtAccessManager ();

  virtual ~NrLbtAccessManager ();

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  /**
   * @brief Set spectrum phy instance for this channel access manager
   * @param spectrumPhy specturm phy instance
   */
  virtual void SetNrSpectrumPhy (Ptr<NrSpectrumPhy> spectrumPhy) override;

  /**
   * \brief Set MAC instance for this channel access manager
   * @param mac gNB mac instance
   */
  virtual void SetNrGnbMac (Ptr<NrGnbMac> mac) override;

  /**
   * \brief Sets the callback function that will be called to notify that
   * the channel access is granted, once that LBT algorithm finishes
   * successfully
   * @param cb the callback function to be called when the channel access is granted
   */
  virtual void SetAccessGrantedCallback (const AccessGrantedCallback &cb) override;

  virtual void SetAccessDeniedCallback (const AccessDeniedCallback &cb) override;

  /**
   * \brief Function used by the user of the CAM to request the channel access
   */
  virtual void RequestAccess () override;

  virtual void Cancel () override;

  /**
   * \brief Set the ED threshold to be used in LBT algorithm and energy detection in dBm
   * @param edThreshold energy detection threshold
   */
  void SetEdThreshold (double edThreshold);

  void navid_change_st_after();
  void force_sensing_navid();

  //void malmalak(uint16_t abbb);

  /**
   * \brief Returns the ED threshold that is used in LBT algorithm and for energy detection
   * @return energy detection threshold value in dBm
   */
  double GetEdThreshold () const;

  /**
   * \brief Gets default defer time that is used by LBT algorithm
   * @return
   */
  virtual const Time& GetDeferTime () const;

  /**
   * Sets the defautl defer time to be used by LBT aalgorithm
   * @param deferTime defer time to be used by LBT algorithm
   */
  void SetDeferTime (const Time deferTime);

  /**
   * \brief Gets the duration of the slot that is configured to be used by LBT
   * @return duration of the slot
   */
  const Time& GetSlotTime () const;

  /**
   * \brief Sets the duration of the slot to be used by the LBT algorithm for
   * @param slotTime duration of the slot
   */
  void SetSlotTime (const Time slotTime);

  /**
   * \brief Gets the MCOT of the transmission opportunity that will be granted to
   * the user of LBT CAM, depends on the priority class
   * @return the maximum channel occupancy time
   */
  const Time& GetMcot () const;

  /**
   * \brief Sets the duration of the transmission opportunity that will be granted to
   * the user of LBT CAM, depends on the priority class
   * @param mcot the maximum channel occupancy time
   */
  void SetMcot (const Time mcot);


protected:
  virtual void DoDispose (void) override;

  virtual void DoInitialize (void) override;

  /**
   * \brief Get the random number of backoff slots to be used in the current
   * execution of the LBT algorithm during the backoff process
   * @return the random number of backoff slots
   */
  virtual uint32_t GetBackoffSlots () = 0;

  /**
   * \brief Function that does various things: notifies the user of the
   * granted access, updates variables that are necessary for handling the
   * next channel access request.
   */
  virtual void SetGrant ();

  /**
   * \brief Gets the last TXOP start time
   * @return the last TXOP start time
   */
  Time GetLastTxopStartTime () const;

  TracedValue <uint32_t> m_currentBackoffSlots;            //!< The last value drawn for backoff
  Ptr<UniformRandomVariable> m_rng;                        //!< The uniform random variable used to choose random value for the backoff counter

protected:
  /**
   * Gets the current value of the backoff counter. Used by LBT algoritm during
   * the backoff procedure.
   * @return
   */
  uint32_t GetCurrentBackoffCount (void) const;

  /**
   * \brief Channel is busy for the given duration
   * @param duration during which the channel is detected as busy
   */
  virtual void TransitionToBusy (Time duration);

  /**
   * \brief Gets the current LBT state
   * @return current LBT state
   */
  LbtState GetLbtState () const;

  /**
   * \brief Function is called when defer period is finished to check if the
   * it is necessary to ener to extended CCA
   */
  virtual void RequestAccessAfterDefer ();

  /**
   * \brief Function is called after backoff process is done to grant the
   * access to the CAM user
   */
  void RequestAccessAfterBackoff ();

  /**
   * \brief Used when the LBT state machine returns from the BUSY state
   */
  void TransitionFromBusy ();

  /**
   * \brief Notify the registered callbacks that the access is granted.
   */
  void NotifyAccessGranted ();

  /**
   * \brief Sets the LBT state.
   * @param state New state for LBT state machine
   */
  void SetLbtState (LbtState state);

  // General LBT configuration attributes
  double m_edThreshold {0.0};                              //!< ED threshold to be used for energy detection
  Time m_slotTime {Seconds (0)};                           //!< The duration of a single slot when listening the channel, that is considered by LBT algorithm
  Time m_deferTime {Seconds (0)};                          //!< The defer time of LBT algorithm
  Time m_mcot {Seconds (0)};                               //!< The maximum channel occupancy time
  
  // Other variables
  LbtState m_state;                                        //!< LBT algorithm state
  uint32_t m_backoffCount;                                 //!< Counter for current backoff remaining
  bool m_grantRequested {false};                           //!< The indicator for tracking whether the channel access grant is requested
  EventId m_waitForDeferEventId;                           //!< The ID of event that is triggered when defer time is over
  EventId m_waitForBackoffEventId;                         //!< The ID of the event that is triggered once that the backoff is over
  EventId m_waitForBusyEventId;                            //!< The ID of the event that is triggered once that the channel busy state is over
  Time m_backoffStartTime {Seconds (0)};                   //!< The backoff start time
  Time m_lastBusyTime {Seconds (0)};                       //!< The last time at which the channel was busy
  Time m_lastTxopStartTime {Seconds (0)};                  //!< The last grant start time
  std::vector<AccessGrantedCallback> m_accessGrantedCb;    //!< The list of registered callacks for the channel access grant notification
  std::vector<AccessDeniedCallback> m_accessDeniedCb;      //!< The list of registered callback for the notification of access denied event
  //TracedValue <uint16_t> kalekiri;
  
};

/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/

/**
 * \ingroup cam-managers
 * \brief Class that implements the Cat2 LBT channel access manager algorithm.
 *
 */
class NrCat2LbtAccessManager : public NrLbtAccessManager
{

public:

  static TypeId GetTypeId (void);

  virtual void RequestAccess () override;

protected:
  virtual void DoDispose (void) override;

  virtual void DoInitialize (void) override;

  virtual uint32_t GetBackoffSlots () override;

  virtual void RequestAccessAfterDefer () override;

  virtual void TransitionToBusy (Time duration) override;

  void NotifyAccessDenied ();
};

/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/

/**
 * \ingroup cam-managers
 * \brief Class that implements the Cat2 LBT channel access manager algorithm.
 *
 */
class NrCat3LbtAccessManager : public NrLbtAccessManager
{

public:
  static TypeId GetTypeId (void);

  /**
   * \brief Sets the value of the contention window that will be used with Cat. 3 LBT
   * @param contentionWindow the contentionWindow to be used with Cat. 3 LBT
   */
  void SetCat3ContentionWindow (const uint32_t& contentionWindow);

  /**
   * \brief Gets the contention window value that is used with Cat. 3 LBT is
   * @return the configured contention window value
   */
  const uint32_t& GetCat3ContentionWindow () const;

protected:
  virtual void DoDispose (void) override;

  virtual void DoInitialize (void) override;

  /**
   * \brief Get the number of backoff slots to be used in the current
   * @return the random number of backoff slots
   */
  virtual uint32_t GetBackoffSlots () override;

private:
  uint32_t m_cat3ContentionWindow {0};                     //!< The fixed CW size that is used by Cat. 3 LBT
};

/**-------------------------------------------------------------------------------------------------------------------------------------------------------------------**/

/**
 * \ingroup cam-managers
 * \brief Class that implements the Cat2 LBT channel access manager algorithm.
 *
 */
class NrCat4LbtAccessManager : public NrLbtAccessManager
{

public:
  enum CWUpdateRule_t
  {
    ALL_NACKS,
    ANY_NACK,
    NACKS_10_PERCENT,
    NACKS_80_PERCENT,
  };

  static TypeId GetTypeId (void);

  /**
   * \brief Function that is called when the HARQ feedback is being received.
   * The HARQ feedback is being processed and based on it is being decided
   * if it is necessary to update the contention window.
   * @param harqFeedback the list of HARQ feedbacks
   */
  void UpdateCwBasedOnHarq (const DlHarqInfo &harqFeedback);

  /**
   * \brief Set MAC instance for this channel access manager
   * @param mac gNB mac instance
   */
  virtual void SetNrGnbMac (Ptr<NrGnbMac> mac) override;

  void SetCat4MinCw (const uint32_t contentionWindow);

  void SetCat4MaxCw (const uint32_t contentionWindow);

  void SetCat4LinBackoff (const uint32_t backoffConst);

  void SetCat4BackoffType (const uint32_t backoffType);

  /**
   * \brief Gets the contention window value that is used with Cat. 3 LBT is
   * @return the configured contention window value
   */
  const uint32_t& GetCat4MinCw () const;

  const uint32_t& GetCat4MaxCw () const;

  const uint32_t& GetCat4LinBackoff () const;

  const uint32_t& GetCat4BackoffType () const;

protected:
  virtual void DoInitialize (void) override;

  /**
   * \brief Get the number of backoff slots to be used in the current
   * @return the random number of backoff slots
   */
  virtual uint32_t GetBackoffSlots () override;

private:
  /**
   * \brief After unsuccessful transmission the contention window should be
   * increased according to the exponential rule.
   */
  void UpdateFailedCw ();

  void UpdateFailedCwConstant ();

  void UpdateFailedCwLinear ();

  /**
   * \brief Resets the current CW size
   */
  void ResetCw (void);

  void ResetCwExp (void);

  // Traced values
  TracedValue <uint32_t> m_cat4ContentionWindowTracedValue;//!< The current value of the contention window, updated by LBT algorithm in the case of variable CW
  
  uint32_t m_cat4CwMin {0};                                //!< The minimum value of the contention window when LBT Cat.4 used, this configuration depends on the class priority and may be in future deprecated in the class priority is introduced as the configuration parameter of LBT CAM
  uint32_t m_cat4CwMax {0};                                //!< The maximum value of the contention window when LBT Cat.4 used, this configuration depends on the class priority and may be in future deprecated in the class priority is introduced as the configuration parameter of LBT CAM
  uint32_t m_linBackoff {4};                         
  CWUpdateRule_t m_cat4CwUpdateRule;                       //!< In case of LBT Cat.4 which CW update rule to use
  uint8_t m_numberOfAttempts {0};                          //!< How many times the algorithm already retried with the same contention window
  uint8_t m_nRetry {0};                                    //!< How many times to retry with the same contention window before reseting it
  uint32_t m_nacks {0};
  uint32_t m_acks  {0};
  uint32_t m_backoffType {3};
};


} // namespace ns3

#endif /* NRLBTACCESSMANAGER_H_ */
