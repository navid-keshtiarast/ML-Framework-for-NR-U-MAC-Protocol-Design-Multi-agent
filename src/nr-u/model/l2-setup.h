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
#ifndef L2_SETUP_H
#define L2_SETUP_H

#include <ns3/net-device-container.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/node-container.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/nr-module.h>
#include <ns3/wifi-module.h>
#include <memory>

namespace ns3 {

/**
 * \brief Interface for constructing everything for the L2 devices (NR, wifi, whatever..)
 */
class L2Setup
{
public:
  /**
   * \brief L2Setup constructor
   * \param gnbNodes
   * \param ueNodes
   */
  L2Setup (const NodeContainer &gnbNodes,
           const NodeContainer &ueNodes);
  /**
   * \brief ~L2Setup
   */
  virtual ~L2Setup ();
  /**
   * \brief Get a container of user devices
   * \return A user device container
   */
  virtual const NetDeviceContainer & GetUeDev () const;
  /**
   * \brief Get a container of gnb devices
   * \return A gnb device container
   */
  virtual const NetDeviceContainer & GetGnbDev () const;

  virtual Ipv4InterfaceContainer ConnectToRemotes (const NodeContainer &remoteHosts, const std::string &base) = 0;
  virtual Ipv4InterfaceContainer AssignIpv4ToUe (const std::unique_ptr<Ipv4AddressHelper> &address) const = 0;
  virtual Ipv4InterfaceContainer AssignIpv4ToStations (const std::unique_ptr<Ipv4AddressHelper> &address) const = 0;
  virtual Ptr<NrHelper> GetNrHelper () = 0;
  virtual Ptr<ns3::NrPointToPointEpcHelper> GetEpcHelper () = 0;
  virtual BandwidthPartInfoPtrVector GetBwpVector () = 0;

  virtual void ChangeGnbTxPower (double newTxPower) = 0;
  virtual void ChangeMcs (uint32_t mcsValue) = 0;
  virtual void ChangeAmpdu (uint32_t newAmpdu) = 0;
  virtual void ChangeAmsdu (uint32_t newAmsdu) = 0;
  virtual void ChangeEdThreshold (uint32_t newEdThreshold) = 0;
  virtual void ChangeSlotTime (uint32_t newSlotTime) = 0;

  /**
   * \brief SinrCb typedef
   * First parameter is node ID, second is the SINR value
   */
  typedef Callback<void, uint32_t, double> SinrCb;

  /**
   * \brief Set Sinr Callback
   * \param cb the callback
   */
  void SetSinrCallback (const SinrCb &cb);

  typedef Callback<void, uint32_t, const Time&> ChannelOccupancyTimeCb;

  void SetChannelOccupancyCallback (const ChannelOccupancyTimeCb &cb);

  typedef Callback<void, uint32_t, uint32_t> MacTxDataFailedCb;

  void SetMacTxDataFailedCb (const MacTxDataFailedCb &cb);

  const NodeContainer & GetGnbNodes () const;

  const NodeContainer & GetUeNodes () const;

protected:
  /**
   * \brief Retrieve GNB node list from the scenario
   * \return the GNB node container
   */
  
  /**
   * \brief Retrieve UE node list from the scenario
   * \return the UE node container
   */
  

protected:
  NetDeviceContainer m_ueDev; //!< user net devices
  NetDeviceContainer m_gnbDev;//!< gnb net devices

  SinrCb m_sinrCb; //!< SINR change callback
  ChannelOccupancyTimeCb m_channelOccupancyTimeCb;
  MacTxDataFailedCb m_macTxDataFailedCb;

private:
  NodeContainer m_gnbNodes;  //!< gnb nodes
  NodeContainer m_ueNodes;   //!< ue nodes
};

} //namespace ns3

#endif // L2_SETUP_H
