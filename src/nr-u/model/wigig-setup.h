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
#ifndef WIGIGSETUP_H
#define WIGIGSETUP_H

#include "l2-setup.h"
#include <unordered_map>

namespace ns3 {

class SpectrumChannel;
class PropagationLossModel;
class MmWave3gppChannel;

/**
 * \brief Setup Wigig as L2 technology
 */
class WigigSetup : public L2Setup
{
public:
  WigigSetup (const NodeContainer &gnbNodes, const NodeContainer &ueNodes,
              const Ptr<SpectrumChannel> &inputSpectrumCh, const Ptr<PropagationLossModel> &inputPathloss,
              const Ptr<MmWave3gppChannel> &inputMmWaveChannel,
              double freq, double bw, double apTxPower, double staTxPower,
              double ccaThresholdAp, double ccaThresholdSta, const std::string &ssID,
              uint16_t apAntennaDim1 = 8, uint16_t apAntennaDim2 = 8,
              uint16_t staAntennaDim1 = 4, uint16_t staAntennaDim2 = 4);
  virtual ~WigigSetup () override;

  virtual Ipv4InterfaceContainer AssignIpv4ToUe (const std::unique_ptr<Ipv4AddressHelper> &address) const override;
  virtual Ipv4InterfaceContainer AssignIpv4ToStations (const std::unique_ptr<Ipv4AddressHelper> &address) const override;

  virtual Ipv4InterfaceContainer ConnectToRemotes (const NodeContainer &remoteHosts, const std::string &base) override;

  typedef Callback<void, uint32_t, Time>  ChannelRequestToAccessDurationCb;

  void SetChannelRTACallback (const ChannelRequestToAccessDurationCb &cb);


private:
  void PhyTxDrop (std::string context, Ptr<const Packet> p);
  void PhyRxDrop (std::string context, Ptr<const Packet> p);
  void TxDataTrace (std::string context, Time t);
  void RxDataTrace (std::string context, double sinr);
  void ChannelRequestToAccessDuration (std::string context, Time oldValue, Time newValue);

  ChannelRequestToAccessDurationCb m_craCb;
};

} // namespace ns3
#endif // WIGIGSETUP_H
