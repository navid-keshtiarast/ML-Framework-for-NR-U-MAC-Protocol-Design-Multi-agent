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
#include "nr-single-bwp-setup.h"
#include <ns3/nr-module.h>
#include <ns3/names.h>
#include <ns3/spectrum-wifi-helper.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/wifi-radio-energy-model-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <ns3/adhoc-wifi-mac.h>
#include <ns3/ampdu-subframe-header.h>
#include <ns3/ampdu-tag.h>
#include <ns3/amsdu-subframe-header.h>
#include <ns3/ap-wifi-mac.h>
#include <ns3/block-ack-agreement.h>
#include <ns3/block-ack-manager.h>
#include <ns3/block-ack-type.h>
#include <ns3/block-ack-window.h>
#include <ns3/capability-information.h>
#include <ns3/channel-access-manager.h>
#include <ns3/ctrl-headers.h>
#include <ns3/edca-parameter-set.h>
#include <ns3/error-rate-model.h>
#include <ns3/extended-capabilities.h>
#include <ns3/frame-capture-model.h>
#include <ns3/frame-exchange-manager.h>
#include <ns3/constant-obss-pd-algorithm.h>
#include <ns3/he-capabilities.h>
#include <ns3/he-configuration.h>
#include <ns3/he-frame-exchange-manager.h>
#include <ns3/he-operation.h>
#include <ns3/he-phy.h>
#include <ns3/he-ppdu.h>
#include <ns3/he-ru.h>
#include <ns3/mu-snr-tag.h>
#include <ns3/multi-user-scheduler.h>
#include <ns3/obss-pd-algorithm.h>
#include <ns3/rr-multi-user-scheduler.h>
#include <ns3/ht-capabilities.h>
#include <ns3/ht-configuration.h>
#include <ns3/ht-frame-exchange-manager.h>
#include <ns3/ht-operation.h>
#include <ns3/ht-phy.h>
#include <ns3/ht-ppdu.h>
#include <ns3/interference-helper.h>
#include <ns3/mac-rx-middle.h>
#include <ns3/mac-tx-middle.h>
#include <ns3/mgt-headers.h>
#include <ns3/mpdu-aggregator.h>
#include <ns3/msdu-aggregator.h>
#include <ns3/nist-error-rate-model.h>
#include <ns3/dsss-error-rate-model.h>
#include <ns3/dsss-parameter-set.h>
#include <ns3/dsss-phy.h>
#include <ns3/dsss-ppdu.h>
#include <ns3/erp-information.h>
#include <ns3/erp-ofdm-phy.h>
#include <ns3/erp-ofdm-ppdu.h>
#include <ns3/ofdm-phy.h>
#include <ns3/ofdm-ppdu.h>
#include <ns3/originator-block-ack-agreement.h>
#include <ns3/phy-entity.h>
#include <ns3/preamble-detection-model.h>
#include <ns3/qos-blocked-destinations.h>
#include <ns3/qos-frame-exchange-manager.h>
#include <ns3/qos-txop.h>
#include <ns3/qos-utils.h>
#include <ns3/aarf-wifi-manager.h>
#include <ns3/aarfcd-wifi-manager.h>
#include <ns3/amrr-wifi-manager.h>
#include <ns3/aparf-wifi-manager.h>
#include <ns3/arf-wifi-manager.h>
#include <ns3/cara-wifi-manager.h>
#include <ns3/constant-rate-wifi-manager.h>
#include <ns3/ideal-wifi-manager.h>
#include <ns3/minstrel-ht-wifi-manager.h>
#include <ns3/minstrel-wifi-manager.h>
#include <ns3/onoe-wifi-manager.h>
#include <ns3/parf-wifi-manager.h>
#include <ns3/rraa-wifi-manager.h>
#include <ns3/rrpaa-wifi-manager.h>
#include <ns3/thompson-sampling-wifi-manager.h>
#include <ns3/recipient-block-ack-agreement.h>
#include <ns3/error-rate-tables.h>
#include <ns3/regular-wifi-mac.h>
#include <ns3/simple-frame-capture-model.h>
#include <ns3/snr-tag.h>
#include <ns3/spectrum-wifi-phy.h>
#include <ns3/ssid.h>
#include <ns3/sta-wifi-mac.h>
#include <ns3/status-code.h>
#include <ns3/supported-rates.h>
#include <ns3/table-based-error-rate-model.h>
#include <ns3/threshold-preamble-detection-model.h>
#include <ns3/txop.h>
#include <ns3/vht-capabilities.h>
#include <ns3/vht-configuration.h>
#include <ns3/vht-frame-exchange-manager.h>
#include <ns3/vht-operation.h>
#include <ns3/vht-phy.h>
#include <ns3/vht-ppdu.h>
#include <ns3/wifi-ack-manager.h>
#include <ns3/wifi-acknowledgment.h>
#include <ns3/wifi-default-ack-manager.h>
#include <ns3/wifi-default-protection-manager.h>
#include <ns3/wifi-information-element-vector.h>
#include <ns3/wifi-information-element.h>
#include <ns3/wifi-mac-header.h>
#include <ns3/wifi-mac-queue-item.h>
#include <ns3/wifi-mac-queue.h>
#include <ns3/wifi-mac-trailer.h>
#include <ns3/wifi-mac.h>
#include <ns3/wifi-mode.h>
#include <ns3/wifi-mpdu-type.h>
#include <ns3/wifi-net-device.h>
#include <ns3/wifi-phy-band.h>
#include <ns3/wifi-phy-common.h>
#include <ns3/wifi-phy-listener.h>
#include <ns3/wifi-phy-operating-channel.h>
#include <ns3/wifi-phy-state-helper.h>
#include <ns3/wifi-phy-state.h>
#include <ns3/wifi-phy.h>
#include <ns3/wifi-ppdu.h>
#include <ns3/wifi-protection-manager.h>
#include <ns3/wifi-protection.h>
#include <ns3/wifi-psdu.h>
#include <ns3/wifi-radio-energy-model.h>
#include <ns3/wifi-remote-station-info.h>
#include <ns3/wifi-remote-station-manager.h>
#include <ns3/wifi-spectrum-phy-interface.h>
#include <ns3/wifi-spectrum-signal-parameters.h>
#include <ns3/wifi-standards.h>
#include <ns3/wifi-tx-current-model.h>
#include <ns3/wifi-tx-parameters.h>
#include <ns3/wifi-tx-timer.h>
#include <ns3/wifi-tx-vector.h>
#include <ns3/wifi-utils.h>
#include <ns3/yans-error-rate-model.h>
#include <ns3/yans-wifi-channel.h>
#include <ns3/yans-wifi-phy.h>
#include <ns3/object-map.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/nr-lbt-access-manager.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>

namespace ns3 {

static void
AttachToClosestGnb (Ptr<NrHelper> helper, Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices)
{
  NS_ASSERT_MSG (gnbDevices.GetN () > 0, "empty gNB device container");
  Vector uepos = ueDevice->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
  double minDistance = std::numeric_limits<double>::infinity ();
  Ptr<NetDevice> closestGnbDevice;
  for (NetDeviceContainer::Iterator i = gnbDevices.Begin (); i != gnbDevices.End (); ++i)
    {
      Vector gnbpos = (*i)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
      double distance = CalculateDistance (uepos, gnbpos);
      if (distance < minDistance)
        {
          minDistance = distance;
          closestGnbDevice = *i;
        }
    }
  NS_ASSERT (closestGnbDevice != nullptr);

  std::cout << "Attaching " << Names::FindName (ueDevice->GetNode ()) << " to " <<
               Names::FindName (closestGnbDevice->GetNode ()) << std::endl;

  helper->AttachToEnb (ueDevice, closestGnbDevice);
}

static void
AttachToClosestGnb (Ptr<NrHelper> helper, NetDeviceContainer ueDevices, NetDeviceContainer gnbDevices)
{
  for (NetDeviceContainer::Iterator i = ueDevices.Begin (); i != ueDevices.End (); i++)
    {
      AttachToClosestGnb (helper, *i, gnbDevices);
    }
}

NrSingleBwpSetup::NrSingleBwpSetup (const NodeContainer &gnbNodes, 
                                    const NodeContainer &ueNodes, 
                                    ns3::Ptr<NrHelper> nrHelper, 
                                    ns3::Ptr<ns3::NrPointToPointEpcHelper> epcHelper,
                                    Ptr<IdealBeamformingHelper> idealBeamformingHelper,
                                    const Ptr<SpectrumChannel> &channel,
                                    const Ptr<ThreeGppPropagationLossModel> &propagation,
                                    const Ptr<ThreeGppSpectrumPropagationLossModel> &spectrumPropagation,
                                    double freq, double bw, uint32_t num, double gnbTxPower, double ueTxPower,
                                    const std::unordered_map<uint32_t, uint32_t> &ueGnbMap,
                                    const std::string &gnbCamType, const std::string &ueCamType,
                                    const std::string &scheduler, const BandwidthPartInfo::Scenario &scenario, int Duplex,
                                    std::string lbtBackoffType,
                                    double lbtEDThreshold, double lbtSlotTime, double lbtDeferTime, double lbtMcot,
                                    double cat2ED, double cat2DeferTime,
                                    uint32_t cat3CW,
                                    uint32_t cat4RetryLimit, uint32_t cat4MinCw, uint32_t cat4MaxCw, uint32_t cat4CwUpdateRule, uint32_t cat4LinearBackoff,
                                    double onoffChangeTime,
                                    uint32_t mcsValue)
  : L2Setup (gnbNodes, ueNodes),
    m_ueGnbMap (ueGnbMap)
{
  std::cout << "Configuring NR with " << gnbNodes.GetN ()
            << " gNBs, and " << ueNodes.GetN () << " UEs" << std::endl;
  
  for (auto it = gnbNodes.Begin(); it != gnbNodes.End(); ++it)
    {
      std::stringstream ss;
      ss << "GNB-NR-" << (*it)->GetId();
      Names::Add(ss.str (), *it);
    }

  for (auto it = ueNodes.Begin(); it != ueNodes.End(); ++it)
    {
      std::stringstream ss;
      ss << "UE-NR-" << (*it)->GetId();
      Names::Add(ss.str (), *it);
    }
  
  // setup the NR simulation
  m_helper = nrHelper;

  // Configure beamforming method
  //m_helper->SetBeamformingHelper (idealBeamformingHelper);

  // Configure scheduler
  m_helper->SetSchedulerTypeId (TypeId::LookupByName(scheduler));
  
  /*
  // Configure BWPs
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;  // each band is composed of a single component carrier
  CcBwpCreator::SimpleOperationBandConf bandConf (freq, bw, numCcPerBand, scenario);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);
  
  // Use provided channel, propagation, and fading models (instead of initializing them in InitializeOperationBand)
  m_helper->InitializeOperationBand (&band, 0x00); //flag to avoid initializing them
  NS_ASSERT (band.m_cc.size () == 1 && band.m_cc[0]->m_bwp.size ());
  const auto & bwp = band.m_cc[0]->m_bwp[0];
  bwp->m_channel = channel;
  bwp->m_propagation = propagation;
  bwp->m_3gppChannel = spectrumPropagation;

  allBwps = CcBwpCreator::GetAllBwps ({band});
  
  // Configure CAMs
  m_helper->SetGnbChannelAccessManagerTypeId (TypeId::LookupByName(gnbCamType));
  m_helper->SetUeChannelAccessManagerTypeId (TypeId::LookupByName(ueCamType));
  
  //m_epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  m_epcHelper = epcHelper;
  m_helper->SetEpcHelper (m_epcHelper);
  m_helper->Initialize ();
  
  // install NR net devices
  m_gnbDev = m_helper->InstallGnbDevice (GetGnbNodes (), allBwps);
  m_ueDev = m_helper->InstallUeDevice (GetUeNodes (), allBwps);
  
  double gnbX = pow (10, gnbTxPower / 10);
  double ueX = pow (10, ueTxPower / 10);

  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      phy->SetNumerology (num);
      phy->SetTxPower (10 * log10 (gnbX));

      Ptr<NrSpectrumPhy> gnbSpectrumPhy = phy->GetSpectrumPhy ();
      std::stringstream nodeId;
      nodeId << (*it)->GetNode ()->GetId ();
      gnbSpectrumPhy->TraceConnect ("RxPacketTraceEnb", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::GnbReception, this));
      gnbSpectrumPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::TxDataTrace, this));
      gnbSpectrumPhy->TraceConnect ("TxCtrlTrace", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::TxCtrlTrace, this));
    }

  m_ueNum = m_ueDev.GetN ();

  for (auto it = m_ueDev.Begin (); it != m_ueDev.End (); ++it)
    {
      Ptr<NrUePhy> phy = m_helper->GetUePhy (*it, 0);
      phy->SetNumerology (num);
      phy->SetTxPower (10 * log10 (ueX));

      Ptr<NrSpectrumPhy> ueSpectrumPhy = phy->GetSpectrumPhy ();
      std::stringstream nodeId;
      nodeId << (*it)->GetNode ()->GetId ();

      ueSpectrumPhy->TraceConnect ("RxPacketTraceUe", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::UeReception, this));
      ueSpectrumPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::TxDataTrace, this));
      ueSpectrumPhy->TraceConnect ("TxCtrlTrace", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::TxCtrlTrace, this));
    }
  
  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = m_ueDev.Begin (); it != m_ueDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }
  */

  if(Duplex == 0)
  {
    BandwidthPartInfoPtrVector allBwps;
    m_helper->SetGnbChannelAccessManagerTypeId (TypeId::LookupByName(gnbCamType));
    
    /*
    Update 24.08.2023
    Updating mac parameters
    */ 
    if (gnbCamType == "ns3::NrLbtAccessManager" || gnbCamType == "ns3::NrCat2LbtAccessManager" || gnbCamType == "ns3::NrCat3LbtAccessManager" || gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      m_helper->SetGnbChannelAccessManagerAttribute ("EnergyDetectionThreshold",DoubleValue(lbtEDThreshold));
      m_helper->SetGnbChannelAccessManagerAttribute ("Slot",TimeValue (MicroSeconds (lbtSlotTime)));
      m_helper->SetGnbChannelAccessManagerAttribute ("DeferTime",TimeValue (MicroSeconds (lbtDeferTime)));
      m_helper->SetGnbChannelAccessManagerAttribute ("Mcot",TimeValue (MilliSeconds (lbtMcot)));
      
      if (gnbCamType == "ns3::NrCat2LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat2EDThreshold",DoubleValue(cat2ED));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat2DeferTime",TimeValue (MicroSeconds (cat2DeferTime)));
      }
      else if (gnbCamType == "ns3::NrCat3LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat3ContentionWindow",UintegerValue(cat3CW));
      }
      else if (gnbCamType == "ns3::NrCat4LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("RetryLimit",UintegerValue(cat4RetryLimit));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4MinCw",UintegerValue(cat4MinCw));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4MaxCw",UintegerValue(cat4MaxCw));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4CwUpdateRule",EnumValue(cat4CwUpdateRule));
      }
    }
    else if (gnbCamType == "ns3::NrOnOffAccessManager")
    {
      m_helper->SetGnbChannelAccessManagerAttribute ("OnOffChangeTime",TimeValue (MilliSeconds (onoffChangeTime)));
    }

    /*
    Update 19.12.2023
    Updating backoff algorithm
    */
    if (gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      if (lbtBackoffType == "Linear")
      {
        uint32_t backoffType = 0;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4LinearBackoff",UintegerValue(cat4LinearBackoff));
      }
      else if (lbtBackoffType == "Exponential")
      {
        uint32_t backoffType = 1;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
      else if (lbtBackoffType == "Constant" )
      {
        uint32_t backoffType = 2;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
      else if (lbtBackoffType == "Binary")
      {
        uint32_t backoffType = 3;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
    } 
    

    m_helper->SetUeChannelAccessManagerTypeId (TypeId::LookupByName(ueCamType));
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf (freq, bw, numCcPerBand, scenario);
    CcBwpCreator::SimpleOperationBandConf bandConfUL (2.535e9, bw, numCcPerBand, scenario);
    //bandConf.m_numBwp=2;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);
    OperationBandInfo bandUL = ccBwpCreator.CreateOperationBandContiguousCc (bandConfUL);
    m_helper->InitializeOperationBand (&band, 0x00);
    const auto & bwp = band.m_cc[0]->m_bwp[0];
    bwp->m_channel = channel;
    bwp->m_propagation = propagation;
    bwp->m_3gppChannel = spectrumPropagation;
    m_helper->InitializeOperationBand (&band);
    m_helper->InitializeOperationBand (&bandUL);

    //14.11.2023
    uint32_t bwpIdForLowLat = 0;
    // TODO check later when QoS is in place, that the type of bearer coresponds to the
    // type of traffic gNb routing between Bearer and bandwidth part
    m_helper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                                 UintegerValue(bwpIdForLowLat));

    // Ue routing between Bearer and bandwidth part
    m_helper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                                UintegerValue(bwpIdForLowLat));
    //m_helper->SetSchedulerTypeId (TypeId::LookupByName(scheduler));
    allBwps = CcBwpCreator::GetAllBwps ({band,bandUL});
    m_allBwps = allBwps;
    m_helper->SetPathlossAttribute("ShadowingEnabled", BooleanValue (false));
    m_epcHelper = epcHelper;
    m_helper->SetEpcHelper (m_epcHelper);
    m_helper->Initialize ();  
    
    
    // install NR net devices
    m_gnbDev = m_helper->InstallGnbDevice (GetGnbNodes (), allBwps);
    m_ueDev = m_helper->InstallUeDevice (GetUeNodes (), allBwps);

    /* 
    03.01.2024
    Install multiple NR net devices in one node
    m_ueDev = m_helper->InstallUeDevice (GetUeNodes (), allBwps);
    m_gnbDev.Add (m_helper->InstallMultipleGnbDevice (GetGnbNodes (), allBwps));
    */

    for(uint32_t f=0;f<m_gnbDev.GetN();f++)
    {
      std::cout << "Error test" << std::endl;
      m_helper->GetGnbPhy (m_gnbDev.Get (f), 0)->SetAttribute ("Numerology", UintegerValue (0));
      m_helper->GetGnbPhy (m_gnbDev.Get (f), 0)->SetAttribute ("Pattern", StringValue ("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
      //m_helper->GetGnbPhy (m_gnbDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (23.0));
      //m_helper->GetUePhy (m_ueDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (23.0));
      
      //21.11.2023
      if (mcsValue <= 28)
      {
        m_helper->GetScheduler (m_gnbDev.Get (f), 0)->SetAttribute("McsValue", UintegerValue(mcsValue));
      }

      // BWP2, FDD-UL
      m_helper->GetGnbPhy (m_gnbDev.Get (f), 1)->SetAttribute ("Numerology", UintegerValue (0));
      m_helper->GetGnbPhy (m_gnbDev.Get (f), 1)->SetAttribute ("Pattern", StringValue ("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
      //m_helper->GetGnbPhy (m_gnbDev.Get (0), 1)->SetAttribute ("TxPower", DoubleValue (23.0));
      //m_helper->GetUePhy (m_ueDev.Get (0), 1)->SetAttribute ("TxPower", DoubleValue (23.0));
      m_helper->GetBwpManagerGnb (m_gnbDev.Get (f))->SetOutputLink (1, 0);
      m_helper->GetBwpManagerUe (m_ueDev.Get (f))->SetOutputLink (0, 1);
    }
  }

  if(Duplex==1)
  {
  // Configure BWPs
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;  // each band is composed of a single component carrier
  CcBwpCreator::SimpleOperationBandConf bandConf (freq, bw, numCcPerBand, scenario);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);

  // Use provided channel, propagation, and fading models (instead of initializing them in InitializeOperationBand)
  // m_helper->InitializeOperationBand (&band, 0x00); //flag to avoid initializing them
  // NS_ASSERT (band.m_cc.size () == 1 && band.m_cc[0]->m_bwp.size ());
  // const auto & bwp = band.m_cc[0]->m_bwp[0];
  // bwp->m_channel = channel;
  // bwp->m_propagation = propagation;
  // bwp->m_3gppChannel = spectrumPropagation;
  m_helper->InitializeOperationBand (&band);
  allBwps = CcBwpCreator::GetAllBwps ({band});
  m_helper->SetGnbChannelAccessManagerTypeId (TypeId::LookupByName(gnbCamType));

  //14.11.2023
  uint32_t bwpIdForLowLat = 0;
  // TODO check later when QoS scheduler is in place, that the type of bearer coresponds to the
  // type of traffic gNb routing between Bearer and bandwidth part
  m_helper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                               UintegerValue(bwpIdForLowLat));

  // Ue routing between Bearer and bandwidth part
  m_helper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                              UintegerValue(bwpIdForLowLat));

  /*
  Update 24.08.2023
  Updating mac parameters
  */ 
  if (gnbCamType == "ns3::NrLbtAccessManager" || gnbCamType == "ns3::NrCat2LbtAccessManager" || gnbCamType == "ns3::NrCat3LbtAccessManager" || gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      m_helper->SetGnbChannelAccessManagerAttribute ("EnergyDetectionThreshold",DoubleValue(lbtEDThreshold));
      m_helper->SetGnbChannelAccessManagerAttribute ("Slot",TimeValue (MicroSeconds (lbtSlotTime)));
      m_helper->SetGnbChannelAccessManagerAttribute ("DeferTime",TimeValue (MicroSeconds (lbtDeferTime)));
      m_helper->SetGnbChannelAccessManagerAttribute ("Mcot",TimeValue (MilliSeconds (lbtMcot)));
      
      if (gnbCamType == "ns3::NrCat2LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat2EDThreshold",DoubleValue(cat2ED));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat2DeferTime",TimeValue (MicroSeconds (cat2DeferTime)));
      }
      else if (gnbCamType == "ns3::NrCat3LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat3ContentionWindow",UintegerValue(cat3CW));
      }
      else if (gnbCamType == "ns3::NrCat4LbtAccessManager")
      {
        m_helper->SetGnbChannelAccessManagerAttribute ("RetryLimit",UintegerValue(cat4RetryLimit));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4MinCw",UintegerValue(cat4MinCw));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4MaxCw",UintegerValue(cat4MaxCw));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4CwUpdateRule",EnumValue(cat4CwUpdateRule));
      }
    }
    else if (gnbCamType == "ns3::NrOnOffAccessManager")
    {
      m_helper->SetGnbChannelAccessManagerAttribute ("OnOffChangeTime",TimeValue (MilliSeconds (onoffChangeTime)));
    }

  /*
  Update 19.12.2023
  Updating backoff algorithm
  */
  if (gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      if (lbtBackoffType == "Linear")
      {
        uint32_t backoffType = 0;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
        m_helper->SetGnbChannelAccessManagerAttribute ("Cat4LinearBackoff",UintegerValue(cat4LinearBackoff));
      }
      else if (lbtBackoffType == "Exponential")
      {
        uint32_t backoffType = 1;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
      else if (lbtBackoffType == "Constant" )
      {
        uint32_t backoffType = 2;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
      else if (lbtBackoffType == "Binary")
      {
        uint32_t backoffType = 3;
        m_helper->SetGnbChannelAccessManagerAttribute ("BackOffType",UintegerValue(backoffType));
      }
    }

  m_helper->SetUeChannelAccessManagerTypeId (TypeId::LookupByName(ueCamType));
  m_epcHelper = epcHelper;
  m_helper->SetEpcHelper (m_epcHelper);
  m_helper->Initialize ();

  // install NR net devices
  m_gnbDev = m_helper->InstallGnbDevice (GetGnbNodes (), allBwps);
  m_ueDev = m_helper->InstallUeDevice (GetUeNodes (), allBwps);

  }
  // Configure CAMs
  
  //m_helper->EnableTraces();
  //m_helper->EnableRlcTraces();

  // install NR net devices

  double gnbX = pow (10, gnbTxPower / 10);
  double ueX = pow (10, ueTxPower / 10);
  
  //////double itiri=0;
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      phy->SetNumerology (num);
      phy->SetTxPower (10 * log10 (gnbX));

      Ptr<NrSpectrumPhy> gnbSpectrumPhy = phy->GetSpectrumPhy ();

      //gnbSpectrumPhy->GetSpectrumChannel()->AddRx(gnbSpectrumPhy);
      
      std::stringstream nodeId;
      nodeId << (*it)->GetNode ()->GetId ();
      gnbSpectrumPhy->TraceConnect ("RxPacketTraceEnb", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::GnbReception, this));
      gnbSpectrumPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::TxDataTrace, this));
      gnbSpectrumPhy->TraceConnect ("TxCtrlTrace", nodeId.str (),
                                    MakeCallback (&NrSingleBwpSetup::TxCtrlTrace, this));
      
      /////////
      // Ptr<Node> node = (*it)->GetNode ();
      // Ptr<NrGnbNetDevice> nrgnbNetDevice = (*it)->GetObject<NrGnbNetDevice> ();
      // Ptr<SpectrumChannel> downlinkSpectrumChannel = nrgnbNetDevice->GetPhy (0)->GetSpectrumPhy()->GetSpectrumChannel();

      
      // SpectrumWifiPhyHelper spectrumPhy;
      // spectrumPhy.SetChannel (downlinkSpectrumChannel);
      // WifiHelper wifi;
      // wifi.SetStandard (WIFI_STANDARD_80211ax_6GHZ);
      // WifiMacHelper mac;
      // //spectrumPhy.Set ("ShortGuardEnabled", BooleanValue (false));
      // spectrumPhy.Set ("ChannelWidth", UintegerValue (20));
      // spectrumPhy.Set ("TxGain", DoubleValue (0));
      // spectrumPhy.Set ("RxGain", DoubleValue (0));
      // spectrumPhy.Set ("TxPowerStart", DoubleValue (0));
      // spectrumPhy.Set ("TxPowerEnd", DoubleValue (0));
      // spectrumPhy.Set ("RxNoiseFigure", DoubleValue (7));
      // //spectrumPhy.Set ("Receivers", UintegerValue (2));
      // //spectrumPhy.Set ("Transmitters", UintegerValue (2));
      // spectrumPhy.Set ("ChannelNumber", UintegerValue (1));
      // wifi.SetRemoteStationManager ("ns3::IdealWifiManager");
      // //which implements a Wi-Fi MAC that does not perform any kind of beacon generation, probing, or association
      // mac.SetType ("ns3::AdhocWifiMac");
      // Ptr<NetDevice> monitor = (wifi.Install (spectrumPhy, mac, monitornode.Get(itiri))).Get (0);
      // Ptr<WifiPhy> wifiPhy = monitor->GetObject<WifiNetDevice> ()->GetPhy ();
      // Ptr<SpectrumWifiPhy> spectrumWifiPhy = DynamicCast<SpectrumWifiPhy> (wifiPhy);
      // UintegerValue numCols, numRows;
      // Ptr<UniformPlanarArray> antennaArray = CreateObject<UniformPlanarArray> ();
      // antennaArray->GetAttribute ("NumColumns", numCols);
      // antennaArray->GetAttribute ("NumRows", numRows);
      // antennaArray->SetBeamformingVector (CreateQuasiOmniBfv (numCols.Get (), numRows.Get ()));
      // // // registering AP devices to 3GPP model
      // std::cout<<"hihoy";
      // spectrumPropagation->AddDevice (monitor, antennaArray);
      

      // // // registering STAs devices to 3GPP model
     
      // //spectrumPropagation->AddDevice (monitor, antennaArray);
   
      // //Ptr<MacLow> macLow = monitor->GetObject<WifiNetDevice>()->GetMac();
      // itiri++;
      // Ptr<NrGnbPhy> nrPhy = (*it)->GetObject<NrGnbNetDevice> ()->GetPhy (0);
      // Ptr<NrGnbMac> nrMac = (*it)->GetObject<NrGnbNetDevice> ()->GetMac (0);
      // Ptr<NrChAccessManager> nrlbta=nrPhy->GetCam();
      // std::cout<<nrlbta;
      /////////////
      //Ptr<NrCat4LbtAccessManager> nrlbta4=DynamicCast<NrCat4LbtAccessManager>(nrlbta);
      //Ptr<NrCat4LbtAccessManager> nrlbta4 = CreateObject<NrCat4LbtAccessManager> ();
      //nrlbtAccessManager->SetWifiPhy (spectrumWifiPhy);
      //nrlbta4->SetWifiPhy (spectrumWifiPhy);
      //std::cout<<nrlbta4;
      //nrlbta4->SetNrGnbMac(nrMac);
      //nrlbta4->SetNrGnbPhy(nrPhy);
      //nrlbtAccessManager->SetGnbbMac(nrMac);
      //nrlbtAccessManager->SetNrGnbPhy(nrPhy);
      //nrlbtAccessManager->SetNrSpectrumPhy(nrgnbNetDevice->GetPhy (0)->GetSpectrumPhy());

    }

    // for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    // {
      
    //   Ptr<NrGnbNetDevice> nrgnbNetDevice1 = m_gnbDev.Get(0)->GetObject<NrGnbNetDevice> ();
    //   Ptr<NrGnbNetDevice> nrgnbNetDevice2 = m_gnbDev.Get(1)->GetObject<NrGnbNetDevice> ();
    //  Ptr<SpectrumChannel> SpectrumC=nrgnbNetDevice1->GetPhy (0)->GetSpectrumPhy()->GetSpectrumChannel();
    // std::cout<<"injara bebin2"<<nrgnbNetDevice2->GetPhy (0)->GetSpectrumPhy()->GetSpectrumChannel();
    // nrgnbNetDevice2->GetPhy (0)->GetSpectrumPhy()->SetChannel(SpectrumC);


  m_ueNum = m_ueDev.GetN ();

  for (auto it = m_ueDev.Begin (); it != m_ueDev.End (); ++it)
    {
      Ptr<NrUePhy> phy = m_helper->GetUePhy (*it, 0);
      phy->SetNumerology (num);
      phy->SetTxPower (10 * log10 (ueX));

      Ptr<NrSpectrumPhy> ueSpectrumPhy = phy->GetSpectrumPhy ();
      
      std::stringstream nodeId;
      nodeId << (*it)->GetNode ()->GetId ();

      ueSpectrumPhy->TraceConnect ("RxPacketTraceUe", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::UeReception, this));
      ueSpectrumPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::TxDataTrace, this));
      ueSpectrumPhy->TraceConnect ("TxCtrlTrace", nodeId.str (),
                                   MakeCallback (&NrSingleBwpSetup::TxCtrlTrace, this));
      
    }

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

  for (auto it = m_ueDev.Begin (); it != m_ueDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }


}

NrSingleBwpSetup::~NrSingleBwpSetup ()
{

}

Ipv4InterfaceContainer
NrSingleBwpSetup::ConnectToRemotes (const NodeContainer &remoteHosts, const std::string &base)
{
  // create the internet and install the IP stack on the UEs
  // get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = m_epcHelper->GetPgwNode ();
  // connect a remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));

  std::cout << "Connecting remote hosts to EPC with a 100 Gb/s link, MTU 2500 B, delay 0 s" << std::endl;

  NetDeviceContainer internetDevices;
  for (auto it = remoteHosts.Begin (); it != remoteHosts.End (); ++it)
    {
      internetDevices.Add (p2ph.Install (pgw, *it));
    }

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase (base.c_str (), "255.255.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  return internetIpIfaces;
}

Ipv4InterfaceContainer
NrSingleBwpSetup::AssignIpv4ToUe (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
  NS_UNUSED (address);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  InternetStackHelper internet;
  internet.Install (GetUeNodes ());
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = m_epcHelper->AssignUeIpv4Address (GetUeDev ());

  // Set the default gateway for the UEs
  for (uint32_t j = 0; j < GetUeNodes ().GetN (); ++j)
    {
      auto ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (GetUeNodes ().Get (j)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (m_epcHelper->GetUeDefaultGatewayAddress (), 1);
    }


  // if no map is provided, then attach UEs to the closest gNB
  if (m_ueGnbMap.empty ())
    {
      std::cout << "Strategy for UEs attachment: to the closest gNB. " << std::endl;
      AttachToClosestGnb (m_helper, GetUeDev (), GetGnbDev ());
    }
  else
    {
      std::cout << "Strategy for UEs attachment: manual mapping. " << std::endl;

      for (const auto & v : m_ueGnbMap)
        {
          if (v.first < GetUeDev ().GetN () && v.second < GetGnbDev ().GetN ())
            {
              std::cout << " attaching " << v.first << " to " << v.second << std::endl;
              m_helper->AttachToEnb (GetUeDev ().Get (v.first), GetGnbDev ().Get (v.second));
            }
        }
    }

  return ueIpIface;
}

Ipv4InterfaceContainer
NrSingleBwpSetup::AssignIpv4ToStations (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
  NS_UNUSED (address);
  return Ipv4InterfaceContainer ();
}

void
NrSingleBwpSetup::TxDataTrace (std::string context, Time t)
{
  if (!m_channelOccupancyTimeCb.IsNull ())
    {
      m_channelOccupancyTimeCb (static_cast<uint32_t> (std::stoul (context)), t);
    }
}

void
NrSingleBwpSetup::TxCtrlTrace (std::string context, Time t)
{
  if (!m_channelOccupancyTimeCb.IsNull ())
    {
      m_channelOccupancyTimeCb (static_cast<uint32_t> (std::stoul (context)), t);
    }
}

void
NrSingleBwpSetup::GnbReception (std::string context, RxPacketTraceParams params)
{

  if (!m_sinrCb.IsNull ())
    {
      m_sinrCb (static_cast<uint32_t> (std::stoul (context)), params.m_sinr);
    }
  if (!m_macTxDataFailedCb.IsNull ())
    {
      if (params.m_corrupt)
        {
          m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), params.m_tbSize);
        }
    }
}

void
NrSingleBwpSetup::UeReception (std::string context, RxPacketTraceParams params)
{
  if (!m_sinrCb.IsNull ())
    {
      m_sinrCb (static_cast<uint32_t> (std::stoul (context)), params.m_sinr);
    }

  if (!m_macTxDataFailedCb.IsNull ())
    {
      if (params.m_corrupt)
        {
          m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), params.m_tbSize);
        }
    }
}

Ptr<NrHelper> NrSingleBwpSetup::GetNrHelper ()
{
  return m_helper;
}

Ptr<ns3::NrPointToPointEpcHelper> NrSingleBwpSetup::GetEpcHelper ()
{
  return m_epcHelper;
}

BandwidthPartInfoPtrVector NrSingleBwpSetup::GetBwpVector ()
{
  return m_allBwps;
}

void NrSingleBwpSetup::ChangeGnbTxPower (double newTxPower)
{
  // std::cout << "Debug ChangeGnbTxPower()" << std::endl;

  double gnbX = pow (10, newTxPower / 10);

  //////double itiri=0;
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      phy->SetTxPower (10 * log10 (gnbX));
    }
}

void NrSingleBwpSetup::ChangeMcot (Time newMcot)
{
  // std::cout << "Debug ChangeMcot()" << std::endl;
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrLbtAccessManager> nrLbtCam = DynamicCast<NrLbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetMcot (newMcot);
    }
}

void NrSingleBwpSetup::ChangeBackoffType (uint32_t newBackoffType)
{
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrCat4LbtAccessManager> nrLbtCam = DynamicCast<NrCat4LbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetCat4BackoffType (newBackoffType);
    }
}

void NrSingleBwpSetup::ChangeDeferTime (Time newDeferTime)
{
    // std::cout << "Debug ChangeDeferTime()" << std::endl;
    for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrLbtAccessManager> nrLbtCam = DynamicCast<NrLbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetDeferTime (newDeferTime);
    }
}

void NrSingleBwpSetup::ChangeMinCw (uint32_t newMinCw)
{
  // std::cout << "Debug ChangeMinCw()" << std::endl;
  for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrCat4LbtAccessManager> nrLbtCam = DynamicCast<NrCat4LbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetCat4MinCw (newMinCw);
    }
}

void NrSingleBwpSetup::ChangeMcs (uint32_t mcsValue)
{
  // std::cout << "Debug ChangeMcs()" << std::endl;
  for(uint32_t f=0;f < m_gnbDev.GetN(); f++)
  { 
    if (mcsValue <= 28)
    {
      m_helper->GetScheduler (m_gnbDev.Get (f), 0)->SetAttribute("McsValue", UintegerValue(mcsValue));
    }
  }
}

void NrSingleBwpSetup::ChangeAmpdu (uint32_t newAmsdu)
{

}

void NrSingleBwpSetup::ChangeAmsdu (uint32_t newAmsdu)
{

}

void NrSingleBwpSetup::ChangeEdThreshold (double newEdThreshold)
{
    // std::cout << "Debug ChangeEdThreshold()" << std::endl;
    for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrLbtAccessManager> nrLbtCam = DynamicCast<NrLbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetEdThreshold (newEdThreshold);
    }
}

void NrSingleBwpSetup::ChangeSlotTime (uint32_t newSlotTime)
{
    // std::cout << "Debug ChangeSlotTime()" << std::endl;
    for (auto it = m_gnbDev.Begin (); it != m_gnbDev.End (); ++it)
    {
      Ptr<NrGnbPhy> phy = m_helper->GetGnbPhy (*it, 0);
      Ptr<NrLbtAccessManager> nrLbtCam = DynamicCast<NrLbtAccessManager> (phy->GetCam ());
      nrLbtCam->SetSlotTime (MicroSeconds(newSlotTime));
    }
}

} // namespace ns3
