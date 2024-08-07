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
#ifndef SIMULATIONHELPER_H
#define SIMULATIONHELPER_H
#include <ns3/node.h>
#include <ns3/ipv4.h>
#include <ns3/ipv4-routing-protocol.h>
#include <ns3/ipv4-interface.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/arp-cache.h>
#include <ns3/output-stream-wrapper.h>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
namespace ns3 {

static std::string
GetRoutingTable (const Ptr<Node> n)
{
  Ptr <Ipv4> ipv4 = n->GetObject <Ipv4> ();
  if (ipv4 == nullptr)
    {
      std::cout << " Node " << n->GetId() << " has no ipv4 " << std::endl;
      return "";
    }
  std::stringstream stream;
  Ptr<OutputStreamWrapper> routingstream = Create<OutputStreamWrapper> (&stream);
  ipv4->GetRoutingProtocol ()->PrintRoutingTable (routingstream);
  return stream.str ();
}

static void
PrintRoutingTable (const NodeContainer &nodes)
{
  for (auto it = nodes.Begin (); it != nodes.End (); ++it)
    {
      std::cout << Names::FindName (*it) << " routing table:\n" << GetRoutingTable (*it) << std::endl;
    }
}

static void
PrintIpAddress (const NodeContainer &nodes)
{
  for (auto it = nodes.Begin (); it != nodes.End (); ++it)
    {
      Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4> ();
      if (ipv4 == nullptr)
        {
          std::cout << " Node " << (*it)->GetId() << " has no ipv4 " << std::endl;
          continue;
        }

      std::cout << Names::FindName (*it) << " has " <<
        ipv4->GetNInterfaces () << " interfaces, ip addresses:\n";

      for (uint32_t i = 0; i < ipv4->GetNInterfaces (); ++i)
        {
          for (uint32_t j = 0; j < ipv4->GetNAddresses (i); ++j)
            {
              Ipv4InterfaceAddress iaddr = ipv4->GetAddress (i, j);
              std::cout << "interface " << i << ", addr " << j
                        << ", ip: " << iaddr.GetLocal () << " mask: " << iaddr.GetMask ()
                        << std::endl;
            }
        }

    }
}

void
PopulateArpCache (void)
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365));

  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
    {
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      if (ip == nullptr)
        {
          continue;
        }
      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);
      for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
        {
          Ptr<Ipv4Interface> ipIface = (j->second)->GetObject<Ipv4Interface> ();
          NS_ASSERT (ipIface != nullptr);
          Ptr<NetDevice> device = ipIface->GetDevice ();
          NS_ASSERT (device != nullptr);
          Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress ());
          for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++)
            {
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal ();
              if (ipAddr == Ipv4Address::GetLoopback ())
                continue;
              ArpCache::Entry *entry = arp->Add (ipAddr);
              //entry->MarkWaitReply (0);
              //entry->MarkAlive (addr);
              entry->SetMacAddress (addr);
              entry->MarkPermanent ();
            }
        }
    }

  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
    {
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      if (ip == nullptr)
        {
          continue;
        }
      ObjectVectorValue interfaces;
      ip->GetAttribute("InterfaceList", interfaces);
      for(ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
        {
          Ptr<Ipv4Interface> ipIface = (j->second)->GetObject<Ipv4Interface> ();
          ipIface->SetAttribute ("ArpCache", PointerValue (arp));
        }
    }
}

}
#endif // SIMULATIONHELPER_H
