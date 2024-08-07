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
#ifndef NR_ON_OFF_ACCESS_MANAGER_H
#define NR_ON_OFF_ACCESS_MANAGER_H

#include <ns3/nr-ch-access-manager.h>

namespace ns3 {

class NrOnOffAccessManager : public NrChAccessManager
{
public:
  /**
   * \brief Get the type ID
   * \return the type id
   */
  static TypeId GetTypeId (void);

  NrOnOffAccessManager ();
  ~NrOnOffAccessManager () override;

  virtual void RequestAccess () override;
  virtual void SetAccessGrantedCallback (const AccessGrantedCallback &cb) override;
  virtual void SetAccessDeniedCallback (const AccessDeniedCallback &cb) override;
  virtual void Cancel () override;
  void SetChangeTime (const Time& changeTime);
  const Time& GetChangeTime () const;

private:
  void ShutDown ();
  void Start ();
private:
  std::vector<AccessGrantedCallback> m_accessGrantedCb;
  bool m_grant {false};
  bool m_call {false};
  EventId m_changeStateEvent;
  Time m_changeStateTime;
  Time m_changeTime;
};

} // namespace ns3
#endif // NR_ON_OFF_ACCESS_MANAGER_H
