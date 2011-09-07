/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
* Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
* Author: Marco Miozzo <marco.miozzo@cttc.es>
*/

#ifndef LTE_TEST_SHADOWING_MODEL_H
#define LTE_TEST_SHADOWING_MODEL_H

#include "ns3/spectrum-value.h"

#include "ns3/test.h"

#include <ns3/buildings-mobility-model.h>
#include <ns3/buildings-propagation-loss-model.h>


using namespace ns3;


/**
* Test 1.1 shadowing calculation
*/
class LteShadowingTestSuite : public TestSuite
{
  public:
    LteShadowingTestSuite ();
};


class LteShadowingTestCase : public TestCase
{
  public:
    LteShadowingTestCase (Ptr<BuildingsMobilityModel> m1, Ptr<BuildingsMobilityModel> m2, double refValue, double sigmaRef, std::string name);
    virtual ~LteShadowingTestCase ();
    
  private:
    virtual void DoRun (void);
    
    Ptr<BuildingsMobilityModel> m_node1;
    Ptr<BuildingsMobilityModel> m_node2;
    double m_lossRef;   // pathloss value (without shadowing)
    double m_sigmaRef;
    
};

class LteShadowingSystemTestCase : public TestCase
{
  public:
    LteShadowingSystemTestCase (std::string name, double snrDb, double dist, uint16_t mcsIndex);
    LteShadowingSystemTestCase ();
    virtual ~LteShadowingSystemTestCase ();
    
    void DlScheduling (uint32_t frameNo, uint32_t subframeNo, uint16_t rnti,
                       uint8_t mcsTb1, uint16_t sizeTb1, uint8_t mcsTb2, uint16_t sizeTb2);
                       
  private:
    virtual void DoRun (void);
    
    double m_snrDb;
    double m_distance;
    uint16_t m_mcsIndex;
};

#endif /*LTE_TEST_SHADOWING_MODEL_H*/