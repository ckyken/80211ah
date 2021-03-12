#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

#include <fstream>
#include <math.h>

#define TestCase_NumCommPair 1

using namespace ns3;

std::ofstream Lab3_PerformanceDataFile("Lab3_Performance.txt", std::ofstream::out);

//Function-----------------------------------------------------------------------------------------
double LinearScale2dB(double LinearScale_Value)
{
  double dB_Value = 10 * (std::log10(LinearScale_Value));
  return dB_Value;
}

double dB2LinearScale(double dB_Value)
{
  double LinearScale_Value = std::pow(10.0, dB_Value / 10.0);
  return LinearScale_Value;
}

double dBm2Watt(double dBm_Value)
{
  double mW_Value = std::pow(10.0, dBm_Value / 10.0);
  return mW_Value / 1000.0;
}

double Watt2dBm(double Watt_Value)
{
  double dBm_Value = std::log10(Watt_Value * 1000.0) * 10.0;
  return dBm_Value;
}

double TwoRayGround_CS_RX_Threshold_Calculator(double TX_Power_dBm, double TX_Antenna_Gain, double RX_Antenna_Gain, double TX_Height_meter, double RX_Height_meter, double System_Loss, double Range_meter, double Lambda_meter, double MinDistance_meter)
{
  if (Range_meter <= MinDistance_meter)
  {
    return TX_Power_dBm;
  }
  double Crossover_Dist = (4 * M_PI * TX_Height_meter * RX_Height_meter) / Lambda_meter;
  double tmp = 0;
  if (Range_meter <= Crossover_Dist)
  {
    double numerator = Lambda_meter * Lambda_meter;
    tmp = M_PI * Range_meter;
    double denominator = 16 * tmp * tmp * System_Loss;
    double pr = 10 * std::log10(numerator / denominator);

    return TX_Power_dBm + pr;
  }
  else
  {
    tmp = TX_Height_meter * RX_Height_meter;
    double rayNumerator = tmp * tmp;
    tmp = Range_meter * Range_meter;
    double rayDenominator = tmp * tmp * System_Loss;
    double rayPr = 10 * std::log10(rayNumerator / rayDenominator);

    return TX_Power_dBm + rayPr;
  }
}
////------------------------------------------------------------------------------------------------
void Lab3_Exp(int Performance_Type, int Node_HorizonSpacing_meter)
{
  //Create node ==> 2 nodes 1 pair
  int Num_WiFiSTA_Node = 4;
  NodeContainer WiFiSTA_Node;
  WiFiSTA_Node.Create(Num_WiFiSTA_Node);

  //Set node's  position
  int Start_XPos_meter, Start_YPos_meter, Start_ZPos_meter;

  MobilityHelper WiFiSTA_Mobility;
  Start_XPos_meter = Start_YPos_meter = Start_ZPos_meter = 0;
  Ptr<ListPositionAllocator> WiFiSTA_PositionAllocator = CreateObject<ListPositionAllocator>();
  WiFiSTA_PositionAllocator->Add(Vector(Start_XPos_meter,
                                        Start_YPos_meter,
                                        Start_ZPos_meter));
  WiFiSTA_PositionAllocator->Add(Vector(Start_XPos_meter + Node_HorizonSpacing_meter,
                                        Start_YPos_meter,
                                        Start_ZPos_meter));
  WiFiSTA_Mobility.SetPositionAllocator(WiFiSTA_PositionAllocator);
  WiFiSTA_Mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  WiFiSTA_Mobility.Install(WiFiSTA_Node);

  //set Type of 802.11
  WifiHelper WiFi = WifiHelper::Default();

  std::string WiFi_Standard_Protocol = "802.11ah";
  if (WiFi_Standard_Protocol == "802.11a")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211a);
  else if (WiFi_Standard_Protocol == "802.11b")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211b);
  else if (WiFi_Standard_Protocol == "802.11g")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211g);
  else if (WiFi_Standard_Protocol == "802.11n-2.4GHz")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
  else if (WiFi_Standard_Protocol == "802.11n-5GHz")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
  else if (WiFi_Standard_Protocol == "802.11ah")
    WiFi.SetStandard(WIFI_PHY_STANDARD_80211ah);

  //WiFi.SetRemoteStationManager("ns3::IdealWifiManager");
  //std::string phyMode("OfdmRate18Mbps");
  std::string phyMode("OfdmRate1_8MbpsBW1MHz");
  WiFi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue(phyMode),
                               "ControlMode", StringValue(phyMode));

  /**
  ///////Lab3--------------------------------------------------------
  double CenterFrequency_Hz = 5.0e+09, SystemLoss = 1, MinDistance_meter = 0, NodeHeight_meter = 1.5;
  YansWifiChannelHelper WiFiChannel;
  WiFiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  WiFiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
                                 "Frequency", DoubleValue(CenterFrequency_Hz),
                                 "SystemLoss", DoubleValue(SystemLoss),
                                 "MinDistance", DoubleValue(MinDistance_meter),
                                 "HeightAboveZ", DoubleValue(NodeHeight_meter));
  double TxPower_Watt = 0.28183815, TxAntennaGain = 1, RxAntennaGain = 1,
         TxHeight_meter = NodeHeight_meter, RxHeight_meter = NodeHeight_meter,
         CS_Range_meter = 1500, TX_Range_meter = 1000, ///  300/200
      Lambda_meter = (299792458.0 / CenterFrequency_Hz);

  double CS_Threshold_dBm = TwoRayGround_CS_RX_Threshold_Calculator(Watt2dBm(TxPower_Watt), TxAntennaGain, RxAntennaGain,
                                                                    TxHeight_meter, RxHeight_meter, SystemLoss, CS_Range_meter, Lambda_meter, MinDistance_meter);

  double RX_Threshold_dBm = TwoRayGround_CS_RX_Threshold_Calculator(Watt2dBm(TxPower_Watt), TxAntennaGain, RxAntennaGain,
                                                                    TxHeight_meter, RxHeight_meter, SystemLoss, TX_Range_meter, Lambda_meter, MinDistance_meter);

  YansWifiPhyHelper WiFiPhy = YansWifiPhyHelper::Default();
  WiFiPhy.Set("TxPowerStart", DoubleValue(Watt2dBm(TxPower_Watt)));
  WiFiPhy.Set("TxPowerEnd", DoubleValue(Watt2dBm(TxPower_Watt)));
  WiFiPhy.Set("TxGain", DoubleValue(LinearScale2dB(TxAntennaGain)));
  WiFiPhy.Set("RxGain", DoubleValue(LinearScale2dB(RxAntennaGain)));
  WiFiPhy.Set("CcaMode1Threshold", DoubleValue(CS_Threshold_dBm));
  WiFiPhy.Set("EnergyDetectionThreshold", DoubleValue(RX_Threshold_dBm));
  WiFiPhy.SetChannel(WiFiChannel.Create());

  S1gWifiMacHelper WiFiMAC = S1gWifiMacHelper::Default();
  WiFiMAC.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer WifiSTA_PhyMac_Wrapper = WiFi.Install(WiFiPhy, WiFiMAC, WiFiSTA_Node);
  ///////----------------------------------------------------------------------------------
**/
  ///////Lab5------------------------------------------------------------------------------
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  wifiPhy.Set("RxGain", DoubleValue(-10));
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel(wifiChannel.Create());
  S1gWifiMacHelper wifiMac = S1gWifiMacHelper();

  wifiMac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer devices = WiFi.Install(wifiPhy, wifiMac, WiFiSTA_Node);

  InternetStackHelper internet;
  internet.Install(WiFiSTA_Node);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ifcont = ipv4.Assign(devices);

  int CBR_PacketSize_Byte = 1024;
  std::string CBR_Rate_bps = "800kbps";
  double Traffic_StartTime_sec = 1.0, Traffic_EndTime_sec = 3.0, TotalSimulationTime_sec = 4.0;
  int CBR_Port = 812;
  ApplicationContainer CBR_Tx_App, CBR_Rx_App;

  int Tx_Index[TestCase_NumCommPair] = {0}, Rx_Index[TestCase_NumCommPair] = {1};

  for (int counter = 0; counter < TestCase_NumCommPair; ++counter)
  {
    OnOffHelper CBR_Client("ns3::UdpSocketFactory",
                           InetSocketAddress(ifcont.GetAddress(Rx_Index[counter]), CBR_Port));
    CBR_Client.SetConstantRate(DataRate(CBR_Rate_bps), CBR_PacketSize_Byte);
    CBR_Tx_App.Add(CBR_Client.Install(WiFiSTA_Node.Get(Tx_Index[counter])));
    CBR_Tx_App.Start(Seconds(Traffic_StartTime_sec));
    CBR_Tx_App.Stop(Seconds(Traffic_EndTime_sec));
  }
  // ---------- [ Receiver Setting ] ---------- //
  for (int counter = 0; counter < TestCase_NumCommPair; ++counter)
  {
    PacketSinkHelper CBR_Server("ns3::UdpSocketFactory",
                                InetSocketAddress(ifcont.GetAddress(Rx_Index[counter]), CBR_Port));
    CBR_Rx_App.Add(CBR_Server.Install(WiFiSTA_Node.Get(Rx_Index[counter])));
    CBR_Rx_App.Start(Seconds(Traffic_StartTime_sec));
    CBR_Rx_App.Stop(Seconds(TotalSimulationTime_sec));
  }

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // Start Simulation
  AnimationInterface anim("Lab3.xml");

  Simulator::Stop(Seconds(TotalSimulationTime_sec));
  Simulator::Run();

  // Show Statistics Information
  double Avg_SystemThroughput_bps = 0.0;
  double Avg_PLR = 0.0;
  double Avg_Base = 0.0;

  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator counter = stats.begin(); counter != stats.end(); ++counter)
  {
    Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(counter->first);
    if (ft.destinationPort == CBR_Port)
    {
      switch (Performance_Type)
      {
      case 0:
      {
        Avg_SystemThroughput_bps +=
            ((double)((counter->second.rxBytes) * 8) /
             (double)(counter->second.timeLastRxPacket.GetSeconds() - counter->second.timeFirstTxPacket.GetSeconds()));

        ++Avg_Base;

        break;
      }
      case 1:
      {
        Avg_PLR +=
            ((double)(counter->second.txPackets - counter->second.rxPackets) /
             (double)(counter->second.txPackets));

        ++Avg_Base;

        break;
      }
      default:
        break;
      }
    }
  }

  switch (Performance_Type)
  {
  case 0:
  {
    Avg_SystemThroughput_bps = Avg_SystemThroughput_bps / Avg_Base;

    std::cout << Avg_SystemThroughput_bps
              << " [bps]\n";

    Lab3_PerformanceDataFile << Avg_SystemThroughput_bps << " ";

    break;
  }
  case 1:
  {
    Avg_PLR = Avg_PLR / Avg_Base;

    std::cout << Avg_PLR
              << "\n";

    Lab3_PerformanceDataFile << Avg_PLR << " ";

    break;
  }
  default:
    break;
  }
  // ===============================================================

  // ======== [ 9 : Cleanup ] ========
  Simulator::Destroy();
  // ==================================

  return;
}

int main(int argc, char *argv[])
{
  int Performance_Type[] = {0, 1}, // 0 : Avg System Throughput ; 1 : Avg Packet Loss Ratio
      Node_HorizonSpacing_meter[] = {50, 100, 150, 190, 220, 280, 320, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400, 1600, 1800, 2000, 5000},
      Num_Distance_Node12 = sizeof Node_HorizonSpacing_meter / sizeof *Node_HorizonSpacing_meter;

  std::cout << "****************** [ Lab 3's Simulation Starts ] ******************\n";
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "#########################################################\n";
  std::cout << "Average System Throughput : \n";
  Lab3_PerformanceDataFile << "\n\nAvg System Throughput : \n\n";
  std::cout << "#########################################################\n";
  for (int counter = 0; counter < Num_Distance_Node12; ++counter)
  {
    std::cout << "[ "
              << "Node1and2_Distance = " << Node_HorizonSpacing_meter[counter] << "m ] => ";
    Lab3_Exp(Performance_Type[0], Node_HorizonSpacing_meter[counter]);

    if (counter != (Num_Distance_Node12 - 1))
    {
      std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n";

      Lab3_PerformanceDataFile << " ";
    }
  }
  std::cout << "#########################################################\n";
  std::cout << "Average Packet Loss Ratio : \n";
  Lab3_PerformanceDataFile << "\n\nAvg Packet Loss Ratio : \n\n";
  std::cout << "#########################################################\n";
  for (int counter = 0; counter < Num_Distance_Node12; ++counter)
  {
    std::cout << "[ "
              << "Node1and2_Distance = " << Node_HorizonSpacing_meter[counter] << "m ] => ";
    Lab3_Exp(Performance_Type[1], Node_HorizonSpacing_meter[counter]);

    if (counter != (Num_Distance_Node12 - 1))
    {
      std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n";

      Lab3_PerformanceDataFile << " ";
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "****************** [ Lab 3's Simulation Ends ] ******************\n";

  Lab3_PerformanceDataFile.close();

  CommandLine cmd;

  cmd.Parse(argc, argv);

  return 0;
}
