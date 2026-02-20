/* 5G NR Drone Rescue Simulation
 * Mobility / Doppler Impact Scenario
 * Based on cttc-nr-demo (NR v2.4 compatible)
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/point-to-point-module.h"
 #include "ns3/antenna-module.h"
 #include "ns3/nr-module.h"
 #include "ns3/flow-monitor-module.h"
 
 #include <fstream>
 #include <vector>
 #include <cmath>
 
 using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneRescue5G");

// Statistics collector for SINR
struct PhyStats
{
  std::vector<double> sinrValues;
  
  void AddSinr(double sinr) { sinrValues.push_back(sinr); }
  
  double GetAvgSinr() const 
  { 
    if (sinrValues.empty()) return 0.0;
    double sum = 0.0;
    for (double s : sinrValues) sum += s;
    return sum / sinrValues.size();
  }
  
  double GetMinSinr() const
  {
    if (sinrValues.empty()) return 0.0;
    double min = sinrValues[0];
    for (double s : sinrValues) if (s < min) min = s;
    return min;
  }
  
  double GetMaxSinr() const
  {
    if (sinrValues.empty()) return 0.0;
    double max = sinrValues[0];
    for (double s : sinrValues) if (s > max) max = s;
    return max;
  }
};

// Global stats collector
PhyStats g_phyStats;

// Callback for SINR reporting
void ReportSinr(uint16_t cellId, uint16_t rnti, double avgSinr, uint16_t bwpId, uint8_t streamId)
{
  // Convert linear SINR to dB
  double sinrDb = 10.0 * log10(avgSinr);
  g_phyStats.AddSinr(sinrDb);
}
 
 int
 main(int argc, char *argv[])
 {
   double simTime = 20.0;
   CommandLine cmd;
   cmd.Parse(argc, argv);
 
   /* ================= SCENARIO ================= */
   std::vector<double> droneSpeeds = {0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 32.5, 35.0, 37.5, 40.0, 42.5, 45.0, 47.5, 50.0};
   double fixedAltitude = 20.0; // LOS altitude
 
   std::ofstream csv("results_speed.csv");
   csv << "Speed(m/s),AvgDelay(ms),Jitter(ms),PacketLoss(%),Throughput(Mbps),AvgSINR(dB),BLER(%)\n";
   csv.close();
 
   for (double speed : droneSpeeds)
   {
     Simulator::Destroy();
 
     /* ================= NODES ================= */
     NodeContainer gnbNodes, ueNodes, remoteHost;
     gnbNodes.Create(1);
     ueNodes.Create(1);
     remoteHost.Create(1);
 
     /* ================= MOBILITY ================= */
     MobilityHelper mobility;
 
     // gNB on truck mast
     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
     mobility.Install(gnbNodes);
     gnbNodes.Get(0)->GetObject<MobilityModel>()
       ->SetPosition(Vector(0.0, 0.0, 15.0));
 
     // UAV with variable speed
     mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
     mobility.Install(ueNodes);
 
     Ptr<ConstantVelocityMobilityModel> uavMob =
       ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
 
     uavMob->SetPosition(Vector(50.0, 0.0, fixedAltitude));
     uavMob->SetVelocity(Vector(speed, 0.0, 0.0));
 
     /* ================= CHANNEL ================= */
     Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                        TimeValue(MilliSeconds(0)));
     Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                        TimeValue(MilliSeconds(0)));
 
     /* ================= NR + EPC ================= */
     Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
     Ptr<NrPointToPointEpcHelper> epcHelper =
       CreateObject<NrPointToPointEpcHelper>();
     Ptr<IdealBeamformingHelper> beamformingHelper =
       CreateObject<IdealBeamformingHelper>();
 
     nrHelper->SetEpcHelper(epcHelper);
     nrHelper->SetBeamformingHelper(beamformingHelper);
 
     nrHelper->SetSchedulerTypeId(
       TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    
    LogComponentEnable("NrMacSchedulerTdmaRR", LOG_LEVEL_INFO);
 
     nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
     nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
 
     nrHelper->SetPathlossAttribute("ShadowingEnabled",
                                    BooleanValue(false));
 
     nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(35.0));
     nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(1));
     nrHelper->SetGnbPhyAttribute(
       "Pattern",
       StringValue("DL|DL|DL|DL|F|UL|UL|UL|UL|UL|"));
 
     /* ================= BAND ================= */
     CcBwpCreator ccBwpCreator;
     CcBwpCreator::SimpleOperationBandConf bandConf(
       3.5e9,
       40e6,
       1,
       //BandwidthPartInfo::UMi_StreetCanyon);
       BandwidthPartInfo::UMa);
     OperationBandInfo band =
       ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
 
     nrHelper->InitializeOperationBand(&band);
     BandwidthPartInfoPtrVector allBwps =
       CcBwpCreator::GetAllBwps({band});
 
     /* ================= ANTENNA ================= */
     nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
     nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
     nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
     nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
 
     /* ================= DEVICES ================= */
     NetDeviceContainer gnbDevs =
       nrHelper->InstallGnbDevice(gnbNodes, allBwps);
     NetDeviceContainer ueDevs =
       nrHelper->InstallUeDevice(ueNodes, allBwps);
 
     for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
       DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
 
     for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
       DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
 
     /* ================= INTERNET ================= */
     InternetStackHelper internet;
     internet.Install(ueNodes);
     internet.Install(remoteHost);
 
     Ptr<Node> pgw = epcHelper->GetPgwNode();
 
     PointToPointHelper p2p;
     p2p.SetDeviceAttribute("DataRate",
                            DataRateValue(DataRate("100Gb/s")));
     p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
 
     NetDeviceContainer p2pDevs =
       p2p.Install(pgw, remoteHost.Get(0));
 
     Ipv4AddressHelper ipv4;
     ipv4.SetBase("1.0.0.0", "255.0.0.0");
     Ipv4InterfaceContainer internetIfaces = ipv4.Assign(p2pDevs);
 
     epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
     Ipv4StaticRoutingHelper routing;
     Ptr<Ipv4StaticRouting> ueRoute =
       routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
     ueRoute->SetDefaultRoute(
       epcHelper->GetUeDefaultGatewayAddress(), 1);
 
     nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);

     /* ================= PHY STATISTICS COLLECTION ================= */
     // Reset stats for this iteration
     g_phyStats = PhyStats();
     
     // Connect to SINR trace source for each UE
     for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
     {
       Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(i), 0);
       uePhy->TraceConnectWithoutContext("DlDataSinr", 
                                         MakeCallback(&ReportSinr));
     }

    /* ================= URLLC TRAFFIC ================= */
    uint16_t port = 5000;

    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(remoteHost.Get(0));
    serverApp.Start(Seconds(0.1));
    serverApp.Stop(Seconds(simTime));

    UdpClientHelper client(internetIfaces.GetAddress(1), port);
    client.SetAttribute("PacketSize", UintegerValue(100));
    client.SetAttribute("Interval", TimeValue(MicroSeconds(100)));
    client.SetAttribute("MaxPackets", UintegerValue(1000000));

    ApplicationContainer clientApp = client.Install(ueNodes.Get(0));
    clientApp.Start(Seconds(0.2));
    clientApp.Stop(Seconds(simTime));
 
     /* ================= FLOW MONITOR ================= */
     FlowMonitorHelper flowmon;
     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
     Simulator::Stop(Seconds(simTime));
     Simulator::Run();
 
     monitor->CheckForLostPackets();
     auto stats = monitor->GetFlowStats();
 
     uint32_t tx = 0, rx = 0;
     uint64_t txBytes = 0, rxBytes = 0;
     double delay = 0.0;
     double jitter = 0.0;
     Time firstRxTime = Seconds(0);
     Time lastRxTime = Seconds(0);
 
     for (auto &f : stats)
     {
       tx += f.second.txPackets;
       rx += f.second.rxPackets;
       txBytes += f.second.txBytes;
       rxBytes += f.second.rxBytes;
       
       if (f.second.rxPackets > 0)
       {
         delay += f.second.delaySum.GetSeconds() / f.second.rxPackets;
         
         // Jitter is calculated as average jitter (jitterSum / (rxPackets - 1))
         // jitterCount == rxPackets - 1 according to FlowMonitor documentation
         if (f.second.rxPackets > 1)
         {
           jitter +=
             f.second.jitterSum.GetSeconds() /
             (f.second.rxPackets - 1);
         }
         
         // Track time window for throughput calculation
         if (firstRxTime == Seconds(0) || f.second.timeFirstRxPacket < firstRxTime)
           firstRxTime = f.second.timeFirstRxPacket;
         if (f.second.timeLastRxPacket > lastRxTime)
           lastRxTime = f.second.timeLastRxPacket;
       }
     }
     RngSeedManager::SetSeed(1); RngSeedManager::SetRun(1);
     double loss = (tx > 0) ? (1.0 - (double)rx / tx) * 100 : 0;
     
     // Calculate throughput (Mbps)
     double throughput = 0.0;
     double rxDuration = (lastRxTime - firstRxTime).GetSeconds();
     if (rxDuration > 0 && rxBytes > 0)
     {
       throughput = (rxBytes * 8.0) / (rxDuration * 1000000.0); // Convert to Mbps
     }
     
     // Get PHY statistics
     double avgSinr = g_phyStats.GetAvgSinr();
     // BLER approximation: Packet loss at application layer is influenced by PHY BLER
     double blerApprox = loss; // Using packet loss as approximation
 
     std::ofstream out("results_speed.csv", std::ios::app);
     out << speed << "," 
         << delay * 1000 << "," 
         << jitter * 1000 << "," 
         << loss << ","
         << throughput << ","
         << avgSinr << ","
         << blerApprox << "\n";
     out.close();
   }
   
   return 0;
 }
 