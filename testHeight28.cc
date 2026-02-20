/* 5G NR Drone Rescue Simulation - Altitude Impact with mmWave
 * Based on cttc-nr-demo (NR v2.4 compatible)
 * Uses successful configuration from testSpeed: RealisticBeamforming, numerology=3,
 * higher TxPower, added DL traffic, long simTime
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
 #include <iostream>
 #include <iomanip>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("DroneRescueAltitudeMmWave");
 
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
 };
 
 PhyStats g_phyStats;
 
 // Callback for SINR reporting
 void ReportSinr(uint16_t cellId, uint16_t rnti, double avgSinr, uint16_t bwpId, uint8_t streamId)
 {
   double sinrDb = 10.0 * log10(avgSinr);
   g_phyStats.AddSinr(sinrDb);
 }
 
 int
 main(int argc, char *argv[])
 {
   // Fixed seed for reproducibility
   RngSeedManager::SetSeed(1);
   RngSeedManager::SetRun(1);
 
   double simTime = 20.0;  // Long simulation for stable stats
 
   CommandLine cmd(__FILE__);
   cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
   cmd.Parse(argc, argv);
 
   /* ================= SCENARIO ================= */
   std::vector<double> droneHeights = {1.5, 10.0, 20.0, 30.0, 40.0, 50.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0};
   double fixedSpeed = 25.0;  // Moderate speed to combine mobility effect
 
   std::ofstream csv("results_height_mmWave.csv");
   csv << "Altitude(m),AvgDelay(ms),Jitter(ms),PacketLoss(%),Throughput(Mbps),AvgSINR(dB),BLER(%)\n";
   csv.close();
 
   for (double height : droneHeights)
   {
     Simulator::Destroy();
     g_phyStats = PhyStats();  // Reset stats
 
     /* ================= NODES ================= */
     NodeContainer gnbNodes, ueNodes, remoteHost;
     gnbNodes.Create(1);
     ueNodes.Create(1);
     remoteHost.Create(1);
 
     /* ================= MOBILITY ================= */
     MobilityHelper mobility;
 
     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
     mobility.Install(gnbNodes);
     gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 15.0));
 
     mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
     mobility.Install(ueNodes);
     Ptr<ConstantVelocityMobilityModel> uavMob = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
     uavMob->SetPosition(Vector(50.0, 0.0, height));
     uavMob->SetVelocity(Vector(fixedSpeed, 0.0, 0.0));
 
     /* ================= CHANNEL ================= */
     Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
     Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
 
     /* ================= NR + EPC ================= */
     Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
     Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
 
    // Realistic Beamforming - key for good performance
    Ptr<RealisticBeamformingHelper> beamformingHelper = CreateObject<RealisticBeamformingHelper>();
    beamformingHelper->SetBeamformingMethod(RealisticBeamformingAlgorithm::GetTypeId());
    nrHelper->SetBeamformingHelper(beamformingHelper);
    
    // Configure RealisticBfManager attributes via nrHelper
    nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());
    nrHelper->SetGnbBeamManagerAttribute("TriggerEvent", EnumValue(RealisticBfManager::SRS_COUNT));
    nrHelper->SetGnbBeamManagerAttribute("UpdateDelay", TimeValue(MilliSeconds(5)));  // Frequent update
 
     nrHelper->SetEpcHelper(epcHelper);
 
     nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
 
     nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
     nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
 
     nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
 
     // Higher TxPower for better coverage at high altitude
     nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(37.0));
     nrHelper->SetUePhyAttribute("TxPower", DoubleValue(26.0));
 
     // Numerology 3 for better Doppler and latency tolerance
     nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(3));
 
     /* ================= BAND - mmWave 28 GHz ================= */
     CcBwpCreator ccBwpCreator;
     CcBwpCreator::SimpleOperationBandConf bandConf(
         28e9,
         100e6,
         1,
         BandwidthPartInfo::UMi_StreetCanyon);
 
     OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
     nrHelper->InitializeOperationBand(&band);
     BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
 
     /* ================= ANTENNA ================= */
     nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
     nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
     nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
     nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
 
     /* ================= DEVICES ================= */
     NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
     NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
 
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
     p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
     p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));
 
     NetDeviceContainer p2pDevs = p2p.Install(pgw, remoteHost.Get(0));
 
     Ipv4AddressHelper ipv4;
     ipv4.SetBase("1.0.0.0", "255.0.0.0");
     Ipv4InterfaceContainer internetIfaces = ipv4.Assign(p2pDevs);
 
     Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
     Ipv4StaticRoutingHelper routing;
     routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
         ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
 
     nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
 
     /* ================= PHY STATISTICS COLLECTION ================= */
     for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
     {
       Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(i), 0);
       uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&ReportSinr));
     }
 
     /* ================= URLLC UPLINK TRAFFIC ================= */
     uint16_t ulPort = 5000;
 
     UdpServerHelper ulServer(ulPort);
     ApplicationContainer ulServerApp = ulServer.Install(remoteHost.Get(0));
     ulServerApp.Start(Seconds(0.1));
     ulServerApp.Stop(Seconds(simTime));
 
     UdpClientHelper ulClient(internetIfaces.GetAddress(1), ulPort);
     ulClient.SetAttribute("PacketSize", UintegerValue(100));
     ulClient.SetAttribute("Interval", TimeValue(MicroSeconds(500)));  // 500us để ổn định hơn
     ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
 
     ApplicationContainer ulClientApp = ulClient.Install(ueNodes.Get(0));
     ulClientApp.Start(Seconds(2.0));
     ulClientApp.Stop(Seconds(simTime));
 
     /* ================= DOWNLINK TRAFFIC (trigger SINR + balance scheduler) ================= */
     uint16_t dlPort = 5001;
 
     UdpServerHelper dlServer(dlPort);
     ApplicationContainer dlServerApp = dlServer.Install(ueNodes.Get(0));
     dlServerApp.Start(Seconds(0.1));
     dlServerApp.Stop(Seconds(simTime));
 
     UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
     dlClient.SetAttribute("PacketSize", UintegerValue(50));
     dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(2)));
     dlClient.SetAttribute("MaxPackets", UintegerValue(10000));
 
     ApplicationContainer dlClientApp = dlClient.Install(remoteHost.Get(0));
     dlClientApp.Start(Seconds(2.5));
     dlClientApp.Stop(Seconds(simTime));
 
     /* ================= FLOW MONITOR ================= */
     FlowMonitorHelper flowmon;
     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
     Simulator::Stop(Seconds(simTime));
     Simulator::Run();
 
     monitor->CheckForLostPackets();
     auto stats = monitor->GetFlowStats();
 
     uint32_t tx = 0, rx = 0;
     uint64_t txBytes = 0, rxBytes = 0;
     double delaySum = 0.0;
     double jitterSum = 0.0;
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
         delaySum += f.second.delaySum.GetSeconds() / f.second.rxPackets;
 
         if (f.second.rxPackets > 1)
         {
           jitterSum += f.second.jitterSum.GetSeconds() / (f.second.rxPackets - 1);
         }
 
         if (firstRxTime == Seconds(0) || f.second.timeFirstRxPacket < firstRxTime)
           firstRxTime = f.second.timeFirstRxPacket;
         if (f.second.timeLastRxPacket > lastRxTime)
           lastRxTime = f.second.timeLastRxPacket;
       }
     }
 
     double loss = (tx > 0) ? (1.0 - (double)rx / tx) * 100.0 : 0.0;
     double avgDelay = (rx > 0) ? delaySum * 1000.0 : 0.0;
     double avgJitter = (rx > 1) ? jitterSum * 1000.0 : 0.0;
 
     double rxDuration = (lastRxTime - firstRxTime).GetSeconds();
     double throughput = (rxDuration > 0 && rxBytes > 0) ? (rxBytes * 8.0) / (rxDuration * 1000000.0) : 0.0;
 
     double avgSinr = g_phyStats.GetAvgSinr();
     double blerApprox = loss;
 
     std::ofstream out("results_height_mmWave.csv", std::ios::app);
     out << height << ","
         << avgDelay << ","
         << avgJitter << ","
         << loss << ","
         << throughput << ","
         << avgSinr << ","
         << blerApprox << "\n";
     out.close();
 
     std::cout << "Altitude: " << height << " m | Loss: " << std::fixed << std::setprecision(2) << loss
               << "% | Delay: " << avgDelay << " ms | Throughput: " << throughput << " Mbps | SINR: " << avgSinr << " dB\n";
   }
 
   std::cout << "\nSimulation completed. Results saved to results_height_mmWave.csv\n";
   return 0;
 }