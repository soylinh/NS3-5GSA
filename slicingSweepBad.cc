/* 5G SA Private Network for Emergency UAV - Network Slicing under Bad Conditions Sweep
 * mmWave 28 GHz - Sweep speed, altitude, TxPower in bad ranges to show slicing benefit
 * Two flows: URLLC (C2 Control), eMBB (Video)
 * Output CSV for easy plotting and comparison
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/point-to-point-module.h"
 #include "ns3/nr-module.h"
 #include "ns3/flow-monitor-module.h"
 
 #include <iostream>
 #include <iomanip>
 #include <fstream>
 #include <vector>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("Uav5GSlicingSweepBad");
 
 struct SliceStats
 {
   std::string name;
   uint16_t port;
   uint32_t txPackets = 0;
   uint32_t rxPackets = 0;
   uint64_t txBytes = 0;
   uint64_t rxBytes = 0;
   double delaySum = 0.0;
   double jitterSum = 0.0;
   Time firstRxTime = Seconds(0);
   Time lastRxTime = Seconds(0);
 
   double GetLoss() const { return (txPackets > 0) ? (1.0 - (double)rxPackets / txPackets) * 100.0 : 0.0; }
   double GetDelayMs() const { return (rxPackets > 0) ? delaySum * 1000.0 : 0.0; }
   double GetJitterMs() const { return (rxPackets > 1) ? jitterSum * 1000.0 : 0.0; }
   double GetThroughputMbps() const
   {
     double duration = (lastRxTime - firstRxTime).GetSeconds();
     return (duration > 0 && rxBytes > 0) ? (rxBytes * 8.0) / (duration * 1000000.0) : 0.0;
   }
 };
 
 int main(int argc, char *argv[])
 {
   Time simTime = Seconds(20.0);
 
   // Bad conditions from sweep results
   std::vector<double> speeds = {15.0, 20.0, 35.0, 45.0};         // High outage speeds
   std::vector<double> altitudes = {120.0, 140.0, 160.0};        // High altitude outage
   std::vector<double> gnbPowers = {30.0, 34.0};                 // Low gNB power
   std::vector<double> uePowers = {20.0, 22.0};                   // Low UE power (drone low battery)
 
   CommandLine cmd(__FILE__);
   cmd.AddValue("simTime", "Simulation time", simTime);
   cmd.Parse(argc, argv);
 
   // CSV header
   std::ofstream csv("slicing_bad.csv");
   csv << "Speed(m/s),Altitude(m),gNB_TxPower(dBm),UE_TxPower(dBm),"
       << "URLLC_Loss(%),URLLC_Delay(ms),URLLC_Throughput(Mbps),"
       << "eMBB_Loss(%),eMBB_Delay(ms),eMBB_Throughput(Mbps)\n";
   csv.close();
 
   for (double speed : speeds)
   {
     for (double height : altitudes)
     {
       for (double gnbPower : gnbPowers)
       {
         for (double uePower : uePowers)
         {
           Simulator::Destroy();
           RngSeedManager::SetSeed(1);
           RngSeedManager::SetRun(1);
 
           /* Nodes & Mobility */
           NodeContainer gnbNode, ueNode, remoteHost;
           gnbNode.Create(1);
           ueNode.Create(1);
           remoteHost.Create(1);
 
           MobilityHelper mobility;
           mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
           mobility.Install(gnbNode);
           gnbNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 15.0));
 
           mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
           mobility.Install(ueNode);
           Ptr<ConstantVelocityMobilityModel> uavMob = ueNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
           uavMob->SetPosition(Vector(50.0, 0.0, height));
           uavMob->SetVelocity(Vector(speed, 0.0, 0.0));
 
           /* NR + EPC */
           Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
           Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
 
           // Realistic Beamforming
           Ptr<RealisticBeamformingHelper> beamHelper = CreateObject<RealisticBeamformingHelper>();
           beamHelper->SetBeamformingMethod(RealisticBeamformingAlgorithm::GetTypeId());
           nrHelper->SetBeamformingHelper(beamHelper);
           
           // Configure RealisticBfManager attributes via nrHelper
           nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());
           nrHelper->SetGnbBeamManagerAttribute("TriggerEvent", EnumValue(RealisticBfManager::SRS_COUNT));
           nrHelper->SetGnbBeamManagerAttribute("UpdateDelay", TimeValue(MilliSeconds(5)));
 
           nrHelper->SetEpcHelper(epcHelper);
           nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
 
           nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbPower));
           nrHelper->SetUePhyAttribute("TxPower", DoubleValue(uePower));
           nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(3));
 
           /* Band - mmWave */
           CcBwpCreator ccBwpCreator;
           CcBwpCreator::SimpleOperationBandConf bandConf(28e9, 100e6, 1, BandwidthPartInfo::UMi_StreetCanyon);
           OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
           nrHelper->InitializeOperationBand(&band);
           BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
 
           /* Devices */
           NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNode, allBwps);
           NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNode, allBwps);
           for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it) DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
           for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it) DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
 
           /* Internet */
           InternetStackHelper internet;
           internet.Install(ueNode);
           internet.Install(remoteHost);
 
           Ptr<Node> pgw = epcHelper->GetPgwNode();
           PointToPointHelper p2p;
           p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
           p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));
           NetDeviceContainer p2pDevs = p2p.Install(pgw, remoteHost.Get(0));
 
           Ipv4AddressHelper ipv4;
           ipv4.SetBase("1.0.0.0", "255.0.0.0");
           Ipv4InterfaceContainer internetIfaces = ipv4.Assign(p2pDevs);
 
           epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
           Ipv4StaticRoutingHelper routingHelper;
           routingHelper.GetStaticRouting(ueNode.Get(0)->GetObject<Ipv4>())->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
 
           nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
 
           /* Slicing */
           uint16_t c2Port = 9000;
           uint16_t videoPort = 9001;
 
           EpsBearer urllcBearer(EpsBearer::GBR_CONV_VOICE);
           EpsBearer embbBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
 
           Ptr<EpcTft> urllcTft = Create<EpcTft>();
           EpcTft::PacketFilter pf;
           pf.remotePortStart = pf.remotePortEnd = c2Port;
           urllcTft->Add(pf);
           pf.localPortStart = pf.localPortEnd = c2Port;
           urllcTft->Add(pf);
 
           Ptr<EpcTft> embbTft = Create<EpcTft>();
           pf.remotePortStart = pf.remotePortEnd = videoPort;
           embbTft->Add(pf);
           pf.localPortStart = pf.localPortEnd = videoPort;
           embbTft->Add(pf);
 
           nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), urllcBearer, urllcTft);
           nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), embbBearer, embbTft);
 
           /* Applications */
           Ipv4Address remoteAddr = internetIfaces.GetAddress(1);
 
           // URLLC C2
           UdpServerHelper c2Server(c2Port);
           ApplicationContainer c2ServerApp = c2Server.Install(remoteHost.Get(0));
           c2ServerApp.Start(Seconds(1.0));
           c2ServerApp.Stop(simTime);
 
           UdpClientHelper c2Client(remoteAddr, c2Port);
           c2Client.SetAttribute("Interval", TimeValue(MicroSeconds(500)));
           c2Client.SetAttribute("PacketSize", UintegerValue(100));
           ApplicationContainer c2ClientApp = c2Client.Install(ueNode.Get(0));
           c2ClientApp.Start(Seconds(3.0));
           c2ClientApp.Stop(simTime);
 
           // eMBB Video
           UdpServerHelper videoServer(videoPort);
           ApplicationContainer videoServerApp = videoServer.Install(remoteHost.Get(0));
           videoServerApp.Start(Seconds(1.0));
           videoServerApp.Stop(simTime);
 
           UdpClientHelper videoClient(remoteAddr, videoPort);
           videoClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
           videoClient.SetAttribute("PacketSize", UintegerValue(1200));
           ApplicationContainer videoClientApp = videoClient.Install(ueNode.Get(0));
           videoClientApp.Start(Seconds(3.0));
           videoClientApp.Stop(simTime);
 
           // DL traffic
           uint16_t dlPort = 5001;
           UdpServerHelper dlServer(dlPort);
           ApplicationContainer dlServerApp = dlServer.Install(ueNode.Get(0));
           dlServerApp.Start(Seconds(1.0));
           dlServerApp.Stop(simTime);
 
           UdpClientHelper dlClient(internetIfaces.GetAddress(1), dlPort);
           dlClient.SetAttribute("PacketSize", UintegerValue(50));
           dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
           ApplicationContainer dlClientApp = dlClient.Install(remoteHost.Get(0));
           dlClientApp.Start(Seconds(3.5));
           dlClientApp.Stop(simTime);
 
           /* Flow Monitor */
           FlowMonitorHelper flowmon;
           Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
           Simulator::Stop(simTime);
           Simulator::Run();
 
           monitor->CheckForLostPackets();
           Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
           auto stats = monitor->GetFlowStats();
 
           SliceStats urllc{"URLLC", c2Port};
           SliceStats embb{"eMBB", videoPort};
 
           for (auto &flow : stats)
           {
             auto t = classifier->FindFlow(flow.first);
             if (t.destinationPort == c2Port || t.sourcePort == c2Port)
             {
               urllc.txPackets += flow.second.txPackets;
               urllc.rxPackets += flow.second.rxPackets;
               urllc.txBytes += flow.second.txBytes;
               urllc.rxBytes += flow.second.rxBytes;
               if (flow.second.rxPackets > 0) urllc.delaySum += flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
               if (flow.second.rxPackets > 1) urllc.jitterSum += flow.second.jitterSum.GetSeconds() / (flow.second.rxPackets - 1);
               if (urllc.firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < urllc.firstRxTime) urllc.firstRxTime = flow.second.timeFirstRxPacket;
               if (flow.second.timeLastRxPacket > urllc.lastRxTime) urllc.lastRxTime = flow.second.timeLastRxPacket;
             }
             else if (t.destinationPort == videoPort || t.sourcePort == videoPort)
             {
               embb.txPackets += flow.second.txPackets;
               embb.rxPackets += flow.second.rxPackets;
               embb.txBytes += flow.second.txBytes;
               embb.rxBytes += flow.second.rxBytes;
               if (flow.second.rxPackets > 0) embb.delaySum += flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
               if (flow.second.rxPackets > 1) embb.jitterSum += flow.second.jitterSum.GetSeconds() / (flow.second.rxPackets - 1);
               if (embb.firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < embb.firstRxTime) embb.firstRxTime = flow.second.timeFirstRxPacket;
               if (flow.second.timeLastRxPacket > embb.lastRxTime) embb.lastRxTime = flow.second.timeLastRxPacket;
             }
           }
 
           // Write to CSV
           std::ofstream out("slicing_bad.csv", std::ios::app);
           out << speed << "," << height << "," << gnbPower << "," << uePower << ","
               << urllc.GetLoss() << "," << urllc.GetDelayMs() << "," << urllc.GetThroughputMbps() << ","
               << embb.GetLoss() << "," << embb.GetDelayMs() << "," << embb.GetThroughputMbps() << "\n";
           out.close();
 
           std::cout << "Completed: Speed=" << speed << " Altitude=" << height
                     << " gNB=" << gnbPower << " UE=" << uePower << "\n";
         }
       }
     }
   }
 
   std::cout << "\nSweep completed. Results saved to slicing_bad.csv\n";
   return 0;
 }