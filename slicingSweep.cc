/* 5G SA Private Network for Emergency UAV - Network Slicing under Sweep Conditions
 * Based on slicing.cc + improvements from sweep results (speed, altitude, TxPower)
 * Configuration optimized from sweep:
 * - mmWave 28 GHz, UMi_StreetCanyon
 * - Speed: 25 m/s, Altitude: 50 m
 * - TxPower: gNB 38 dBm, UE 26 dBm
 * - Numerology 3, RealisticBeamforming
 * - Two flows: URLLC (C2 Control), eMBB (Video)
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
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("Uav5GSlicingUnderSweep");
 
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
 };
 
 int main(int argc, char *argv[])
 {
   Time simTime = Seconds(20.0);
 
   double uavSpeed = 25.0;   // From speed sweep: stable, low loss
   double uavHeight = 50.0;  // From altitude sweep: good performance
   double gnbTxPower = 38.0; // From TxPower sweep: high reliability
   double ueTxPower = 26.0;
 
   CommandLine cmd(__FILE__);
   cmd.AddValue("simTime", "Simulation time", simTime);
   cmd.Parse(argc, argv);
 
   RngSeedManager::SetSeed(1);
   RngSeedManager::SetRun(1);
 
   /* ======================= Nodes ======================= */
   NodeContainer gnbNode, ueNode, remoteHost;
   gnbNode.Create(1);
   ueNode.Create(1);
   remoteHost.Create(1);
 
   /* ======================= Mobility ======================= */
   MobilityHelper mobility;
   mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
   mobility.Install(gnbNode);
   gnbNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 15.0));
 
   mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
   mobility.Install(ueNode);
   Ptr<ConstantVelocityMobilityModel> uavMob = ueNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
   uavMob->SetPosition(Vector(50.0, 0.0, uavHeight));
   uavMob->SetVelocity(Vector(uavSpeed, 0.0, 0.0));
 
   /* ======================= NR + EPC ======================= */
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
 
   nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPower));
   nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));
   nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(3));  // Low latency
 
   /* ======================= Band - mmWave ======================= */
   CcBwpCreator ccBwpCreator;
   CcBwpCreator::SimpleOperationBandConf bandConf(
       28e9, 100e6, 1, BandwidthPartInfo::UMi_StreetCanyon);
   OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
   nrHelper->InitializeOperationBand(&band);
   BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
 
   /* ======================= Devices ======================= */
   NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNode, allBwps);
   NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNode, allBwps);
 
   for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
     DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
   for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
     DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
 
   /* ======================= Internet ======================= */
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
   routingHelper.GetStaticRouting(ueNode.Get(0)->GetObject<Ipv4>())
       ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
 
   nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
 
   /* ======================= QoS-based Slicing ======================= */
   uint16_t c2Port = 9000;     // URLLC: C2 Command & Control
   uint16_t videoPort = 9001;  // eMBB: Video Streaming
 
   EpsBearer urllcBearer(EpsBearer::GBR_CONV_VOICE);        // Low latency, high priority
   EpsBearer embbBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT); // High throughput
 
   // TFT for URLLC (C2)
   Ptr<EpcTft> urllcTft = Create<EpcTft>();
   EpcTft::PacketFilter c2UlFilter;
   c2UlFilter.remotePortStart = c2Port;
   c2UlFilter.remotePortEnd = c2Port;
   urllcTft->Add(c2UlFilter);
   EpcTft::PacketFilter c2DlFilter;
   c2DlFilter.localPortStart = c2Port;
   c2DlFilter.localPortEnd = c2Port;
   urllcTft->Add(c2DlFilter);
 
   // TFT for eMBB (Video)
   Ptr<EpcTft> embbTft = Create<EpcTft>();
   EpcTft::PacketFilter videoUlFilter;
   videoUlFilter.remotePortStart = videoPort;
   videoUlFilter.remotePortEnd = videoPort;
   embbTft->Add(videoUlFilter);
   EpcTft::PacketFilter videoDlFilter;
   videoDlFilter.localPortStart = videoPort;
   videoDlFilter.localPortEnd = videoPort;
   embbTft->Add(videoDlFilter);
 
   // Activate dedicated bearers
   nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), urllcBearer, urllcTft);
   nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), embbBearer, embbTft);
 
   /* ======================= Applications ======================= */
   Ipv4Address remoteAddr = internetIfaces.GetAddress(1);
 
   // URLLC: C2 Control (small packets, high frequency)
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
 
   // eMBB: Video Streaming (larger packets, moderate rate)
   UdpServerHelper videoServer(videoPort);
   ApplicationContainer videoServerApp = videoServer.Install(remoteHost.Get(0));
   videoServerApp.Start(Seconds(1.0));
   videoServerApp.Stop(simTime);
 
   UdpClientHelper videoClient(remoteAddr, videoPort);
   videoClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));  // ~10 Mbps target
   videoClient.SetAttribute("PacketSize", UintegerValue(1200));
   ApplicationContainer videoClientApp = videoClient.Install(ueNode.Get(0));
   videoClientApp.Start(Seconds(3.0));
   videoClientApp.Stop(simTime);
 
   // Light DL traffic to balance
   uint16_t dlPort = 5001;
   UdpServerHelper dlServer(dlPort);
   ApplicationContainer dlServerApp = dlServer.Install(ueNode.Get(0));
   dlServerApp.Start(Seconds(1.0));
   dlServerApp.Stop(simTime);
 
   UdpClientHelper dlClient(epcHelper->GetUeDefaultGatewayAddress(), dlPort);  // Use correct address if needed
   dlClient.SetAttribute("PacketSize", UintegerValue(50));
   dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
   ApplicationContainer dlClientApp = dlClient.Install(remoteHost.Get(0));
   dlClientApp.Start(Seconds(3.5));
   dlClientApp.Stop(simTime);
 
   /* ======================= Flow Monitor ======================= */
   FlowMonitorHelper flowmon;
   Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
   Simulator::Stop(simTime);
   Simulator::Run();
 
   monitor->CheckForLostPackets();
   Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
   auto stats = monitor->GetFlowStats();
 
   SliceStats urllcStats{"URLLC (C2 Control)", c2Port};
   SliceStats embbStats{"eMBB (Video)", videoPort};
 
   for (auto &flow : stats)
   {
     auto t = classifier->FindFlow(flow.first);
     if (t.destinationPort == c2Port || t.sourcePort == c2Port)
     {
       urllcStats.txPackets += flow.second.txPackets;
       urllcStats.rxPackets += flow.second.rxPackets;
       urllcStats.txBytes += flow.second.txBytes;
       urllcStats.rxBytes += flow.second.rxBytes;
       urllcStats.delaySum += flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
       urllcStats.jitterSum += (flow.second.rxPackets > 1) ? flow.second.jitterSum.GetSeconds() / (flow.second.rxPackets - 1) : 0;
       if (urllcStats.firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < urllcStats.firstRxTime)
         urllcStats.firstRxTime = flow.second.timeFirstRxPacket;
       if (flow.second.timeLastRxPacket > urllcStats.lastRxTime)
         urllcStats.lastRxTime = flow.second.timeLastRxPacket;
     }
     else if (t.destinationPort == videoPort || t.sourcePort == videoPort)
     {
       embbStats.txPackets += flow.second.txPackets;
       embbStats.rxPackets += flow.second.rxPackets;
       embbStats.txBytes += flow.second.txBytes;
       embbStats.rxBytes += flow.second.rxBytes;
       embbStats.delaySum += flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
       embbStats.jitterSum += (flow.second.rxPackets > 1) ? flow.second.jitterSum.GetSeconds() / (flow.second.rxPackets - 1) : 0;
       if (embbStats.firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < embbStats.firstRxTime)
         embbStats.firstRxTime = flow.second.timeFirstRxPacket;
       if (flow.second.timeLastRxPacket > embbStats.lastRxTime)
         embbStats.lastRxTime = flow.second.timeLastRxPacket;
     }
   }
 
   auto printStats = [](const SliceStats &s) {
     std::cout << "=== " << s.name << " (Port " << s.port << ") ===\n";
     std::cout << "Tx Packets: " << s.txPackets << " | Rx Packets: " << s.rxPackets << "\n";
     double loss = (s.txPackets > 0) ? (1.0 - (double)s.rxPackets / s.txPackets) * 100.0 : 0.0;
     std::cout << "Packet Loss: " << std::fixed << std::setprecision(4) << loss << "%\n";
     if (s.rxPackets > 0)
     {
       std::cout << "Avg Delay: " << std::setprecision(3) << s.delaySum * 1000.0 << " ms\n";
       std::cout << "Avg Jitter: " << std::setprecision(3) << s.jitterSum * 1000.0 << " ms\n";
       double duration = (s.lastRxTime - s.firstRxTime).GetSeconds();
       double thr = (duration > 0) ? (s.rxBytes * 8.0) / (duration * 1000000.0) : 0.0;
       std::cout << "Throughput: " << std::setprecision(3) << thr << " Mbps\n";
     }
     std::cout << "\n";
   };
 
   std::cout << "\n===== NETWORK SLICING PERFORMANCE (mmWave under Sweep Conditions) =====\n";
   std::cout << "Speed: " << uavSpeed << " m/s | Altitude: " << uavHeight << " m | gNB TxPower: " << gnbTxPower << " dBm | UE TxPower: " << ueTxPower << " dBm\n\n";
 
   printStats(urllcStats);
   printStats(embbStats);
 
   Simulator::Destroy();
   return 0;
 }