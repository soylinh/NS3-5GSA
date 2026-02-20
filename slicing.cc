/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * 5G SA Private Network for Emergency UAV
 * QoS-based Network Slicing (ns-3.38 + NR v2.4)
 *
 * Flow 1: C2 Command & Control  → URLLC-like slice (GBR)
 * Flow 2: Video Streaming      → eMBB-like slice (Non-GBR)
 *
 * Slicing mechanism:
 *  - EPS Bearers + Traffic Flow Templates (TFT)
 *  - No BWP parameter (not supported in this version)
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/point-to-point-helper.h"
 
 #include "ns3/nr-module.h"
 #include "ns3/antenna-module.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <iomanip>
#include <fstream>
 
 using namespace ns3;
 
NS_LOG_COMPONENT_DEFINE("Uav5GSlicingFixed");

// Enable logging for debugging TFT classification
// LogComponentEnable("EpcTftClassifier", LOG_LEVEL_ALL);
// LogComponentEnable("EpcUeNas", LOG_LEVEL_ALL);
 
 int
 main(int argc, char *argv[])
 {
   Time simTime = Seconds(20.0);
 
  double uavHeight = 50.0;
  double uavSpeed  = 20.0;
 
   CommandLine cmd;
   cmd.AddValue("uavHeight", "UAV altitude (m)", uavHeight);
   cmd.AddValue("uavSpeed",  "UAV speed (m/s)",  uavSpeed);
   cmd.Parse(argc, argv);
 
   /* =======================
    * Nodes
    * ======================= */
  NodeContainer gnbNode, ueNode, remoteHost;
   gnbNode.Create(1);
  ueNode.Create(1);
   remoteHost.Create(1);
 
   /* =======================
    * Mobility
    * ======================= */
   MobilityHelper mobility;
 
   mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
   mobility.Install(gnbNode);
   gnbNode.Get(0)->GetObject<MobilityModel>()
       ->SetPosition(Vector(0.0, 0.0, 30.0));
 
   mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(ueNode);
 
   Ptr<ConstantVelocityMobilityModel> uavMob =
      ueNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
   uavMob->SetPosition(Vector(0.0, 0.0, uavHeight));
   uavMob->SetVelocity(Vector(uavSpeed, 0.0, 0.0));
 
   /* =======================
   * NR + EPC
    * ======================= */
   Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> beamHelper = CreateObject<IdealBeamformingHelper>();

  nrHelper->SetEpcHelper(epcHelper);
  nrHelper->SetBeamformingHelper(beamHelper);
 
   nrHelper->SetSchedulerTypeId(
       TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
 
  nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
  nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
  nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
   nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
 
   /* =======================
   * Spectrum (Single BWP)
    * ======================= */
   CcBwpCreator ccBwpCreator;
  auto bandConf = CcBwpCreator::SimpleOperationBandConf(
      3.5e9, 100e6, 1, BandwidthPartInfo::UMi_StreetCanyon);
 
  OperationBandInfo band =
      ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
 
   nrHelper->InitializeOperationBand(&band);
  auto allBwps = CcBwpCreator::GetAllBwps({band});
 
   /* =======================
    * Devices
    * ======================= */
  auto gnbDevs = nrHelper->InstallGnbDevice(gnbNode, allBwps);
  auto ueDevs  = nrHelper->InstallUeDevice(ueNode, allBwps);

  for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
    DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
  for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
    DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
 
   /* =======================
    * Internet
    * ======================= */
   InternetStackHelper internet;
  internet.Install(ueNode);
   internet.Install(remoteHost);

  Ptr<Node> pgw = epcHelper->GetPgwNode();
 
   PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
  p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
 
  auto p2pDevs = p2p.Install(pgw, remoteHost.Get(0));
 
   Ipv4AddressHelper ipv4;
   ipv4.SetBase("1.0.0.0", "255.0.0.0");
  auto internetIfaces = ipv4.Assign(p2pDevs);
 
  epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
   Ipv4StaticRoutingHelper routing;
  routing.GetStaticRouting(ueNode.Get(0)->GetObject<Ipv4>())
      ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

  routing.GetStaticRouting(remoteHost.Get(0)->GetObject<Ipv4>())
      ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

  nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);

  /* =======================
   * QoS-based Slicing
   * ======================= */
  uint16_t c2Port = 9000;
  uint16_t videoPort = 9001;

  EpsBearer urllcBearer(EpsBearer::GBR_CONV_VOICE);
  EpsBearer embbBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);

  // TFT filters: Match packets by destination port
  // Based on ns-3 examples, we add both UPLINK and DOWNLINK filters to same TFT
  // For UPLINK (UE -> Remote Host): remotePort = destination port (server port)
  // For DOWNLINK (Remote Host -> UE): localPort = destination port (server port on UE)
  
  Ptr<EpcTft> urllcTft = Create<EpcTft>();
  
  // UPLINK filter: UE sends to server port 9000
  EpcTft::PacketFilter c2UlFilter;
  c2UlFilter.remotePortStart = c2Port;  // Destination port (server port)
  c2UlFilter.remotePortEnd = c2Port;
  // localPort defaults to 0-65535 (any source port)
  // direction defaults to BIDIRECTIONAL, but we'll match UPLINK
  urllcTft->Add(c2UlFilter);
  
  // DOWNLINK filter: Server sends to UE port 9000
  EpcTft::PacketFilter c2DlFilter;
  c2DlFilter.localPortStart = c2Port;   // Destination port (server port on UE)
  c2DlFilter.localPortEnd = c2Port;
  // remotePort defaults to 0-65535 (any source port)
  urllcTft->Add(c2DlFilter);

  Ptr<EpcTft> embbTft = Create<EpcTft>();
  
  // UPLINK filter: UE sends to server port 9001
  EpcTft::PacketFilter videoUlFilter;
  videoUlFilter.remotePortStart = videoPort;  // Destination port (server port)
  videoUlFilter.remotePortEnd = videoPort;
  // localPort defaults to 0-65535 (any source port)
  embbTft->Add(videoUlFilter);
  
  // DOWNLINK filter: Server sends to UE port 9001
  EpcTft::PacketFilter videoDlFilter;
  videoDlFilter.localPortStart = videoPort;   // Destination port (server port on UE)
  videoDlFilter.localPortEnd = videoPort;
  // remotePort defaults to 0-65535 (any source port)
  embbTft->Add(videoDlFilter);

  // Activate dedicated bearers
  // Default bearer is created automatically when IP is assigned
  // Dedicated bearers are for network slicing with specific QoS
  nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), urllcBearer, urllcTft);
  nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), embbBearer, embbTft);
  
  // Give time for bearer activation to complete through EPC signaling
  // Applications start at 2.0s to ensure bearers are ready
 
   /* =======================
    * Applications
    * ======================= */
  Ipv4Address remoteAddr = internetIfaces.GetAddress(1);
 
   UdpServerHelper c2Server(c2Port);
  ApplicationContainer c2ServerApp = c2Server.Install(remoteHost.Get(0));
   c2ServerApp.Start(Seconds(0.5));
   c2ServerApp.Stop(simTime);
 
  UdpClientHelper c2Client(remoteAddr, c2Port);
   c2Client.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
   c2Client.SetAttribute("PacketSize", UintegerValue(64));
  c2Client.SetAttribute("MaxPackets", UintegerValue(0));
  ApplicationContainer c2ClientApp = c2Client.Install(ueNode.Get(0));
  c2ClientApp.Start(Seconds(5.0));  // Increased delay to ensure RRC connection and bearer activation
   c2ClientApp.Stop(simTime);
 
   UdpServerHelper videoServer(videoPort);
  ApplicationContainer videoServerApp = videoServer.Install(remoteHost.Get(0));
   videoServerApp.Start(Seconds(0.5));
   videoServerApp.Stop(simTime);
 
  UdpClientHelper videoClient(remoteAddr, videoPort);
  videoClient.SetAttribute("Interval", TimeValue(MilliSeconds(33)));
  videoClient.SetAttribute("PacketSize", UintegerValue(1200));
   videoClient.SetAttribute("MaxPackets", UintegerValue(0));
  ApplicationContainer videoClientApp = videoClient.Install(ueNode.Get(0));
  videoClientApp.Start(Seconds(5.0));  // Increased delay to ensure RRC connection and bearer activation
   videoClientApp.Stop(simTime);
 
   /* =======================
   * Flow Monitor
    * ======================= */
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

   Simulator::Stop(simTime);
   Simulator::Run();

  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  auto stats = monitor->GetFlowStats();

  // Statistics structure for each slice
  struct SliceStats {
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

  SliceStats urllcStats, embbStats;
  urllcStats.name = "URLLC (C2 Control)";
  urllcStats.port = c2Port;
  embbStats.name = "eMBB (Video)";
  embbStats.port = videoPort;

  // Collect statistics for each flow
  std::cout << "\n=========== FLOW DEBUG INFO ===========\n";
  for (auto &flow : stats)
  {
    auto t = classifier->FindFlow(flow.first);
    std::cout << "Flow " << flow.first << ": " 
              << t.sourceAddress << ":" << t.sourcePort
              << " -> " << t.destinationAddress << ":" << t.destinationPort
              << " | Tx=" << flow.second.txPackets
              << " Rx=" << flow.second.rxPackets << "\n";
    
    // Identify flow by destination port (server port for uplink)
    if (t.destinationPort == c2Port || t.sourcePort == c2Port)
    {
      urllcStats.txPackets += flow.second.txPackets;
      urllcStats.rxPackets += flow.second.rxPackets;
      urllcStats.txBytes += flow.second.txBytes;
      urllcStats.rxBytes += flow.second.rxBytes;
      urllcStats.delaySum += flow.second.delaySum.GetSeconds();
      urllcStats.jitterSum += flow.second.jitterSum.GetSeconds();
      
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
      embbStats.delaySum += flow.second.delaySum.GetSeconds();
      embbStats.jitterSum += flow.second.jitterSum.GetSeconds();
      
      if (embbStats.firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < embbStats.firstRxTime)
        embbStats.firstRxTime = flow.second.timeFirstRxPacket;
      if (flow.second.timeLastRxPacket > embbStats.lastRxTime)
        embbStats.lastRxTime = flow.second.timeLastRxPacket;
    }
  }

  // Print detailed statistics
  std::cout << "\n========================================================\n";
  std::cout << "      NETWORK SLICING PERFORMANCE METRICS\n";
  std::cout << "========================================================\n\n";

  auto printSliceStats = [](const SliceStats &s) {
    std::cout << "  " << s.name << " (Port " << s.port << ")\n";
    std::cout << "  " << std::string(50, '-') << "\n";
    
    std::cout << "  Transmitted Packets: " << s.txPackets << "\n";
    std::cout << "  Received Packets:    " << s.rxPackets << "\n";
    
    double loss = (s.txPackets > 0) ? 
                  (1.0 - (double)s.rxPackets / s.txPackets) * 100.0 : 0.0;
    std::cout << "  Packet Loss:         " << std::fixed << std::setprecision(2) 
              << loss << "%\n";
    
    if (s.rxPackets > 0)
    {
      double avgDelay = s.delaySum / s.rxPackets;
      std::cout << "  Avg Delay:           " << std::setprecision(3) 
                << avgDelay * 1000.0 << " ms\n";
      
      if (s.rxPackets > 1)
      {
        double avgJitter = s.jitterSum / (s.rxPackets - 1);
        std::cout << "  Avg Jitter:          " << std::setprecision(3) 
                  << avgJitter * 1000.0 << " ms\n";
      }
      else
      {
        std::cout << "  Avg Jitter:          N/A (need at least 2 packets)\n";
      }
      
      double rxDuration = (s.lastRxTime - s.firstRxTime).GetSeconds();
      if (rxDuration > 0)
      {
        double throughput = (s.rxBytes * 8.0) / (rxDuration * 1000000.0);
        std::cout << "  Throughput:          " << std::setprecision(2) 
                  << throughput << " Mbps\n";
      }
      else
      {
        std::cout << "  Throughput:          N/A\n";
      }
    }
    else
    {
      std::cout << "  Avg Delay:           N/A (no packets received)\n";
      std::cout << "  Avg Jitter:          N/A (no packets received)\n";
      std::cout << "  Throughput:          N/A (no packets received)\n";
    }
    std::cout << "\n";
  };

  std::cout << "SLICE 1: " << urllcStats.name << "\n";
  printSliceStats(urllcStats);

  std::cout << "SLICE 2: " << embbStats.name << "\n";
  printSliceStats(embbStats);

  // Comparison summary
  std::cout << "========================================================\n";
  std::cout << "              SLICE COMPARISON SUMMARY\n";
  std::cout << "========================================================\n";
  
  if (urllcStats.rxPackets > 0 && embbStats.rxPackets > 0)
  {
    double urllcDelay = urllcStats.delaySum / urllcStats.rxPackets * 1000.0;
    double embbDelay = embbStats.delaySum / embbStats.rxPackets * 1000.0;
    
    double urllcJitter = (urllcStats.rxPackets > 1) ? 
                          urllcStats.jitterSum / (urllcStats.rxPackets - 1) * 1000.0 : 0.0;
    double embbJitter = (embbStats.rxPackets > 1) ? 
                        embbStats.jitterSum / (embbStats.rxPackets - 1) * 1000.0 : 0.0;
    
    double urllcLoss = (urllcStats.txPackets > 0) ? 
                       (1.0 - (double)urllcStats.rxPackets / urllcStats.txPackets) * 100.0 : 0.0;
    double embbLoss = (embbStats.txPackets > 0) ? 
                      (1.0 - (double)embbStats.rxPackets / embbStats.txPackets) * 100.0 : 0.0;
    
    double urllcDuration = (urllcStats.lastRxTime - urllcStats.firstRxTime).GetSeconds();
    double embbDuration = (embbStats.lastRxTime - embbStats.firstRxTime).GetSeconds();
    double urllcThr = (urllcDuration > 0) ? 
                      (urllcStats.rxBytes * 8.0) / (urllcDuration * 1000000.0) : 0.0;
    double embbThr = (embbDuration > 0) ? 
                     (embbStats.rxBytes * 8.0) / (embbDuration * 1000000.0) : 0.0;
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Metric              URLLC          eMBB          Difference\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << "Delay (ms)          " << std::setw(10) << urllcDelay 
              << "    " << std::setw(10) << embbDelay 
              << "    " << std::setw(10) << (urllcDelay - embbDelay) << "\n";
    std::cout << "Jitter (ms)         " << std::setw(10) << urllcJitter 
              << "    " << std::setw(10) << embbJitter 
              << "    " << std::setw(10) << (urllcJitter - embbJitter) << "\n";
    std::cout << "Packet Loss (%)     " << std::setw(10) << urllcLoss 
              << "    " << std::setw(10) << embbLoss 
              << "    " << std::setw(10) << (urllcLoss - embbLoss) << "\n";
    std::cout << std::setprecision(2);
    std::cout << "Throughput (Mbps)   " << std::setw(10) << urllcThr 
              << "    " << std::setw(10) << embbThr 
              << "    " << std::setw(10) << (urllcThr - embbThr) << "\n";
  }
  
  std::cout << "\n";
  std::cout << "Note: URLLC uses GBR_CONV_VOICE bearer (low latency priority)\n";
  std::cout << "      eMBB uses NGBR_VIDEO_TCP_DEFAULT bearer (high throughput)\n";
  std::cout << "========================================================\n\n";

  // Save to CSV
  std::ofstream csv("slicing_results.csv");
  csv << "Slice,Port,TxPackets,RxPackets,PacketLoss(%),AvgDelay(ms),AvgJitter(ms),Throughput(Mbps)\n";
  
  auto writeSliceToCsv = [&csv](const SliceStats &s) {
    double loss = (s.txPackets > 0) ? 
                  (1.0 - (double)s.rxPackets / s.txPackets) * 100.0 : 0.0;
    double avgDelay = (s.rxPackets > 0) ? s.delaySum / s.rxPackets * 1000.0 : 0.0;
    double avgJitter = (s.rxPackets > 1) ? 
                       s.jitterSum / (s.rxPackets - 1) * 1000.0 : 0.0;
    
    double rxDuration = (s.lastRxTime - s.firstRxTime).GetSeconds();
    double throughput = (rxDuration > 0 && s.rxBytes > 0) ? 
                         (s.rxBytes * 8.0) / (rxDuration * 1000000.0) : 0.0;
    
    csv << s.name << "," << s.port << "," << s.txPackets << "," << s.rxPackets 
        << "," << std::fixed << std::setprecision(2) << loss
        << "," << std::setprecision(3) << avgDelay
        << "," << avgJitter
        << "," << std::setprecision(2) << throughput << "\n";
  };
  
  writeSliceToCsv(urllcStats);
  writeSliceToCsv(embbStats);
  csv.close();
  
  std::cout << "Results saved to: slicing_results.csv\n\n";

   Simulator::Destroy();
   return 0;
 }
 