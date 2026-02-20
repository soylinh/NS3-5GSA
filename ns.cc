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
#include <iostream>
#include <iomanip>
 
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
   // Set random seed for reproducibility
   RngSeedManager::SetSeed(12345);
   
   double simTime = 10.0;  // Increased from 3.0s to 10.0s for better statistics
   CommandLine cmd;
   cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
   cmd.Parse(argc, argv);
   
   // Enable detailed logging for debugging (optional, can be disabled)
   // LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
   // LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
 
   /* ================= SCENARIO ================= */
   std::vector<double> droneSpeeds = {0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
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
 
     nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
     nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
 
     nrHelper->SetPathlossAttribute("ShadowingEnabled",
                                    BooleanValue(false));
 
     nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
     nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(3));  // Higher numerology = shorter slots = lower latency
     // UL-heavy TDD pattern for URLLC uplink traffic (C2 control from drone)
     // Need at least some DL slots for control signaling (PDCCH, PDSCH)
     // Pattern: DL|DL|F|UL|UL|UL|UL|UL|UL|UL| (2 DL, 1 Flexible, 7 UL)
     nrHelper->SetGnbPhyAttribute(
       "Pattern",
       StringValue("DL|DL|F|UL|UL|UL|UL|UL|UL|UL|"));  // UL-heavy but with DL for control
 
     /* ================= BAND ================= */
     CcBwpCreator ccBwpCreator;
     CcBwpCreator::SimpleOperationBandConf bandConf(
       28e9,
       100e6,
       1,
       BandwidthPartInfo::UMi_StreetCanyon);
 
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
     // Reduced core network latency for URLLC (100 microseconds instead of 10ms)
     p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));
 
     NetDeviceContainer p2pDevs =
       p2p.Install(pgw, remoteHost.Get(0));
 
     Ipv4AddressHelper ipv4;
     ipv4.SetBase("1.0.0.0", "255.0.0.0");
     Ipv4InterfaceContainer internetIfaces = ipv4.Assign(p2pDevs);
 
     Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
     Ipv4StaticRoutingHelper routing;
     Ptr<Ipv4StaticRouting> ueRoute =
       routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
     ueRoute->SetDefaultRoute(
       epcHelper->GetUeDefaultGatewayAddress(), 1);
 
     nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);

     /* ================= NETWORK SLICING: URLLC QoS ================= */
     // Create URLLC bearer with GBR (Guaranteed Bit Rate) for low latency
     EpsBearer urllcBearer(EpsBearer::GBR_CONV_VOICE);
     
     // Create TFT (Traffic Flow Template) to classify URLLC traffic by port
     uint16_t urllcPort = 5000;
     Ptr<EpcTft> urllcTft = Create<EpcTft>();
     
     // UPLINK filter: UE sends to server port
     EpcTft::PacketFilter urllcUlFilter;
     urllcUlFilter.remotePortStart = urllcPort;
     urllcUlFilter.remotePortEnd = urllcPort;
     urllcTft->Add(urllcUlFilter);
     
     // DOWNLINK filter: Server sends to UE port
     EpcTft::PacketFilter urllcDlFilter;
     urllcDlFilter.localPortStart = urllcPort;
     urllcDlFilter.localPortEnd = urllcPort;
     urllcTft->Add(urllcDlFilter);
     
     // Activate dedicated EPS bearer for URLLC traffic
     // This provides QoS guarantees (low latency, high reliability)
     nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(0), urllcBearer, urllcTft);

     /* ================= PHY STATISTICS COLLECTION ================= */
     // Reset stats for this iteration
     g_phyStats = PhyStats();
     
     // Connect to SINR trace source for each UE
     // Note: DlDataSinr only fires when DL data is transmitted
     // For UL-heavy traffic, we might get fewer SINR reports
     for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
     {
       Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(i), 0);
       if (uePhy)
       {
         // DlDataSinr is correct for NR - fires when DL data is received
       uePhy->TraceConnectWithoutContext("DlDataSinr", 
                                         MakeCallback(&ReportSinr));
       }
     }

    /* ================= URLLC TRAFFIC ================= */
    // URLLC traffic: Small packets, high frequency (low latency requirement)
    uint16_t port = urllcPort;

    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(remoteHost.Get(0));
    serverApp.Start(Seconds(0.1));
    serverApp.Stop(Seconds(simTime));

    UdpClientHelper client(internetIfaces.GetAddress(1), port);
    client.SetAttribute("PacketSize", UintegerValue(100));
    client.SetAttribute("Interval", TimeValue(MicroSeconds(500)));
    client.SetAttribute("MaxPackets", UintegerValue(1000000));

    ApplicationContainer clientApp = client.Install(ueNodes.Get(0));
    // Start after bearer activation (give time for RRC connection and bearer setup)
    clientApp.Start(Seconds(2.0));  // Increased from 1.0s to 2.0s
    clientApp.Stop(Seconds(simTime));
    
    /* ================= DOWNLINK TRAFFIC (ACK from server) ================= */
    // Add downlink traffic to trigger DL SINR reports and simulate bidirectional communication
    Ipv4Address ueAddress = ueIpIface.GetAddress(0);
    uint16_t dlPort = 5001;  // Different port for downlink
    
    UdpServerHelper dlServer(dlPort);
    ApplicationContainer dlServerApp = dlServer.Install(ueNodes.Get(0));
    dlServerApp.Start(Seconds(0.1));
    dlServerApp.Stop(Seconds(simTime));
    
    UdpClientHelper dlClient(ueAddress, dlPort);
    dlClient.SetAttribute("PacketSize", UintegerValue(50));  // Small ACK packets
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(1)));  // Less frequent than UL
    dlClient.SetAttribute("MaxPackets", UintegerValue(10000));
    ApplicationContainer dlClientApp = dlClient.Install(remoteHost.Get(0));
    dlClientApp.Start(Seconds(2.5));  // Start slightly after UL traffic
    dlClientApp.Stop(Seconds(simTime));
 
     /* ================= FLOW MONITOR ================= */
     FlowMonitorHelper flowmon;
     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
     Simulator::Stop(Seconds(simTime));
     Simulator::Run();
 
     monitor->CheckForLostPackets();
     Ptr<Ipv4FlowClassifier> classifier =
         DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
     auto stats = monitor->GetFlowStats();
 
     // Collect URLLC flow statistics
     uint32_t tx = 0, rx = 0;
     uint64_t txBytes = 0, rxBytes = 0;
     double delay = 0.0;
     double jitter = 0.0;
     Time firstRxTime = Seconds(0);
     Time lastRxTime = Seconds(0);
 
     // Debug: Show all flows
     std::cout << "\n=========== FLOW DEBUG INFO (Speed: " << speed << " m/s) ===========\n";
     for (auto &flow : stats)
     {
       auto t = classifier->FindFlow(flow.first);
       std::cout << "Flow " << flow.first << ": " 
                 << t.sourceAddress << ":" << t.sourcePort
                 << " -> " << t.destinationAddress << ":" << t.destinationPort
                 << " | Tx=" << flow.second.txPackets
                 << " Rx=" << flow.second.rxPackets << "\n";
       
       // Collect URLLC flow statistics (filter by port)
       if (t.destinationPort == urllcPort || t.sourcePort == urllcPort)
       {
         tx += flow.second.txPackets;
         rx += flow.second.rxPackets;
         txBytes += flow.second.txBytes;
         rxBytes += flow.second.rxBytes;
       
         if (flow.second.rxPackets > 0)
       {
           delay += flow.second.delaySum.GetSeconds() / flow.second.rxPackets;
         
           if (flow.second.rxPackets > 1)
         {
             jitter += flow.second.jitterSum.GetSeconds() / 
                      (flow.second.rxPackets - 1);
         }
         
           if (firstRxTime == Seconds(0) || flow.second.timeFirstRxPacket < firstRxTime)
             firstRxTime = flow.second.timeFirstRxPacket;
           if (flow.second.timeLastRxPacket > lastRxTime)
             lastRxTime = flow.second.timeLastRxPacket;
         }
       }
     }
 
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
     double blerApprox = loss; // Using packet loss as approximation
 
     // Console output for URLLC slice
     std::cout << "\n========================================================\n";
     std::cout << "      URLLC NETWORK SLICING PERFORMANCE METRICS\n";
     std::cout << "      (Speed: " << speed << " m/s)\n";
     std::cout << "========================================================\n\n";
     
     std::cout << "URLLC Slice (Port " << urllcPort << ")\n";
     std::cout << std::string(50, '-') << "\n";
     std::cout << "  Bearer Type:         GBR_CONV_VOICE (Guaranteed Bit Rate)\n";
     std::cout << "  Transmitted Packets: " << tx << "\n";
     std::cout << "  Received Packets:    " << rx << "\n";
     std::cout << "  Packet Loss:         " << std::fixed << std::setprecision(2) 
               << loss << "%\n";
     
     if (rx > 0)
     {
       std::cout << "  Avg Delay:           " << std::setprecision(3) 
                 << delay * 1000.0 << " ms\n";
       if (rx > 1)
       {
         std::cout << "  Avg Jitter:          " << std::setprecision(3) 
                   << jitter * 1000.0 << " ms\n";
       }
       std::cout << "  Throughput:          " << std::setprecision(3) 
                 << throughput << " Mbps\n";
     }
     else
     {
       std::cout << "  Avg Delay:           N/A (no packets received)\n";
       std::cout << "  Avg Jitter:          N/A (no packets received)\n";
       std::cout << "  Throughput:          N/A (no packets received)\n";
     }
     
     std::cout << "  Avg SINR:            " << std::setprecision(2) 
               << avgSinr << " dB\n";
     std::cout << "  BLER (approx):       " << std::setprecision(2) 
               << blerApprox << "%\n";
     std::cout << "\n========================================================\n\n";
 
     // Write to CSV for speed sweep analysis
     std::ofstream out("results_ns.csv", std::ios::app);
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
 