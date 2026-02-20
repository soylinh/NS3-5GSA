/* 5G Standalone Private Network with Network Slicing
 * Real-world scenario: Emergency UAV Rescue Operation
 * 
 * Network Slices:
 *  1. URLLC Slice: Critical C2 (Command & Control) traffic - Low latency, high reliability
 *  2. eMBB Slice: Video streaming from UAV - High throughput
 * 
 * Demonstrates:
 *  - 5G SA architecture with EPC
 *  - QoS-based network slicing using EPS bearers
 *  - Traffic Flow Templates (TFT) for slice classification
 *  - Real-world performance metrics
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
#include <map>
#include <string>
#include <numeric>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PrivateNetwork5G");

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

// Global stats collector
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
  // Set random seed for reproducibility
  RngSeedManager::SetSeed(12345);
  
  double simTime = 15.0;
  double uavSpeed = 20.0;  // m/s
  double uavHeight = 50.0; // meters
  
  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
  cmd.AddValue("uavSpeed", "UAV speed (m/s)", uavSpeed);
  cmd.AddValue("uavHeight", "UAV altitude (m)", uavHeight);
  cmd.Parse(argc, argv);
  
  /* ================= SCENARIO ================= */
  // Emergency UAV rescue operation:
  // - gNB: Base station at ground control center
  // - UE: UAV flying at altitude, moving at speed
  // - Remote Host: Command center server
  
  /* ================= NODES ================= */
  NodeContainer gnbNodes, ueNodes, remoteHost;
  gnbNodes.Create(1);
  ueNodes.Create(1);
  remoteHost.Create(1);
  
  /* ================= MOBILITY ================= */
  MobilityHelper mobility;
  
  // gNB: Fixed position at ground control center
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(gnbNodes);
  gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 30.0));
  
  // UAV: Moving at constant velocity
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(ueNodes);
  Ptr<ConstantVelocityMobilityModel> uavMob = 
      ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
  uavMob->SetPosition(Vector(0.0, 0.0, uavHeight));
  uavMob->SetVelocity(Vector(uavSpeed, 0.0, 0.0));
  
  // Remote Host: Fixed position
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(remoteHost);
  remoteHost.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
  
  /* ================= NR SETUP ================= */
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
  
  nrHelper->SetEpcHelper(epcHelper);
  nrHelper->SetBeamformingHelper(beamformingHelper);
  
  nrHelper->SetSchedulerTypeId(
      TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
  
  nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
  nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
  
  // Channel configuration for UAV scenario
Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
    TimeValue(MilliSeconds(100)));
Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
    TimeValue(MilliSeconds(100)));

  nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
  nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", 
      TimeValue(MilliSeconds(100)));
  
  nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
  nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(2));
  // Balanced TDD pattern for both UL and DL traffic
  nrHelper->SetGnbPhyAttribute("Pattern",
      StringValue("DL|DL|F|UL|UL|UL|UL|UL|UL|UL|"));
  
  /* ================= BAND ================= */
  CcBwpCreator ccBwpCreator;
  CcBwpCreator::SimpleOperationBandConf bandConf(
      3.5e9,      // 3.5 GHz frequency
      100e6,      // 100 MHz bandwidth
      1,          // Single component carrier
      BandwidthPartInfo::UMi_StreetCanyon);
  
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  nrHelper->InitializeOperationBand(&band);
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
  
  /* ================= DEVICES ================= */
  NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
  NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
  
  for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
    DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
  for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
    DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
  
  /* ================= INTERNET STACK ================= */
  InternetStackHelper internet;
  internet.Install(ueNodes);
  internet.Install(remoteHost);
  
  Ptr<Node> pgw = epcHelper->GetPgwNode();
  
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
  p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(100)));
  
  NetDeviceContainer p2pDevs = p2p.Install(pgw, remoteHost.Get(0));
  
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIfaces = ipv4.Assign(p2pDevs);
  
  Ipv4InterfaceContainer ueIf = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
  
  Ipv4StaticRoutingHelper routing;
  routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
      ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
  
  routing.GetStaticRouting(remoteHost.Get(0)->GetObject<Ipv4>())
      ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
  
  nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
  
  /* ================= NETWORK SLICING: EPS BEARERS ================= */
  // URLLC Slice: Critical C2 traffic (Port 5000)
  // eMBB Slice: Video streaming (Port 5001)
  
  uint16_t urllcPort = 5000;
  uint16_t embbPort = 5001;
  
  // Get UE device for bearer activation
  Ptr<NetDevice> ueNetDev = ueDevs.Get(0);
  Ptr<NrUeNetDevice> ueNrDev = DynamicCast<NrUeNetDevice>(ueNetDev);

  // URLLC Bearer: GBR_CONV_VOICE for low latency, high reliability
  Ptr<EpcTft> tftUrllc = Create<EpcTft>();
  EpcTft::PacketFilter pfUrllcUl;
  pfUrllcUl.remotePortStart = urllcPort;
  pfUrllcUl.remotePortEnd = urllcPort;
  pfUrllcUl.direction = EpcTft::UPLINK;
  tftUrllc->Add(pfUrllcUl);
  
  EpcTft::PacketFilter pfUrllcDl;
  pfUrllcDl.localPortStart = urllcPort;
  pfUrllcDl.localPortEnd = urllcPort;
  pfUrllcDl.direction = EpcTft::DOWNLINK;
  tftUrllc->Add(pfUrllcDl);
  
  nrHelper->ActivateDedicatedEpsBearer(ueDevs, EpsBearer::GBR_CONV_VOICE, tftUrllc);
  
  // eMBB Bearer: NGBR_VIDEO_TCP_DEFAULT for high throughput
  Ptr<EpcTft> tftEmbB = Create<EpcTft>();
  EpcTft::PacketFilter pfEmbBUl;
  pfEmbBUl.remotePortStart = embbPort;
  pfEmbBUl.remotePortEnd = embbPort;
  pfEmbBUl.direction = EpcTft::UPLINK;
  tftEmbB->Add(pfEmbBUl);
  
  EpcTft::PacketFilter pfEmbBDl;
  pfEmbBDl.localPortStart = embbPort;
  pfEmbBDl.localPortEnd = embbPort;
  pfEmbBDl.direction = EpcTft::DOWNLINK;
  tftEmbB->Add(pfEmbBDl);
  
  nrHelper->ActivateDedicatedEpsBearer(ueDevs, EpsBearer::NGBR_VIDEO_TCP_DEFAULT, tftEmbB);
  
  /* ================= APPLICATIONS ================= */
  // Wait for RRC connection and bearer activation
  Time appStartTime = Seconds(2.0);
  
  // URLLC: C2 Command & Control (Uplink from UAV to control center)
  // Small packets, high frequency, low latency requirement
  UdpClientHelper urllcClient(remoteHost.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), urllcPort);
  urllcClient.SetAttribute("MaxPackets", UintegerValue(1000000));
  urllcClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));  // 100 packets/sec
  urllcClient.SetAttribute("PacketSize", UintegerValue(100));  // Small C2 packets
  
  ApplicationContainer urllcClientApp = urllcClient.Install(ueNodes.Get(0));
  urllcClientApp.Start(appStartTime);
  urllcClientApp.Stop(Seconds(simTime));
  
  UdpServerHelper urllcServer(urllcPort);
  ApplicationContainer urllcServerApp = urllcServer.Install(remoteHost.Get(0));
  urllcServerApp.Start(Seconds(0.0));
  urllcServerApp.Stop(Seconds(simTime));

  // eMBB: Video streaming (Uplink from UAV)
  // Large packets, continuous stream, high throughput requirement
  UdpClientHelper embbClient(remoteHost.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), embbPort);
  embbClient.SetAttribute("MaxPackets", UintegerValue(1000000));
  embbClient.SetAttribute("Interval", TimeValue(MilliSeconds(20)));  // 50 packets/sec (~0.4 Mbps)
  embbClient.SetAttribute("PacketSize", UintegerValue(1000));  // Video packets
  
  ApplicationContainer embbClientApp = embbClient.Install(ueNodes.Get(0));
  embbClientApp.Start(appStartTime);
  embbClientApp.Stop(Seconds(simTime));
  
  UdpServerHelper embbServer(embbPort);
  ApplicationContainer embbServerApp = embbServer.Install(remoteHost.Get(0));
  embbServerApp.Start(Seconds(0.0));
  embbServerApp.Stop(Seconds(simTime));
  
  // Downlink traffic for SINR measurement (ACK-like packets)
  UdpClientHelper dlClient(ueIf.GetAddress(0), urllcPort);
  dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
  dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
  dlClient.SetAttribute("PacketSize", UintegerValue(50));
  
  ApplicationContainer dlClientApp = dlClient.Install(remoteHost.Get(0));
  dlClientApp.Start(appStartTime + Seconds(0.1));
  dlClientApp.Stop(Seconds(simTime));
  
  /* ================= SINR TRACING ================= */
  Ptr<NrUePhy> uePhy = ueNrDev->GetPhy(0);
  uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&ReportSinr));
  
  /* ================= FLOW MONITOR ================= */
  FlowMonitorHelper flowMonitor;
  Ptr<FlowMonitor> monitor = flowMonitor.InstallAll();
  
  /* ================= SIMULATION ================= */
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  
  /* ================= STATISTICS COLLECTION ================= */
  monitor->CheckForLostPackets();
  
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowMonitor.GetClassifier());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  
  std::map<std::string, FlowMonitor::FlowStats> sliceStats;
  
  for (auto it = stats.begin(); it != stats.end(); ++it)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
    
    // Classify flows by port
    if (t.destinationPort == urllcPort || t.sourcePort == urllcPort)
    {
      sliceStats["URLLC"] = it->second;
    }
    else if (t.destinationPort == embbPort || t.sourcePort == embbPort)
    {
      sliceStats["eMBB"] = it->second;
    }
  }
  
  /* ================= RESULTS OUTPUT ================= */
  std::cout << "\n";
  std::cout << "========================================================\n";
  std::cout << "  5G SA PRIVATE NETWORK - NETWORK SLICING RESULTS\n";
  std::cout << "  Scenario: Emergency UAV Rescue Operation\n";
  std::cout << "  UAV Speed: " << uavSpeed << " m/s\n";
  std::cout << "  UAV Altitude: " << uavHeight << " m\n";
  std::cout << "========================================================\n\n";
  
  for (auto &kv : sliceStats)
  {
    const std::string &sliceName = kv.first;
    const FlowMonitor::FlowStats &f = kv.second;
    
    double rxBytes = f.rxBytes;
    double rxPackets = f.rxPackets;
    double txPackets = f.txPackets;
    
    double loss = (txPackets > 0) ? ((txPackets - rxPackets) * 100.0 / txPackets) : 0.0;
    double delay = (rxPackets > 0) ? (f.delaySum.GetSeconds() / rxPackets * 1000.0) : 0.0;
    double jitter = (rxPackets > 1) ? (f.jitterSum.GetSeconds() / (rxPackets - 1) * 1000.0) : 0.0;
    double throughput = (rxBytes * 8.0) / (simTime * 1000000.0);
    double avgSinr = g_phyStats.GetAvgSinr();
    double bler = loss;
    
    std::cout << "[" << sliceName << " SLICE]\n";
    std::cout << "  Bearer Type: ";
    if (sliceName == "URLLC")
      std::cout << "GBR_CONV_VOICE (Guaranteed Bit Rate)\n";
    else
      std::cout << "NGBR_VIDEO_TCP_DEFAULT (Non-GBR)\n";
    std::cout << "  Transmitted Packets: " << (uint32_t)txPackets << "\n";
    std::cout << "  Received Packets:    " << (uint32_t)rxPackets << "\n";
    std::cout << "  Packet Loss:         " << std::fixed << std::setprecision(2) << loss << "%\n";
    std::cout << "  Avg Delay:           " << std::fixed << std::setprecision(3) << delay << " ms\n";
    std::cout << "  Avg Jitter:           " << std::fixed << std::setprecision(3) << jitter << " ms\n";
    std::cout << "  Throughput:          " << std::fixed << std::setprecision(3) << throughput << " Mbps\n";
    std::cout << "  Avg SINR:            " << std::fixed << std::setprecision(2) << avgSinr << " dB\n";
    std::cout << "  BLER (approx):       " << std::fixed << std::setprecision(2) << bler << "%\n";
    std::cout << "\n";
  }
  
  // Write to CSV
  std::ofstream csv("results_slicing.csv");
  csv << "Slice,Throughput(Mbps),Delay(ms),Jitter(ms),PacketLoss(%),SINR(dB),BLER(%)\n";
  
  for (auto &kv : sliceStats)
  {
    const std::string &sliceName = kv.first;
    const FlowMonitor::FlowStats &f = kv.second;
    
    double rxBytes = f.rxBytes;
    double rxPackets = f.rxPackets;
    double txPackets = f.txPackets;
    
    double loss = (txPackets > 0) ? ((txPackets - rxPackets) * 100.0 / txPackets) : 0.0;
    double delay = (rxPackets > 0) ? (f.delaySum.GetSeconds() / rxPackets * 1000.0) : 0.0;
    double jitter = (rxPackets > 1) ? (f.jitterSum.GetSeconds() / (rxPackets - 1) * 1000.0) : 0.0;
    double throughput = (rxBytes * 8.0) / (simTime * 1000000.0);
    double avgSinr = g_phyStats.GetAvgSinr();
    double bler = loss;
    
    csv << sliceName << ","
        << throughput << ","
        << delay << ","
        << jitter << ","
        << loss << ","
        << avgSinr << ","
        << bler << "\n";
  }
  csv.close();
  
  std::cout << "Results saved to: results_slicing.csv\n";
  std::cout << "========================================================\n\n";
  
  Simulator::Destroy();
  return 0;
}
