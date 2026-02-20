/* 5G NR UAV Multi-Parameter Sweep Simulation
 * Altitude – Speed – Packet Size – Tx Power – Frequency Band
 * Compatible with NR module (ns-3.41+ / CTTC NR v2.4 style)
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/applications-module.h"
 
 #include "ns3/nr-module.h"
 #include "ns3/antenna-module.h"
 #include "ns3/point-to-point-module.h"
 #include "ns3/flow-monitor-module.h"
 
 #include <fstream>
 #include <vector>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("NrUavMultiSweep");
 
 struct BandConfig
 {
   double centerFreq;
   double bandwidth;
   BandwidthPartInfo::Scenario scenario;
   std::string name;
 };
 
 int
 main(int argc, char *argv[])
 {
   Time simTime = Seconds(10.0);
 
   std::vector<double> altitudes = {1.5, 20.0, 50.0, 100.0};
   std::vector<double> speeds    = {0.0, 10.0, 30.0, 60.0};
 
   std::vector<uint32_t> packetSizes = {128, 512, 1200};
   std::vector<double> txPowers = {23.0, 30.0, 37.0};
 
   std::vector<BandConfig> bands = {
     {2.6e9,  20e6,  BandwidthPartInfo::UMa, "2.6GHz"},
     {3.5e9,  40e6,  BandwidthPartInfo::UMi_StreetCanyon, "3.5GHz"},
     {28e9, 100e6,  BandwidthPartInfo::UMi_StreetCanyon, "28GHz"}
   };
 
   std::ofstream out("results_all.csv");
   out << "Band,TxPower(dBm),PacketSize(B),Altitude(m),Speed(m/s),"
       << "AvgDelay(ms),Jitter(ms),PacketLoss(%),Throughput(Mbps)\n";
 
   for (const auto &band : bands)
   {
     for (double txPower : txPowers)
     {
       for (uint32_t pktSize : packetSizes)
       {
         for (double altitude : altitudes)
         {
           for (double speed : speeds)
           {
             Simulator::Destroy();
 
             // ---------------- NR Helpers ----------------
             Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
             Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
             Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
             
             nrHelper->SetEpcHelper(epcHelper);
             nrHelper->SetBeamformingHelper(beamformingHelper);
 
             nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
             nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
 
             nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
             nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
             nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
             nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
 
             // ---------------- Band configuration ----------------
             CcBwpCreator ccBwpCreator;
             CcBwpCreator::SimpleOperationBandConf bandConf(
               band.centerFreq,
               band.bandwidth,
               1,
               band.scenario);
 
             OperationBandInfo bandInfo = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
             nrHelper->InitializeOperationBand(&bandInfo);
             BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({bandInfo});
 
             // ---------------- Nodes ----------------
             NodeContainer gnbNodes, ueNodes;
             gnbNodes.Create(1);
             ueNodes.Create(1);
 
             // ---------------- Mobility ----------------
             MobilityHelper mobility;
 
             mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
             mobility.Install(gnbNodes);
             gnbNodes.Get(0)->GetObject<MobilityModel>()
               ->SetPosition(Vector(0.0, 0.0, 25.0));
 
             mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
             mobility.Install(ueNodes);
 
             Ptr<ConstantVelocityMobilityModel> uavMob =
               ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
 
             uavMob->SetPosition(Vector(100.0, 0.0, altitude));
             uavMob->SetVelocity(Vector(speed, 0.0, 0.0));
 
             // ---------------- Install Devices ----------------
             NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
             NetDeviceContainer ueDevs  = nrHelper->InstallUeDevice(ueNodes, allBwps);

             // Update configuration for all devices
             for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
             {
               DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
             }
             for (auto it = ueDevs.Begin(); it != ueDevs.End(); ++it)
             {
               DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
             }
 
             // ---------------- Internet ----------------
             InternetStackHelper internet;
             internet.Install(ueNodes);

             // PGW node already has internet stack installed by EPC helper
             Ptr<Node> pgw = epcHelper->GetPgwNode();
 
             Ipv4InterfaceContainer ueIp =
               epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
             Ipv4StaticRoutingHelper routing;
             Ptr<Ipv4StaticRouting> ueRoute =
               routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
             ueRoute->SetDefaultRoute(
               epcHelper->GetUeDefaultGatewayAddress(), 1);

             nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
 
             // ---------------- Applications ----------------
             uint16_t port = 4000;
 
             UdpServerHelper server(port);
             ApplicationContainer serverApp = server.Install(pgw);
             serverApp.Start(Seconds(0.5));
             serverApp.Stop(simTime);
 
             UdpClientHelper client(ueIp.GetAddress(0), port);
             client.SetAttribute("Interval", TimeValue(MilliSeconds(20)));
             client.SetAttribute("PacketSize", UintegerValue(pktSize));
             client.SetAttribute("MaxPackets", UintegerValue(1000000));
 
             ApplicationContainer clientApp = client.Install(ueNodes.Get(0));
             clientApp.Start(Seconds(1.0));
             clientApp.Stop(simTime);
 
             // ---------------- Flow Monitor ----------------
             FlowMonitorHelper flowmon;
             Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
             // ---------------- Run ----------------
             Simulator::Stop(simTime);
             Simulator::Run();
 
             // ---------------- Statistics ----------------
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
                 
                 if (f.second.rxPackets > 1)
                 {
                   jitter +=
                     f.second.jitterSum.GetSeconds() /
                     (f.second.rxPackets - 1);
                 }
                 
                 if (firstRxTime == Seconds(0) || f.second.timeFirstRxPacket < firstRxTime)
                   firstRxTime = f.second.timeFirstRxPacket;
                 if (f.second.timeLastRxPacket > lastRxTime)
                   lastRxTime = f.second.timeLastRxPacket;
               }
             }

             double loss = (tx > 0) ? (1.0 - (double)rx / tx) * 100 : 0;
             
             // Calculate throughput (Mbps)
             double throughput = 0.0;
             double rxDuration = (lastRxTime - firstRxTime).GetSeconds();
             if (rxDuration > 0 && rxBytes > 0)
             {
               throughput = (rxBytes * 8.0) / (rxDuration * 1000000.0);
             }
             
             double avgDelay = (rx > 0) ? delay : 0.0;
             double avgJitter = (rx > 1) ? jitter : 0.0;
 
             // ---------------- Save ----------------
             out << band.name << ","
                 << txPower << ","
                 << pktSize << ","
                 << altitude << ","
                 << speed << ","
                 << avgDelay * 1000 << ","
                 << avgJitter * 1000 << ","
                 << loss << ","
                 << throughput << "\n";
           }
         }
       }
     }
   }
 
   out.close();
   std::cout << "Simulation finished. Results saved to results_all.csv\n";
   return 0;
 }
 