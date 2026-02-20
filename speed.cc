/* 5G NR Private Network – UAV Doppler Impact Simulation
 * File: testSpeedDoppler.cc
 * Purpose: Isolate Doppler effect by fixing UE–gNB distance
 * NR version: cttc-nr-demo (NR v2.4 compatible)
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
 
 NS_LOG_COMPONENT_DEFINE("TestSpeedDoppler");
 
 /* ================= PHY STATISTICS ================= */
 struct PhyStats
 {
   std::vector<double> sinrDb;
 
   void Add(double sinr)
   {
     sinrDb.push_back(sinr);
   }
 
   double Avg() const
   {
     if (sinrDb.empty()) return 0.0;
     double s = 0.0;
     for (double v : sinrDb) s += v;
     return s / sinrDb.size();
   }
 
   double Std() const
   {
     if (sinrDb.size() < 2) return 0.0;
     double mean = Avg();
     double var = 0.0;
     for (double v : sinrDb)
       var += (v - mean) * (v - mean);
     return std::sqrt(var / (sinrDb.size() - 1));
   }
 };
 
 PhyStats g_phyStats;
 
 /* ================= SINR CALLBACK ================= */
 void
 ReportSinr(uint16_t cellId,
            uint16_t rnti,
            double avgSinr,
            uint16_t bwpId,
            uint8_t streamId)
 {
   double sinrDb = 10.0 * std::log10(avgSinr);
   g_phyStats.Add(sinrDb);
 }
 
 /* ================= VELOCITY REVERSAL ================= */
 void
 ReverseVelocity(Ptr<ConstantVelocityMobilityModel> mob)
 {
   Vector v = mob->GetVelocity();
   mob->SetVelocity(Vector(-v.x, v.y, v.z));
 }
 
 /* ================= MAIN ================= */
 int
 main(int argc, char* argv[])
 {
   double simTime = 20.0;
   double altitude = 20.0;
   double centerDistance = 60.0;
   int runs = 5; // multi-run averaging
 
   CommandLine cmd;
   cmd.Parse(argc, argv);
 
   RngSeedManager::SetSeed(1);
 
   std::vector<double> speeds =
   {
     0.0, 5.0, 10.0, 15.0, 20.0,
     25.0, 30.0, 35.0, 40.0, 45.0, 50.0
   };
 
   std::ofstream csv("results_speed_doppler.csv");
   csv << "Speed(m/s),AvgDelay(ms),Jitter(ms),PacketLoss(%),"
          "Throughput(Mbps),AvgSINR(dB),SINR_Std(dB)\n";
   csv.close();
 
   for (double speed : speeds)
   {
     double lossSum = 0.0, delaySum = 0.0, jitterSum = 0.0;
     double sinrMeanSum = 0.0, sinrStdSum = 0.0;
     double throughputSum = 0.0;
 
     for (int r = 0; r < runs; ++r)
     {
       Simulator::Destroy();
       g_phyStats = PhyStats();
       RngSeedManager::SetRun(r + 1);
 
       /* ================= NODES ================= */
       NodeContainer gnbNodes, ueNodes, remoteHost;
       gnbNodes.Create(1);
       ueNodes.Create(1);
       remoteHost.Create(1);
 
       /* ================= MOBILITY ================= */
       MobilityHelper mobility;
 
       mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
       mobility.Install(gnbNodes);
       gnbNodes.Get(0)->GetObject<MobilityModel>()
         ->SetPosition(Vector(0.0, 0.0, 15.0));
 
       mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
       mobility.Install(ueNodes);
 
       Ptr<ConstantVelocityMobilityModel> uavMob =
         ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
 
       uavMob->SetPosition(Vector(centerDistance, 0.0, altitude));
       uavMob->SetVelocity(Vector(speed, 0.0, 0.0));
 
       for (double t = 1.0; t < simTime; t += 1.0)
       {
         Simulator::Schedule(Seconds(t), &ReverseVelocity, uavMob);
       }
 
       /* ================= CHANNEL ================= */
       Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                          TimeValue(MilliSeconds(0)));
       Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                          TimeValue(MilliSeconds(0)));
 
       /* ================= NR + EPC ================= */
       Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
       Ptr<NrPointToPointEpcHelper> epcHelper =
         CreateObject<NrPointToPointEpcHelper>();
 
       nrHelper->SetEpcHelper(epcHelper);
 
       nrHelper->SetSchedulerTypeId(
         TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
 
       nrHelper->SetHarqEnabled(true); // expose PHY errors
 
       nrHelper->SetDlErrorModel("ns3::NrEesmIrT1");
       nrHelper->SetUlErrorModel("ns3::NrEesmIrT1");
 
       nrHelper->SetPathlossAttribute("ShadowingEnabled",
                                      BooleanValue(false));
 
       nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(35.0));
       nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(0)); // 15 kHz
 
       /* ================= BAND ================= */
       CcBwpCreator ccBwpCreator;
       CcBwpCreator::SimpleOperationBandConf bandConf(
         3.5e9, 40e6, 1, BandwidthPartInfo::UMa);
 
       OperationBandInfo band =
         ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
 
       nrHelper->InitializeOperationBand(&band);
       auto allBwps = CcBwpCreator::GetAllBwps({band});
 
       /* ================= ANTENNAS ================= */
       nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
       nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
       nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
       nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
 
       /* ================= DEVICES ================= */
       auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
       auto ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
 
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
       p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
 
       auto p2pDevs = p2p.Install(pgw, remoteHost.Get(0));
 
       Ipv4AddressHelper ipv4;
       ipv4.SetBase("1.0.0.0", "255.0.0.0");
       auto ifaces = ipv4.Assign(p2pDevs);
 
       epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
 
       Ipv4StaticRoutingHelper routing;
       auto ueRoute =
         routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
       ueRoute->SetDefaultRoute(
         epcHelper->GetUeDefaultGatewayAddress(), 1);
 
       nrHelper->AttachToClosestEnb(ueDevs, gnbDevs);
 
       /* ================= SINR TRACE ================= */
       Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevs.Get(0), 0);
       uePhy->TraceConnectWithoutContext("DlDataSinr",
                                         MakeCallback(&ReportSinr));
 
       /* ================= TRAFFIC ================= */
       uint16_t port = 5000;
      UdpServerHelper server(port);
      auto serverApps = server.Install(remoteHost.Get(0));
      serverApps.Start(Seconds(0.1));
 
       UdpClientHelper client(ifaces.GetAddress(1), port);
       client.SetAttribute("PacketSize", UintegerValue(100));
       client.SetAttribute("Interval", TimeValue(MicroSeconds(200)));
       client.SetAttribute("MaxPackets", UintegerValue(1000000));
 
      auto clientApps = client.Install(ueNodes.Get(0));
      clientApps.Start(Seconds(0.2));
 
       /* ================= FLOW MONITOR ================= */
       FlowMonitorHelper flowmon;
       Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
       Simulator::Stop(Seconds(simTime));
       Simulator::Run();
 
       monitor->CheckForLostPackets();
       auto stats = monitor->GetFlowStats();
 
       uint32_t tx = 0, rx = 0;
       uint64_t rxBytes = 0;
       double delay = 0.0, jitter = 0.0;
       Time tFirst, tLast;
 
       for (auto& f : stats)
       {
         tx += f.second.txPackets;
         rx += f.second.rxPackets;
         rxBytes += f.second.rxBytes;
 
         if (f.second.rxPackets > 0)
         {
           delay += f.second.delaySum.GetSeconds() / f.second.rxPackets;
           if (f.second.rxPackets > 1)
             jitter += f.second.jitterSum.GetSeconds() /
                       (f.second.rxPackets - 1);
 
           tFirst = f.second.timeFirstRxPacket;
           tLast  = f.second.timeLastRxPacket;
         }
       }
 
       double loss = (tx > 0) ? (1.0 - (double)rx / tx) * 100.0 : 0.0;
       double throughput = (tLast > tFirst) ?
         (rxBytes * 8.0) / ((tLast - tFirst).GetSeconds() * 1e6) : 0.0;
 
       lossSum += loss;
       delaySum += delay;
       jitterSum += jitter;
       throughputSum += throughput;
       sinrMeanSum += g_phyStats.Avg();
       sinrStdSum += g_phyStats.Std();
     }
 
     std::ofstream out("results_speed_doppler.csv", std::ios::app);
     out << speed << ","
         << (delaySum / runs) * 1000 << ","
         << (jitterSum / runs) * 1000 << ","
         << (lossSum / runs) << ","
         << (throughputSum / runs) << ","
         << (sinrMeanSum / runs) << ","
         << (sinrStdSum / runs) << "\n";
     out.close();
   }
 
   return 0;
 }
 