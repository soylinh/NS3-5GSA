/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * KỊCH BẢN: 5G Private Network - Cứu hộ khẩn cấp (URLLC)
 * Dựa trên template chuẩn cttc-nr-demo.cc của ns-3.38/nr-v2.4
 * Đã tinh chỉnh: 1 gNB, 1 Drone, Numerology 3, Shadowing ON, Gói tin nhỏ.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RescueScenarioDemo");

int
main(int argc, char* argv[])
{
    // --- 1. THIẾT LẬP THAM SỐ KỊCH BẢN CỨU HỘ ---
    
    // Topology: 1 Xe trạm phát (gNB) và 1 Robot/Drone (UE)
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 1; 
    
    bool logging = false;
    bool doubleOperationalBand = false; // Chỉ dùng 1 băng tần cho đơn giản

    // Traffic URLLC (Điều khiển Robot):
    // Gói tin cực nhỏ (100 bytes) để giảm trễ đóng gói
    uint32_t udpPacketSizeULL = 100; 
    // Gửi liên tục 2000 gói/giây (Interval 0.5ms) để Real-time
    uint32_t lambdaULL = 2000;      

    // Traffic nền (Không dùng trong kịch bản này, nhưng giữ lại để tránh lỗi code)
    uint32_t udpPacketSizeBe = 1000;
    uint32_t lambdaBe = 100;

    // Thời gian mô phỏng: 10 giây để lấy mẫu ổn định
    Time simTime = Seconds(10.0);
    Time udpAppStartTime = MilliSeconds(400);

    // Cấu hình Vật lý 5G (PHY Layer):
    // Numerology 3 (120 kHz SCS) -> Slot time 0.125ms -> CHÌA KHÓA CỦA URLLC
    uint16_t numerologyBwp1 = 3; 
    
    // Tần số mmWave 28 GHz (Băng thông rộng)
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 100e6; // 100 MHz
    
    double totalTxPower = 30; // 30 dBm (1 Watt) - Công suất trạm nhỏ trên xe

    std::string simTag = "rescue-urllc";
    std::string outputDir = "./";

    // Xử lý tham số dòng lệnh (Giữ nguyên cấu trúc template)
    CommandLine cmd(__FILE__);
    cmd.AddValue("gNbNum", "Number of gNbs", gNbNum);
    cmd.AddValue("ueNumPergNb", "Number of UE per gNb", ueNumPergNb);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.Parse(argc, argv);

    // Bật log nếu cần
    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // --- 2. TẠO VỊ TRÍ (MOBILITY) ---
    // Sử dụng GridScenarioHelper nhưng cấu hình cho 1 node
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);
    
    // Đặt Drone cách trạm 100m (Vùng an toàn để test)
    gridScenario.SetHorizontalBsDistance(100.0); 
    gridScenario.SetVerticalBsDistance(100.0);
    gridScenario.SetBsHeight(15.0); // Trạm trên xe cao 15m
    gridScenario.SetUtHeight(5.0);  // Drone bay thấp 5m
    
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNbNum);
    gridScenario.SetUtNumber(ueNumPergNb * gNbNum);
    gridScenario.SetScenarioHeight(3); 
    gridScenario.SetScenarioLength(3); 
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    // Phân loại UE: Vì chỉ có 1 UE (index 0), nó sẽ vào nhóm LowLat (URLLC)
    NodeContainer ueLowLatContainer;
    NodeContainer ueVoiceContainer;

    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);
        if (j % 2 == 0) 
        {
            ueLowLatContainer.Add(ue); // UE số 0 vào đây
        }
        else
        {
            ueVoiceContainer.Add(ue);
        }
    }

    // --- 3. CẤU HÌNH 5G RAN & CORE ---
    // Dùng NrPointToPointEpcHelper (Mô hình NR kết nối Core)
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Cấu hình Băng tần & Mô hình Kênh
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; 

    // QUAN TRỌNG: Sử dụng UMi_StreetCanyon (Hẻm phố) để mô phỏng đổ nát
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::UMi_StreetCanyon);

    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);

    // Cấu hình Vật cản (Shadowing) - BẬT LÊN để đúng ngữ cảnh khắc nghiệt
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    nrHelper->InitializeOperationBand(&band1);

    // Tính toán công suất phát
    double x = pow(10, totalTxPower / 10);
    double totalBandwidth = bandwidthBand1;
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Cấu hình Beamforming & Delay Core
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0))); // Local Core delay = 0

    // Cấu hình Antenna (MIMO 2x4 cho UE, 4x8 cho gNB)
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));

    // --- 4. CÀI ĐẶT THIẾT BỊ ---
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueLowLatNetDev = nrHelper->InstallUeDevice(ueLowLatContainer, allBwps);
    NetDeviceContainer ueVoiceNetDev = nrHelper->InstallUeDevice(ueVoiceContainer, allBwps);

    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueLowLatNetDev, randomStream);

    // Cấu hình Numerology 3 cho gNB (URLLC)
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(totalTxPower));

    // Update Config
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    for (auto it = ueLowLatNetDev.Begin(); it != ueLowLatNetDev.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // --- 5. KẾT NỐI SERVER & ROUTING ---
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Kết nối Local Breakout (PGW <-> Server)
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0001))); // 0.1ms LAN delay
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    
    // Định tuyến Server -> UE
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    
    internet.Install(gridScenario.GetUserTerminals());

    Ipv4InterfaceContainer ueLowLatIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLowLatNetDev));

    // Set Default Gateway cho UE
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(
            gridScenario.GetUserTerminals().Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach (Kết nối vào mạng)
    nrHelper->AttachToClosestEnb(ueLowLatNetDev, enbNetDev);

    // --- 6. TRIỂN KHAI TRAFFIC (URLLC SLICING) ---
    uint16_t dlPortLowLat = 1234;
    ApplicationContainer serverApps;

    // A. UE (Drone) lắng nghe lệnh
    UdpServerHelper dlPacketSinkLowLat(dlPortLowLat);
    serverApps.Add(dlPacketSinkLowLat.Install(ueLowLatContainer));

    // B. Server gửi lệnh điều khiển (URLLC Traffic)
    UdpClientHelper dlClientLowLat;
    dlClientLowLat.SetAttribute("RemotePort", UintegerValue(dlPortLowLat));
    dlClientLowLat.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientLowLat.SetAttribute("PacketSize", UintegerValue(udpPacketSizeULL)); // 100 bytes
    dlClientLowLat.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambdaULL))); // 0.5 ms

    // C. Cấu hình Slicing: Bearer QCI 80 (Low Latency)
    EpsBearer lowLatBearer(EpsBearer::NGBR_LOW_LAT_EMBB);
    Ptr<EpcTft> lowLatTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfLowLat;
    dlpfLowLat.localPortStart = dlPortLowLat;
    dlpfLowLat.localPortEnd = dlPortLowLat;
    lowLatTft->Add(dlpfLowLat);

    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < ueLowLatContainer.GetN(); ++i)
    {
        Ptr<Node> ue = ueLowLatContainer.Get(i);
        Ptr<NetDevice> ueDevice = ueLowLatNetDev.Get(i);
        Address ueAddress = ueLowLatIpIface.GetAddress(i);

        dlClientLowLat.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClientLowLat.Install(remoteHost));

        // Kích hoạt Bearer URLLC
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, lowLatBearer, lowLatTft);
    }

    // Chạy ứng dụng
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // --- 7. THU THẬP KẾT QUẢ ---
    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(gridScenario.GetUserTerminals());

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    NS_LOG_UNCOND("--- STARTING RESCUE SIMULATION ---");
    Simulator::Stop(simTime);
    Simulator::Run();

    // In kết quả ra màn hình
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        if (t.destinationPort == dlPortLowLat) {
            NS_LOG_UNCOND("\n=== URLLC CONTROL FLOW RESULT (Server -> Drone) ===");
            NS_LOG_UNCOND("  Tx Packets : " << i->second.txPackets);
            NS_LOG_UNCOND("  Rx Packets : " << i->second.rxPackets);
            NS_LOG_UNCOND("  Packet Loss: " << i->second.txPackets - i->second.rxPackets);
            
            double delay = i->second.delaySum.GetSeconds() / i->second.rxPackets * 1000;
            NS_LOG_UNCOND("  MEAN DELAY : " << delay << " ms");
            
            double jitter = i->second.jitterSum.GetSeconds() / i->second.rxPackets * 1000;
            NS_LOG_UNCOND("  JITTER     : " << jitter << " ms");
        }
    }

    Simulator::Destroy();
    return 0;
}