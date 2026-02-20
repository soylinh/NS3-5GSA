/* Rescue simulation with per-flow metrics: delay, jitter, packet loss
 * - Uses raw UDP sockets to include sender timestamp (ns) in packet payload
 * - Computes per-flow: txPackets, rxPackets, mean delay, jitter (avg abs diff of successive delays), packet loss
 * - Default: numerology=2, bandwidth=400 MHz to avoid bwInRbg==0 on ns-3.38
 * Replace scratch/test.cc with this file and rebuild.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/lte-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/antenna-module.h"

#include <map>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RescueWithMetrics");

struct FlowMetrics {
  uint64_t txPackets = 0;
  uint64_t rxPackets = 0;
  double sumDelaySec = 0.0; // sum of per-packet one-way delays (seconds)
  double sumAbsDiffDelaySec = 0.0; // sum of abs differences between successive delays
  double lastDelaySec = -1.0; // last packet delay seen
};

static std::map<uint16_t, FlowMetrics> metricsByPort; // keyed by destination port

// Helper: send a packet with current timestamp (ns) as 8-byte header (network byte order not required here)
static void SendTimestampedPacket(Ptr<Socket> socket, uint32_t pktSize, Ipv4Address dstAddr, uint16_t dstPort)
{
  // total packet size: 8 bytes timestamp + (pktSize - 8) payload
  uint32_t body = pktSize >= 8 ? pktSize - 8 : 0;
  // prepare timestamp
  uint64_t tns = Simulator::Now().GetNanoSeconds();
  // create packet
  Ptr<Packet> p = Create<Packet>(0);
  // add timestamp as 8 bytes
  p->AddAtEnd(Create<Packet>((uint8_t*)&tns, sizeof(tns)));
  if (body > 0) {
    Ptr<Packet> payload = Create<Packet>(body);
    p->AddAtEnd(payload);
  }
  Address to = InetSocketAddress(dstAddr, dstPort);
  socket->SendTo(p, 0, to);
  // increase tx count for that port
  metricsByPort[dstPort].txPackets++;
}

// Periodic sender function
static void ScheduleNextSend(Ptr<Socket> socket, uint32_t pktSize, Ipv4Address dstAddr, uint16_t dstPort, Time interval, Time stop)
{
  if (Simulator::Now() >= stop) return;
  SendTimestampedPacket(socket, pktSize, dstAddr, dstPort);
  Simulator::Schedule(interval, &ScheduleNextSend, socket, pktSize, dstAddr, dstPort, interval, stop);
}

// Receiver callback: parse timestamp, compute one-way delay and jitter
static void ReceivePacketCallback(Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet;
  while ((packet = socket->RecvFrom(from))) {
    uint16_t srcPort = 0, dstPort = 0;
    InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
    srcPort = address.GetPort();
    // For get local port (destination port) we can get socket's bound address
    // But easier: use a hack: get socket->GetSockName
    Address local;
    socket->GetSockName(local);
    InetSocketAddress localAddr = InetSocketAddress::ConvertFrom(local);
    dstPort = localAddr.GetPort();

    // Extract timestamp (first 8 bytes) if present
    uint64_t sendTns = 0;
    if (packet->GetSize() >= (int)sizeof(sendTns)) {
      // read first 8 bytes
      uint8_t buf[8];
      packet->CopyData(buf, 8);
      // reconstruct uint64_t from bytes (assuming little-endian same as sender env)
      memcpy(&sendTns, buf, sizeof(sendTns));
    }
    double nowSec = Simulator::Now().GetSeconds();
    double delaySec = 0.0;
    if (sendTns != 0) {
      uint64_t nowns = Simulator::Now().GetNanoSeconds();
      if (nowns >= sendTns) {
        delaySec = double(nowns - sendTns) / 1e9;
      } else {
        // simulator time shouldn't go backwards, but handle
        delaySec = 0.0;
      }
    }

    FlowMetrics &m = metricsByPort[dstPort];
    m.rxPackets++;
    m.sumDelaySec += delaySec;
    if (m.lastDelaySec >= 0.0) {
      m.sumAbsDiffDelaySec += std::fabs(delaySec - m.lastDelaySec);
    }
    m.lastDelaySec = delaySec;
  }
}

int
main(int argc, char *argv[])
{
  // --- 1. PARAMETERS ---
  double simTime = 5.0;

  // PHY params (use safe numerology)
  double frequency = 3.5e9;      // 28 GHz
  double bandwidth = 100e6;     // 400 MHz
  double txPower = 30.0;        // 30 dBm
  double noiseFigure = 9.0;
  uint16_t numerology = 2;      // use 2 (SCS=60kHz) as safe default to avoid bwInRbg=0

  // Traffic
  uint32_t pktSizeControl = 100;
  Time intervalControl = MilliSeconds(0.5);

  uint32_t pktSizeVideo = 1472;
  Time intervalVideo = MicroSeconds(580);

  double droneSpeed = 20.0;
  double droneHeight = 25.0;
  double gnbHeight = 15.0;

  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time", simTime);
  cmd.Parse(argc, argv);

  // --- 2. NODES ---
  NodeContainer droneNodes; NodeContainer gnbNodes; NodeContainer serverNodes;
  droneNodes.Create(1); gnbNodes.Create(1); serverNodes.Create(1);

  // --- 3. MOBILITY ---
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
  gnbPos->Add(Vector(0.0, 0.0, gnbHeight));
  mobility.SetPositionAllocator(gnbPos);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(gnbNodes);
  mobility.Install(serverNodes);

  Ptr<ListPositionAllocator> droneStartPos = CreateObject<ListPositionAllocator>();
  droneStartPos->Add(Vector(50.0, 0.0, droneHeight));
  mobility.SetPositionAllocator(droneStartPos);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(droneNodes);
  droneNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(droneSpeed, 0.0, 0.0));

  // --- 4. NR & EPC ---
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
  beamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  nrHelper->SetBeamformingHelper(beamformingHelper);
  nrHelper->SetEpcHelper(epcHelper);
  nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

  CcBwpCreator ccBwpCreator;
  CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, 1, BandwidthPartInfo::UMi_StreetCanyon);
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
  nrHelper->InitializeOperationBand(&band);

  nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
  nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
  nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(noiseFigure));

  // --- 5. INSTALL DEVICES ---
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
  if (allBwps.empty()) {
    NS_FATAL_ERROR("BWP Configuration Failed! Vector is empty. Try changing numerology/bandwidth.");
  }

  NS_LOG_UNCOND("--- RESCUE SIM CONFIG ---");
  NS_LOG_UNCOND("Frequency (Hz): " << frequency);
  NS_LOG_UNCOND("Bandwidth (Hz): " << bandwidth);
  NS_LOG_UNCOND("Numerology: " << numerology);
  NS_LOG_UNCOND("TxPower (dBm): " << txPower);
  NS_LOG_UNCOND("Drone Height (m): " << droneHeight);
  NS_LOG_UNCOND("Drone Speed (m/s): " << droneSpeed);

  NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
  NetDeviceContainer droneNetDev = nrHelper->InstallUeDevice(droneNodes, allBwps);

  // --- 6. Internet & routing ---
  InternetStackHelper internet;
  internet.Install(droneNodes);
  internet.Install(serverNodes);
  PointToPointHelper p2p; p2p.SetDeviceAttribute("DataRate", StringValue("100Gbps")); p2p.SetChannelAttribute("Delay", StringValue("0.1ms"));
  NetDeviceContainer internetDevices = p2p.Install(epcHelper->GetPgwNode(), serverNodes.Get(0));
  Ipv4AddressHelper ipv4h; ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  Ipv4InterfaceContainer droneIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(droneNetDev));

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> serverStaticRouting = ipv4RoutingHelper.GetStaticRouting(serverNodes.Get(0)->GetObject<Ipv4>());
  serverStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

  nrHelper->AttachToClosestEnb(droneNetDev, gnbNetDev);

  // --- 7. TRAFFIC USING RAW SOCKETS (so we can timestamp) ---
  uint16_t portControl = 1234; // server -> drone
  uint16_t portVideo = 8080;   // drone -> server

  // Create receiver sockets
  // Control sink on drone
  Ptr<Socket> droneSinkSocket = Socket::CreateSocket(droneNodes.Get(0), UdpSocketFactory::GetTypeId());
  InetSocketAddress droneLocal = InetSocketAddress(Ipv4Address::GetAny(), portControl);
  droneSinkSocket->Bind(droneLocal);
  droneSinkSocket->SetRecvCallback(MakeCallback(&ReceivePacketCallback));

  // Video sink on server
  Ptr<Socket> serverSinkSocket = Socket::CreateSocket(serverNodes.Get(0), UdpSocketFactory::GetTypeId());
  InetSocketAddress serverLocal = InetSocketAddress(Ipv4Address::GetAny(), portVideo);
  serverSinkSocket->Bind(serverLocal);
  serverSinkSocket->SetRecvCallback(MakeCallback(&ReceivePacketCallback));

  // Create sender sockets
  // Control sender (on server) sending to drone
  Ptr<Socket> controlSender = Socket::CreateSocket(serverNodes.Get(0), UdpSocketFactory::GetTypeId());
  // bind to ephemeral port
  controlSender->Bind();
  Ipv4Address droneAddr = droneIpIface.GetAddress(0);
  Time controlStart = Seconds(0.1);
  Time controlStop = Seconds(simTime);
  Simulator::Schedule(controlStart, &ScheduleNextSend, controlSender, pktSizeControl, droneAddr, portControl, intervalControl, controlStop);

  // Video sender (on drone) sending to server
  Ptr<Socket> videoSender = Socket::CreateSocket(droneNodes.Get(0), UdpSocketFactory::GetTypeId());
  videoSender->Bind();
  Ipv4Address serverAddr = internetIpIfaces.GetAddress(1); // pgw-server link
  Time videoStart = Seconds(0.2);
  Time videoStop = Seconds(simTime);
  Simulator::Schedule(videoStart, &ScheduleNextSend, videoSender, pktSizeVideo, serverAddr, portVideo, intervalVideo, videoStop);

  // --- 8. MONITOR & RUN ---
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

  NS_LOG_UNCOND("--- STARTING RESCUE SIMULATION WITH METRICS ---");
  NS_LOG_UNCOND("Drone Speed: " << droneSpeed << " m/s");

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  // After run: compute and print metrics per port
  NS_LOG_UNCOND("\n=== METRICS SUMMARY ===");
  for (auto &kv : metricsByPort) {
    uint16_t port = kv.first;
    FlowMetrics &m = kv.second;
    double meanDelayMs = (m.rxPackets > 0) ? (m.sumDelaySec / m.rxPackets * 1000.0) : 0.0;
    double avgJitterMs = (m.rxPackets > 1) ? (m.sumAbsDiffDelaySec / (m.rxPackets - 1) * 1000.0) : 0.0;
    uint64_t tx = m.txPackets;
    uint64_t rx = m.rxPackets;
    uint64_t lost = (tx >= rx) ? (tx - rx) : 0;
    double lossRate = (tx > 0) ? (double)lost / (double)tx * 100.0 : 0.0;

    NS_LOG_UNCOND("Port: " << port);
    NS_LOG_UNCOND("  Tx Packets : " << tx);
    NS_LOG_UNCOND("  Rx Packets : " << rx);
    NS_LOG_UNCOND("  Lost Packets: " << lost << " (" << lossRate << "%)");
    NS_LOG_UNCOND("  Mean One-way Delay : " << meanDelayMs << " ms");
    NS_LOG_UNCOND("  Avg Jitter (abs diff delays) : " << avgJitterMs << " ms");
  }

  monitor->CheckForLostPackets();
  Simulator::Destroy();
  return 0;
}
