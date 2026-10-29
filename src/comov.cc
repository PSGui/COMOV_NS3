    /*
 * COMOV - Pedidos de prioridade V2I em IEEE 802.11p (ns-3.48)
 *
 * Adaptado de examples/wireless/wifi-simple-adhoc.cc:
 * Copyright (c) 2009 The Boeing Company
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Etapa: RSU fixa + N autocarros moveis (N configuravel).
 * Mantem-se UDP broadcast, como na versao funcional anterior,
 * para testar apenas a transicao de 1 para N autocarros.
 * Os CSV registam os pacotes de cada autocarro individualmente.
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/seq-ts-header.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Comov");

struct PacketData
{
    uint32_t busId = 0;      // 1, 2, ..., nBuses
    uint32_t busPacket = 0;  // indice local do pacote
    double txTime = 0.0;    // segundos
    double distance = 0.0;  // metros, no instante TX
    bool received = false;
    double rxTime = 0.0;    // segundos
    double delayMs = 0.0;   // milissegundos
};

static uint32_t gPhyRxDrops = 0;

static void OnPhyRxDrop(Ptr<const Packet> packet, WifiPhyRxfailureReason reason)
{
    ++gPhyRxDrops;
    NS_LOG_UNCOND("PHY RX DROP | t=" << Simulator::Now().GetSeconds() << " s | reason="<< reason);
}

static uint32_t gNextSeq = 0;
static uint32_t gTxPackets = 0;
static uint32_t gRxPackets = 0;
static double gTotalDelayMs = 0.0;
static std::vector<uint32_t> gTxByBus;
static std::vector<uint32_t> gRxByBus;
static std::vector<double> gDelayByBus;
static std::map<uint32_t, PacketData> gRecords;

// Ficheiros compatíveis com Excel em portugues: ';' e virgula decimal.
class CsvDecimalComma : public std::numpunct<char>
{
  protected:
    char do_decimal_point() const override
    {
        return ',';
    }
};

static void
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    while ((packet = socket->Recv()))
    {
        SeqTsHeader header;
        if (packet->GetSize() < header.GetSerializedSize())
        {
            NS_LOG_UNCOND("RX ERROR | pacote demasiado pequeno");
            continue;
        }
        packet->RemoveHeader(header);
        const uint32_t seq = header.GetSeq();
        const auto it = gRecords.find(seq);
        if (it == gRecords.end())
        {
            NS_LOG_UNCOND("RX ERROR | pacote sem registo TX: " << seq);
            continue;
        }
        if (it->second.received)
        {
            NS_LOG_UNCOND("RX DUPLICATE | pacote " << seq);
            continue;
        }

        const double delayMs = (Simulator::Now() - header.GetTs()).GetSeconds() * 1000.0;
        PacketData& data = it->second;
        data.received = true;
        data.rxTime = Simulator::Now().GetSeconds();
        data.delayMs = delayMs;

        ++gRxPackets;
        ++gRxByBus[data.busId - 1];
        gTotalDelayMs += delayMs;
        gDelayByBus[data.busId - 1] += delayMs;

        NS_LOG_UNCOND("RX | BUS " << data.busId << " | seq " << seq
                      << " | delay " << delayMs << " ms");
    }
}

static void
GenerateTraffic(Ptr<Socket> socket,
                uint32_t busId,
                Ptr<MobilityModel> rsuMobility,
                uint32_t pktSize,
                uint32_t remaining,
                uint32_t busPacket,
                Time pktInterval)
{
    if (remaining == 0)
    {
        socket->Close();
        return;
    }

    Ptr<MobilityModel> busMobility = socket->GetNode()->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!busMobility, "BUS sem MobilityModel");

    SeqTsHeader header;
    const uint32_t seq = ++gNextSeq; // numero global unico para todos os BUS
    header.SetSeq(seq);
    const uint32_t headerSize = header.GetSerializedSize();
    NS_ABORT_MSG_IF(pktSize < headerSize, "packetSize deve ser >= 12 bytes");

    Ptr<Packet> packet = Create<Packet>(pktSize - headerSize);
    packet->AddHeader(header);

    const double distance = busMobility->GetDistanceFrom(rsuMobility);
    const int sent = socket->Send(packet);
    if (sent == static_cast<int>(packet->GetSize()))
    {
        PacketData data;
        data.busId = busId;
        data.busPacket = busPacket;
        data.txTime = Simulator::Now().GetSeconds();
        data.distance = distance;
        gRecords[seq] = data;
        ++gTxPackets;
        ++gTxByBus[busId - 1];

        NS_LOG_UNCOND("TX | BUS " << busId << " | seq " << seq
                      << " | t " << data.txTime << " s"
                      << " | d " << distance << " m");
    }
    else
    {
        NS_LOG_UNCOND("TX ERROR | BUS " << busId << " | seq " << seq);
    }

    Simulator::Schedule(pktInterval,
                        &GenerateTraffic,
                        socket,
                        busId,
                        rsuMobility,
                        pktSize,
                        remaining - 1,
                        busPacket + 1,
                        pktInterval);
}

static void
ExportCsv(const std::string& filename)
{
    std::ofstream csv(filename);
    NS_ABORT_MSG_IF(!csv.is_open(), "Nao foi possivel criar CSV: " << filename);
    csv.imbue(std::locale(std::locale::classic(), new CsvDecimalComma));
    csv << std::fixed << std::setprecision(6);
    csv << "BusId;PacoteBus;Sequencia;TX_s;Distancia_m;Recebido;RX_s;Delay_ms\n";

    for (const auto& [seq, data] : gRecords)
    {
        csv << data.busId << ';' << data.busPacket << ';' << seq << ';'
            << data.txTime << ';' << data.distance << ';'
            << (data.received ? 1 : 0) << ';';
        if (data.received)
        {
            csv << data.rxTime << ';' << data.delayMs;
        }
        else
        {
            csv << ';'; // RX_s e Delay_ms vazios quando nao recebido
        }
        csv << '\n';
    }
    csv.close();
    NS_ABORT_MSG_IF(!csv, "Erro de escrita no CSV");
    std::cout << "CSV exportado: " << filename << '\n';
}

int
main(int argc, char* argv[])
{
    const std::string phyMode{"OfdmRate6MbpsBW10MHz"};
    uint32_t nBuses{2};
    double initialDistance{200.0};
    double speed{10.0};
    double busSpacing{0.0}; // 0 m: mesma geometria radio para comparar contencao
    double pathLossExponent{2.0};
    uint32_t packetSize{100};
    uint32_t numPackets{10}; // por autocarro
    Time interPacketInterval{"100ms"};
    bool verbose{false};
    std::string outputCsv{"comov_multibus.csv"};

    CommandLine cmd(__FILE__);
    cmd.AddValue("nBuses", "Numero de autocarros", nBuses);
    cmd.AddValue("initialDistance", "Distancia inicial do primeiro BUS (m)", initialDistance);
    cmd.AddValue("speed", "Velocidade de todos os BUS (m/s)", speed);
    cmd.AddValue("busSpacing", "Espacamento longitudinal entre BUS (m)", busSpacing);
    cmd.AddValue("pathLossExponent", "Expoente de propagacao", pathLossExponent);
    cmd.AddValue("packetSize", "Tamanho do pacote UDP (bytes)", packetSize);
    cmd.AddValue("numPackets", "Pacotes enviados por cada BUS", numPackets);
    cmd.AddValue("interval", "Intervalo entre pacotes por BUS", interPacketInterval);
    cmd.AddValue("verbose", "Ativar logs Wi-Fi", verbose);
    cmd.AddValue("outputCsv", "Nome/caminho do CSV", outputCsv);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(nBuses < 1 || nBuses > 100, "nBuses deve estar entre 1 e 100");
    NS_ABORT_MSG_IF(initialDistance < 0.0 || speed < 0.0 || busSpacing < 0.0,
                    "Distancias e velocidades devem ser nao negativas");
    NS_ABORT_MSG_IF(pathLossExponent <= 0.0, "pathLossExponent deve ser positivo");
    NS_ABORT_MSG_IF(packetSize < 12, "packetSize deve ser >= 12 bytes");
    NS_ABORT_MSG_IF(interPacketInterval <= Seconds(0), "interval deve ser positivo");
    NS_ABORT_MSG_IF(static_cast<uint64_t>(nBuses) * numPackets >
                        std::numeric_limits<uint32_t>::max(),
                    "Numero de sequencias ultrapassa uint32_t");

    gTxByBus.assign(nBuses, 0);
    gRxByBus.assign(nBuses, 0);
    gDelayByBus.assign(nBuses, 0.0);

    std::cout << "\n========== INFO COMOV ==========\n"
              << "Padrao: IEEE 802.11p | modo: " << phyMode << '\n'
              << "RSU: 1 | BUS: " << nBuses << '\n'
              << "Distancia inicial: " << initialDistance << " m\n"
              << "Espacamento: " << busSpacing << " m\n"
              << "Velocidade: " << speed << " m/s\n"
              << "Expoente propagacao: " << pathLossExponent << '\n'
              << "Pacotes por BUS: " << numPackets << '\n'
              << "Tamanho: " << packetSize << " bytes\n"
              << "Intervalo: " << interPacketInterval.GetMilliSeconds() << " ms\n"
              << "Destino: broadcast (baseline anterior)\n"
              << "================================\n";

    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer nodes;
    nodes.Create(nBuses + 1); // 0 = RSU; 1..N = BUS

    WifiHelper wifi;
    if (verbose)
    {
        WifiHelper::EnableLogComponents();
    }
    wifi.SetStandard(WIFI_STANDARD_80211p);

    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("RxGain", DoubleValue(0));
    wifiPhy.Set("ChannelSettings", StringValue("{172, 10, BAND_5GHZ, 0}"));

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                   "Exponent",
                                   DoubleValue(pathLossExponent));
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(phyMode),
                                 "ControlMode",
                                 StringValue(phyMode));
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    Ptr<WifiNetDevice> rsuDevice = DynamicCast<WifiNetDevice>(devices.Get(0));

    NS_ABORT_MSG_IF(!rsuDevice, "RSU sem WifiNetDevice");

    bool connected = rsuDevice->GetPhy()->TraceConnectWithoutContext(
        "PhyRxDrop",
        MakeCallback(&OnPhyRxDrop));

    NS_ABORT_MSG_IF(!connected, "Falha ao ligar PhyRxDrop");

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // RSU
    for (uint32_t bus = 0; bus < nBuses; ++bus)
    {
        positionAlloc->Add(Vector(-initialDistance - bus * busSpacing, 0.0, 0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    Ptr<MobilityModel> rsuMobility = nodes.Get(0)->GetObject<MobilityModel>();
    Ptr<ConstantVelocityMobilityModel> rsuVelocity =
        nodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    rsuVelocity->SetVelocity(Vector(0.0, 0.0, 0.0));

    for (uint32_t bus = 1; bus <= nBuses; ++bus)
    {
        Ptr<ConstantVelocityMobilityModel> model =
            nodes.Get(bus)->GetObject<ConstantVelocityMobilityModel>();
        model->SetVelocity(Vector(speed, 0.0, 0.0));
    }

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(nodes.Get(0), tid);
    recvSink->Bind(InetSocketAddress(Ipv4Address::GetAny(), 80));
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    // Para esta fase conserva-se UDP broadcast do prototipo com 1 BUS.
    // A decisao broadcast/unicast sera fixada antes dos ensaios finais.
    for (uint32_t bus = 1; bus <= nBuses; ++bus)
    {
        Ptr<Socket> source = Socket::CreateSocket(nodes.Get(bus), tid);
        source->SetAllowBroadcast(true);
        source->Connect(InetSocketAddress(Ipv4Address("255.255.255.255"), 80));

        // Gerar desfasamento inicial aleatorio
        Ptr<UniformRandomVariable> startJitter =
            CreateObject<UniformRandomVariable>();

        startJitter->SetAttribute(
            "Min", DoubleValue(0.0));

        startJitter->SetAttribute(
            "Max", DoubleValue(interPacketInterval.GetSeconds()));

        // Stream independente por autocarro
        startJitter->SetStream(1000 + bus);

        const Time start =
            Seconds(1.0 + startJitter->GetValue());

        NS_LOG_UNCOND("BUS " << bus
                    << " | Inicio: " << start.GetSeconds() << " s");

        // Agendar transmissao
        Simulator::ScheduleWithContext(
            nodes.Get(bus)->GetId(),
            start,
            &GenerateTraffic,
            source,
            bus,
            rsuMobility,
            packetSize,
            numPackets,
            1,
            interPacketInterval);
    }

    // Janela comum para N=2/5/10; 0.5 s final para entrega de pacotes.
    const double simulationSeconds = 1.0 + numPackets * interPacketInterval.GetSeconds() + 3.0;
    Simulator::Stop(Seconds(simulationSeconds));
    Simulator::Run();

    const uint32_t lost = gTxPackets - gRxPackets;
    const double reception = gTxPackets ? 100.0 * gRxPackets / gTxPackets : 0.0;
    const double meanDelay = gRxPackets ? gTotalDelayMs / gRxPackets : 0.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n========== RESULTADOS ==========" << '\n';
    std::cout << "Tempo simulado:    " << Simulator::Now().GetSeconds() << " s\n";
    std::cout << "Autocarros:        " << nBuses << '\n';
    std::cout << "Pacotes enviados:  " << gTxPackets << '\n';
    std::cout << "Pacotes recebidos: " << gRxPackets << '\n';
    std::cout << "Pacotes perdidos:  " << lost << '\n';
    std::cout << "Taxa de rececao:   " << reception << " %\n";
    if (gRxPackets)
    {
        std::cout << "Atraso medio:      " << meanDelay << " ms\n";
    }
    else
    {
        std::cout << "Atraso medio:      N/A\n";
    }
    std::cout << "\nPor autocarro:\n";
    for (uint32_t bus = 1; bus <= nBuses; ++bus)
    {
        const uint32_t tx = gTxByBus[bus - 1];
        const uint32_t rx = gRxByBus[bus - 1];
        const double percent = tx ? 100.0 * rx / tx : 0.0;
        std::cout << "BUS " << bus << ": TX=" << tx << " RX=" << rx
                  << " Rececao=" << percent << "%";
        if (rx)
        {
            std::cout << " Delay=" << gDelayByBus[bus - 1] / rx << " ms";
        }
        std::cout << '\n';
    }
    std::cout << "PHY RX drops RSU: " << gPhyRxDrops << "\n";
    std::cout << "================================\n";

    ExportCsv(outputCsv);
    Simulator::Destroy();
    return 0;
}