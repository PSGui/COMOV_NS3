/*
 * COMOV - Comunicacao V2I com IEEE 802.11p (ns-3.48)
 *
 * Adaptado de examples/wireless/wifi-simple-adhoc.cc:
 * Copyright (c) 2009 The Boeing Company
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Cenario:
 *   No 0: RSU fixa em (0, 0, 0)
 *   No 1: autocarro que parte de (-initialDistance, 0, 0)
 *         e se desloca com velocidade constante +speed no eixo X.
 *
 * O autocarro envia pacotes UDP broadcast. O SeqTsHeader de 12 bytes
 * transporta o numero sequencial e o instante de envio.
 * Os resultados sao guardados por pacote num ficheiro CSV para o Excel.
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

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Comov");

// Contadores ao nivel da aplicacao UDP.
uint32_t txPackets = 0;
uint32_t rxPackets = 0;
double totalDelayMs = 0.0;

struct PacketData
{
    double txTime = 0.0;    // segundos
    double distance = 0.0;  // metros, no instante TX
    bool received = false;
    double rxTime = 0.0;    // segundos
    double delayMs = 0.0;   // milissegundos
};

// Chave: numero sequencial (neste momento existe apenas um autocarro).
std::map<uint32_t, PacketData> packetRecords;

// CSV separado por ponto e virgula e com virgula decimal: Excel em PT.
class CsvDecimalComma : public std::numpunct<char>
{
  protected:
    char do_decimal_point() const override
    {
        return ',';
    }
};

void
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;

    while ((packet = socket->Recv()))
    {
        SeqTsHeader header;
        if (packet->GetSize() < header.GetSerializedSize())
        {
            NS_LOG_UNCOND("RX ERROR | Pacote demasiado pequeno para SeqTsHeader");
            continue;
        }

        packet->RemoveHeader(header);

        uint32_t seq = header.GetSeq();
        Time txTime = header.GetTs();
        Time rxTime = Simulator::Now();
        double delayMs = (rxTime - txTime).GetSeconds() * 1000.0;

        auto it = packetRecords.find(seq);
        if (it == packetRecords.end())
        {
            NS_LOG_UNCOND("RX ERROR | Pacote " << seq << " sem registo TX");
            continue;
        }
        if (it->second.received)
        {
            NS_LOG_UNCOND("RX DUPLICATE | Pacote " << seq);
            continue;
        }

        it->second.received = true;
        it->second.rxTime = rxTime.GetSeconds();
        it->second.delayMs = delayMs;

        rxPackets++;
        totalDelayMs += delayMs;

        NS_LOG_UNCOND("RX | Packet " << seq
                       << " | TX: " << txTime.GetSeconds() << " s"
                       << " | RX: " << rxTime.GetSeconds() << " s"
                       << " | Delay: " << delayMs << " ms");
    }
}

static void
GenerateTraffic(Ptr<Socket> socket,
                uint32_t pktSize,
                uint32_t pktCount,
                Time pktInterval)
{
    if (pktCount == 0)
    {
        socket->Close();
        return;
    }

    SeqTsHeader header;  // Timestamp = Simulator::Now() neste instante
    uint32_t seq = txPackets + 1;
    header.SetSeq(seq);

    uint32_t headerSize = header.GetSerializedSize();
    NS_ABORT_MSG_IF(pktSize < headerSize, "packetSize deve ser >= 12 bytes");

    Ptr<Packet> packet = Create<Packet>(pktSize - headerSize);
    packet->AddHeader(header);

    // Medir a distancia real ao longo da trajetoria do autocarro.
    Ptr<MobilityModel> busMobility = socket->GetNode()->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!busMobility, "O autocarro nao tem MobilityModel");

    Vector position = busMobility->GetPosition();
    // Neste cenario, a RSU esta fixa em (0, 0, 0).
    double currentDistance = CalculateDistance(position, Vector(0.0, 0.0, 0.0));

    int bytesSent = socket->Send(packet);

    if (bytesSent == static_cast<int>(packet->GetSize()))
    {
        txPackets++;

        PacketData record;
        record.txTime = Simulator::Now().GetSeconds();
        record.distance = currentDistance;
        packetRecords[seq] = record;

        NS_LOG_UNCOND("TX | Packet " << seq
                       << " | Time: " << record.txTime << " s"
                       << " | Position X: " << position.x << " m"
                       << " | Distance: " << currentDistance << " m"
                       << " | Size: " << packet->GetSize() << " bytes");
    }
    else
    {
        NS_LOG_UNCOND("TX ERROR | Falha ao enviar o pacote " << seq);
    }

    // Mantem o agendamento original, incluindo o ultimo evento de fecho.
    Simulator::Schedule(pktInterval,
                        &GenerateTraffic,
                        socket,
                        pktSize,
                        pktCount - 1,
                        pktInterval);
}

static void
ExportCsv(const std::string& filename)
{
    std::ofstream csv(filename);
    NS_ABORT_MSG_IF(!csv.is_open(), "Nao foi possivel criar o ficheiro CSV: " << filename);

    // Forca virgula decimal sem depender das locales instaladas no Ubuntu.
    csv.imbue(std::locale(std::locale::classic(), new CsvDecimalComma));
    csv << std::fixed << std::setprecision(6);
    csv << "Pacote;TX_s;Distancia_m;Recebido;RX_s;Delay_ms\n";

    for (const auto& [seq, data] : packetRecords)
    {
        csv << seq << ';'
            << data.txTime << ';'
            << data.distance << ';'
            << (data.received ? 1 : 0) << ';';

        if (data.received)
        {
            csv << data.rxTime << ';' << data.delayMs;
        }
        else
        {
            csv << ';'; // RX_s e Delay_ms ficam vazios
        }
        csv << '\n';
    }

    csv.close();
    NS_ABORT_MSG_IF(!csv, "Falha ao escrever o CSV: " << filename);
    std::cout << "CSV exportado: " << filename << '\n';
}

int
main(int argc, char* argv[])
{
    // Parametros da simulacao
    const std::string phyMode{"OfdmRate6MbpsBW10MHz"};
    double initialDistance{200.0};
    double speed{10.0};
    double pathLossExponent{2.0};
    uint32_t packetSize{100};
    uint32_t numPackets{10};
    Time interPacketInterval{"100ms"};
    bool verbose{false};
    std::string outputCsv{"comov_pacotes.csv"};

    CommandLine cmd(__FILE__);
    cmd.AddValue("initialDistance", "Distancia inicial (m)", initialDistance);
    cmd.AddValue("speed", "Velocidade do autocarro (m/s)", speed);
    cmd.AddValue("pathLossExponent", "Expoente de propagacao", pathLossExponent);
    cmd.AddValue("packetSize", "Tamanho do pacote UDP (bytes)", packetSize);
    cmd.AddValue("numPackets", "Numero de pacotes", numPackets);
    cmd.AddValue("interval", "Intervalo entre pacotes", interPacketInterval);
    cmd.AddValue("verbose", "Ativar logs Wi-Fi", verbose);
    cmd.AddValue("outputCsv", "Nome/caminho do CSV de resultados", outputCsv);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(initialDistance < 0.0, "initialDistance nao pode ser negativa");
    NS_ABORT_MSG_IF(speed < 0.0, "speed nao pode ser negativa neste cenario");
    NS_ABORT_MSG_IF(pathLossExponent <= 0.0, "pathLossExponent deve ser positivo");
    NS_ABORT_MSG_IF(packetSize < 12, "packetSize deve ser >= 12 bytes");
    NS_ABORT_MSG_IF(interPacketInterval <= Seconds(0), "interval deve ser positivo");

    std::cout << "\n========== INFO COMOV ==========\n";
    std::cout << "Standard: IEEE 802.11p\n";
    std::cout << "PHY mode: " << phyMode << '\n';
    std::cout << "Distancia inicial: " << initialDistance << " m\n";
    std::cout << "Velocidade: " << speed << " m/s\n";
    std::cout << "Expoente propagacao: " << pathLossExponent << '\n';
    std::cout << "Tamanho pacote: " << packetSize << " bytes\n";
    std::cout << "Numero de pacotes: " << numPackets << '\n';
    std::cout << "Intervalo: " << interPacketInterval.GetMilliSeconds() << " ms\n";
    std::cout << "CSV: " << outputCsv << '\n';
    std::cout << "================================\n\n";

    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer c;
    c.Create(2); // No 0 = RSU; No 1 = BUS

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
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, c);

    // Mobilidade: RSU fixa na origem; BUS aproxima-se da origem.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(-initialDistance, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(c);

    Ptr<ConstantVelocityMobilityModel> rsu =
        c.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    Ptr<ConstantVelocityMobilityModel> bus =
        c.Get(1)->GetObject<ConstantVelocityMobilityModel>();
    rsu->SetVelocity(Vector(0.0, 0.0, 0.0));
    bus->SetVelocity(Vector(speed, 0.0, 0.0));

    InternetStackHelper internet;
    internet.Install(c);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    Ptr<Socket> recvSink = Socket::CreateSocket(c.Get(0), tid);
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> source = Socket::CreateSocket(c.Get(1), tid);
    InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), 80);
    source->SetAllowBroadcast(true);
    source->Connect(remote);

    // PCAP mantido desativado: provocou uma assercao no teste com 10 MHz.
    // wifiPhy.EnablePcap("comov", devices);

    Simulator::ScheduleWithContext(source->GetNode()->GetId(),
                                   Seconds(1),
                                   &GenerateTraffic,
                                   source,
                                   packetSize,
                                   numPackets,
                                   interPacketInterval);

    Simulator::Run();

    Vector busPosition = bus->GetPosition();
    Vector rsuPosition = rsu->GetPosition();
    double finalDistance = CalculateDistance(busPosition, rsuPosition);

    double receptionRate = txPackets > 0 ? 100.0 * rxPackets / txPackets : 0.0;
    double avgDelayMs = rxPackets > 0 ? totalDelayMs / rxPackets : 0.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n========== MOBILIDADE ==========" << '\n';
    std::cout << "Posicao final BUS: " << busPosition.x << " m\n";
    std::cout << "Posicao final RSU: " << rsuPosition.x << " m\n";
    std::cout << "Distancia final:   " << finalDistance << " m\n";
    std::cout << "================================\n";

    std::cout << "\n========== RESULTADOS ==========" << '\n';
    std::cout << "Pacotes enviados:  " << txPackets << '\n';
    std::cout << "Pacotes recebidos: " << rxPackets << '\n';
    std::cout << "Pacotes perdidos:  " << (txPackets - rxPackets) << '\n';
    std::cout << "Taxa de rececao:   " << receptionRate << " %\n";
    if (rxPackets > 0)
    {
        std::cout << "Atraso medio:      " << avgDelayMs << " ms\n";
    }
    else
    {
        std::cout << "Atraso medio:      N/A (sem rececoes)\n";
    }
    std::cout << "================================\n";

    ExportCsv(outputCsv);
    Simulator::Destroy();
    return 0;
}