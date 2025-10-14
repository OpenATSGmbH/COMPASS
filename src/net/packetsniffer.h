/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include <map>
#include <set>
#include <tuple>
#include <string>
#include <limits>

#include <boost/optional.hpp>

struct pcap_pkthdr;
struct pcap;

/**
*/
class PacketSniffer
{
public:
    // ip protocol | ip version | source_ip | source_port | destination_ip | destination_port
    typedef std::tuple<uint8_t, unsigned int, std::string, unsigned int, std::string, unsigned int> Signature;

    /**
     * Accumulated packet data.
     */
    struct Data
    {
        bool valid() const 
        {
            return (packets > 0 && size > 0 && data.size() > 0);
        }

        std::string info() const;

        size_t              packets = 0; // number of packets added
        size_t              size    = 0; // size of added packets
        std::vector<u_char> data;        // accumulated packet byte data (possibly only a portion, depending on filter)
    };

    typedef std::map<Signature, Data> DataPerSignature;

    /**
     * Basic filters for packet sniffing.
    */
    struct BasicFilter
    {
        BasicFilter() {}
        ~BasicFilter() {}

        BasicFilter& startPacketIndex(size_t idx) { start_packet_index = idx; return *this; }
        BasicFilter& endPacketIndex(size_t idx) { end_packet_index = idx; return *this; }
        BasicFilter& maxPackets(size_t n) { max_packets = n; return *this; }
        BasicFilter& maxBytes(size_t n) { max_bytes = n; return *this; }
        BasicFilter& signaturesToRead(const std::set<Signature>& sigs) { signatures = sigs; return *this; }

        bool hasSignatureFilter() const { return !signatures.empty(); }

        size_t start_packet_index  = 0;
        size_t end_packet_index    = std::numeric_limits<size_t>::max();
        size_t max_packets         = std::numeric_limits<size_t>::max();
        size_t max_bytes           = std::numeric_limits<size_t>::max();

        std::set<Signature> signatures;
    };

    typedef BasicFilter PacketFilter;

    /**
     * Filters for packet sniffing.
     */
    struct DataFilter : public BasicFilter
    {
        DataFilter() {}
        ~DataFilter() {}

        DataFilter& startPacketIndex(size_t idx) { start_packet_index = idx; return *this; }
        DataFilter& endPacketIndex(size_t idx) { end_packet_index = idx; return *this; }
        DataFilter& maxPackets(size_t n) { max_packets = n; return *this; }
        DataFilter& maxBytes(size_t n) { max_bytes = n; return *this; }
        DataFilter& signaturesToRead(const std::set<Signature>& sigs) { signatures = sigs; return *this; }

        DataFilter& maxPacketsPerSignature(size_t n) { max_packets_per_sig = n; return *this; }
        DataFilter& maxBytesPerSignature(size_t n) { max_bytes_per_sig = n; return *this; }

        size_t max_packets_per_sig = std::numeric_limits<size_t>::max();
        size_t max_bytes_per_sig   = std::numeric_limits<size_t>::max();
    };

    /**
     * Data chunk read in readFileNext().
     */
    struct Chunk
    {
        Data chunk_data;   // read data
        bool eof = false;  // end of file reached
    };

    enum class Device
    {
        NoDevice = 0,
        File
    };

    enum class ReadStyle
    {
        Accumulate = 0,   // accumulate all packet data in a single data struct
        PerSignature      // accumulate packet data in per-signature data structs
    };

    enum class SignatureMode
    {
        UDPBroadcast = 0, // signature for udp broadcast (dst_ip, dst_port)
        UDPMulticast,     // signature for udp multicast (dst_ip, dst_port)
        UDPUnicast,       // signature for udp unicast (src_ip, dst_ip, dst_port)
        TCP,              // signature for tcp (src_ip, src_port, dst_ip, dst_port)
        Auto              // determine suitable signature from packet information
    };

    enum class Error
    {
        NoError = 0,
        OpenFailed,
        ParseFailed
    };

    /**
    */
    struct ReadConfig
    {
        ReadStyle     read_style     = PacketSniffer::ReadStyle::Accumulate;
        SignatureMode signature_mode = PacketSniffer::SignatureMode::Auto;

        BasicFilter  chunk_filter;    // filter used to determine if the current data chunk has ended
        PacketFilter packet_filter;   // filter used to skip whole packets
        DataFilter   data_filter;     // filter used to skip packet data
    };
    
    PacketSniffer();
    virtual ~PacketSniffer();

    void clear();
    bool openPCAP(const std::string& fn);

    bool readFile(ReadStyle read_style = ReadStyle::Accumulate,
                  const PacketFilter& packet_filter = {},
                  const DataFilter& data_filter = {});
    boost::optional<Chunk> readFileNext(size_t max_packets = 0, 
                                        size_t max_bytes = 0,
                                        const std::set<Signature>& signatures_to_read = std::set<Signature>());

    void digestPCAPPacket(const struct pcap_pkthdr* pkthdr,
                          const u_char* packet,
                          int link_layer_type,
                          const ReadConfig& read_config,
                          bool& chunk_ended);

    void print() const;

    size_t numPacketsRead() const { return num_read_; }
    size_t numPacketsDropped() const { return num_dropped_; }
    size_t numBytesRead() const { return bytes_read_; }
    size_t numPacketsReadTotal() const { return num_read_total_; }
    size_t numPacketsDroppedTotal() const { return num_dropped_total_; }
    size_t numBytesReadTotal() const { return bytes_read_total_; }

    const DataPerSignature& dataPerSignature() const { return data_per_signature_; }
    const Data& data() const { return data_; }

    std::pair<size_t, std::set<int>> unknownLinkTypes() const;
    std::pair<size_t, std::set<int>> unknownEthernetTypes() const;
    std::pair<size_t, std::set<int>> unknownIPProtocols() const;
    void printUnknowns() const;

    bool hasUnknownPacketHeaders() const;

    static bool ipIsMulticast(const std::string& ip, unsigned int ip_version, bool* ok = nullptr);
    static bool ipIsBroadcast(const std::string& ip, unsigned int ip_version, bool* ok = nullptr);
    static Signature signatureForMode(uint8_t ip_protocol,
                                      unsigned int ip_version,
                                      const std::string& src_ip,
                                      unsigned int src_port,
                                      const std::string& dst_ip,
                                      unsigned int dst_port,
                                      SignatureMode mode);
    static SignatureMode determinePacketSignatureMode(const std::string& src_ip,
                                                      unsigned int src_port,
                                                      uint8_t ip_protocol, 
                                                      unsigned int ip_version);
    static bool signatureSourceIPSet(const Signature& signature);
    static bool signatureSourcePortSet(const Signature& signature);
    static std::string ipProtocolInfoToString(uint8_t ip_protocol,
                                              unsigned int ip_version);
    static std::pair<uint8_t, unsigned int> ipProtocolInfoFromString(const std::string& s);
    static std::string signatureToString(const Signature& signature);
    static Signature signatureFromString(const std::string& signature);

    static const std::string  SignatureStringSeparator;
    static const std::string  SignatureStringSeparatorAddresses;
    static const std::string  SignatureIPPortSeparator;
    static const std::string  SignaturePlaceholder;
    static const unsigned int UnspecifiedPort;

private:
    void digestPCAPEtherPacket(int ether_type, 
                               const struct pcap_pkthdr* pkthdr, 
                               const u_char* packet, 
                               unsigned long data_offs,
                               const ReadConfig& read_config);
    Signature signatureForPacket(const std::string& src_ip,
                                 unsigned int src_port,
                                 const std::string& dst_ip,
                                 unsigned int dst_port,
                                 uint8_t ip_protocol,
                                 unsigned int ip_version,
                                 const ReadConfig& read_config) const;
    void addPacket(const std::string& src_ip,
                   unsigned int src_port,
                   const std::string& dst_ip,
                   unsigned int dst_port,
                   uint8_t ip_protocol,
                   unsigned int ip_version,
                   u_char* data,
                   size_t data_len,
                   const ReadConfig& read_config);
    void closePCAPFile();

    bool chunkEnded(const BasicFilter& filter) const;

    bool checkGeneralFilters(const BasicFilter& filter) const;
    bool checkPerSignatureFilters(const Data& data,
                                  const DataFilter& filter) const;
    bool checkSignatureFilter(const Signature& signature,
                              const BasicFilter& filter) const;

    size_t packet_idx_        = 0;
    size_t num_read_          = 0;
    size_t num_dropped_       = 0;
    size_t bytes_read_        = 0;
    size_t num_read_total_    = 0;
    size_t num_dropped_total_ = 0;
    size_t bytes_read_total_  = 0;

    bool reached_eof_ = false;

    DataPerSignature data_per_signature_;
    Data             data_;

    std::vector<int> unknown_link_types_;
    std::vector<int> unknown_eth_types_;
    std::vector<int> unknown_ip_prot_;

    pcap*   pcap_file_       = nullptr;
    Device  device_          = Device::NoDevice;
    int     link_layer_type_ = -1;
};
