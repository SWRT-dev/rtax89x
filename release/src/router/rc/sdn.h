#ifndef __SDN_H__
#define __SDN_H__

#include "firewall_sdn.h"

#define SDN_FEATURE_URL_FILTER 0x00000001
#define SDN_FEATURE_NWS_FILTER 0x00000002
#define SDN_FEATURE_FIREWALL 0x00000004
#define SDN_FEATURE_SDN_IPTABLES 0x00000008

#define SDN_FEATURE_DNSMASQ 0x00000010
#define SDN_FEATURE_VPNC 0x00000020
#define SDN_FEATURE_VPNS 0x00000040
#define SDN_FEATURE_GRE 0x00000080

#define SDN_FEATURE_SDN_INTERNAL_ACCESS 0x00000100

#define SDN_FEATURE_ALL_FIREWALL (SDN_FEATURE_URL_FILTER | SDN_FEATURE_NWS_FILTER | SDN_FEATURE_FIREWALL | SDN_FEATURE_VPNC | SDN_FEATURE_VPNS | SDN_FEATURE_GRE | SDN_FEATURE_SDN_IPTABLES | SDN_FEATURE_SDN_INTERNAL_ACCESS)
#define SDN_FEATURE_ALL 0xFFFFFFFF

#define sdn_dnsmasq_pid_path "/var/run/dnsmasq-%d.pid"

extern const char sdn_dir[];

#define ALL_SDN 255
#define LAN_IN_SDN_IDX 0

int handle_sdn_feature(const int sdn_idx, const unsigned long features, const int action);

#endif
