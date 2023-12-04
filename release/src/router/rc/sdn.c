#include <rc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <shared.h>
#include "vpnc_fusion.h"

#if defined(RTCONFIG_DNSFILTER)
#include "dnsfilter.h"
#endif

#if defined(RTCONFIG_SMARTDNS)
const char sdservers[];
#endif
extern const char dmservers[];

const char sdn_dir[] = "/tmp/.sdn";
static int _handle_sdn_dnsmasq(const MTLAN_T *pmtl, const int action);

int handle_sdn_feature(const int sdn_idx, const unsigned long features, const int action)
{
	FILE *fp_filter = NULL, *fp_nat = NULL, *fp_mangle = NULL;
	char file_filter[128], file_nat[128], file_mangle[128];

#ifdef RTCONFIG_IPV6
	FILE *fpv6_filter = NULL, *fpv6_nat = NULL, *fpv6_mangle = NULL;
	char filev6_filter[128], filev6_nat[128], filev6_mangle[128];
#endif

	MTLAN_T *pmtl = NULL;
	size_t mtl_sz = 0;
	int i;
	char logaccept[32], logdrop[32];
	int restart_all_sdn;

	//_dprintf("[%s, %d]<%d, %x, %d>\n", __FUNCTION__, __LINE__, sdn_idx, features, action);

	// check dir exist
	if (!d_exists(sdn_dir))
	{
		mkdir(sdn_dir, 0777);
	}

	get_drop_accept(logdrop, sizeof(logdrop), logaccept, sizeof(logaccept));

	pmtl = (MTLAN_T *)INIT_MTLAN(sizeof(MTLAN_T));
	if (pmtl)
	{
		// feature active by independent sdn
		if (sdn_idx == ALL_SDN)
		{
			get_mtlan(pmtl, &mtl_sz);
			restart_all_sdn = 1;
		}
		else
		{
			get_mtlan_by_idx(SDNFT_TYPE_SDN, sdn_idx, pmtl, &mtl_sz);
			restart_all_sdn = 0;
		}

		for (i = 0; i < mtl_sz; ++i)
		{
			if (features & SDN_FEATURE_ALL_FIREWALL)
			{
				create_iptables_file(IPTABLE_TYPE_FILTER, pmtl[i].sdn_t.sdn_idx, &fp_filter, file_filter, sizeof(file_filter), 0);
				create_iptables_file(IPTABLE_TYPE_NAT, pmtl[i].sdn_t.sdn_idx, &fp_nat, file_nat, sizeof(file_nat), 0);
				create_iptables_file(IPTABLE_TYPE_MANGLE, pmtl[i].sdn_t.sdn_idx, &fp_mangle, file_mangle, sizeof(file_mangle), 0);
#ifdef RTCONFIG_IPV6
				if (ipv6_enabled())
				{
					create_iptables_file(IPTABLE_TYPE_FILTER, pmtl[i].sdn_t.sdn_idx, &fpv6_filter, filev6_filter, sizeof(filev6_filter), 1);
					create_iptables_file(IPTABLE_TYPE_NAT, pmtl[i].sdn_t.sdn_idx, &fpv6_nat, filev6_nat, sizeof(filev6_nat), 1);
					create_iptables_file(IPTABLE_TYPE_MANGLE, pmtl[i].sdn_t.sdn_idx, &fpv6_mangle, filev6_mangle, sizeof(filev6_mangle), 1);
				}
#endif
			}

			if (features & SDN_FEATURE_DNSMASQ)
			{
				if (pmtl[i].sdn_t.sdn_idx) // ignore  main LAN
				{
					_dprintf("[%s]DO: SDN DNSMASQ\n", __FUNCTION__);
					_handle_sdn_dnsmasq(&pmtl[i], action);
				}
			}
			if (features & SDN_FEATURE_SDN_IPTABLES)
			{
				_dprintf("[%s]DO: SDN_FEATURE_SDN_IPTABLES\n", __FUNCTION__);
				if (pmtl[i].sdn_t.sdn_idx) // ignore  main LAN
				{
					update_SDN_iptables(&pmtl[i], logdrop, logaccept);
				}
			}
			if (features & SDN_FEATURE_URL_FILTER)
			{
				_dprintf("[%s]DO: URL FILTER\n", __FUNCTION__);
#ifdef RTCONFIG_IPV6
				write_URLFilter_SDN(&pmtl[i], sdn_idx == ALL_SDN ? 1 : 0, logdrop, fp_filter, fpv6_filter);
#else
				write_URLFilter_SDN(&pmtl[i], sdn_idx == ALL_SDN ? 1 : 0, logdrop, fp_filter);
#endif
			}
			if (features & SDN_FEATURE_NWS_FILTER)
			{
				_dprintf("[%s]DO: NETWORK SERVICE  FILTER\n", __FUNCTION__);
#ifdef RTCONFIG_IPV6
				write_NwServiceFilter_SDN(&pmtl[i], sdn_idx == ALL_SDN ? 1 : 0, logdrop, logaccept, fp_filter, fpv6_filter);
#else
				write_NwServiceFilter_SDN(&pmtl[i], sdn_idx == ALL_SDN ? 1 : 0, logdrop, logaccept, fp_filter);
#endif
			}
			if (features & SDN_FEATURE_FIREWALL)
			{
				_dprintf("[%s, %d]TODO: FIREWALL\n", __FUNCTION__, __LINE__);
			}
#ifdef RTCONFIG_VPN_FUSION_SUPPORT_INTERFACE
			if (features & SDN_FEATURE_VPNC)
			{
				// set routing rule
				vpnc_set_iif_routing_rule(pmtl[i].enable ? pmtl[i].sdn_t.vpnc_idx : 0, pmtl[i].nw_t.ifname);
			}
#endif
			if (features & SDN_FEATURE_ALL_FIREWALL)
			{
				close_n_restore_iptables_file(IPTABLE_TYPE_FILTER, &fp_filter, file_filter, 0);
				close_n_restore_iptables_file(IPTABLE_TYPE_NAT, &fp_nat, file_nat, 0);
				close_n_restore_iptables_file(IPTABLE_TYPE_MANGLE, &fp_mangle, file_mangle, 0);
#ifdef RTCONFIG_IPV6
				if (ipv6_enabled())
				{
					close_n_restore_iptables_file(IPTABLE_TYPE_FILTER, &fpv6_filter, filev6_filter, 1);
					close_n_restore_iptables_file(IPTABLE_TYPE_NAT, &fpv6_nat, filev6_nat, 1);
					close_n_restore_iptables_file(IPTABLE_TYPE_MANGLE, &fpv6_mangle, filev6_mangle, 1);
				}
#endif
				// Handle jump rule here.
				if (features & SDN_FEATURE_URL_FILTER)
				{
					handle_URLFilter_jump_rule(&pmtl[i]);
				}
				if (features & SDN_FEATURE_NWS_FILTER)
				{
					handle_NwServiceFilter_jump_rule(&pmtl[i]);
				}
			}
		}

		// common feature, not for a specific SDN
		if (features & SDN_FEATURE_SDN_INTERNAL_ACCESS)
		{
			handle_SDN_internal_access(logdrop, logaccept);
		}
		// some vpn handled by script
		if (features & SDN_FEATURE_VPNC)
		{
			_dprintf("[%s]DO: VPNC\n", __FUNCTION__);
#ifdef RTCONFIG_VPN_FUSION
			vpnc_set_iptables_rule_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
#ifdef RTCONFIG_OPENVPN
			update_ovpn_client_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
#ifdef RTCONFIG_WIREGUARD
			update_wgc_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
		}
		if (features & SDN_FEATURE_VPNS)
		{
			_dprintf("[%s]DO: VPNS\n", __FUNCTION__);
#ifdef RTCONFIG_WIREGUARD
			update_wgs_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
#ifdef RTCONFIG_IPSEC
			update_ipsec_server_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
#ifdef RTCONFIG_OPENVPN
			update_ovpn_server_by_sdn(pmtl, mtl_sz, restart_all_sdn);
#endif
		}
#ifdef RTCONFIG_GRE
		if (features & SDN_FEATURE_GRE)
		{
			_dprintf("[%s]DO: GRE\n", __FUNCTION__);
			update_gre_by_sdn(pmtl, mtl_sz, restart_all_sdn);
		}
#endif

		FREE_MTLAN((void *)pmtl);
		return 0;
	}
	return -1;
}

static int _write_dhcp_reservation(FILE *fp, const int dhcpres_idx)
{
	char *nv, *nvp, *b;
	char *mac, *ip, *dns, *hostname;
	char name[32];

	if (!fp)
		return -1;

	// get matched dhcpresX_rl
	snprintf(name, sizeof(name), "dhcpres%d_rl", dhcpres_idx);
	nv = nvp = strdup(nvram_safe_get(name));

	// parser the rule list
	while (nv && (b = strsep(&nvp, "<")) != NULL)
	{
		if (vstrsep(b, ">", &mac, &ip, &dns, &hostname) < 2) // mac and ip are mandatory
			continue;

		if (!mac)
			continue;

		// write dns
		if (dns)
		{
			struct in_addr in4;
#ifdef RTCONFIG_IPV6
			struct in6_addr in6;

			if (*dns && inet_pton(AF_INET6, dns, &in6) > 0 &&
				!IN6_IS_ADDR_UNSPECIFIED(&in6) && !IN6_IS_ADDR_LOOPBACK(&in6))
			{
				fprintf(fp, "dhcp-option=tag:%s,option6:23,%s\n", mac, dns);
			}
			else
#endif
				if (*dns && inet_pton(AF_INET, dns, &in4) > 0 &&
					in4.s_addr != INADDR_ANY && in4.s_addr != INADDR_LOOPBACK && in4.s_addr != INADDR_BROADCAST)
			{
				fprintf(fp, "dhcp-option=tag:%s,6,%s\n", mac, dns);
			}
			else
				dns = NULL;
		}

		// write dhcp static lease
		if (*ip == '\0')
		{
			if (dns)
				fprintf(fp, "dhcp-host=%s,set:%s\n", mac, mac);
			continue;
		}

		// if ((sdn_ipaddr & sdn_netmask_addr) == sdn_net_addr)
		{
			if (hostname && hostname[0] != '\0')
				fprintf(fp, "dhcp-host=%s,set:%s,%s,%s\n", mac, mac, hostname, ip);
			else
				fprintf(fp, "dhcp-host=%s,set:%s,%s\n", mac, mac, ip);
			continue;
		}
	}
	if (nv)
		free(nv);

	return 0;
}

static int _gen_sdn_dnsmasq_conf(const MTLAN_T *pmtl, char *config_file, const size_t path_len)
{
	FILE *fp;
	char resolv_path[64];
	int resolv_flag = 0, n;
#if defined(RTCONFIG_DNSFILTER)
	int count;
	dnsf_srv_entry_t dnsfsrv;
#endif

	if (!pmtl || !config_file)
		return -1;

	snprintf(resolv_path, sizeof(resolv_path), vpnc_resolv_path, pmtl->sdn_t.vpnc_idx);
	if (!access(resolv_path, F_OK))
		resolv_flag = 1;

	// create /etc/dnsmasq-<sdn_ifname>.conf
	snprintf(config_file, path_len, "/etc/dnsmasq-%d.conf", pmtl->sdn_t.sdn_idx);
	unlink(config_file);
	fp = fopen(config_file, "w");

	if (fp)
	{
		fprintf(fp, "pid-file=" sdn_dnsmasq_pid_path "\n", pmtl->sdn_t.sdn_idx);
		fprintf(fp, "user=nobody\n");
		fprintf(fp, "bind-interfaces\n");
		fprintf(fp, "listen-address=%s\n", pmtl->nw_t.addr);
		fprintf(fp, "no-resolv\n");
#if defined(RTCONFIG_SMARTDNS)
		fprintf(fp, "servers-file=%s\n", resolv_flag ? resolv_path : nvram_match("smartdns_enable", "1") ? sdservers : dmservers);

#else
		fprintf(fp, "servers-file=%s\n", resolv_flag ? resolv_path : dmservers);
#endif
		fprintf(fp, "no-poll\n");
		fprintf(fp, "no-negcache\n");
		fprintf(fp, "cache-size=1500\n");
		fprintf(fp, "min-port=4096\n");

		// TODO: need to handle dhcp_enable
		if (pmtl->nw_t.dhcp_enable)
		{
			fprintf(fp, "dhcp-authoritative\n");
			fprintf(fp, "dhcp-range=%s,%s,%s,%s,%ds\n", pmtl->nw_t.ifname, pmtl->nw_t.dhcp_min, pmtl->nw_t.dhcp_max, pmtl->nw_t.netmask, pmtl->nw_t.dhcp_lease);
			fprintf(fp, "dhcp-option=%s,3,%s\n", pmtl->nw_t.ifname, pmtl->nw_t.addr);
		}

		/* limit number of outstanding requests */
		{
			int max_queries = nvram_get_int("max_dns_queries");
#if defined(RTCONFIG_SOC_IPQ8064)
			if (max_queries == 0)
				max_queries = 1500;
#endif
			if (max_queries)
				fprintf(fp, "dns-forward-max=%d\n", max(150, min(max_queries, 10000)));
		}

		if (pmtl->nw_t.domain_name[0] != '\0')
		{
			fprintf(fp, "domain=%s\n"
						"expand-hosts\n",
					pmtl->nw_t.domain_name); // expand hostnames in hosts file
		}
		if (nvram_get_int("dns_fwd_local") != 1)
		{
			fprintf(fp, "bogus-priv\n"		// don't forward private reverse lookups upstream
						"domain-needed\n"); // don't forward plain name queries upstream
			if (pmtl->nw_t.domain_name[0] != '\0')
				fprintf(fp, "local=/%s/\n", pmtl->nw_t.domain_name); // don't forward local domain queries upstream
		}

		// write dhcp reservation
		if (pmtl->nw_t.dhcp_res)
		{
			_write_dhcp_reservation(fp, pmtl->nw_t.dhcp_res_idx);
		}

#ifdef RTCONFIG_DNSSEC
#ifdef RTCONFIG_DNSPRIVACY
		if (nvram_get_int("dnspriv_enable") && nvram_get_int("dnssec_enable") == 2)
		{
			fprintf(fp, "proxy-dnssec\n");
		}
		else
#endif
			if (nvram_get_int("dnssec_enable"))
		{
			fprintf(fp, "trust-anchor=.,20326,8,2,E06D44B80B8F1D39A95C0B0D7C65D08458E880409BBC683457104237C7F8EC8D\n"
						"dnssec\n");

			/* If NTP isn't set yet, wait until rc's ntp signals us to start validating time */
			if (!nvram_get_int("ntp_ready"))
			{
				fprintf(fp, "dnssec-no-timecheck\n");
			}

			if (nvram_match("dnssec_check_unsigned_x", "0"))
			{
				fprintf(fp, "dnssec-check-unsigned=no\n");
			}
		}
#endif
		if (nvram_match("dns_norebind", "1"))
		{
			fprintf(fp, "stop-dns-rebind\n");
		}

		/* Instruct clients like Firefox to not auto-enable DoH */
		n = nvram_get_int("dns_priv_override");
		if ((n == 1) ||
			(n == 0 && (
#ifdef RTCONFIG_DNSPRIVACY
						   nvram_get_int("dnspriv_enable") ||
#endif
						   (nvram_get_int("dnsfilter_enable_x") && nvram_get_int("dnsfilter_mode"))) // DNSFilter enabled in Global mode
			 ))
		{
			fprintf(fp, "address=/use-application-dns.net/\n");
		}

		/* Protect against VU#598349 */
		fprintf(fp, "dhcp-name-match=set:wpad-ignore,wpad\n");
		fprintf(fp, "dhcp-ignore-names=tag:wpad-ignore\n");

		/* dhcp-script */
		fprintf(fp, "dhcp-script=/sbin/dhcpc_lease\n");

#if defined(RTCONFIG_AMAS)
		fprintf(fp, "script-arp\n");
#endif

#if defined(RTCONFIG_DNSFILTER) && defined(RTCONFIG_IPV6)
		if (pmtl->sdn_t.dnsf_idx != DNSF_SRV_UNFILTERED)
		{
			count = get_dns_filter(AF_INET6, pmtl->sdn_t.dnsf_idx, &dnsfsrv);
			if (count > 0)
			{
				fprintf(fp, "dhcp-option=lan,option6:23,[%s]", dnsfsrv.server1);
				if (count > 1)
				{
					fprintf(fp, ",[%s]", dnsfsrv.server2);
				}
				fprintf(fp, "\n");
			}
		}
#endif

		// TODO: Set VPN server

		fclose(fp);
		return 0;
	}
	return -1;
}

static int _handle_sdn_dnsmasq(const MTLAN_T *pmtl, const int action)
{
	char config_path[128] = "";

	if (!pmtl)
		return -1;

	if (action & RC_SERVICE_STOP)
	{
		snprintf(config_path, sizeof(config_path), sdn_dnsmasq_pid_path, pmtl->sdn_t.sdn_idx);
		kill_pidfile_tk(config_path);
	}

	if (action & RC_SERVICE_START)
	{
		if (pmtl->sdn_t.sdn_idx && pmtl->enable)
		{
			// generate dnsmasq config file
			if (!_gen_sdn_dnsmasq_conf(pmtl, config_path, sizeof(config_path)))
			{
				eval("dnsmasq", "-C", config_path, "--log-async");
			}
		}
	}
	return 0;
}
