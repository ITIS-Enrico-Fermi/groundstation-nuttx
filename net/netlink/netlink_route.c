/****************************************************************************
 * net/netlink/netlink_route.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <net/route.h>
#include <netpacket/netlink.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/neighbor.h>
#include <nuttx/net/netlink.h>

#include "netdev/netdev.h"
#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "netlink/netlink.h"
#include "utils/utils.h"

#ifdef CONFIG_NETLINK_ROUTE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_NET_ARP) && !defined(CONFIG_NET_IPv6)
#  undef CONFIG_NETLINK_DISABLE_GETNEIGH
#  define CONFIG_NETLINK_DISABLE_GETNEIGH 1
#endif

#if !defined(CONFIG_NET_ROUTE) || (!defined(CONFIG_NET_IPv4) && \
    !defined(CONFIG_NET_IPv6))
#  undef CONFIG_NETLINK_DISABLE_GETROUTE
#  define CONFIG_NETLINK_DISABLE_GETROUTE 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RTM_GETLINK:  Enumerate network devices */

struct getlink_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct ifinfomsg iface;
  struct rtattr    attr;
  uint8_t          data[IFNAMSIZ];  /* IFLA_IFNAME is the only attribute supported */
};

struct getlink_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getlink_recvfrom_response_s payload;
};

/* RTM_GETNEIGH:  Get neighbor table entry */

struct getneigh_recvfrom_response_s
{
  struct nlmsghdr hdr;
  struct ndmsg    msg;
  struct rtattr   attr;
  uint8_t         data[1];
};

#define SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(n) \
  (sizeof(struct getneigh_recvfrom_response_s) + (n) - 1)

struct getneigh_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getneigh_recvfrom_response_s payload;
};

#define SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(n) \
  (sizeof(struct getneigh_recvfrom_rsplist_s) + (n) - 1)

/* RTM_GETROUTE.  Get routing tables */

struct getroute_recvfrom_ipv4addr_s
{
  struct rtattr attr;
  in_addr_t     addr;
};

struct getroute_recvfrom_ipv4response_s
{
  struct nlmsghdr hdr;
  struct rtmsg    rte;
  struct getroute_recvfrom_ipv4addr_s dst;
  struct getroute_recvfrom_ipv4addr_s genmask;
  struct getroute_recvfrom_ipv4addr_s gateway;
};

struct getroute_recvfrom_ipv4resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_ipv4response_s payload;
};

struct getroute_recvfrom_ipv6addr_s
{
  struct rtattr  attr;
  net_ipv6addr_t addr;
};

struct getroute_recvfrom_ipv6response_s
{
  struct nlmsghdr hdr;
  struct rtmsg    rte;
  struct getroute_recvfrom_ipv6addr_s dst;
  struct getroute_recvfrom_ipv6addr_s genmask;
  struct getroute_recvfrom_ipv6addr_s gateway;
};

struct getroute_recvfrom_ipv6resplist_s
{
  sq_entry_t flink;
  struct getroute_recvfrom_ipv6response_s payload;
};

/* RTM_GETADDR:  Get the specified network device address info */

struct getaddr_recvfrom_response_s
{
  struct nlmsghdr  hdr;
  struct ifaddrmsg ifaddr;
  struct rtattr    attr;
#ifndef CONFIG_NET_IPv6
  struct in_addr   local_addr;  /* IFA_LOCAL is the only attribute supported */
#else
  struct in6_addr  local_addr;  /* IFA_LOCAL is the only attribute supported */
#endif
};

struct getaddr_recvfrom_rsplist_s
{
  sq_entry_t flink;
  struct getaddr_recvfrom_response_s payload;
};

/* netdev_foreach() callback */

struct nlroute_sendto_request_s
{
  struct nlmsghdr hdr;
  struct rtgenmsg gen;
};

struct nlroute_info_s
{
  NETLINK_HANDLE handle;
  FAR const struct nlroute_sendto_request_s *req;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETLINK_VALIDATE_POLICY
#  ifdef CONFIG_NET_IPv4
static const struct nla_policy g_ifa_ipv4_policy[] =
{
  {0},                                              /* IFA_UNSPEC */
  {NLA_U32, 0, NULL},                               /* IFA_ADDRESS */
  {NLA_U32, 0, NULL},                               /* IFA_LOCAL */
  {NLA_STRING, IFNAMSIZ - 1, NULL},                 /* IFA_LABEL */
  {NLA_U32, 0, NULL},                               /* IFA_BROADCAST */
  {0},                                              /* IFA_ANYCAST */
  {NLA_UNSPEC, sizeof(struct ifa_cacheinfo), NULL}, /* IFA_CACHEINFO */
  {0},                                              /* IFA_MULTICAST */
  {NLA_U32, 0, NULL},                               /* IFA_FLAGS */
  {NLA_U32, 0, NULL},                               /* IFA_RT_PRIORITY */
};

static_assert(sizeof(g_ifa_ipv4_policy) / sizeof(g_ifa_ipv4_policy[0]) ==
              IFA_MAX + 1, "The policy definition has changed,"
              " please check it");
#  endif
#  ifdef CONFIG_NET_IPv6
static const struct nla_policy g_ifa_ipv6_policy[] =
{
  {0},                                              /* IFA_UNSPEC */
  {0, sizeof(struct in6_addr), NULL},               /* IFA_ADDRESS */
  {0, sizeof(struct in6_addr), NULL},               /* IFA_LOCAL */
  {0},                                              /* IFA_LABEL */
  {0},                                              /* IFA_BROADCAST */
  {0},                                              /* IFA_ANYCAST */
  {NLA_UNSPEC, sizeof(struct ifa_cacheinfo), NULL}, /* IFA_CACHEINFO */
  {0},                                              /* IFA_MULTICAST */
  {0, sizeof(uint32_t), NULL},                      /* IFA_FLAGS */
  {0, sizeof(uint32_t), NULL},                      /* IFA_RT_PRIORITY */
};

static_assert(sizeof(g_ifa_ipv6_policy) / sizeof(g_ifa_ipv6_policy[0]) ==
              IFA_MAX + 1, "The policy definition has changed,"
              " please check it");
#  endif
#else
#  define g_ifa_ipv4_policy NULL
#  define g_ifa_ipv6_policy NULL
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_get_device
 *
 * Description:
 *   Generate one device response.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static FAR struct netlink_response_s *
netlink_get_device(FAR struct net_driver_s *dev,
                   FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getlink_recvfrom_rsplist_s *alloc;
  FAR struct getlink_recvfrom_response_s *resp;
  int up = IFF_IS_UP(dev->d_flags);

  /* Allocate the response buffer */

  alloc = (FAR struct getlink_recvfrom_rsplist_s *)
    kmm_zalloc(sizeof(struct getlink_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                   = &alloc->payload;

  resp->hdr.nlmsg_len    = sizeof(struct getlink_recvfrom_response_s);
  resp->hdr.nlmsg_type   = up ? RTM_NEWLINK : RTM_DELLINK;
  resp->hdr.nlmsg_flags  = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq    = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid    = req ? req->hdr.nlmsg_pid : 0;

  resp->iface.ifi_family = req ? req->gen.rtgen_family : AF_PACKET;
  resp->iface.ifi_type   = dev->d_lltype;
#ifdef CONFIG_NETDEV_IFINDEX
  resp->iface.ifi_index  = dev->d_ifindex;
#endif
  resp->iface.ifi_flags  = dev->d_flags;
  resp->iface.ifi_change = 0xffffffff;

  resp->attr.rta_len     = RTA_LENGTH(strnlen(dev->d_ifname, IFNAMSIZ));
  resp->attr.rta_type    = IFLA_IFNAME;

  strlcpy((FAR char *)resp->data, dev->d_ifname, IFNAMSIZ);

  /* Finally, return the response */

  return (FAR struct netlink_response_s *)alloc;
}
#endif

#ifdef CONFIG_NET_IPv4
static uint32_t make_mask(int prefixlen)
{
  if (prefixlen)
    {
      return HTONL(UINT32_MAX << (32 - prefixlen));
    }

  return 0;
}

static uint8_t mask_len(uint32_t nmask)
{
  uint8_t prefixlen = 0;
  uint32_t hmask = NTOHL(nmask);

  for (; hmask; ++prefixlen, hmask <<= 1);

  return prefixlen;
}
#endif

/****************************************************************************
 * Name: netlink_get_ifaddr
 *
 * Description:
 *   Generate one interface address response.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static FAR struct netlink_response_s *
netlink_get_ifaddr(FAR struct net_driver_s *dev, int domain, int type,
                   FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getaddr_recvfrom_rsplist_s *alloc;
  FAR struct getaddr_recvfrom_response_s *resp;

  /* Allocate the response buffer */

  alloc = (FAR struct getaddr_recvfrom_rsplist_s *)
    kmm_zalloc(sizeof(struct getaddr_recvfrom_rsplist_s));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return NULL;
    }

  /* Initialize the response buffer */

  resp                    = &alloc->payload;

  resp->hdr.nlmsg_len     = sizeof(struct getaddr_recvfrom_response_s);
  resp->hdr.nlmsg_type    = type;
  resp->hdr.nlmsg_flags   = req ? req->hdr.nlmsg_flags : 0;
  resp->hdr.nlmsg_seq     = req ? req->hdr.nlmsg_seq : 0;
  resp->hdr.nlmsg_pid     = req ? req->hdr.nlmsg_pid : 0;

  resp->ifaddr.ifa_family = domain;
#ifdef CONFIG_NETDEV_IFINDEX
  resp->ifaddr.ifa_index  = dev->d_ifindex;
#endif
  resp->ifaddr.ifa_flags  = IFA_F_PERMANENT;
  resp->ifaddr.ifa_scope  = RT_SCOPE_UNIVERSE;

  resp->attr.rta_type     = IFA_LOCAL;
#ifdef CONFIG_NET_IPv4
  if (domain == AF_INET)
    {
      resp->attr.rta_len = RTA_LENGTH(sizeof(struct in_addr));
      memcpy(&resp->local_addr, &dev->d_ipaddr, sizeof(struct in_addr));
      resp->ifaddr.ifa_prefixlen = mask_len(dev->d_netmask);
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (domain == AF_INET6)
    {
      resp->attr.rta_len = RTA_LENGTH(sizeof(struct in6_addr));
      memcpy(&resp->local_addr, dev->d_ipv6addr, sizeof(struct in6_addr));
      resp->ifaddr.ifa_prefixlen = net_ipv6_mask2pref(dev->d_ipv6netmask);
    }
#endif

  /* Finally, return the response */

  return (FAR struct netlink_response_s *)alloc;
}
#endif

/****************************************************************************
 * Name: netlink_get_terminator
 *
 * Description:
 *   Generate one NLMSG_DONE response.
 *
 ****************************************************************************/

static FAR struct netlink_response_s *
netlink_get_terminator(FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct netlink_response_s *resp;
  FAR struct nlmsghdr *hdr;

  /* Allocate the list terminator */

  resp = kmm_zalloc(sizeof(struct netlink_response_s));
  if (resp == NULL)
    {
      nerr("ERROR: Failed to allocate response terminator.\n");
      return NULL;
    }

  /* Initialize and send the list terminator */

  hdr              = &resp->msg;
  hdr->nlmsg_len   = sizeof(struct nlmsghdr);
  hdr->nlmsg_type  = NLMSG_DONE;
  hdr->nlmsg_flags = req ? req->hdr.nlmsg_flags : 0;
  hdr->nlmsg_seq   = req ? req->hdr.nlmsg_seq : 0;
  hdr->nlmsg_pid   = req ? req->hdr.nlmsg_pid : 0;

  /* Finally, return the response */

  return resp;
}

/****************************************************************************
 * Name: netlink_add_terminator
 *
 * Description:
 *   Add one NLMSG_DONE response to handle.
 *
 ****************************************************************************/

static int netlink_add_terminator(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct netlink_response_s * resp;

  resp = netlink_get_terminator(req);
  if (resp == NULL)
    {
      return -ENOMEM;
    }

  netlink_add_response(handle, resp);
  return OK;
}

/****************************************************************************
 * Name: netlink_get_devlist
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
static int netlink_device_callback(FAR struct net_driver_s *dev,
                                   FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s * resp;

  resp = netlink_get_device(dev, info->req);
  if (resp == NULL)
    {
      return -ENOMEM;
    }

  netlink_add_response(info->handle, resp);
  return OK;
}

static int netlink_get_devlist(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* Visit each device */

  info.handle = handle;
  info.req    = req;

  net_lock();
  ret = netdev_foreach(netlink_device_callback, &info);
  net_unlock();
  if (ret < 0)
    {
      return ret;
    }

  return netlink_add_terminator(handle, req);
}
#endif

/****************************************************************************
 * Name: netlink_get_arptable()
 *
 * Description:
 *   Return the entire ARP table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ARP) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static int netlink_get_arptable(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getneigh_recvfrom_rsplist_s *entry;
  unsigned int ncopied;
  size_t allocsize;
  size_t tabsize;
  size_t rspsize;

  /* Preallocate memory to hold the maximum sized ARP table
   * REVISIT:  This is probably excessively large and could cause false
   * memory out conditions.  A better approach would be to actually count
   * the number of valid entries in the ARP table.
   */

  tabsize   = CONFIG_NET_ARPTAB_SIZE * sizeof(struct arpreq);
  rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
  allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

  entry = (FAR struct getneigh_recvfrom_rsplist_s *)kmm_zalloc(allocsize);
  if (entry == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return -ENOMEM;
    }

  /* Populate the entry */

  memcpy(&entry->payload.hdr, &req->hdr, sizeof(struct nlmsghdr));
  entry->payload.hdr.nlmsg_len  = rspsize;
  entry->payload.msg.ndm_family = req->gen.rtgen_family;
  entry->payload.attr.rta_len   = RTA_LENGTH(tabsize);

  /* Lock the network so that the ARP table will be stable, then copy
   * the ARP table into the allocated memory.
   */

  net_lock();
  ncopied = arp_snapshot((FAR struct arpreq *)entry->payload.data,
                         CONFIG_NET_ARPTAB_SIZE);
  net_unlock();

  /* Now we have the real number of valid entries in the ARP table and
   * we can trim the allocation.
   */

  if (ncopied < CONFIG_NET_ARPTAB_SIZE)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize = ncopied * sizeof(struct arpreq);
      rspsize = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = (FAR struct getneigh_recvfrom_rsplist_s *)
        kmm_realloc(entry, allocsize);

      if (newentry != NULL)
        {
           entry = newentry;
        }

      entry->payload.hdr.nlmsg_len = rspsize;
      entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  /* Finally, add the data to the list of pending responses */

  netlink_add_response(handle, (FAR struct netlink_response_s *)entry);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_nbtable()
 *
 * Description:
 *   Return the entire IPv6 neighbor table.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETNEIGH)
static int netlink_get_nbtable(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  FAR struct getneigh_recvfrom_rsplist_s *entry;
  unsigned int ncopied;
  size_t allocsize;
  size_t tabsize;
  size_t rspsize;

  /* Preallocate memory to hold the maximum sized Neighbor table
   * REVISIT:  This is probably excessively large and could cause false
   * memory out conditions.  A better approach would be to actually count
   * the number of valid entries in the Neighbor table.
   */

  tabsize   = CONFIG_NET_IPv6_NCONF_ENTRIES *
              sizeof(struct neighbor_entry_s);
  rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
  allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

  entry = (FAR struct getneigh_recvfrom_rsplist_s *)kmm_zalloc(allocsize);
  if (entry == NULL)
    {
      nerr("ERROR: Failed to allocate response buffer.\n");
      return -ENOMEM;
    }

  /* Populate the entry */

  memcpy(&entry->payload.hdr, &req->hdr, sizeof(struct nlmsghdr));
  entry->payload.hdr.nlmsg_len  = rspsize;
  entry->payload.msg.ndm_family = req->gen.rtgen_family;
  entry->payload.attr.rta_len   = RTA_LENGTH(tabsize);

  /* Lock the network so that the Neighbor table will be stable, then
   * copy the Neighbor table into the allocated memory.
   */

  net_lock();
  ncopied = neighbor_snapshot(
    (FAR struct neighbor_entry_s *)entry->payload.data,
    CONFIG_NET_IPv6_NCONF_ENTRIES);
  net_unlock();

  /* Now we have the real number of valid entries in the Neighbor table
   * and we can trim the allocation.
   */

  if (ncopied < CONFIG_NET_IPv6_NCONF_ENTRIES)
    {
      FAR struct getneigh_recvfrom_rsplist_s *newentry;

      tabsize   = ncopied * sizeof(struct neighbor_entry_s);
      rspsize   = SIZEOF_NLROUTE_RECVFROM_RESPONSE_S(tabsize);
      allocsize = SIZEOF_NLROUTE_RECVFROM_RSPLIST_S(tabsize);

      newentry = (FAR struct getneigh_recvfrom_rsplist_s *)
        kmm_realloc(entry, allocsize);

      if (newentry != NULL)
        {
           entry = newentry;
        }

      entry->payload.hdr.nlmsg_len = rspsize;
      entry->payload.attr.rta_len  = RTA_LENGTH(tabsize);
    }

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(handle, (FAR struct netlink_response_s *)entry);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_ipv4_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_ipv4_route(FAR struct net_route_ipv4_s *route,
                              FAR void *arg)
{
  FAR struct getroute_recvfrom_ipv4resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv4response_s *resp;
  FAR struct nlroute_info_s *info;

  DEBUGASSERT(route != NULL && arg != NULL);
  info = (FAR struct nlroute_info_s *)arg;

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv4resplist_s *)
    kmm_zalloc(sizeof(struct getroute_recvfrom_ipv4resplist_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  /* Format the response */

  resp                  = &alloc->payload;
  resp->hdr.nlmsg_len   = sizeof(struct getroute_recvfrom_ipv4response_s);
  resp->hdr.nlmsg_type  = RTM_NEWROUTE;
  resp->hdr.nlmsg_flags = info->req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq   = info->req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid   = info->req->hdr.nlmsg_pid;

  resp->rte.rtm_family   = info->req->gen.rtgen_family;
  resp->rte.rtm_table    = RT_TABLE_MAIN;
  resp->rte.rtm_protocol = RTPROT_STATIC;
  resp->rte.rtm_scope    = RT_SCOPE_SITE;

  resp->dst.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->dst.attr.rta_type = RTA_DST;
  resp->dst.addr          = route->target;

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  resp->genmask.addr          = route->netmask;

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(in_addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  resp->gateway.addr          = route->router;

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(info->handle, (FAR struct netlink_response_s *)alloc);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_ipv4route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_get_ipv4route(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* Visit each routing table entry */

  info.handle = handle;
  info.req    = req;

  ret = net_foreachroute_ipv4(netlink_ipv4_route, &info);
  if (ret < 0)
    {
      return ret;
    }

  /* Terminate the routing table */

  return netlink_add_terminator(handle, req);
}
#endif

/****************************************************************************
 * Name: netlink_ipv6_route
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_ipv6_route(FAR struct net_route_ipv6_s *route,
                              FAR void *arg)
{
  FAR struct getroute_recvfrom_ipv6resplist_s *alloc;
  FAR struct getroute_recvfrom_ipv6response_s *resp;
  FAR struct nlroute_info_s *info;

  DEBUGASSERT(route != NULL && arg != NULL);
  info = (FAR struct nlroute_info_s *)arg;

  /* Allocate the response */

  alloc = (FAR struct getroute_recvfrom_ipv6resplist_s *)
    kmm_zalloc(sizeof(struct getroute_recvfrom_ipv6resplist_s));
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  /* Format the response */

  resp                  = &alloc->payload;
  resp->hdr.nlmsg_len   = sizeof(struct getroute_recvfrom_ipv6response_s);
  resp->hdr.nlmsg_type  = RTM_NEWROUTE;
  resp->hdr.nlmsg_flags = info->req->hdr.nlmsg_flags;
  resp->hdr.nlmsg_seq   = info->req->hdr.nlmsg_seq;
  resp->hdr.nlmsg_pid   = info->req->hdr.nlmsg_pid;

  resp->rte.rtm_family   = info->req->gen.rtgen_family;
  resp->rte.rtm_table    = RT_TABLE_MAIN;
  resp->rte.rtm_protocol = RTPROT_STATIC;
  resp->rte.rtm_scope    = RT_SCOPE_SITE;

  resp->dst.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->dst.attr.rta_type = RTA_DST;
  net_ipv6addr_copy(resp->dst.addr, route->target);

  resp->genmask.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->genmask.attr.rta_type = RTA_GENMASK;
  net_ipv6addr_copy(resp->genmask.addr, route->netmask);

  resp->gateway.attr.rta_len  = RTA_LENGTH(sizeof(net_ipv6addr_t));
  resp->gateway.attr.rta_type = RTA_GATEWAY;
  net_ipv6addr_copy(resp->gateway.addr, route->router);

  /* Finally, add the response to the list of pending responses */

  netlink_add_response(info->handle, (FAR struct netlink_response_s *)alloc);
  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_ip6vroute
 *
 * Description:
 *   Dump a list of all network devices of the specified type.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_GETROUTE)
static int netlink_get_ip6vroute(NETLINK_HANDLE handle,
                              FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* Visit each routing table entry */

  info.handle = handle;
  info.req    = req;

  ret = net_foreachroute_ipv6(netlink_ipv6_route, &info);
  if (ret < 0)
    {
      return ret;
    }

  /* Terminate the routing table */

  return netlink_add_terminator(handle, req);
}
#endif

/****************************************************************************
 * Name: netlink_new_ipv4addr
 *
 * Description:
 *   Set the ipv4 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_NEWADDR)
static int netlink_new_ipv4addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv4_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  if (ifm->ifa_prefixlen > 32 || !tb[IFA_LOCAL])
    {
      return -EINVAL;
    }

  net_lock();
  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      net_unlock();
      return -ENODEV;
    }

  dev->d_ipaddr  = nla_get_in_addr(tb[IFA_LOCAL]);
  dev->d_netmask = make_mask(ifm->ifa_prefixlen);

  netlink_device_notify_ipaddr(dev, RTM_NEWADDR, AF_INET);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_new_ipv6addr
 *
 * Description:
 *   Set the ipv6 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_NEWADDR)
static int netlink_new_ipv6addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv6_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  if (ifm->ifa_prefixlen > 128 || !tb[IFA_LOCAL])
    {
      return -EINVAL;
    }

  net_lock();
  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      net_unlock();
      return -ENODEV;
    }

  memcpy(dev->d_ipv6addr, nla_data(tb[IFA_LOCAL]), 16);
  net_ipv6_pref2mask(ifm->ifa_prefixlen, dev->d_ipv6netmask);

  netlink_device_notify_ipaddr(dev, RTM_NEWADDR, AF_INET6);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_del_ipv4addr
 *
 * Description:
 *   Clear the ipv4 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NETLINK_DISABLE_DELADDR)
static int netlink_del_ipv4addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv4_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  net_lock();
  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      net_unlock();
      return -ENODEV;
    }

  if (tb[IFA_LOCAL] && dev->d_ipaddr != nla_get_in_addr(tb[IFA_LOCAL]))
    {
      net_unlock();
      return -EADDRNOTAVAIL;
    }

  dev->d_ipaddr  = 0;

  netlink_device_notify_ipaddr(dev, RTM_DELADDR, AF_INET);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_del_ipv6addr
 *
 * Description:
 *   Clear the ipv6 address.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NETLINK_DISABLE_DELADDR)
static int netlink_del_ipv6addr(NETLINK_HANDLE handle,
                                FAR const struct nlmsghdr *nlmsg)
{
  FAR struct net_driver_s *dev;
  FAR struct ifaddrmsg *ifm = NLMSG_DATA(nlmsg);
  FAR struct nlattr *tb[IFA_MAX + 1];
  struct netlink_ext_ack extack;
  int ret;

  ret = nlmsg_parse(nlmsg, sizeof(*ifm), tb, IFA_MAX, g_ifa_ipv6_policy,
                    &extack);
  if (ret < 0)
    {
      return ret;
    }

  net_lock();
  dev = netdev_findbyindex(ifm->ifa_index);

  if (dev == NULL)
    {
      net_unlock();
      return -ENODEV;
    }

  if (tb[IFA_LOCAL] && dev->d_ipaddr != nla_get_in_addr(tb[IFA_LOCAL]))
    {
      net_unlock();
      return -EADDRNOTAVAIL;
    }

  memset(&dev->d_ipv6addr, 0, sizeof(net_ipv6addr_t));

  netlink_device_notify_ipaddr(dev, RTM_DELADDR, AF_INET6);
  net_unlock();

  return OK;
}
#endif

/****************************************************************************
 * Name: netlink_get_addr
 *
 * Description:
 *   Clear the ipv4/ipv6 address.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETADDR
static int netlink_addr_callback(FAR struct net_driver_s *dev,
                                   FAR void *arg)
{
  FAR struct nlroute_info_s *info = arg;
  FAR struct netlink_response_s *resp;

  resp = netlink_get_ifaddr(dev, info->req->gen.rtgen_family, RTM_NEWADDR,
                            info->req);
  if (resp == NULL)
    {
      return -ENOMEM;
    }

  netlink_add_response(info->handle, resp);
  return OK;
}

static int netlink_get_addr(NETLINK_HANDLE handle,
                            FAR const struct nlroute_sendto_request_s *req)
{
  struct nlroute_info_s info;
  int ret;

  /* Visit each device */

  info.handle = handle;
  info.req    = req;

  net_lock();
  ret = netdev_foreach(netlink_addr_callback, &info);
  net_unlock();
  if (ret < 0)
    {
      return ret;
    }

  return netlink_add_terminator(handle, req);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_route_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

ssize_t netlink_route_sendto(NETLINK_HANDLE handle,
                             FAR const struct nlmsghdr *nlmsg,
                             size_t len, int flags,
                             FAR const struct sockaddr_nl *to,
                             socklen_t tolen)
{
  FAR const struct nlroute_sendto_request_s *req =
    (FAR const struct nlroute_sendto_request_s *)nlmsg;
  int ret;

  DEBUGASSERT(handle != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_nl));

  /* Handle according to the message type */

  switch (nlmsg->nlmsg_type)
    {
#ifndef CONFIG_NETLINK_DISABLE_GETLINK
      /* Dump a list of all devices */

      case RTM_GETLINK:

        /* Generate the response */

        ret = netlink_get_devlist(handle, req);
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETNEIGH
      /* Retrieve ARP/Neighbor Tables */

      case RTM_GETNEIGH:
#ifdef CONFIG_NET_ARP
        /* Retrieve the ARP table in its entirety. */

        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_get_arptable(handle, req);
          }
        else
#endif

#ifdef CONFIG_NET_IPv6
        /* Retrieve the IPv6 neighbor table in its entirety. */

        if (req->gen.rtgen_family == AF_INET6)
          {
             ret = netlink_get_nbtable(handle, req);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif /* !CONFIG_NETLINK_DISABLE_GETNEIGH */

#ifndef CONFIG_NETLINK_DISABLE_GETROUTE
      /* Retrieve the IPv4 or IPv6 routing table */

      case RTM_GETROUTE:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_get_ipv4route(handle, req);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_get_ip6vroute(handle, req);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_NEWADDR
      /* Set the IPv4 or IPv6 address */

      case RTM_NEWADDR:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_new_ipv4addr(handle, nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_new_ipv6addr(handle, nlmsg);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_DELADDR
      /* Clear the IPv4 or IPv6 address */

      case RTM_DELADDR:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_del_ipv4addr(handle, nlmsg);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_del_ipv6addr(handle, nlmsg);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

#ifndef CONFIG_NETLINK_DISABLE_GETADDR
      /* Get the IPv4 or IPv6 address */

      case RTM_GETADDR:
#ifdef CONFIG_NET_IPv4
        if (req->gen.rtgen_family == AF_INET)
          {
            ret = netlink_get_addr(handle, req);
          }
        else
#endif
#ifdef CONFIG_NET_IPv6
        if (req->gen.rtgen_family == AF_INET6)
          {
            ret = netlink_get_addr(handle, req);
          }
        else
#endif
          {
            ret = -EAFNOSUPPORT;
          }
        break;
#endif

      default:
        ret = -ENOSYS;
        break;
    }

  /* On success, return the size of the request that was processed */

  if (ret >= 0)
    {
      ret = len;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_device_notify()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#ifndef CONFIG_NETLINK_DISABLE_GETLINK
void netlink_device_notify(FAR struct net_driver_s *dev)
{
  FAR struct netlink_response_s *resp;

  DEBUGASSERT(dev != NULL);

  resp = netlink_get_device(dev, NULL);
  if (resp != NULL)
    {
      netlink_add_broadcast(RTNLGRP_LINK, resp);

      resp = netlink_get_terminator(NULL);
      if (resp != NULL)
        {
          netlink_add_broadcast(RTNLGRP_LINK, resp);
        }
    }
}
#endif

/****************************************************************************
 * Name: netlink_device_notify_ipaddr()
 *
 * Description:
 *   Perform the route broadcast for the NETLINK_ROUTE protocol.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETLINK_DISABLE_NEWADDR) || \
    !defined(CONFIG_NETLINK_DISABLE_DELADDR) || \
    !defined(CONFIG_NETLINK_DISABLE_GETADDR)
void netlink_device_notify_ipaddr(FAR struct net_driver_s *dev,
                                  int type, int domain)
{
  FAR struct netlink_response_s *resp;
  int group;

  DEBUGASSERT(dev != NULL);

  resp = netlink_get_ifaddr(dev, domain, type, NULL);
  if (resp != NULL)
    {
#ifdef CONFIG_NET_IPv4
      if (domain == AF_INET)
        {
          group = RTNLGRP_IPV4_IFADDR;
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (domain == AF_INET6)
        {
          group = RTNLGRP_IPV6_IFADDR;
        }
      else
#endif
        {
          nwarn("netlink_device_notify_ipaddr unknown type %d domain %d\n",
                type, domain);
          return;
        }

      netlink_add_broadcast(group, resp);

      resp = netlink_get_terminator(NULL);
      if (resp != NULL)
        {
          netlink_add_broadcast(group, resp);
        }
    }
}
#endif

#endif /* CONFIG_NETLINK_ROUTE */
