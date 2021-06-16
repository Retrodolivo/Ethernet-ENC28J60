#ifndef _NET_H_
#define _NET_H_

#include "main.h"
#define IP_ADDR {192,168,1,197}

#define be16toword(a) 	((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define ETH_ARP be16toword(0x0806)
#define ETH_IP be16toword(0x0800)
#define ARP_ETH be16toword(0x0001)
#define ARP_IP be16toword(0x0800)
#define ARP_REQUEST be16toword(1)
#define ARP_REPLY be16toword(2)
#define IP_ICMP 1
#define IP_TCP 6
#define IP_UDP 17
#define ICMP_REQ 8
#define ICMP_REPLY 0

typedef struct enc28j60_frame
{
	uint8_t addr_dest[6];
	uint8_t addr_src[6];
	uint16_t type;
	uint8_t data[];
} enc28j60_frame_ptr;

typedef struct arp_msg
{
  uint16_t net_tp;
  uint16_t proto_tp;
  uint8_t macaddr_len;
  uint8_t ipaddr_len;
  uint16_t op;
  uint8_t macaddr_src[6];
  uint8_t ipaddr_src[4];
  uint8_t macaddr_dst[6];
  uint8_t ipaddr_dst[4];
} arp_msg_ptr;

typedef struct ip_pkt
{
	uint8_t verlen;
	uint8_t ts;
	uint16_t len;
	uint16_t id;
	uint16_t fl_frg_of;
	uint8_t ttl;
	uint8_t prt;
	uint16_t cs;
	uint8_t ipaddr_src[4];
	uint8_t ipaddr_dst[4];
	uint8_t data[];
} ip_pkt_ptr;

typedef struct icmp_pkt
{
	uint8_t msg_tp;
	uint8_t msg_cd;
	uint16_t cs;
	uint16_t id;
	uint16_t num;
	uint8_t data[];	
} icmp_pkt_ptr;

void net_init(void);
void net_pool(void);
void eth_read(enc28j60_frame_ptr *frame, uint16_t len);
void eth_send(enc28j60_frame_ptr *frame, uint16_t len);
void arp_send(enc28j60_frame_ptr *frame);
uint8_t ip_read(enc28j60_frame_ptr *frame, uint16_t len);
uint8_t icmp_read(enc28j60_frame_ptr *frame, uint16_t len);
uint8_t ip_send(enc28j60_frame_ptr *frame, uint16_t len);

#endif /*_NET_H_*/

