#include "net.h"

extern UART_HandleTypeDef huart1;

uint8_t net_buff[ENC28J60_MAXFRAME];
uint8_t ipaddr[4] = IP_ADDR;
extern uint8_t macaddr[6];
char str1[60] = {0};


void net_init(void)
{
	enc28j60_init();
}

uint16_t checksum(uint8_t *ptr, uint16_t len)
{
	uint32_t sum = 0;
	while(len > 0)
	{
		sum += (uint16_t) (((uint32_t)*ptr<<8) | *(ptr + 1));
		ptr += 2;
		len -= 2;
	}
	if(len) 
		sum += ((uint32_t)*ptr)<<8;
	while(sum>>16) 
		sum = (uint16_t)sum + (sum>>16);
	return ~be16toword((uint16_t)sum);
}

uint8_t ip_read(enc28j60_frame_ptr *frame, uint16_t len)
{
  uint8_t res = 0;
  ip_pkt_ptr *ip_pkt = (void*)(frame->data);
  if((ip_pkt->verlen == 0x45) && (!memcmp(ip_pkt->ipaddr_dst, ipaddr, 4)))
  {
		/*data lenght*/
		len = be16toword(ip_pkt->len) - sizeof(ip_pkt_ptr);
		if(ip_pkt->prt == IP_ICMP)
		{
			icmp_read(frame, len);
		}
		else if(ip_pkt->prt == IP_TCP)
		{
			
		}
		else if(ip_pkt->prt == IP_UDP)
		{
		}
  }
  return res;	
}

uint8_t icmp_read(enc28j60_frame_ptr *frame, uint16_t len)
{
  uint8_t res = 0;
	ip_pkt_ptr *ip_pkt = (void*)frame->data;
	icmp_pkt_ptr *icmp_pkt = (void*)ip_pkt->data;
	if ((len >= sizeof(icmp_pkt_ptr)) && (icmp_pkt->msg_tp == ICMP_REQ))
	{
		sprintf(str1, "icmp request\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 0x1000);
		icmp_pkt->msg_tp = ICMP_REPLY;
		icmp_pkt->cs = 0;
		icmp_pkt->cs = checksum((void*)icmp_pkt, len);
		ip_send(frame, len + sizeof(ip_pkt_ptr));		
	}
	
  return res;
}

uint8_t ip_send(enc28j60_frame_ptr *frame, uint16_t len)
{
	uint8_t res=0;
	ip_pkt_ptr *ip_pkt = (void*)frame->data;
	ip_pkt->len = be16toword(len);
	ip_pkt->fl_frg_of = 0;
	ip_pkt->ttl = 128;
	ip_pkt->cs = 0;
	memcpy(ip_pkt->ipaddr_dst, ip_pkt->ipaddr_src, 4);
	memcpy(ip_pkt->ipaddr_src, ipaddr, 4);
	ip_pkt->cs = checksum((void*)ip_pkt, sizeof(ip_pkt_ptr));
	eth_send(frame, len);
	
	return res;
}

uint8_t arp_read(enc28j60_frame_ptr *frame, uint16_t len)
{
  uint8_t res = 0;
  arp_msg_ptr *msg = (void *)(frame->data);
	if (len >= sizeof(arp_msg_ptr))
	{
		if ((msg->net_tp == ARP_ETH) && (msg->proto_tp == ARP_IP))
		{
			if ((msg->op == ARP_REQUEST)&&(!memcmp(msg->ipaddr_dst, ipaddr, 4)))
			{
				sprintf(str1, "request\r\nmac_src %02X:%02X:%02X:%02X:%02X:%02X\r\n", msg->macaddr_src[0], msg->macaddr_src[1], 
				msg->macaddr_src[2], msg->macaddr_src[3], msg->macaddr_src[4], msg->macaddr_src[5]);
				HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 0x1000);
				sprintf(str1, "ip_src %d.%d.%d.%d\r\n", msg->ipaddr_src[0], msg->ipaddr_src[1] ,msg->ipaddr_src[2], msg->ipaddr_src[3]);
				HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 0x1000);
				sprintf(str1,"nmac_dst %02X:%02X:%02X:%02X:%02X:%02X\r\n", msg->macaddr_dst[0], msg->macaddr_dst[1], 
				msg->macaddr_dst[2], msg->macaddr_dst[3], msg->macaddr_dst[4], msg->macaddr_dst[5]);
				HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 0x1000);
				sprintf(str1, "ip_dst %d.%d.%d.%d\r\n", msg->ipaddr_dst[0],msg->ipaddr_dst[1],msg->ipaddr_dst[2],msg->ipaddr_dst[3]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
				res = 1;				
			}
		}
	}
	return res;	
}

void eth_send(enc28j60_frame_ptr *frame, uint16_t len)
{
	memcpy(frame->addr_dest, frame->addr_src, 6);
	memcpy(frame->addr_src, macaddr, 6);
	enc28j60_packet_tx((void*)frame, len + sizeof(enc28j60_frame_ptr));
}

void arp_send(enc28j60_frame_ptr *frame)
{
  arp_msg_ptr *msg = (void*)frame->data;
  msg->op = ARP_REPLY;
	memcpy(msg->macaddr_dst, msg->macaddr_src, 6);
	memcpy(msg->macaddr_src, macaddr, 6);
	memcpy(msg->macaddr_src, macaddr, 6);
	memcpy(msg->ipaddr_dst, msg->ipaddr_src, 4);
	memcpy(msg->ipaddr_src, ipaddr, 4);
	eth_send(frame, sizeof(arp_msg_ptr));
}

void eth_read(enc28j60_frame_ptr *frame, uint16_t len)
{
  if (len >= sizeof(enc28j60_frame_ptr))
  {
		if(frame->type == ETH_ARP)
		{
			sprintf(str1,"%02X:%02X:%02X:%02X:%02X:%02X-%02X:%02X:%02X:%02X:%02X:%02X; %d; arp\r\n",
			frame->addr_src[0],frame->addr_src[1],frame->addr_src[2],frame->addr_src[3],frame->addr_src[4],frame->addr_src[5],
			frame->addr_dest[0],frame->addr_dest[1],frame->addr_dest[2],frame->addr_dest[3],frame->addr_dest[4],frame->addr_dest[5], len);
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
			if(arp_read(frame, len - sizeof(enc28j60_frame_ptr)))
			{
				arp_send(frame);
			}
		}
		else if(frame->type == ETH_IP)
		{
			sprintf(str1,"%02X:%02X:%02X:%02X:%02X:%02X-%02X:%02X:%02X:%02X:%02X:%02X; %d; ip\r\n",
			frame->addr_src[0],frame->addr_src[1],frame->addr_src[2],frame->addr_src[3],frame->addr_src[4],frame->addr_src[5],
			frame->addr_dest[0],frame->addr_dest[1],frame->addr_dest[2],frame->addr_dest[3],frame->addr_dest[4],frame->addr_dest[5], len);
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
			ip_read(frame, len - sizeof(ip_pkt_ptr));
		}
  }	
}

void net_pool(void)
{
	uint16_t len;
	enc28j60_frame_ptr *frame = (void *)net_buff;
	while((len = enc28j60_packet_rx(net_buff, sizeof(net_buff))) > 0)
    eth_read(frame, len);
}

