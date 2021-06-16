#include "enc28j60.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

static uint8_t enc28j60_read_op(uint8_t op_code, uint8_t addr);
static void enc28j60_read_buff(uint16_t len, uint8_t *data);
static void enc28j60_set_bank(uint8_t addr);
static void enc28j60_write_reg_byte(uint8_t addr, uint8_t data);
static uint8_t enc28j60_read_reg_byte(uint8_t addr);
static void enc28j60_write_reg(uint8_t addr,uint16_t data);
static void enc28j60_write_phy(uint8_t addr,uint16_t data);
static void enc28j60_write_buf(uint16_t len, uint8_t* data);
static uint8_t spi_write_read_byte(uint8_t tx_byte);
static void error(void);

static uint8_t enc28j60Bank;
static int next_packet_ptr;
uint8_t macaddr[6] = MAC_ADDR;


void enc28j60_init(void)
{
	LED_OFF;
	enc28j60_write_op(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	HAL_Delay(2);
	while(!enc28j60_read_op(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY);
	
	/*buffer init*/
	enc28j60_write_reg(ERXST, RXSTART_INIT);
  enc28j60_write_reg(ERXRDPT, RXSTART_INIT);
  enc28j60_write_reg(ERXND, RXSTOP_INIT);
  enc28j60_write_reg(ETXST, TXSTART_INIT);
  enc28j60_write_reg(ETXND, TXSTOP_INIT);
	
	/*enable broadcast*/
	enc28j60_write_reg_byte(ERXFCON, enc28j60_read_reg_byte(ERXFCON) | ERXFCON_BCEN);
	
	/*set data link layer*/
	enc28j60_write_reg_byte(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	enc28j60_write_reg_byte(MACON2, 0x00);
	enc28j60_write_op(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
	enc28j60_write_reg(MAIPG, 0x0C12);
	enc28j60_write_reg_byte(MABBIPG,0x12);
	enc28j60_write_reg(MAMXFL,MAX_FRAMELEN);
	enc28j60_write_reg_byte(MAADR5,macaddr[0]);
	enc28j60_write_reg_byte(MAADR4,macaddr[1]);
	enc28j60_write_reg_byte(MAADR3,macaddr[2]);
	enc28j60_write_reg_byte(MAADR2,macaddr[3]);
	enc28j60_write_reg_byte(MAADR1,macaddr[4]);
	enc28j60_write_reg_byte(MAADR0,macaddr[5]);

	/*set physical layer*/
	enc28j60_write_reg_byte(MAADR0,macaddr[5]);
	enc28j60_write_phy(PHCON2,PHCON2_HDLDIS);
	/*leds config*/
	enc28j60_write_phy(PHLCON,PHLCON_LACFG2 | PHLCON_LBCFG2 | PHLCON_LBCFG1 | 
										 PHLCON_LBCFG0 | PHLCON_LFRQ0 | PHLCON_STRCH);
	/*enable global interrupts*/									 
	enc28j60_set_bank(ECON1);
  enc28j60_write_op(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);
  enc28j60_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

void enc28j60_write_op(uint8_t op_code, uint8_t addr, uint8_t data)
{
	ENC_CS_LOW;
	spi_write_byte(op_code | (addr & ADDR_MASK));
	spi_write_byte(data);
	ENC_CS_HIGH;
}

static uint8_t enc28j60_read_op(uint8_t op_code, uint8_t addr)
{
	uint8_t reg_val = 0;
	
	ENC_CS_LOW;
	spi_write_byte(op_code | (addr & ADDR_MASK));
	spi_write_byte(0x00);
	/*skip dummy byte if MAC or MII Reg*/
	if(addr & 0x80)
		spi_read_byte();
	reg_val = spi_read_byte();
	ENC_CS_HIGH;
	
	return reg_val;
}

static void enc28j60_read_buff(uint16_t len, uint8_t *data)
{
	ENC_CS_LOW;
	spi_write_byte(ENC28J60_READ_BUF_MEM);
	while(len--)
		*data++ = spi_write_read_byte(0x00);
	ENC_CS_HIGH;
}

static void enc28j60_set_bank(uint8_t addr)
{
	if((addr & BANK_MASK) != enc28j60Bank)
  {
		enc28j60_write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
		enc28j60Bank = addr & BANK_MASK;
		enc28j60_write_op(ENC28J60_BIT_FIELD_SET, ECON1, enc28j60Bank >> 5);
  }
}

static void enc28j60_write_reg_byte(uint8_t addr, uint8_t data)
{
	enc28j60_set_bank(addr);
  enc28j60_write_op(ENC28J60_WRITE_CTRL_REG, addr, data);
}

static uint8_t enc28j60_read_reg_byte(uint8_t addr)
{
  enc28j60_set_bank(addr);
  return enc28j60_read_op(ENC28J60_READ_CTRL_REG, addr);
}

static void enc28j60_write_reg(uint8_t addr,uint16_t data)
{
  enc28j60_write_reg_byte(addr, data);
  enc28j60_write_reg_byte(addr + 1, data>>8);
}

static void enc28j60_write_phy(uint8_t addr,uint16_t data)
{
  enc28j60_write_reg_byte(MIREGADR, addr);
  enc28j60_write_reg(MIWR, data);
  while(enc28j60_read_reg_byte(MISTAT) & MISTAT_BUSY);
}

uint16_t enc28j60_packet_rx(uint8_t *buf, uint16_t buflen)
{
	uint16_t len = 0;
	if(enc28j60_read_reg_byte(EPKTCNT) > 0)
	{
		enc28j60_write_reg(ERDPT, next_packet_ptr);
		struct
		{
			uint16_t next_packet;
			uint16_t byte_count;
			uint16_t status;
		} header;
		enc28j60_read_buff(sizeof header, (uint8_t *)&header);
		next_packet_ptr = header.next_packet;
		/*remove CRC from len*/
		len = header.byte_count - 4;
		if(len > buflen)
			len = buflen;
		if((header.status & 0x80) == 0)
			len = 0;
		else
			enc28j60_read_buff(len, buf);
		buf[len] = 0;
		if(next_packet_ptr - 1 > RXSTOP_INIT)
			enc28j60_write_reg(ERXRDPT, RXSTOP_INIT);
		else
			enc28j60_write_reg(ERXRDPT, next_packet_ptr-1);
		enc28j60_write_op(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	}
	return len;
}

void enc28j60_packet_tx(uint8_t *buf, uint16_t buflen)
{
	while(enc28j60_read_op(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
  {
		if(enc28j60_read_reg_byte(EIR) & EIR_TXERIF)
		{
			enc28j60_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60_write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
  }
  enc28j60_write_reg(EWRPT,TXSTART_INIT);
  enc28j60_write_reg(ETXND,TXSTART_INIT + buflen);
  enc28j60_write_buf(1, (uint8_t*)"x00");
  enc28j60_write_buf(buflen, buf);
  enc28j60_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}
	
static void enc28j60_write_buf(uint16_t len, uint8_t* data)
{
  ENC_CS_LOW;
  spi_write_byte(ENC28J60_WRITE_BUF_MEM);
  while(len--)
		spi_write_byte(*data++);
  ENC_CS_HIGH;

}




static void error(void)
{
	LED_ON;
}

static uint8_t spi_write_read_byte(uint8_t tx_byte)
{
	uint8_t rx_byte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1, &tx_byte, &rx_byte, 1, 1000) != HAL_OK)
		error();
	
	return rx_byte;
}

void spi_write_byte(uint8_t tx_byte)
{
	spi_write_read_byte(tx_byte);
}

uint8_t spi_read_byte(void)
{
	uint8_t rx_byte = spi_write_read_byte(0xFF);
	
	return rx_byte;
}



