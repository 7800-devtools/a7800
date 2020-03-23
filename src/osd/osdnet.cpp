// license:BSD-3-Clause
// copyright-holders:Carl
#include "emu.h"
#include "osdnet.h"

static class std::vector<std::unique_ptr<osd_netdev::entry_t>> netdev_list;

void add_netdev(const char *name, const char *description, create_netdev func)
{
	auto entry = make_unique_clear<osd_netdev::entry_t>();
	entry->id = netdev_list.size();
	strncpy(entry->name, name, 255);
	entry->name[255] = '\0';
	strncpy(entry->description, (description != nullptr) ? description : "(no name)", 255);
	entry->description[255] = '\0';
	entry->func = func;
	netdev_list.push_back(std::move(entry));
}

void clear_netdev()
{
	netdev_list.clear();
}

const std::vector<std::unique_ptr<osd_netdev::entry_t>>& get_netdev_list()
{
	return netdev_list;
}

class osd_netdev *open_netdev(int id, class device_network_interface *ifdev, int rate)
{
	for(auto &entry : netdev_list)
		if(entry->id==id)
			return entry->func(entry->name, ifdev, rate);
	return nullptr;
}

osd_netdev::osd_netdev(class device_network_interface *ifdev, int rate)
{
	m_dev = ifdev;
	m_stop = false;
	m_timer = ifdev->device().machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(osd_netdev::recv), this));
	m_timer->adjust(attotime::from_hz(rate), 0, attotime::from_hz(rate));
}

osd_netdev::~osd_netdev()
{
	m_stop = true;
// nasty hack to prevent Segmentation fault on emulation stop
//  m_timer->reset();
}

int osd_netdev::send(uint8_t *buf, int len)
{
	return 0;
}

void osd_netdev::recv(void *ptr, int param)
{
	uint8_t *buf;
	int len;
	//const char atalkmac[] = { 0x09, 0x00, 0x07, 0xff, 0xff, 0xff };
	while((!m_stop) && (len = recv_dev(&buf)))
	{
#if 0
		if(buf[0] & 1)
		{
			if(memcmp("\xff\xff\xff\xff\xff\xff", buf, 6) && memcmp(atalkmac, buf, 6) && !m_dev->mcast_chk(buf, len)) continue;
		}
		else {
			//const unsigned char *ourmac = (const unsigned char *)get_mac();
			//printf("our mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X dst mac: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", ourmac[0], ourmac[1], ourmac[2], ourmac[3], ourmac[4], ourmac[5], buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
			if(memcmp(get_mac(), buf, 6) && !get_promisc()) continue;
		}
#endif

		m_dev->recv_cb(buf, len);
	}
}

int osd_netdev::recv_dev(uint8_t **buf)
{
	return 0;
}

void osd_netdev::set_mac(const char *mac)
{
}

void osd_netdev::set_promisc(bool promisc)
{
}

bool osd_netdev::get_promisc()
{
	if(m_dev)
		return m_dev->get_promisc();
	return false;
}

const char *osd_netdev::get_mac()
{
	if(m_dev)
		return m_dev->get_mac();
	return "\0\0\0\0\0\0";
}

int netdev_count()
{
	return netdev_list.size();
}

void osd_list_network_adapters(void)
{
	#ifdef USE_NETWORK
	int num_devs = netdev_list.size();

	if (num_devs == 0)
	{
		printf("No network adapters were found\n");
		return;
	}

	printf("Available network adapters:\n");
	for (auto &entry : netdev_list)
	{
		printf("    %s\n", entry->description);
	}

	#else
	printf("Network is not supported in this build\n");
	#endif
}
