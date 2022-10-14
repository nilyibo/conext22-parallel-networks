// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-        
#include <math.h>
#include "config.h"
#include "tcppacket.h"

double drand() {
    int r=rand();
    int m=RAND_MAX;
    double d = (double)r/(double)m;
    return d;
}

int pareto(int xm, int mean){
    double oneoveralpha = ((double)mean-xm)/mean;
    return (int)((double)xm/pow(drand(),oneoveralpha));
}

double exponential(double lambda){
    return -log(drand())/lambda;
}

mem_b memFromPkt(double pkts) {
    mem_b m = (mem_b)(ceil(pkts * Packet::data_packet_size()));
    return m;
}

linkspeed_bps speedFromPktps(double packetsPerSec) {
    double bitpersec = packetsPerSec*8*Packet::data_packet_size();
    linkspeed_bps spd = (linkspeed_bps) bitpersec;
    return spd;
}

double speedAsPktps(linkspeed_bps bps) {
    double pktps = ((double)bps)/(8.0*Packet::data_packet_size());
    return pktps;
}

mem_pkts memFromPkts(double pkts) {
    return (int)(ceil(pkts));
}

