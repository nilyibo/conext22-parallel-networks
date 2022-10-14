// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-
#ifndef _ECN_PRIORITY_QUEUE_H
#define _ECN_PRIORITY_QUEUE_H
#include "queue.h"
/*
 * A modified ECN queue that has two priority levels.
 */

#include <list>
#include "config.h"
#include "eventlist.h"
#include "network.h"
#include "loggertypes.h"

class ECNPriorityQueue : public Queue {
 public:
    ECNPriorityQueue(linkspeed_bps bitrate, mem_b maxsize, EventList &eventlist, QueueLogger* logger, mem_b drop);
    void receivePacket(Packet & pkt) override;
    void completeService() override;
 protected:
   void beginService() override;
 private:
    enum ServingQueue {
        NONE,
        LOW,
        HIGH
    };

    list<Packet*> _enqueued_low;
    list<Packet*> _enqueued_high;
    mem_b _K;
    int _state_send;

    ServingQueue serving;

    inline bool empty() {
       return _enqueued_low.empty() && _enqueued_high.empty();
    }
};

#endif
