// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-        
#include "ecn_priority_queue.h"
#include <math.h>
#include "ecn.h"
#include "queue_lossless.h"
#include <iostream>

ECNPriorityQueue::ECNPriorityQueue(linkspeed_bps bitrate, mem_b maxsize, EventList& eventlist, QueueLogger* logger, mem_b  K)
    : Queue(bitrate,maxsize,eventlist,logger),
      _K(K)
{
    _state_send = LosslessQueue::READY;
    this->serving = ServingQueue::NONE;
}


void
ECNPriorityQueue::receivePacket(Packet & pkt)
{
    //is this a PAUSE packet?
    if (pkt.type()==ETH_PAUSE){
        EthPausePacket* p = (EthPausePacket*)&pkt;
        
        if (p->sleepTime()>0){
            //remote end is telling us to shut up.
            //assert(_state_send == LosslessQueue::READY);
            if (queuesize()>0)
                //we have a packet in flight
                _state_send = LosslessQueue::PAUSE_RECEIVED;
            else
                _state_send = LosslessQueue::PAUSED;
            
            //cout << timeAsMs(eventlist().now()) << " " << _name << " PAUSED "<<endl;
        }
        else {
            //we are allowed to send!
            _state_send = LosslessQueue::READY;
            //cout << timeAsMs(eventlist().now()) << " " << _name << " GO "<<endl;
            
            //start transmission if we have packets to send!
            if(queuesize()>0)
                beginService();
        }
        
        pkt.free();
        return;
    }

    bool high_priority = pkt.is_high_priority();

    while (_queuesize+pkt.size() > _maxsize) {
        Packet *packet_to_drop;
        if (high_priority && !_enqueued_low.empty()) {
            // Drop from low priority queue if not empty
            packet_to_drop = _enqueued_low.front();
            _enqueued_low.pop_front();
            _queuesize -= packet_to_drop->size();
        } else {
            // Drop it if it's low priority or low priority queue is empty
            packet_to_drop = &pkt;
        }

        if (_logger)
            _logger->logQueue(*this, QueueLogger::PKT_DROP, *packet_to_drop);
        packet_to_drop->flow().logTraffic(pkt, *this, TrafficLogger::PKT_DROP);
        packet_to_drop->free();
        _num_drops++;

        if (packet_to_drop == &pkt) // Stop if the incoming packet is dropped.
            return;
    }
    pkt.flow().logTraffic(pkt, *this, TrafficLogger::PKT_ARRIVE);

    //mark on enqueue
    //    if (_queuesize > _K)
    //  pkt.set_flags(pkt.flags() | ECN_CE);

    /* enqueue the packet */
    bool queueWasEmpty = this->empty();
    if (high_priority)
        _enqueued_high.push_front(&pkt);
    else
        _enqueued_low.push_front(&pkt);
    _queuesize += pkt.size();
    if (_logger) _logger->logQueue(*this, QueueLogger::PKT_ENQUEUE, pkt);

    if (queueWasEmpty && _state_send==LosslessQueue::READY) {
        /* schedule the dequeue event */
        assert(_enqueued_low.size() + _enqueued_high.size() == 1);
        beginService();
    }
}

void
ECNPriorityQueue::beginService()
{
    list<Packet*> *q;
    // Serve from high priority queue first
    if (!_enqueued_high.empty()) {
        q = &_enqueued_high;
        serving = ServingQueue::HIGH;
    } else if (!_enqueued_low.empty()) {
        q = &_enqueued_low;
        serving = ServingQueue::LOW;
    } else {
        // Shouldn't happen
        serving = ServingQueue::NONE;
        assert(0);
    }

    eventlist().sourceIsPendingRel(*this, drainTime(q->back()));
}

void
ECNPriorityQueue::completeService()
{
    /* dequeue the packet */
    assert(!this->empty());
    assert(serving != ServingQueue::NONE);

    auto &queue = (serving == ServingQueue::HIGH) ? _enqueued_high : _enqueued_low;
    Packet* pkt = queue.back();
    queue.pop_back();

    if (_state_send==LosslessQueue::PAUSE_RECEIVED)
        _state_send = LosslessQueue::PAUSED;
    
    //mark on deque
    if (_queuesize > _K)
          pkt->set_flags(pkt->flags() | ECN_CE);

    _queuesize -= pkt->size();
    pkt->flow().logTraffic(*pkt, *this, TrafficLogger::PKT_DEPART);
    if (_logger) _logger->logQueue(*this, QueueLogger::PKT_SERVICE, *pkt);

    /* tell the packet to move on to the next pipe */
    pkt->sendOn();

    if (!this->empty() && _state_send==LosslessQueue::READY) {
        /* schedule the next dequeue event */
        beginService();
    }
}
