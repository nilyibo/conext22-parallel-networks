// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-        
#include "tcp_transfer.h"
#include "mtcp.h"
#include "math.h"
#include <iostream>
#include "config.h"

////////////////////////////////////////////////////////////////
//  TCP PERIODIC SOURCE
////////////////////////////////////////////////////////////////

uint64_t generateFlowSize(){
  //  if (drand()>0.99)
  //return 8000000;// * (0.75 + drand()/2);
  //else
  return 90000;// * (0.5 + drand());
}

TcpSrcTransfer::TcpSrcTransfer(TcpLogger* logger, TrafficLogger* pktLogger, EventList &eventlist,
                               uint64_t bytes_to_send, vector<const Route*>* p, 
                               EventSource* stopped) : TcpSrc(logger,pktLogger,eventlist)
{
  _is_active = false;  
  _ssthresh = 0xffffffff;
  //_cwnd = 90000;
  _bytes_to_send = bytes_to_send;
  set_flowsize(_bytes_to_send+_mss);
  _paths = p;

  _flow_stopped = stopped;

  //#if PACKET_SCATTER
  //set_paths(p);
  //#endif
}

void TcpSrcTransfer::reset() {
  this->reset(_bytes_to_send, 1);
}

void TcpSrcTransfer::reset(uint64_t bb, int shouldRestart){
  _sawtooth = 0;
  _rtt_avg = timeFromMs(0);
  _rtt_cum = timeFromMs(0);
  _highest_sent = 0;
  _effcwnd = 0;
  _ssthresh = 0xffffffff;
  _last_acked = 0;
  _dupacks = 0;
  _mdev = 0;
  _rto = timeFromMs(10);
  _recoverq = 0;
  _in_fast_recovery = false;
  _established = false;
  
  _rtx_timeout_pending = false;
  _RFC2988_RTO_timeout = timeInf;
  
  //_bytes_to_send = bb;

  if (shouldRestart)
    eventlist().sourceIsPendingRel(*this,timeFromMs(0));
}


void 
TcpSrcTransfer::connect(const Route& routeout, const Route& routeback, TcpSink& sink, simtime_picosec starttime)
{
  _is_active = false;

  TcpSrc::connect(routeout,routeback,sink,starttime);
}

void 
TcpSrcTransfer::doNextEvent() {
  if (!_is_active){
    _is_active = true;

    //delete _route;
    if (_paths!=NULL){
      Route* rt = new Route(*(_paths->at(rand()%_paths->size())));
      rt->push_back(_sink);
      _route = rt;
    }

    //should reset route here!
    //how?
    ((TcpSinkTransfer*)_sink)->reset();

    _started = eventlist().now();
    cout << "Flow ID " << id << " (" << _bytes_to_send << "B) starting at " << timeAsSec(_started) << "s" << endl;
    startflow();
  } else
    TcpSrc::doNextEvent();
}

void 
TcpSrcTransfer::receivePacket(Packet& pkt){
  if (_is_active){
    TcpSrc::receivePacket(pkt);

    if (_bytes_to_send>0){
      if (!_mSrc && _last_acked>=_bytes_to_send){
        _is_active = false;
        flow_completed_ = true;

        cout << "Flow ID " << id << " (" << _bytes_to_send << "B) finished after " << timeAsSec(eventlist().now()-_started) << "s" << endl;
        
        if (_flow_stopped){
          _flow_stopped->doNextEvent(this);
        }
        else 
          reset(_bytes_to_send,1);
      }
      else if (_mSrc){
        if (_last_acked >= _bytes_to_send/_mSrc->_subflows.size() && _mSrc->compute_total_bytes()>=_bytes_to_send){
          //log finish time

          _is_active = false;
          flow_completed_ = true;
          cout << "Subflow ID " << id << " (" << _bytes_to_send << "B) finished after " << timeAsSec(eventlist().now()-_started) << "s" << endl;

          bool all_finished = true;
          for (auto & subflow : _mSrc->_subflows) {
            auto *t = dynamic_cast<TcpSrcTransfer *>(subflow);
            if (t->_is_active) {
              all_finished = false;
              break;
            }
          }

          if (all_finished) {
            cout << "Flow ID " << _mSrc->id << " (" << _mSrc->compute_total_bytes() << "B) finished after " << timeAsSec(eventlist().now()-_started) << "s" << endl;
            _mSrc->triggerStopHandler();
          }

          if (_flow_stopped) {
            _flow_stopped->doNextEvent(this);
          } else {
            //reset all the subflows, including this one.
            // int bb = generateFlowSize();

            // int subflows_to_activate = bb >= 1000000 ? 8:1;
            // int crt_subflow = 0;

            for (const auto subflow: _mSrc->_subflows){
              auto crt = dynamic_cast<TcpSrcTransfer *>(subflow);
              crt->_is_active = false;
              crt->reset(0, 1);
              // crt_subflow++;
            }
          }
        }
      }
    }
  }
  else {
    pkt.flow().logTraffic(pkt,*this,TrafficLogger::PKT_RCVDESTROY);
    pkt.free();
  }
}

void TcpSrcTransfer::rtx_timer_hook(simtime_picosec now, simtime_picosec period) {
  if (!_is_active) return;

  if (now <= _RFC2988_RTO_timeout || _RFC2988_RTO_timeout==timeInf) return;
  if (_highest_sent == 0) return;

  cerr << "Transfer timeout: active " << _is_active << " bytes to send " << _bytes_to_send << " sent " << _last_acked << " established? " << _established << " HSENT " << _highest_sent << endl;

  TcpSrc::rtx_timer_hook(now,period);
}

////////////////////////////////////////////////////////////////
//  Tcp Transfer SINK
////////////////////////////////////////////////////////////////

TcpSinkTransfer::TcpSinkTransfer() : TcpSink() 
{
}

void TcpSinkTransfer::reset(){
  _cumulative_ack = 0;
  _received.clear();

  //queue logger sampling?
}
