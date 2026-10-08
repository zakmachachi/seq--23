#pragma once
#include "HeldFxProtocol.h"

// Latest-held priority; release falls back to the preceding held pad.
// Physical ownership survives scope exit so release can never toggle a step.
class HeldFxButtons {
public:
    using Send=bool (*)(void*,uint8_t,uint8_t);
    void Begin(Send send,void* context){send_=send;context_=context;pending_=true;}
    void Scope(uint8_t scope){if(scope!=scope_){scope_=scope;Clear();}}
    void Clear(){count_=0;selected_=0;pending_=true;}
    bool Press(uint8_t key){
        if(!scope_ || key>=16)return false;
        Release(key);owned_|=uint16_t(1u<<key);
        stack_[count_++]=key;Choose();return true;
    }
    bool Release(uint8_t key){
        if(key>=16 || !(owned_&(1u<<key)))return false;
        owned_&=~uint16_t(1u<<key);
        for(uint8_t i=0;i<count_;++i)if(stack_[i]==key){
            for(uint8_t j=i+1;j<count_;++j)stack_[j-1]=stack_[j];
            --count_;break;
        }
        Choose();return true;
    }
    void Service(uint32_t now){
        if(!send_)return;
        if(pending_){
            if(!send_(context_,heldfx::SELECT,selected_))return;
            pending_=false;lastSend_=now;
        }
        if(selected_ && uint32_t(now-lastSend_)>=heldfx::HEARTBEAT_MS
           && send_(context_,heldfx::KEEPALIVE,127))lastSend_=now;
    }
    uint8_t Selected() const {return selected_;} // 0 off; 1..16 pad address
    uint16_t Held() const {uint16_t m=0;for(uint8_t i=0;i<count_;++i)m|=1u<<stack_[i];return m;}
private:
    void Choose(){uint8_t next=count_?stack_[count_-1]+1:0;if(next!=selected_){selected_=next;pending_=true;}}
    Send send_=nullptr;void* context_=nullptr;
    uint8_t stack_[16]={},count_=0,scope_=0,selected_=0;
    uint16_t owned_=0;uint32_t lastSend_=0;bool pending_=false;
};
