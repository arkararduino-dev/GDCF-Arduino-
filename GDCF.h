#ifndef GDCF_H
#define GDCF_H
#include <Arduino.h>
#include "pico/multicore.h"
#include "pico/stdlib.h"
struct GDCFToken {
    bool valid; uint8_t epoch, tag; float data; uint8_t src_x, src_y, dst_x, dst_y, hop_count;
    GDCFToken() : valid(false), epoch(0), tag(0), data(0.0f), src_x(0), src_y(0), dst_x(0), dst_y(0), hop_count(16) {}
    GDCFToken(bool v, uint8_t ep, uint8_t t, float d, uint8_t sx, uint8_t sy, uint8_t dx, uint8_t dy) :
        valid(v), epoch(ep), tag(t), data(d), src_x(sx), src_y(sy), dst_x(dx), dst_y(dy), hop_count(16) {}
};
enum GDCFPrimitive { PRIM_MAC, PRIM_ADD, PRIM_RELU, PRIM_IDENTITY, PRIM_COUNTER };
class GDCNode {
public:
    uint8_t x,y; GDCFPrimitive primitive; uint8_t state, epoch;
    static const int MAX_BUF=4;
    struct Buffer {
        GDCFToken tok[MAX_BUF]; int head=0,tail=0,cnt=0;
        bool push(const GDCFToken& t){ if(cnt>=MAX_BUF)return false; tok[tail]=t; tail=(tail+1)%MAX_BUF; cnt++; return true; }
        bool pop(GDCFToken& t){ if(cnt==0)return false; t=tok[head]; head=(head+1)%MAX_BUF; cnt--; return true; }
        GDCFToken& front(){ return tok[head]; }
        bool empty(){ return cnt==0; }
    };
    Buffer buf_n, buf_s, buf_e, buf_w;
    float acc; GDCFToken out_token; bool out_ready; uint32_t ops_done, ops_skip; int max_cnt;
    int8_t pin_act=-1, pin_out_pulse=-1, pin_in_trig=-1;
    GDCNode() : x(0),y(0),primitive(PRIM_IDENTITY),state(0),epoch(0),acc(0),out_ready(false),ops_done(0),ops_skip(0),max_cnt(0) {}
    void setup(uint8_t _x, uint8_t _y, GDCFPrimitive p){ x=_x; y=_y; primitive=p; state=1; }
    bool canFire(){
        if(primitive==PRIM_MAC){ if(buf_w.empty()||buf_n.empty())return false; return buf_w.front().valid&&buf_n.front().valid&&buf_w.front().epoch==epoch&&buf_n.front().epoch==epoch; }
        if(primitive==PRIM_RELU||primitive==PRIM_IDENTITY){ if(buf_w.empty())return false; return buf_w.front().valid&&buf_w.front().epoch==epoch; }
        if(primitive==PRIM_COUNTER){ if(buf_w.empty())return false; return buf_w.front().valid; }
        return false;
    }
    void evaluate(){
        if(primitive==PRIM_MAC){ GDCFToken a,b; buf_w.pop(a); buf_n.pop(b); acc=a.data*b.data+acc; out_token=GDCFToken(true,epoch,0,acc,x,y,0,0); ops_done++; }
        else if(primitive==PRIM_RELU){ GDCFToken in; buf_w.pop(in); out_token=GDCFToken(true,epoch,0,in.data>0?in.data:0,x,y,0,0); ops_done++; }
        else if(primitive==PRIM_IDENTITY){ GDCFToken in; buf_w.pop(in); out_token=GDCFToken(true,epoch,0,in.data,x,y,0,0); ops_done++; }
        else if(primitive==PRIM_COUNTER){ GDCFToken in; buf_w.pop(in); float count=in.data+1; if(count>=max_cnt) out_token=GDCFToken(true,epoch,4,count,x,y,in.src_x,in.src_y); else out_token=GDCFToken(true,epoch,0,count,x,y,0,0); ops_done++; }
        state=3; out_ready=true;
    }
    void skipInvalid(){ if(primitive==PRIM_MAC&&!buf_w.empty()&&!buf_n.empty()) if(!buf_w.front().valid||!buf_n.front().valid){ GDCFToken d; buf_w.pop(d); buf_n.pop(d); ops_skip++; } }
    void step(){
        if(pin_in_trig!=-1&&digitalRead(pin_in_trig)==HIGH) buf_w.push(GDCFToken(true,epoch,0,1.0f,255,255,x,y));
        if(state==0) state=1;
        if(state==1){ skipInvalid(); if(canFire()){ state=2; evaluate(); } }
    }
    void updatePins(){ if(pin_act!=-1) digitalWrite(pin_act,(state==2||state==3)?HIGH:LOW); }
    bool hasOutput(){ return out_ready; }
    GDCFToken consumeOutput(){ out_ready=false; state=1; return out_token; }
};
class GDCFRouter {
public:
    GDCNode* grid; uint8_t w,h;
    GDCFRouter(GDCNode* g, uint8_t w_, uint8_t h_) : grid(g), w(w_), h(h_) {}
    bool route(GDCFToken& t, GDCNode& src){
        if(src.x==t.dst_x&&src.y==t.dst_y) return true;
        uint8_t nx=src.x, ny=src.y, port=0;
        if(src.x<t.dst_x){ nx++; port=3; } else if(src.x>t.dst_x){ nx--; port=2; }
        else if(src.y<t.dst_y){ ny++; port=0; } else if(src.y>t.dst_y){ ny--; port=1; }
        int idx=ny*w+nx;
        if(nx>=w||ny>=h) return false;
        switch(port){ case 0: return grid[idx].buf_n.push(t); case 1: return grid[idx].buf_s.push(t); case 2: return grid[idx].buf_e.push(t); default: return grid[idx].buf_w.push(t); }
    }
};
class GDCFFabric {
public:
    uint8_t w,h; GDCNode* nodes; GDCFRouter* router; mutex_t mtx;
    GDCFFabric(uint8_t w_, uint8_t h_) : w(w_), h(h_) {
        nodes=new GDCNode[w*h]; router=new GDCFRouter(nodes,w,h); mutex_init(&mtx);
        for(int y=0;y<h;y++) for(int x=0;x<w;x++) nodes[y*w+x].setup(x,y,PRIM_IDENTITY);
    }
    ~GDCFFabric(){ delete[] nodes; delete router; }
    void setNode(uint8_t x, uint8_t y, GDCFPrimitive p){ nodes[y*w+x].primitive=p; }
    void autoActivityPins(int startPin){ for(int i=0;i<w*h;i++){ nodes[i].pin_act=startPin+i; pinMode(nodes[i].pin_act,OUTPUT); } }
    void setActivityPin(uint8_t x, uint8_t y, int pin){ nodes[y*w+x].pin_act=pin; pinMode(pin,OUTPUT); }
    void setInputTriggerPin(uint8_t x, uint8_t y, int pin){ nodes[y*w+x].pin_in_trig=pin; pinMode(pin,INPUT); }
    void setOutputPulsePin(uint8_t x, uint8_t y, int pin){ nodes[y*w+x].pin_out_pulse=pin; pinMode(pin,OUTPUT); }
    void begin(){}
    void update(){
        mutex_enter_blocking(&mtx);
        for(int i=0;i<w*h;i++) nodes[i].step();
        for(int i=0;i<w*h;i++){
            if(nodes[i].hasOutput()){
                GDCFToken t=nodes[i].consumeOutput();
                if(nodes[i].pin_out_pulse!=-1){ digitalWrite(nodes[i].pin_out_pulse,HIGH); delayMicroseconds(10); digitalWrite(nodes[i].pin_out_pulse,LOW); }
                router->route(t,nodes[i]);
            }
        }
        for(int i=0;i<w*h;i++) nodes[i].updatePins();
        mutex_exit(&mtx);
    }
    bool inject(uint8_t x, uint8_t y, uint8_t dir, const GDCFToken& t){
        mutex_enter_blocking(&mtx); bool ok=false;
        switch(dir){ case 0: ok=nodes[y*w+x].buf_n.push(t); break; case 1: ok=nodes[y*w+x].buf_s.push(t); break; case 2: ok=nodes[y*w+x].buf_e.push(t); break; case 3: ok=nodes[y*w+x].buf_w.push(t); break; }
        mutex_exit(&mtx); return ok;
    }
    void lock(){ mutex_enter_blocking(&mtx); }
    void unlock(){ mutex_exit(&mtx); }
};
#endif