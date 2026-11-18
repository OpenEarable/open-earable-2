#include <TWIM.h>
#define private public
#include "MAXM86161.h"
#undef private
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <limits>

const device fake_device{};
std::atomic<uint64_t> clock_us{0};
static uint8_t regs[256];
static std::vector<uint8_t> fifo;
static size_t largest_read=0;
int i2c_burst_read(const device *,uint16_t,uint8_t reg,uint8_t *out,uint32_t len) {
    if(reg==REG_FIFO_DATA) {
        assert(len<=FIFO_SIZE*BYTES_PER_CH && len<=fifo.size());
        largest_read=std::max(largest_read,(size_t)len);
        std::memcpy(out,fifo.data(),len);fifo.erase(fifo.begin(),fifo.begin()+len);
    } else { assert(len==1);out[0]=regs[reg]; }
    return 0;
}
int i2c_burst_write(const device *,uint16_t,uint8_t reg,const uint8_t *in,uint32_t len) {
    assert(len==1);regs[reg]=in[0];return 0;
}
static void feed(const std::vector<unsigned> &tags,int counter=-1) {
    fifo.clear();unsigned n=0;
    for(unsigned tag:tags) {
        uint32_t word=(tag<<19)|(++n);
        fifo.push_back(word>>16);fifo.push_back(word>>8);fifo.push_back(word);
    }
    regs[REG_FIFO_DATA_COUNTER]=counter<0?tags.size():counter;
}
int main() {
    MAXM86161 sensor(&I2C2);ppg_sample out[128]{};
    const uint8_t map[4][4]={{1,0,0,0},{1,0,0,0},{1,2,0,0},{1,2,0,3}};
    for(unsigned count=1;count<=LED_NUM;++count) {
        assert(sensor.set_exposure_count(count)==0);
        std::vector<unsigned> tags;for(unsigned i=1;i<=count;++i)tags.push_back(i);
        feed(tags);assert(sensor.read(out,1)==1);
        for(unsigned i=0;i<count;++i)assert(out[0][map[count-1][i]]==i+1);
    }
    std::puts("PASS: all four exposure layouts preserve sample ordering");
    assert(sensor.set_exposure_count(4)==0);feed({1,2});assert(sensor.read(out,1)==0);
    feed({3,4});assert(sensor.read(out,1)==1 && out[0][1]==1 && out[0][2]==2 && out[0][0]==1 && out[0][3]==2);
    assert(sensor.set_exposure_count(3)==0);feed({13,14,15});assert(sensor.read(out,1)==1);
    std::puts("PASS: split frames and picket-fence replacement tags assemble correctly");
    for(unsigned tag=0;tag<=31;++tag) {
        if((tag>=1 && tag<=4)||(tag>=13 && tag<=15))continue;
        assert(sensor.set_exposure_count(4)==0);sensor._pending_exposure_mask=1;
        feed({tag});assert(sensor.read(out,1)==0);
    }
    for(unsigned count:{0U,5U,255U}) {
        sensor._exposure_count=count;sensor._pending_exposure_mask=1;
        feed({31});assert(sensor.read(out,1)==0);
    }
    assert(sensor.set_exposure_count(1)==0);sensor._exposure_output_indices[0]=255;
    feed({1});assert(sensor.read(out,1)==0 && sensor._pending_exposure_mask==0);
    std::puts("PASS: reserved tags, invalid counts and invalid output indices cannot index arrays");
    assert(sensor.set_exposure_count(4)==0);std::vector<unsigned> full;
    for(int i=0;i<FIFO_SIZE;++i)full.push_back(i%4+1);
    feed(full,255);assert(sensor.read(out,std::numeric_limits<size_t>::max())==FIFO_SIZE/4);
    assert(largest_read==FIFO_SIZE*BYTES_PER_CH);
    feed({});assert(sensor.read(out,1)==0);
    assert(sensor.read(nullptr,1)==0 && sensor.read(out,0)==0);
    std::puts("PASS: oversized FIFO counters and caller capacities cannot overflow the DMA buffer");
}
