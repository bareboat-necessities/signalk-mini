#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <signalk_mini.hpp>
#define OK(x) do{if(!(x)){std::fprintf(stderr,"failed %s:%d %s\n",__FILE__,__LINE__,#x);std::exit(1);}}while(0)
#define NEAR(a,b,e) do{if(fabsl((long double)(a)-(long double)(b))>(long double)(e)){std::exit(2);}}while(0)
static std::string line(const char*b){char out[128];OK(nmea0183::append_checksum(b,out,sizeof(out)));return out;}
int main(){using Real=float;signalk_mini::ModelStore<Real> store;signalk_mini::Nmea0183Input<Real> in(store);const char*bodies[]={"GPRMC,142533.00,A,4042.6142,N,07400.4168,W,5.4,083.2,010726,13.1,W,A","HCHDT,081.7,T","IIMWV,045.0,R,12.8,N,A","SDDPT,5.6,0.4"};for(auto b:bodies){auto l=line(b);OK(in.feed_line(l.c_str(),7,100)==nmea0183::Status::Applied);}auto&m=store.model();NEAR(m.navigation.latitude_deg.value,40.7102367L,0.0001L);NEAR(m.navigation.longitude_deg.value,-74.0069467L,0.0001L);NEAR(m.navigation.sog_mps.value,2.7779999L,0.001L);NEAR(m.wind.apparent_speed_mps.value,6.5848889L,0.001L);NEAR(m.water.depth_below_transducer_m.value,5.6L,0.001L);signalk_mini::SignalKMapper<Real> mapper;signalk_mini::ModelChange ch;bool got=false;while(store.changes().pop(ch)){signalk_mini::SignalKMappedValue<Real> out;if(mapper.map_change(store.model(),ch,out)&&out.path&&std::strcmp(out.path,"environment.wind.angleApparent")==0)got=true;}OK(got);return 0;}
