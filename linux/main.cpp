#include <iostream>
#include <string>
#include <signalk_mini.hpp>
int main(){using Real=float;signalk_mini::ModelStore<Real> store;signalk_mini::Nmea0183Input<Real> input(store);std::string line;uint64_t now=0;while(std::getline(std::cin,line)){now+=100000;input.feed_line(line.c_str(),1,now);}std::cout<<"changes="<<store.sequence()<<"\n";}
