#pragma once
struct MdnsClass {
  void addService(const char *,const char *,int) {}
  void addServiceTxt(const char *,const char *,const char *,const char *) {}
};
inline MdnsClass MDNS;
