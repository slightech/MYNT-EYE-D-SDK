#include "mynteye/device/data_caches.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char const *argv[]) {

  DataCaches caches;

  {
    auto&& data1 = caches.GetFixed(12);
    auto&& data2 = caches.GetFixed(12);
    auto&& data3 = caches.GetFixed(120);

    caches.DebugPrint();
  }

  {
    auto&& data1 = caches.GetFixed(12);
    auto&& data2 = caches.GetFixed(12);
    auto&& data3 = caches.GetFixed(120);
    {
      auto&& data4 = caches.GetFixed(120);
    }
    auto&& data4 = caches.GetFixed(120);
    auto&& data5 = caches.GetFixed(120);

    caches.DebugPrint();
  }

  {
    caches.SetProperSizes({12, 110});
    auto&& x = caches.GetProper(10);
    auto&& a = caches.GetProper(12);
    auto&& b = caches.GetProper(100);
    auto&& c = caches.GetProper(200);
  }

  return 0;
}
