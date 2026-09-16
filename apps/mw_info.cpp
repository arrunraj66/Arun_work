#include <iostream>

#include "mw/version.hpp"

int main() {
  const mw::Version v = mw::version();
  std::cout << "custom middleware " << v.major << '.' << v.minor << '.'
            << v.patch << "  (" << mw::version_string() << ")\n";
  return 0;
}
