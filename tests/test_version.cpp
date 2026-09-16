// No test framework yet, on purpose. A test is a program that exits non-zero
// when something is wrong; ctest needs nothing more. GoogleTest arrives when
// there is enough to test that the boilerplate pays for itself.

#include <iostream>

#include "mw/version.hpp"

int main() {
  const mw::Version v = mw::version();

  if (v.major != 0 || v.minor != 1 || v.patch != 0) {
    std::cerr << "version() returned " << v.major << '.' << v.minor << '.'
              << v.patch << ", expected 0.1.0\n";
    return 1;
  }

  if (mw::version_string() != "0.1.0") {
    std::cerr << "version_string() returned '" << mw::version_string()
              << "', expected '0.1.0'\n";
    return 1;
  }

  return 0;
}
