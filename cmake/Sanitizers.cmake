# One INTERFACE target carrying the sanitiser flags.
#
# Unlike warnings, these are linked PUBLIC. A sanitiser instruments the whole
# program: if the library is built with AddressSanitizer and the executable is
# not, you get link errors at best and silent nonsense at worst.

set(MW_SANITIZER "none" CACHE STRING "none | address | thread")
set_property(CACHE MW_SANITIZER PROPERTY STRINGS none address thread)

add_library(mw_sanitizers INTERFACE)

if(MW_SANITIZER STREQUAL "address")
  # ASan and UBSan compose; TSan does not compose with either.
  target_compile_options(mw_sanitizers INTERFACE
    -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  target_link_options(mw_sanitizers INTERFACE
    -fsanitize=address,undefined)
elseif(MW_SANITIZER STREQUAL "thread")
  target_compile_options(mw_sanitizers INTERFACE
    -fsanitize=thread -fno-omit-frame-pointer -g)
  target_link_options(mw_sanitizers INTERFACE
    -fsanitize=thread)
endif()
