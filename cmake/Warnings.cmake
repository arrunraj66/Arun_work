# One INTERFACE target carrying the warning flags.
#
# It is linked PRIVATE by everything we build, so our own code is held to it —
# and never propagates to anyone who links us. A library that forces -Werror on
# its users is a library people stop using.

option(MW_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

add_library(mw_warnings INTERFACE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mw_warnings INTERFACE
    -Wall                 # the sensible baseline
    -Wextra               # the rest of the sensible baseline
    -Wpedantic            # no compiler extensions sneaking in
    -Wshadow              # a local hiding a member is almost always a bug
    -Wnon-virtual-dtor    # deleting through a base pointer without a virtual dtor
    -Wold-style-cast      # forces static_cast, which is greppable
    -Wcast-align          # casts that break alignment on ARM but not on x86
    -Wunused              # dead code
    -Woverloaded-virtual  # a derived function that hides rather than overrides
    -Wconversion          # silent narrowing: the classic float/size_t trap
    -Wsign-conversion     # signed/unsigned mixing
    -Wdouble-promotion    # a float silently becoming a double
    -Wformat=2)           # printf format strings checked properly
  if(MW_WARNINGS_AS_ERRORS)
    target_compile_options(mw_warnings INTERFACE -Werror)
  endif()
elseif(MSVC)
  target_compile_options(mw_warnings INTERFACE /W4 /permissive-)
  if(MW_WARNINGS_AS_ERRORS)
    target_compile_options(mw_warnings INTERFACE /WX)
  endif()
endif()
