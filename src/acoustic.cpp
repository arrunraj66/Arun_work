#include "mw/acoustic.hpp"

namespace mw {

std::string_view to_string(Priority p) noexcept {
  switch (p) {
    case Priority::kBulk:   return "bulk";
    case Priority::kNormal: return "normal";
    case Priority::kUrgent: return "urgent";
  }
  return "unknown";
}

}  // namespace mw