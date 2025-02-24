#pragma once

#include <type_traits>

template <typename E>
concept EnumType = std::is_enum_v<E>;
