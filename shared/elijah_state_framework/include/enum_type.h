#pragma once

#include <type_traits>

template <typename E, typename U = uint8_t>
concept EnumType = std::is_enum_v<E> && std::is_same_v<std::underlying_type_t<E>, U>;
