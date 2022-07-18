/* ============================================================================================================================ *//**
 * @file       traits.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 1st April 2022 5:00:00 pm
 * @modified   Wednesday, 25th May 2022 11:05:58 pm
 * @project    scan-tools
 * @brief      Helper traits used to implement LaserScanMatcher class
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __SCAN_TOOLS_POINT_CLOUD_ASSEMBLER_TRAITS_H__
#define __SCAN_TOOLS_POINT_CLOUD_ASSEMBLER_TRAITS_H__

/* =========================================================== Includes =========================================================== */

#include <tuple>
#include <type_traits>

/* ========================================================== Namespaces ========================================================== */

namespace scan_tools {
namespace traits {

/* ============================================================ Traits ============================================================ */

namespace details {

    template<template<typename> typename Template, typename>
    struct resolve_variant_by_tuple { };

    template<template<typename> typename Template, typename ...T>
    struct resolve_variant_by_tuple<Template, std::tuple<T...>> {
        using type = std::variant<Template<T>...>;
    };
    
}

/**
 * @brief Helper traits defining a specialization of std::variant given for
 *    list of specializations of the given @p Template specialized on each
 *    of types by the stored std::tuple type
 * 
 * @tparam Template 
 *    template whose specialization will be stored in variant
 * @tparam T 
 *    tuple defining list of tyopes to specialize @p Template at
 */
template<template<typename> typename Template, typename T>
struct resolve_variant_by_tuple : 
    public details::resolve_variant_by_tuple<Template, T> 
{ };

namespace details {

    template< std::size_t N, typename T, typename Tuple>
    constexpr std::size_t get_index_of() {

        static_assert(N < std::tuple_size<Tuple>::value, "[index_of] The element is not in the tuple");

        if constexpr(std::is_same<T, typename std::tuple_element<N, Tuple>::type>::value )
            return N;
        else
            return get_index_of<N + 1, T, Tuple>();
    }

    template<typename T, typename Tuple>
    struct index_of {
        static constexpr size_t value = get_index_of<0, T, Tuple>();
    };

}

/**
 * @brief Helper trait defining @a value field storing index of the given @p T type
 *   in the list of types of the @p Tuple specialization of std::tuple
 * 
 * @tparam T 
 *    type to be found
 * @tparam Tuple 
 *    tuple defining list of types to search
 */
template<typename T, typename Tuple>
struct index_of : 
    public details::index_of<T, Tuple> 
{ };

/* ================================================================================================================================ */

} // End namespace traits
} // End namespace scan_tools

#endif
