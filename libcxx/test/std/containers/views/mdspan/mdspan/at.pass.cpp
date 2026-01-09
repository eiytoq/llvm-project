//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// UNSUPPORTED: c++03, c++11, c++14, c++17, c++20, c++23

// <mdspan>

// template<class... OtherIndexTypes>
//   constexpr reference at(OtherIndexTypes... indices) const;
//
// template<class OtherIndexType>
//   constexpr reference at(span<OtherIndexType, rank()> indices) const;
//
// template<class OtherIndexType>
//   constexpr reference at(const array<OtherIndexType, rank()>& indices) const;
//
// Constraints:
//   * sizeof...(OtherIndexTypes) == extents_type::rank() is true,
//   * (is_convertible_v<OtherIndexTypes, index_type> && ...) is true,
//   * (is_nothrow_constructible_v<index_type, OtherIndexTypes> && ...) is true.
//
// Throws:
//   * std::out_of_range if extents_type::index-cast(indices) is not a multidimensional index in extents_.

#include <array>
#include <cassert>
#include <mdspan>
#include <span> // dynamic_extent
#ifndef TEST_HAS_NO_EXCEPTIONS
#  include <vector>
#endif

#include "test_macros.h"

#include "../ConvertibleToIntegral.h"
#include "../CustomTestLayouts.h"

template <class MDS, class... Indices>
concept at_constraints = requires(MDS m, Indices... idxs) {
  { m.at(idxs...) } -> std::same_as<typename MDS::reference>;
};

template <class MDS, class... Indices>
  requires(at_constraints<MDS, Indices...>)
constexpr bool check_at_constraints(MDS m, Indices... idxs) {
  TEST_IGNORE_NODISCARD m.at(idxs...);
  return true;
}

template <class MDS, class... Indices>
constexpr bool check_at_constraints(MDS, Indices...) {
  return false;
}

template <class MDS, class... Args>
constexpr void iterate(MDS mds, Args... args) {
  constexpr int r = static_cast<int>(MDS::extents_type::rank()) - 1 - static_cast<int>(sizeof...(Args));

  if constexpr (r == -1) {
    [[maybe_unused]] int* ptr_accessor = &(mds.accessor().access(mds.data_handle(), mds.mapping()(args...)));
    std::array<typename MDS::index_type, MDS::rank()> args_arr{static_cast<typename MDS::index_type>(args)...};

    // mdspan.at(indices...)
    [[maybe_unused]] typename MDS::element_type* ptr_at = &mds.at(args...);
    assert(ptr_at == ptr_accessor);

    //  mdspan.at(array)
    [[maybe_unused]] typename MDS::element_type* ptr_arr = &mds.at(args_arr);
    assert(&mds.at(args_arr) == ptr_accessor);

    // mdspan.at(span)
    [[maybe_unused]] typename MDS::element_type* ptr_span = &mds.at(std::span(args_arr));
    assert(ptr_span == ptr_accessor);

  } else {
    for (typename MDS::index_type i = 0; i < mds.extents().extent(r); i++) {
      iterate(mds, i, args...);
    }
  }
}

template <class Mapping>
constexpr void test_iteration(Mapping m) {
  std::array<int, 1024> data;
  using MDS = std::mdspan<int, typename Mapping::extents_type, typename Mapping::layout_type>;
  MDS mds(data.data(), m);
  iterate(mds);
}

template <class Layout>
constexpr void test_layout() {
  constexpr size_t D = std::dynamic_extent;
  test_iteration(construct_mapping(Layout(), std::extents<int>()));
  test_iteration(construct_mapping(Layout(), std::extents<unsigned, D>(1)));
  test_iteration(construct_mapping(Layout(), std::extents<unsigned, D>(7)));
  test_iteration(construct_mapping(Layout(), std::extents<unsigned, 7>()));
  test_iteration(construct_mapping(Layout(), std::extents<unsigned, 7, 8>()));
  test_iteration(construct_mapping(Layout(), std::extents<signed char, D, D, D, D>(1, 1, 1, 1)));

  int data[1];
  // Check at constraint for number of arguments
  static_assert(check_at_constraints(std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), 0));
  static_assert(!check_at_constraints(std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), 0, 0));

  // Check at constraint for convertibility of arguments to index_type
  static_assert(
      check_at_constraints(std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), IntType(0)));
  static_assert(
      !check_at_constraints(std::mdspan(data, construct_mapping(Layout(), std::extents<unsigned, D>(1))), IntType(0)));

  // Check at constraint for no-throw-constructibility of index_type from arguments
  static_assert(!check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<unsigned char, D>(1))), IntType(0)));

  // Check that mixed integrals work: note the second one tests that mdspan casts: layout_wrapping_integral does not accept IntType
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<unsigned char, D, D>(1, 1))), int(0), size_t(0)));
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntType(0)));

  constexpr bool t = true;
  constexpr bool o = false;
  static_assert(!check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<o, o, t, t>(0)));
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<o, t, t, t>(0)));
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<o, t, o, t>(0)));
  static_assert(!check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<t, o, o, t>(0)));
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<t, o, t, o>(0)));
  static_assert(check_at_constraints(
      std::mdspan(data, construct_mapping(Layout(), std::extents<int, D, D>(1, 1))), 0uz, IntConfig<t, t, t, t>(0)));

  // layout_wrapped wouldn't quite work here the way we wrote the check
  // IntConfig has configurable conversion properties: convert from const&, convert from non-const, no-throw-ctor from const&, no-throw-ctor from non-const
  if constexpr (std::is_same_v<Layout, std::layout_left>) {
    static_assert(!check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<o, o, t, t>(0)}));
    static_assert(!check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<o, t, t, t>(0)}));
    static_assert(!check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<t, o, o, t>(0)}));
    static_assert(!check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<t, t, o, t>(0)}));
    static_assert(check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<t, o, t, o>(0)}));
    static_assert(check_at_constraints(
        std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::array{IntConfig<t, t, t, t>(0)}));

    {
      [[maybe_unused]] std::array idx{IntConfig<o, o, t, t>(0)};
      assert(!check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
    {
      [[maybe_unused]] std::array idx{IntConfig<o, t, t, t>(0)};
      assert(!check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
    {
      [[maybe_unused]] std::array idx{IntConfig<t, o, o, t>(0)};
      assert(!check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
    {
      [[maybe_unused]] std::array idx{IntConfig<t, t, o, t>(0)};
      assert(!check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
    {
      [[maybe_unused]] std::array idx{IntConfig<t, o, t, o>(0)};
      assert(check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
    {
      [[maybe_unused]] std::array idx{IntConfig<t, t, t, t>(0)};
      assert(check_at_constraints(
          std::mdspan(data, construct_mapping(Layout(), std::extents<int, D>(1))), std::span(idx)));
    }
  }
}

constexpr bool test() {
  test_layout<std::layout_left>();
  test_layout<std::layout_right>();
  test_layout<layout_wrapping_integral<4>>();
  return true;
}

constexpr bool test_index_casting() {
  using MDS = std::mdspan<int, std::dextents<int, 1>, strict_cast_layout>;

  int data[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  MDS m(data, std::dextents<int, 1>(10));

  SpyIndex idx_val(3);

  {
    assert(&m.at(idx_val) == &data[3]);
  }

  {
    [[maybe_unused]] std::array<SpyIndex, 1> indices{idx_val};
    assert(&m.at(indices) == &data[3]);
  }

  {
    std::array<SpyIndex, 1> arr{idx_val};
    std::span indices(arr);
    assert(&m.at(indices) == &data[3]);
  }

  return true;
}

void test_exceptions() {
#ifndef TEST_HAS_NO_EXCEPTIONS
  std::vector<int> data(100);
  std::mdspan m(data.data(), 10, 10);

  auto check_throw = [&](auto... args) {
    try {
      TEST_IGNORE_NODISCARD m.at(args...);
      assert(false);
    } catch (const std::out_of_range&) {
      return; // Expected
    } catch (...) {
      assert(false);
    }
  };

  assert(m.at(9, 9) == data[99]);

  check_throw(10, 0);
  check_throw(0, 10);
  check_throw(-1, 0);

  check_throw(std::array<int, 2>{10, 0});

  std::array<int, 2> invalid_idx{0, 10};
  check_throw(std::span(invalid_idx));
#endif
}

int main(int, char**) {
  test();
  static_assert(test());

  test_index_casting();
  static_assert(test_index_casting());

  test_exceptions();

  return 0;
}