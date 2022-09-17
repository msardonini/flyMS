/**
 * @file yaml_serialization.h
 * @author Mike Sardonini (msardonini@gmail.com)
 * @brief yaml_serialization.h. This file contains code to assist with the serialization of C++ data structs to YAML
 * format and back. This is heavily based on the implementation found here:
 * https://stackoverflow.com/a/34165367/8222156. Credit to https://stackoverflow.com/users/2104697/guillaume-racicot
 * @version 0.1
 * @date 2022-09-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <tuple>

template <typename Class, typename T>
struct PropertyImpl {
  constexpr PropertyImpl(T Class::*aMember, const char* aName) : member{aMember}, name{aName} {}

  using Type = T;

  T Class::*member;
  const char* name;
};

template <typename Class, typename T>
constexpr auto property(T Class::*member, const char* name) {
  return PropertyImpl<Class, T>{member, name};
}

template <typename T, T... S, typename F>
constexpr void for_sequence(std::integer_sequence<T, S...>, F&& f) {
  (static_cast<void>(f(std::integral_constant<T, S>{})), ...);
}

// deserialize function
template <typename T>
T from_yaml(const YAML::Node& data) {
  T object;

  // We first get the number of properties
  constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

  // We iterate on the index sequence of size `nbProperties`
  for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
    // get the property
    constexpr auto property = std::get<i>(T::properties);

    // get the type of the property
    using Type = typename decltype(property)::Type;

    // set the value to the member
    object.*(property.member) = data[std::string(property.name)].as<Type>();
  });

  return object;
}

template <typename T>
YAML::Node to_yaml(const T& object) {
  YAML::Node data;

  // We first get the number of properties
  constexpr auto nbProperties = std::tuple_size<decltype(T::properties)>::value;

  // We iterate on the index sequence of size `nbProperties`
  for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
    // get the property
    constexpr auto property = std::get<i>(T::properties);

    // set the value to the member
    data[property.name] = object.*(property.member);
  });

  return data;
}

// /**
//  * @brief The following is an example of how to use the above code to create a serializable struct
//  *
//  */
// struct Dog {
//   std::string barkType;
//   std::string color;
//   int weight = 0;

//   bool operator==(const Dog& rhs) const {
//     return std::tie(barkType, color, weight) == std::tie(rhs.barkType, rhs.color, rhs.weight);
//   }

//   constexpr static auto properties = std::make_tuple(property(&Dog::barkType, "barkType"),
//                                                      property(&Dog::color, "color"), property(&Dog::weight,
//                                                      "weight"));
// };

// int main() {
//   Dog dog(.barkType = "woof", .color = "brown", .weight = 10);
//   auto dog_node = to_yaml(dog);
//   auto dog2 = from_yaml<Dog>(dog_node);

//   if (dog == dog2) {
//     std::cout << "Success!" << std::endl;
//   } else {
//     std::cout << "Failure!" << std::endl;
//   }
// }
