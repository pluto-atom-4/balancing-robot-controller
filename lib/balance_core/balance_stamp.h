#pragma once

// Pure stamp check for the generated gains header vs the generated vectors.
// Header-only, no heap. C++ only compares stamps for equality; it never
// recomputes the gains hash (the Python exporter --check does that).
// Order of checks (first failure wins): schema, then contract, then hash.
// Schema is first because it defines what the other fields mean.

#include <cstdint>

namespace balance_stamp {

struct Stamp {
  uint32_t schema;
  uint32_t contract;
  uint32_t hash;
};

enum class Result : uint8_t { Ok = 0, BadSchema, BadContract, BadHash };

constexpr Result check(const Stamp& gains, const Stamp& vectors, uint32_t expect_schema,
                       uint32_t expect_contract) {
  if (gains.schema != expect_schema || vectors.schema != expect_schema) {
    return Result::BadSchema;
  }
  if (gains.contract != expect_contract || vectors.contract != expect_contract) {
    return Result::BadContract;
  }
  if (gains.hash != vectors.hash) {
    return Result::BadHash;
  }
  return Result::Ok;
}

constexpr const char* name(Result r) {
  switch (r) {
    case Result::Ok: return "Ok";
    case Result::BadSchema: return "BadSchema";
    case Result::BadContract: return "BadContract";
    case Result::BadHash: return "BadHash";
  }
  return "?";
}

}  // namespace balance_stamp
