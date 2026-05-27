// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// CustomData — the GLIM-style escape hatch carried by Measurement,
// EstimationFrame and SubMap. A frontend or module can stash arbitrary payload
// (a descriptor index, a dense slab, a sensor-specific blob) keyed by name,
// so the pipeline extends without changing the shared types' interfaces.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace slamko {

using CustomData = std::unordered_map<std::string, std::shared_ptr<void>>;

}  // namespace slamko
