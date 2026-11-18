#pragma once

#include "status.hpp"

namespace robot
{
// 由 app_start 在启动阶段调用一次。
pnxtypes::status start();
} // namespace robot