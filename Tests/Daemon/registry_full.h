#pragma once
#include "daemon.h"
#include <array>
#include <utility>
template<std::size_t... I> auto TestDaemons(std::index_sequence<I...>)
{
    return std::array<Daemon,sizeof...(I)>{{(static_cast<void>(I),Daemon{10U})...}};
}
inline bool FillTestDaemonRegistry()
{
    static auto monitors=TestDaemons(std::make_index_sequence<32>{});
    for (auto &monitor : monitors) if (!DaemonManager::Register(monitor)) return false;
    return true;
}
