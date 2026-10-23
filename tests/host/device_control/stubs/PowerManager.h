#pragma once
struct PowerManager { int calls = 0; int clear_bonds_and_power_down() { calls++; return -1; } };
extern PowerManager power_manager;
