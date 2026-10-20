#pragma once

void addAuton(const char* name, void (*routine)());
void startAutonSelector();
void runSelectedAuton();
int getSelectedAuton();
