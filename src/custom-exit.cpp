#include "custom-exit.h"

CustomExit::CustomExit(std::function<bool()> exitCondition)
    : exitCondition_(exitCondition) {}

bool CustomExit::shouldExit() const {
    return exitCondition_();
}
