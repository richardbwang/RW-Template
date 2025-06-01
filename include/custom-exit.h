#ifndef CUSTOM_EXIT_H
#define CUSTOM_EXIT_H

#include <functional>

class CustomExit {
public:
    CustomExit(std::function<bool()> exitCondition);
    bool shouldExit() const;

private:
    std::function<bool()> exitCondition_;
};

#endif // CUSTOM_EXIT_H
