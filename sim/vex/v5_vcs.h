// Simulator stand-in for the VEX V5 C++ API (only used by the computer simulator in sim/).
// It covers what RW-Template and typical robot code use. Devices talk to the simulated
// robot in sim/sim-engine.cpp. If your code uses something missing here, the sim build
// will say so; add it below the same way.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "../sim-hooks.h"

namespace vex {
  // ============================================================================
  // UNITS AND CONSTANTS (same names as the VEX SDK)
  // ============================================================================
  enum class percentUnits { pct };
  enum class timeUnits { sec, msec };
  enum class currentUnits { amp };
  enum class voltageUnits { volt, mV };
  enum class powerUnits { watt };
  enum class torqueUnits { Nm, InLb };
  enum class rotationUnits { deg, rev, raw };
  enum class velocityUnits { pct, rpm, dps };
  enum class distanceUnits { mm, in, cm };
  enum class analogUnits { pct, range8bit, range10bit, range12bit, mV };
  enum class temperatureUnits { celsius, fahrenheit };
  enum class directionType { fwd, rev, undefined };
  enum class turnType { left, right };
  enum class brakeType { coast, brake, hold, undefined };
  enum class gearSetting { ratio36_1, ratio18_1, ratio6_1 };
  enum class fontType { mono20, mono30, mono40, mono60, mono15, mono12, prop20, prop30, prop40, prop60, mono10 };
  enum class controllerType { primary, partner };
  enum class axisType { xaxis, yaxis, zaxis };
  enum class orientationType { roll, pitch, yaw };
  enum class sizeType { none, small, medium, large };
  enum class ledState { off, on };

  inline const rotationUnits degrees = rotationUnits::deg, deg = rotationUnits::deg, turns = rotationUnits::rev, rev = rotationUnits::rev;
  inline const percentUnits percent = percentUnits::pct, pct = percentUnits::pct;
  inline const timeUnits seconds = timeUnits::sec, sec = timeUnits::sec, msec = timeUnits::msec;
  inline const distanceUnits inches = distanceUnits::in, mm = distanceUnits::mm;
  inline const directionType forward = directionType::fwd, fwd = directionType::fwd, reverse = directionType::rev;
  inline const turnType left = turnType::left, right = turnType::right;
  inline const axisType xaxis = axisType::xaxis, yaxis = axisType::yaxis, zaxis = axisType::zaxis;
  inline const orientationType roll = orientationType::roll, pitch = orientationType::pitch, yaw = orientationType::yaw;
  inline const voltageUnits volt = voltageUnits::volt;
  inline const currentUnits amp = currentUnits::amp;
  inline const powerUnits watt = powerUnits::watt;
  inline const torqueUnits Nm = torqueUnits::Nm, InLb = torqueUnits::InLb;
  inline const velocityUnits rpm = velocityUnits::rpm, dps = velocityUnits::dps;
  inline const temperatureUnits celsius = temperatureUnits::celsius, fahrenheit = temperatureUnits::fahrenheit;
  inline const brakeType coast = brakeType::coast, brake = brakeType::brake, hold = brakeType::hold;
  inline const gearSetting ratio36_1 = gearSetting::ratio36_1, ratio18_1 = gearSetting::ratio18_1, ratio6_1 = gearSetting::ratio6_1;
  inline const controllerType primary = controllerType::primary, partner = controllerType::partner;
  inline const fontType mono10 = fontType::mono10, mono12 = fontType::mono12, mono15 = fontType::mono15, mono20 = fontType::mono20,
                        mono30 = fontType::mono30, mono40 = fontType::mono40, mono60 = fontType::mono60,
                        monoXS = fontType::mono12, monoS = fontType::mono15, monoM = fontType::mono20, monoL = fontType::mono30,
                        monoXL = fontType::mono40, monoXXL = fontType::mono60, prop20 = fontType::prop20, prop30 = fontType::prop30,
                        prop40 = fontType::prop40, prop60 = fontType::prop60, propM = fontType::prop20, propL = fontType::prop30,
                        propXL = fontType::prop40, propXXL = fontType::prop60;

  inline const int32_t PORT1 = 0, PORT2 = 1, PORT3 = 2, PORT4 = 3, PORT5 = 4, PORT6 = 5, PORT7 = 6, PORT8 = 7,
                       PORT9 = 8, PORT10 = 9, PORT11 = 10, PORT12 = 11, PORT13 = 12, PORT14 = 13, PORT15 = 14,
                       PORT16 = 15, PORT17 = 16, PORT18 = 17, PORT19 = 18, PORT20 = 19, PORT21 = 20, PORT22 = 21;

  inline double toMs(double value, timeUnits units) { return units == timeUnits::sec ? value * 1000.0 : value; }
  inline int brakeMode(brakeType type) { return type == brakeType::coast ? 0 : (type == brakeType::brake ? 1 : 2); }
  inline double fromDeg(double deg, rotationUnits units) {
    return units == rotationUnits::rev ? deg / 360.0 : deg;
  }
  inline double toDeg(double value, rotationUnits units) {
    return units == rotationUnits::rev ? value * 360.0 : value;
  }

  // ============================================================================
  // COLORS
  // ============================================================================
  class color {
   public:
    uint32_t rgb = 0;
    color() {}
    color(uint32_t value) : rgb(value) {}
    color(int r, int g, int b) : rgb(((r & 255) << 16) | ((g & 255) << 8) | (b & 255)) {}
    operator uint32_t() const { return rgb; }
  };
  inline const color black(0x000000), white(0xFFFFFF), red(0xFF0000), green(0x00FF00), blue(0x0000FF),
                     yellow(0xFFFF00), orange(0xFFA500), purple(0xFF00FF), cyan(0x00FFFF), transparent(0);

  // ============================================================================
  // TIME AND THREADS
  // ============================================================================
  inline void wait(double time, timeUnits units = timeUnits::msec) { rwsim::sleepMs(toMs(time, units)); }

  class timer {
    double start_ms = 0;
   public:
    timer() : start_ms(rwsim::nowMs()) {}
    double time(timeUnits units = timeUnits::msec) const {
      double elapsed = rwsim::nowMs() - start_ms;
      return units == timeUnits::sec ? elapsed / 1000.0 : elapsed;
    }
    double value() const { return time(timeUnits::sec); }
    void clear() { start_ms = rwsim::nowMs(); }
    void reset() { clear(); }
    operator uint32_t() const { return (uint32_t)time(); }
    static uint32_t system() { return (uint32_t)rwsim::nowMs(); }
    static uint64_t systemHighResolution() { return (uint64_t)(rwsim::nowMs() * 1000.0); }
  };

  class thread {
    int32_t id = -1;
   public:
    thread() {}
    thread(void (*callback)(void)) { id = rwsim::spawnThread([callback] { callback(); }); }
    thread(int (*callback)(void)) { id = rwsim::spawnThread([callback] { callback(); }); }
    thread(int (*callback)(void*), void* arg) { id = rwsim::spawnThread([callback, arg] { callback(arg); }); }
    thread(void (*callback)(void*), void* arg) { id = rwsim::spawnThread([callback, arg] { callback(arg); }); }
    int32_t get_id() { return id; }
    void join() { while (rwsim::threadRunning(id)) rwsim::sleepMs(10); }
    void detach() {}
    bool joinable() { return rwsim::threadRunning(id); }
    void interrupt() { rwsim::interruptThread(id); }
    static void swap(thread& a, thread& b) { int32_t t = a.id; a.id = b.id; b.id = t; }
  };

  namespace this_thread {
    inline int32_t get_id() { return rwsim::currentThreadId(); }
    inline void sleep_for(uint32_t ms) { rwsim::sleepMs(ms); }
    inline void sleep_until(uint32_t ms) { double d = ms - rwsim::nowMs(); rwsim::sleepMs(d > 0 ? d : 0); }
    inline void yield() { rwsim::sleepMs(0); }
  }

  class task {
    int32_t id = -1;
   public:
    task() {}
    task(int (*callback)(void)) { id = rwsim::spawnThread([callback] { callback(); }); }
    task(int (*callback)(void*), void* arg) { id = rwsim::spawnThread([callback, arg] { callback(arg); }); }
    void stop() { rwsim::interruptThread(id); }
    bool suspend() { return false; }
    bool resume() { return false; }
    int32_t index() { return id; }
    static void sleep(uint32_t ms) { rwsim::sleepMs(ms); }
    static void yield() { rwsim::sleepMs(0); }
  };

  class mutex {
   public:
    void lock() {}
    void unlock() {}
    bool try_lock() { return true; }
  };

  // ============================================================================
  // DEVICES
  // ============================================================================
  class device {
   protected:
    int32_t _port = 0;
   public:
    device() {}
    device(int32_t port) : _port(port) {}
    int32_t index() const { return _port; }
    bool installed() { return true; }
    uint32_t timestamp() { return (uint32_t)rwsim::nowMs(); }
    int32_t value() { return 0; }
  };

  class motor : public device {
    static double freeRpm(gearSetting gears) {
      return gears == gearSetting::ratio6_1 ? 600 : (gears == gearSetting::ratio36_1 ? 100 : 200);
    }
   public:
    // Without a gear setting, keep the motor's cartridge (motor_group makes these to reach its motors)
    motor(int32_t port) : device(port) { rwsim::motorRegister(port, 0); }
    motor(int32_t port, bool reverse) : device(port) { (void)reverse; rwsim::motorRegister(port, 0); }
    motor(int32_t port, gearSetting gears) : device(port) { rwsim::motorRegister(port, freeRpm(gears)); }
    motor(int32_t port, gearSetting gears, bool reverse) : device(port) { (void)reverse; rwsim::motorRegister(port, freeRpm(gears)); }

    // Direction is already "forward" for the simulated robot, whatever the reversed flag says
    void spin(directionType dir, double voltage, voltageUnits units) {
      double volts = units == voltageUnits::mV ? voltage / 1000.0 : voltage;
      rwsim::motorVoltage(_port, dir == directionType::rev ? -volts : volts);
    }
    void spin(directionType dir, double velocity, velocityUnits units) {
      double pct = units == velocityUnits::pct ? velocity : (units == velocityUnits::rpm ? velocity : velocity / 6.0) / rwsim::motorFreeRpm(_port) * 100.0;
      rwsim::motorPercent(_port, dir == directionType::rev ? -pct : pct);
    }
    void spin(directionType dir, double velocity, percentUnits units) { (void)units; rwsim::motorPercent(_port, dir == directionType::rev ? -velocity : velocity); }
    void spin(directionType dir) { double pct = rwsim::motorVelocitySetting(_port); rwsim::motorPercent(_port, dir == directionType::rev ? -pct : pct); }
    void setVelocity(double velocity, velocityUnits units) { rwsim::motorSetVelocity(_port, units == velocityUnits::pct ? velocity : velocity / rwsim::motorFreeRpm(_port) * 100.0); }
    void setVelocity(double velocity, percentUnits units) { (void)units; rwsim::motorSetVelocity(_port, velocity); }
    void stop() { rwsim::motorStop(_port, -1); }
    void stop(brakeType mode) { rwsim::motorStop(_port, brakeMode(mode)); }
    void setStopping(brakeType mode) { rwsim::motorSetStopping(_port, brakeMode(mode)); }
    void setBrake(brakeType mode) { setStopping(mode); }
    void setPosition(double value, rotationUnits units) { rwsim::motorSetPositionDeg(_port, toDeg(value, units)); }
    void resetPosition() { rwsim::motorSetPositionDeg(_port, 0); }
    void resetRotation() { resetPosition(); }
    double position(rotationUnits units = rotationUnits::deg) { return fromDeg(rwsim::motorPositionDeg(_port), units); }
    double rotation(rotationUnits units = rotationUnits::deg) { return position(units); }
    double velocity(velocityUnits units) {
      double r = rwsim::motorRpm(_port);
      return units == velocityUnits::rpm ? r : (units == velocityUnits::dps ? r * 6.0 : r / rwsim::motorFreeRpm(_port) * 100.0);
    }
    double velocity(percentUnits units) { (void)units; return rwsim::motorRpm(_port) / rwsim::motorFreeRpm(_port) * 100.0; }
    bool isSpinning() { return std::fabs(rwsim::motorRpm(_port)) > 1; }
    bool isDone() { return true; }
    double current(currentUnits units = currentUnits::amp) { (void)units; return 0.5; }
    double current(percentUnits units) { (void)units; return 20; }
    double voltage(voltageUnits units = voltageUnits::volt) { (void)units; return 0; }
    double power(powerUnits units = powerUnits::watt) { (void)units; return 0; }
    double torque(torqueUnits units = torqueUnits::Nm) { (void)units; return 0; }
    double efficiency(percentUnits units = percentUnits::pct) { (void)units; return 50; }
    double temperature(percentUnits units) { (void)units; return 20; }
    double temperature(temperatureUnits units = temperatureUnits::celsius) { (void)units; return 30; }
    void setMaxTorque(double value, percentUnits units) { (void)value; (void)units; }
    void setMaxTorque(double value, currentUnits units) { (void)value; (void)units; }
    void setTimeout(int32_t time, timeUnits units) { (void)time; (void)units; }
    void spinFor(directionType dir, double rotation, rotationUnits units, bool waitForCompletion = true) {
      spinToPosition(position(units) + (dir == directionType::rev ? -rotation : rotation), units, waitForCompletion);
    }
    void spinToPosition(double rotation, rotationUnits units, bool waitForCompletion = true) {
      double target = toDeg(rotation, units);
      auto step = [this, target] {
        double error = target - rwsim::motorPositionDeg(_port);
        if (std::fabs(error) < 2) { rwsim::motorStop(_port, 2); return true; }
        rwsim::motorPercent(_port, (error > 0 ? 1 : -1) * std::fmin(50.0, std::fabs(error) * 0.5 + 5));
        return false;
      };
      if (!waitForCompletion) { step(); return; }
      while (!step()) rwsim::sleepMs(10);
    }
  };

  class motor_group {
    std::vector<int32_t> ports;
    void add(motor& m) { ports.push_back(m.index()); }
   public:
    motor_group() {}
    template <class... Motors>
    motor_group(motor& first, Motors&... rest) { add(first); (add(rest), ...); }
    const std::vector<int32_t>& simPorts() const { return ports; }
    int32_t count() const { return (int32_t)ports.size(); }

    void spin(directionType dir, double voltage, voltageUnits units) { for (int32_t p : ports) motor(p).spin(dir, voltage, units); }
    void spin(directionType dir, double velocity, velocityUnits units) { for (int32_t p : ports) motor(p).spin(dir, velocity, units); }
    void spin(directionType dir, double velocity, percentUnits units) { for (int32_t p : ports) motor(p).spin(dir, velocity, units); }
    void spin(directionType dir) { for (int32_t p : ports) motor(p).spin(dir); }
    void setVelocity(double velocity, velocityUnits units) { for (int32_t p : ports) motor(p).setVelocity(velocity, units); }
    void setVelocity(double velocity, percentUnits units) { for (int32_t p : ports) motor(p).setVelocity(velocity, units); }
    void stop() { for (int32_t p : ports) motor(p).stop(); }
    void stop(brakeType mode) { for (int32_t p : ports) motor(p).stop(mode); }
    void setStopping(brakeType mode) { for (int32_t p : ports) motor(p).setStopping(mode); }
    void setPosition(double value, rotationUnits units) { for (int32_t p : ports) motor(p).setPosition(value, units); }
    void resetPosition() { for (int32_t p : ports) motor(p).resetPosition(); }
    // Like the VEX SDK, position and velocity come from the first motor in the group
    double position(rotationUnits units = rotationUnits::deg) { return ports.empty() ? 0 : motor(ports[0]).position(units); }
    double velocity(velocityUnits units) { return ports.empty() ? 0 : motor(ports[0]).velocity(units); }
    double velocity(percentUnits units) { return ports.empty() ? 0 : motor(ports[0]).velocity(units); }
    double current(currentUnits units = currentUnits::amp) { (void)units; return 0.5 * ports.size(); }
    double temperature(temperatureUnits units = temperatureUnits::celsius) { (void)units; return 30; }
    bool isSpinning() { return !ports.empty() && motor(ports[0]).isSpinning(); }
    void spinFor(directionType dir, double rotation, rotationUnits units, bool waitForCompletion = true) {
      for (size_t i = 0; i < ports.size(); i++) motor(ports[i]).spinFor(dir, rotation, units, i + 1 == ports.size() ? waitForCompletion : false);
    }
    void spinToPosition(double rotation, rotationUnits units, bool waitForCompletion = true) {
      for (size_t i = 0; i < ports.size(); i++) motor(ports[i]).spinToPosition(rotation, units, i + 1 == ports.size() ? waitForCompletion : false);
    }
  };

  class inertial : public device {
   public:
    inertial(int32_t port) : device(port) {}
    inertial(int32_t port, turnType dir) : device(port) { (void)dir; }
    void calibrate() { rwsim::imuCalibrate(_port); }
    void startCalibration() { calibrate(); }
    bool isCalibrating() { return rwsim::imuCalibrating(_port); }
    double rotation(rotationUnits units = rotationUnits::deg) { return fromDeg(rwsim::imuRotationDeg(_port), units); }
    double heading(rotationUnits units = rotationUnits::deg) {
      double h = std::fmod(rwsim::imuRotationDeg(_port), 360.0);
      if (h < 0) h += 360.0;
      return fromDeg(h, units);
    }
    double angle(rotationUnits units = rotationUnits::deg) { return heading(units); }
    void setRotation(double value, rotationUnits units) { rwsim::imuSetRotationDeg(_port, toDeg(value, units)); }
    void setHeading(double value, rotationUnits units) {
      double current = rwsim::imuRotationDeg(_port);
      double h = std::fmod(current, 360.0);
      if (h < 0) h += 360.0;
      rwsim::imuSetRotationDeg(_port, current - h + toDeg(value, units));
    }
    void resetRotation() { setRotation(0, rotationUnits::deg); }
    void resetHeading() { setHeading(0, rotationUnits::deg); }
    double gyroRate(axisType axis, velocityUnits units) { (void)units; return axis == axisType::zaxis ? rwsim::imuRateDps(_port) : 0; }
    double acceleration(axisType axis) { (void)axis; return 0; }
    double orientation(orientationType type, rotationUnits units) { return type == orientationType::yaw ? heading(units) : 0; }
    double roll(rotationUnits units = rotationUnits::deg) { (void)units; return 0; }
    double pitch(rotationUnits units = rotationUnits::deg) { (void)units; return 0; }
    double yaw(rotationUnits units = rotationUnits::deg) { return heading(units); }
  };

  class rotation : public device {
   public:
    rotation(int32_t port, bool reverse = false) : device(port) { (void)reverse; }
    double position(rotationUnits units) { return fromDeg(rwsim::rotationSensorDeg(_port), units); }
    double angle(rotationUnits units = rotationUnits::deg) {
      double a = std::fmod(rwsim::rotationSensorDeg(_port), 360.0);
      if (a < 0) a += 360.0;
      return fromDeg(a, units);
    }
    void setPosition(double value, rotationUnits units) { rwsim::rotationSensorSetDeg(_port, toDeg(value, units)); }
    void resetPosition() { setPosition(0, rotationUnits::deg); }
    double velocity(velocityUnits units) { double r = rwsim::rotationSensorRpm(_port); return units == velocityUnits::dps ? r * 6.0 : r; }
    void setReversed(bool value) { (void)value; }
  };

  class distance : public device {
   public:
    distance(int32_t port) : device(port) {}
    double objectDistance(distanceUnits units) {
      double mm_value = rwsim::distanceMm(_port);
      if (mm_value >= 9999) return units == distanceUnits::mm ? 9999 : (units == distanceUnits::in ? 9999 / 25.4 : 999.9);
      return units == distanceUnits::mm ? mm_value : (units == distanceUnits::in ? mm_value / 25.4 : mm_value / 10.0);
    }
    bool isObjectDetected() { return rwsim::distanceMm(_port) < 9999; }
    sizeType objectSize() { return isObjectDetected() ? sizeType::large : sizeType::none; }
    int32_t objectRawSize() { return isObjectDetected() ? 200 : 0; }
    double objectVelocity() { return 0; }
    bool changed() { return true; }
    uint32_t timestamp() { return rwsim::distanceTimestamp(_port); }
  };

  class optical : public device {
   public:
    optical(int32_t port) : device(port) {}
    optical(int32_t port, bool gesture) : device(port) { (void)gesture; }
    void setLight(ledState state) { (void)state; }
    void setLightPower(int32_t value, percentUnits units = percentUnits::pct) { (void)value; (void)units; }
    bool isNearObject() { return false; }
    double hue() { return 0; }
    double brightness(bool raw = false) { (void)raw; return 0; }
    int32_t proximity() { return 0; }
  };

  class vision {
   public:
    class signature {
     public:
      template <class... A> signature(A...) {}
    };
    class code {
     public:
      template <class... A> code(A...) {}
    };
  };

  class triport {
   public:
    class port {
     public:
      int32_t id;
      port(int32_t value) : id(value) {}
    };
    port A{0}, B{1}, C{2}, D{3}, E{4}, F{5}, G{6}, H{7};
  };

  class digital_out {
    bool state = false;
   public:
    digital_out(triport::port& port) { (void)port; }
    void set(bool value) { state = value; }
    int32_t value() { return state ? 1 : 0; }
    digital_out& operator=(bool value) { state = value; return *this; }
  };

  class digital_in {
   public:
    digital_in(triport::port& port) { (void)port; }
    int32_t value() { return 0; }
  };

  class pneumatics {
    bool state = false;
   public:
    pneumatics(triport::port& port) { (void)port; }
    void open() { state = true; }
    void close() { state = false; }
    void set(bool value) { state = value; }
    int32_t value() { return state ? 1 : 0; }
  };

  // ============================================================================
  // BRAIN, CONTROLLER, COMPETITION
  // ============================================================================
  class brain {
   public:
    // Screen drawing does nothing in the simulator
    class lcd {
     public:
      template <class... A> void print(A&&...) {}
      template <class... A> void printAt(A&&...) {}
      template <class... A> void setCursor(A&&...) {}
      template <class... A> void setFont(A&&...) {}
      template <class... A> void setPenColor(A&&...) {}
      template <class... A> void setPenWidth(A&&...) {}
      template <class... A> void setFillColor(A&&...) {}
      template <class... A> void clearScreen(A&&...) {}
      template <class... A> void clearLine(A&&...) {}
      template <class... A> void newLine(A&&...) {}
      template <class... A> void drawLine(A&&...) {}
      template <class... A> void drawRectangle(A&&...) {}
      template <class... A> void drawCircle(A&&...) {}
      template <class... A> void drawPixel(A&&...) {}
      template <class... A> void drawImageFromFile(A&&...) {}
      template <class... A> bool drawImageFromBuffer(A&&...) { return true; }
      template <class... A> void setOrigin(A&&...) {}
      template <class... A> void setClipRegion(A&&...) {}
      template <class... A> void pressed(A&&...) {}
      template <class... A> void released(A&&...) {}
      template <class... A> bool render(A&&...) { return true; }
      int32_t getStringWidth(const char* text) { int32_t n = 0; while (text && text[n]) n++; return n * 10; }
      int32_t getStringHeight(const char* text) { (void)text; return 20; }
      int32_t row() { return 1; }
      int32_t column() { return 1; }
      bool pressing() { return false; }
      int32_t xPosition() { return 0; }
      int32_t yPosition() { return 0; }
    };
    // The simulated SD card is the folder sim/out/sdcard
    class sdcard {
      static std::string path(const char* name) { return std::string("sim/out/sdcard/") + name; }
      static int32_t write(const char* name, uint8_t* buffer, int32_t length, const char* how) {
        FILE* file = std::fopen(path(name).c_str(), how);
        if (!file) return 0;
        int32_t written = (int32_t)std::fwrite(buffer, 1, length, file);
        std::fclose(file);
        return written;
      }
     public:
      bool isInserted() { return true; }
      int32_t savefile(const char* name, uint8_t* buffer, int32_t length) { return write(name, buffer, length, "wb"); }
      int32_t appendfile(const char* name, uint8_t* buffer, int32_t length) { return write(name, buffer, length, "ab"); }
      int32_t loadfile(const char* name, uint8_t* buffer, int32_t length) {
        FILE* file = std::fopen(path(name).c_str(), "rb");
        if (!file) return 0;
        int32_t read = (int32_t)std::fread(buffer, 1, length, file);
        std::fclose(file);
        return read;
      }
      int32_t size(const char* name) {
        FILE* file = std::fopen(path(name).c_str(), "rb");
        if (!file) return 0;
        std::fseek(file, 0, SEEK_END);
        int32_t bytes = (int32_t)std::ftell(file);
        std::fclose(file);
        return bytes;
      }
      bool exists(const char* name) {
        FILE* file = std::fopen(path(name).c_str(), "rb");
        if (file) std::fclose(file);
        return file != nullptr;
      }
    };
    class battery {
     public:
      uint32_t capacity(percentUnits units = percentUnits::pct) { (void)units; return 100; }
      double voltage(voltageUnits units = voltageUnits::volt) { return units == voltageUnits::mV ? 12600 : 12.6; }
      double current(currentUnits units = currentUnits::amp) { (void)units; return 2; }
      double temperature(percentUnits units) { (void)units; return 20; }
      double temperature(temperatureUnits units = temperatureUnits::celsius) { (void)units; return 25; }
    };

    lcd Screen;
    sdcard SDcard;
    battery Battery;
    triport ThreeWirePort;
    vex::timer Timer;

    double timer(timeUnits units) { return units == timeUnits::sec ? rwsim::nowMs() / 1000.0 : rwsim::nowMs(); }
    void playSound(int) {}
  };

  // Joysticks and buttons read as untouched in the simulator
  class controller {
   public:
    class axis {
     public:
      int32_t value() const { return 0; }
      int32_t position(percentUnits units = percentUnits::pct) const { (void)units; return 0; }
      template <class... A> void changed(A&&...) const {}
    };
    class button {
     public:
      bool pressing() const { return false; }
      template <class... A> void pressed(A&&...) const {}
      template <class... A> void released(A&&...) const {}
    };
    class lcd {
     public:
      template <class... A> void print(A&&...) {}
      template <class... A> void setCursor(A&&...) {}
      template <class... A> void clearScreen(A&&...) {}
      template <class... A> void clearLine(A&&...) {}
      template <class... A> void newLine(A&&...) {}
    };

    controller() {}
    controller(controllerType type) { (void)type; }
    axis Axis1, Axis2, Axis3, Axis4;
    button ButtonL1, ButtonL2, ButtonR1, ButtonR2, ButtonUp, ButtonDown, ButtonLeft, ButtonRight,
           ButtonX, ButtonB, ButtonY, ButtonA;
    lcd Screen;
    void rumble(const char*) {}
    bool installed() { return true; }
  };

  class competition {
   public:
    inline static bool bStopTasksBetweenModes = true;
    inline static bool bStopAllTasksBetweenModes = false;
    competition() {}
    void autonomous(void (*callback)(void)) { (void)callback; }
    void drivercontrol(void (*callback)(void)) { (void)callback; }
    bool isEnabled() { return rwsim::competitionEnabled(); }
    bool isAutonomous() { return rwsim::competitionAutonomous(); }
    bool isDriverControl() { return false; }
    bool isCompetitionSwitch() { return true; }
    bool isFieldControl() { return false; }
  };
}
