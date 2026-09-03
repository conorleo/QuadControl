// FMI 2.0 mode exchange host for the Quadcopter FMU.
// Does not modify model sources. Load the unzipped FMU (or pass its path) and step.

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// --- FMI 2.0 types (standard, not model code) ---
using fmi2Component = void*;
using fmi2ComponentEnvironment = void*;
using fmi2FMUstate = void*;
using fmi2ValueReference = unsigned int;
using fmi2Real = double;
using fmi2Integer = int;
using fmi2Boolean = int;
using fmi2Char = char;
using fmi2String = const fmi2Char*;
using fmi2Byte = char;
using fmi2Type = enum { fmi2ModelExchange = 0, fmi2CoSimulation = 1 };
using fmi2Status = enum {
  fmi2OK = 0,
  fmi2Warning = 1,
  fmi2Discard = 2,
  fmi2Error = 3,
  fmi2Fatal = 4,
  fmi2Pending = 5
};
using fmi2StatusKind = enum {
  fmi2DoStepStatus = 0,
  fmi2PendingStatus = 1,
  fmi2LastSuccessfulTime = 2,
  fmi2Terminated = 3
};

#define fmi2True 1
#define fmi2False 0

using fmi2CallbackLogger = void (*)(fmi2ComponentEnvironment, fmi2String, fmi2Status,
                                    fmi2String, fmi2String, ...);
using fmi2CallbackAllocateMemory = void* (*)(std::size_t, std::size_t);
using fmi2CallbackFreeMemory = void (*)(void*);
using fmi2StepFinished = void (*)(fmi2ComponentEnvironment, fmi2Status);

struct fmi2CallbackFunctions {
  fmi2CallbackLogger logger;
  fmi2CallbackAllocateMemory allocateMemory;
  fmi2CallbackFreeMemory freeMemory;
  fmi2StepFinished stepFinished;
  fmi2ComponentEnvironment componentEnvironment;
};

using fmi2GetTypesPlatformTYPE = const char* (*)();
using fmi2GetVersionTYPE = const char* (*)();
using fmi2InstantiateTYPE = fmi2Component (*)(fmi2String, fmi2Type, fmi2String, fmi2String,
                                              const fmi2CallbackFunctions*, fmi2Boolean,
                                              fmi2Boolean);
using fmi2FreeInstanceTYPE = void (*)(fmi2Component);
using fmi2SetupExperimentTYPE = fmi2Status (*)(fmi2Component, fmi2Boolean, fmi2Real, fmi2Real,
                                               fmi2Boolean, fmi2Real);
using fmi2EnterInitializationModeTYPE = fmi2Status (*)(fmi2Component);
using fmi2ExitInitializationModeTYPE = fmi2Status (*)(fmi2Component);
using fmi2TerminateTYPE = fmi2Status (*)(fmi2Component);
using fmi2ResetTYPE = fmi2Status (*)(fmi2Component);
using fmi2SetRealTYPE = fmi2Status (*)(fmi2Component, const fmi2ValueReference[], std::size_t,
                                       const fmi2Real[]);
using fmi2GetRealTYPE = fmi2Status (*)(fmi2Component, const fmi2ValueReference[], std::size_t,
                                       fmi2Real[]);
using fmi2SetTimeTYPE = fmi2Status (*)(fmi2Component, fmi2Real);
using fmi2GetContinuousStatesTYPE = fmi2Status (*)(fmi2Component, fmi2Real[], std::size_t);
using fmi2SetContinuousStatesTYPE = fmi2Status (*)(fmi2Component, const fmi2Real[], std::size_t);
using fmi2GetDerivativesTYPE = fmi2Status (*)(fmi2Component, fmi2Real[], std::size_t);
using fmi2DoStepTYPE = fmi2Status (*)(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);

static const char* kGuid = "{200bdf00-574d-4c95-a530-761da4fc4aa5}";
static const char* kInstanceName = "Quadcopter";

// From modelDescription.xml (Quadcopter FMI 2.0)
static const fmi2ValueReference kThrustVr[4] = {348, 349, 350, 351};
static const fmi2ValueReference kBodyR0Vr[3] = {3, 4, 5};

static const char* statusName(fmi2Status s) {
  switch (s) {
    case fmi2OK:
      return "OK";
    case fmi2Warning:
      return "Warning";
    case fmi2Discard:
      return "Discard";
    case fmi2Error:
      return "Error";
    case fmi2Fatal:
      return "Fatal";
    case fmi2Pending:
      return "Pending";
    default:
      return "?";
  }
}

static void logger(fmi2ComponentEnvironment, fmi2String instanceName, fmi2Status status,
                   fmi2String category, fmi2String message, ...) {
  std::fprintf(stderr, "[FMI %s] %s / %s: ", statusName(status),
               instanceName ? instanceName : "?", category ? category : "?");
  va_list args;
  va_start(args, message);
  std::vfprintf(stderr, message ? message : "", args);
  va_end(args);
  std::fputc('\n', stderr);
}

static void* callocMem(std::size_t nobj, std::size_t size) { return std::calloc(nobj, size); }
static void freeMem(void* obj) { std::free(obj); }

static void check(fmi2Status status, const char* what) {
  if (status != fmi2OK && status != fmi2Warning) {
    std::cerr << what << " failed: " << statusName(status) << "\n";
    std::exit(1);
  }
}

template <typename T>
static T loadSym(HMODULE dll, const char* name) {
  auto fn = reinterpret_cast<T>(GetProcAddress(dll, name));
  if (!fn) {
    std::cerr << "Missing FMI symbol: " << name << "\n";
    std::exit(1);
  }
  return fn;
}

static std::string toFileUri(const fs::path& p) {
  auto abs = fs::absolute(p).lexically_normal();
  std::string s = abs.generic_string();
  // file:///C:/...
  if (s.size() >= 2 && s[1] == ':') {
    return "file:///" + s;
  }
  return "file://" + s;
}

static fs::path findUnzippedFmu(int argc, char** argv) {
  std::vector<fs::path> candidates;
  if (argc > 1) {
    candidates.emplace_back(argv[1]);
  }
  const fs::path here = fs::current_path();
  candidates.push_back(here / "Model");
  candidates.push_back(here / ".." / "Model");

  for (auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c / "binaries" / "win64" / "Quadcopter.dll", ec) &&
        fs::exists(c / "modelDescription.xml", ec)) {
      return fs::absolute(c);
    }
  }
  std::cerr << "Could not find unzipped FMU (need binaries/win64/Quadcopter.dll).\n"
            << "Pass the unzipped FMU directory as argv[1], or run from the repo root.\n";
  std::exit(1);
}

int main(int argc, char** argv) {
  const fs::path fmuDir = findUnzippedFmu(argc, argv);
  const fs::path dllPath = fmuDir / "binaries" / "win64" / "Quadcopter.dll";
  // FMI 2.0: fmuResourceLocation is a URI to the unzipped FMU's resources/ directory.
  const std::string resourceUri = toFileUri(fmuDir / "resources");

  const fmi2Real tStart = 0.0;
  const fmi2Real tStop = 3.0;
  const fmi2Real dt = 0.1;
  const fmi2Real thrust = -30.0;

  std::cout << "FMU dir: " << fmuDir.string() << "\n";
  std::cout << "resourceLocation: " << resourceUri << "\n";
  std::cout << "Co-sim " << tStart << " -> " << tStop << " s, dt=" << dt
            << ", thrust1-4=" << thrust << "\n";

  HMODULE dll = LoadLibraryW(dllPath.wstring().c_str());
  if (!dll) {
    std::cerr << "LoadLibrary failed for " << dllPath.string() << " (error " << GetLastError()
              << ")\n";
    return 1;
  }

  auto fmi2GetVersion = loadSym<fmi2GetVersionTYPE>(dll, "fmi2GetVersion");
  auto fmi2Instantiate = loadSym<fmi2InstantiateTYPE>(dll, "fmi2Instantiate");
  auto fmi2FreeInstance = loadSym<fmi2FreeInstanceTYPE>(dll, "fmi2FreeInstance");
  auto fmi2SetupExperiment = loadSym<fmi2SetupExperimentTYPE>(dll, "fmi2SetupExperiment");
  auto fmi2EnterInitializationMode =
      loadSym<fmi2EnterInitializationModeTYPE>(dll, "fmi2EnterInitializationMode");
  auto fmi2ExitInitializationMode =
      loadSym<fmi2ExitInitializationModeTYPE>(dll, "fmi2ExitInitializationMode");
  auto fmi2Terminate = loadSym<fmi2TerminateTYPE>(dll, "fmi2Terminate");
  auto fmi2SetReal = loadSym<fmi2SetRealTYPE>(dll, "fmi2SetReal");
  auto fmi2GetReal = loadSym<fmi2GetRealTYPE>(dll, "fmi2GetReal");
  auto fmi2SetTime = loadSym<fmi2SetTimeTYPE>(dll, "fmi2SetTime");
  auto fmi2GetContinuousStates =
      loadSym<fmi2GetContinuousStatesTYPE>(dll, "fmi2GetContinuousStates");
  auto fmi2SetContinuousStates =
      loadSym<fmi2SetContinuousStatesTYPE>(dll, "fmi2SetContinuousStates");
  auto fmi2GetDerivatives = loadSym<fmi2GetDerivativesTYPE>(dll, "fmi2GetDerivatives");
  auto fmi2DoStep = loadSym<fmi2DoStepTYPE>(dll, "fmi2DoStep");

  std::cout << "FMI version: " << fmi2GetVersion() << "\n";

  fmi2CallbackFunctions callbacks{};
  callbacks.logger = logger;
  callbacks.allocateMemory = callocMem;
  callbacks.freeMemory = freeMem;
  callbacks.stepFinished = nullptr;
  callbacks.componentEnvironment = nullptr;

  // loggingOn=false: OpenModelica otherwise dumps every internal ME call (GetDerivatives,
  // SetContinuousStates, ...) on logFmi2Call for each doStep. Pass --verbose to enable.
  const fmi2Boolean loggingOn = (argc > 2 && std::strcmp(argv[2], "--verbose") == 0) ? fmi2True
                               : (argc > 1 && std::strcmp(argv[1], "--verbose") == 0) ? fmi2True
                                                                                     : fmi2False;

  fmi2Component c =
      fmi2Instantiate(kInstanceName, fmi2ModelExchange, kGuid, resourceUri.c_str(), &callbacks,
                      fmi2False, loggingOn);
  if (!c) {
    std::cerr << "fmi2Instantiate returned NULL\n";
    return 1;
  }

  check(fmi2SetupExperiment(c, fmi2False, 0.0, tStart, fmi2True, tStop), "fmi2SetupExperiment");
  check(fmi2EnterInitializationMode(c), "fmi2EnterInitializationMode");

  const fmi2Real thrusts[4] = {thrust, thrust, thrust, thrust};
  check(fmi2SetReal(c, kThrustVr, 4, thrusts), "fmi2SetReal(thrust)");

  check(fmi2ExitInitializationMode(c), "fmi2ExitInitializationMode");

  constexpr std::size_t kNx = 12;
  fmi2Real states[kNx]{};
  fmi2Real derivatives[kNx]{};
  check(fmi2GetContinuousStates(c, states, kNx), "fmi2GetContinuousStates");

  std::printf("%10s %12s %12s %12s\n", "t", "x", "y", "z");
  fmi2Real pos[3]{};
  check(fmi2GetReal(c, kBodyR0Vr, 3, pos), "fmi2GetReal(body.r_0)");
  std::printf("%10.4f %12.6f %12.6f %12.6f\n", tStart, pos[0], pos[1], pos[2]);

  fmi2Real time = tStart;
  int printEvery = static_cast<int>(std::lround(0.1 / dt));
  int step = 0;
  while (time + dt <= tStop) {
    check(fmi2SetReal(c, kThrustVr, 4, thrusts), "fmi2SetReal(thrust)");

    check(fmi2SetTime(c, time), "fmi2SetTime");
    check(fmi2GetDerivatives(c, derivatives, kNx), "fmi2GetDerivatives");
    for (std::size_t i = 0; i < kNx; ++i) {
      states[i] += dt * derivatives[i];
    }
    check(fmi2SetContinuousStates(c, states, kNx), "fmi2SetContinuousStates");

    time += dt;
    check(fmi2SetTime(c, time), "fmi2SetTime");
    ++step;
    if (step % printEvery == 0 || time + dt > tStop) {
      check(fmi2GetReal(c, kBodyR0Vr, 3, pos), "fmi2GetReal(body.r_0)");
      std::printf("%10.4f %12.6f %12.6f %12.6f\n", time, pos[0], pos[1], pos[2]);
    }
  }

  check(fmi2Terminate(c), "fmi2Terminate");
  fmi2FreeInstance(c);
  FreeLibrary(dll);
  std::cout << "Done.\n";
  return 0;
}
