#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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

#ifdef _WIN32
using DynamicLibrary = HMODULE;
#else
using DynamicLibrary = void*;
#endif

static std::string platformLibraryName() {
#ifdef _WIN32
  return "Quadcopter.dll";
#else
  return "Quadcopter.so";
#endif
}

static std::string platformLibraryDir() {
#ifdef _WIN32
  return "win64";
#else
  return "linux64";
#endif
}

static DynamicLibrary loadDynamicLibrary(const fs::path& path) {
#ifdef _WIN32
  return LoadLibraryW(path.wstring().c_str());
#else
  return dlopen(path.string().c_str(), RTLD_LAZY);
#endif
}

static void unloadDynamicLibrary(DynamicLibrary dll) {
#ifdef _WIN32
  if (dll) {
    FreeLibrary(dll);
  }
#else
  if (dll) {
    dlclose(dll);
  }
#endif
}

static std::string lastLibraryError() {
#ifdef _WIN32
  return std::to_string(GetLastError());
#else
  const char* err = dlerror();
  return err ? err : "unknown";
#endif
}

// --- FMI 2.0 types (shared) ---
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
static T loadSym(DynamicLibrary dll, const char* name) {
#ifdef _WIN32
  auto fn = reinterpret_cast<T>(GetProcAddress(dll, name));
#else
  dlerror();
  auto fn = reinterpret_cast<T>(dlsym(dll, name));
#endif
  if (!fn) {
    std::cerr << "Missing FMI symbol: " << name;
#ifndef _WIN32
    std::cerr << " (" << lastLibraryError() << ")";
#endif
    std::cerr << "\n";
    std::exit(1);
  }
  return fn;
}

static std::string toFileUri(const fs::path& p) {
  auto abs = fs::absolute(p).lexically_normal();
  std::string s = abs.generic_string();
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

  const std::string libDir = platformLibraryDir();
  const std::string libName = platformLibraryName();

  for (auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c / "binaries" / libDir / libName, ec) &&
        fs::exists(c / "modelDescription.xml", ec)) {
      return fs::absolute(c);
    }
  }
  std::cerr << "Could not find unzipped FMU (need binaries/" << libDir << "/" << libName
            << ").\n"
            << "Pass the unzipped FMU directory as argv[1], or run from the repo root.\n";
  std::exit(1);
}

struct FMU {
  DynamicLibrary dll = nullptr;
  fmi2Component c = nullptr;
  // function pointers
  fmi2FreeInstanceTYPE fmi2FreeInstance = nullptr;
  fmi2TerminateTYPE fmi2Terminate = nullptr;
  fmi2SetRealTYPE fmi2SetReal = nullptr;
  fmi2GetRealTYPE fmi2GetReal = nullptr;
  fmi2DoStepTYPE fmi2DoStep = nullptr;
  fmi2SetTimeTYPE fmi2SetTime = nullptr;
  fmi2GetContinuousStatesTYPE fmi2GetContinuousStates = nullptr;
  fmi2SetContinuousStatesTYPE fmi2SetContinuousStates = nullptr;
  fmi2GetDerivativesTYPE fmi2GetDerivatives = nullptr;
};

static FMU Initialise(int argc, char** argv, fmi2Type type, fmi2Real tStart,
                      const fmi2Real* tStop, fmi2Boolean loggingOn,
                      const fmi2ValueReference* thrustVr, std::size_t nThrust,
                      const fmi2Real* thrusts) {
  FMU fm;
  const fs::path fmuDir = findUnzippedFmu(argc, argv);
  const fs::path dllPath = fmuDir / "binaries" / platformLibraryDir() / platformLibraryName();
  const std::string resourceUri = toFileUri(fmuDir / "resources");

  std::cout << "FMU dir: " << fmuDir.string() << "\n";
  std::cout << "resourceLocation: " << resourceUri << "\n";

  fm.dll = loadDynamicLibrary(dllPath);
  if (!fm.dll) {
    std::cerr << "LoadLibrary failed for " << dllPath.string() << " (error "
              << lastLibraryError() << ")\n";
    std::exit(1);
  }

  auto fmi2GetVersion = loadSym<fmi2GetVersionTYPE>(fm.dll, "fmi2GetVersion");
  auto fmi2Instantiate = loadSym<fmi2InstantiateTYPE>(fm.dll, "fmi2Instantiate");
  fm.fmi2FreeInstance = loadSym<fmi2FreeInstanceTYPE>(fm.dll, "fmi2FreeInstance");
  auto fmi2SetupExperiment = loadSym<fmi2SetupExperimentTYPE>(fm.dll, "fmi2SetupExperiment");
  auto fmi2EnterInitializationMode =
      loadSym<fmi2EnterInitializationModeTYPE>(fm.dll, "fmi2EnterInitializationMode");
  auto fmi2ExitInitializationMode =
      loadSym<fmi2ExitInitializationModeTYPE>(fm.dll, "fmi2ExitInitializationMode");
  fm.fmi2Terminate = loadSym<fmi2TerminateTYPE>(fm.dll, "fmi2Terminate");
  fm.fmi2SetReal = loadSym<fmi2SetRealTYPE>(fm.dll, "fmi2SetReal");
  fm.fmi2GetReal = loadSym<fmi2GetRealTYPE>(fm.dll, "fmi2GetReal");
  // optional ME functions
  fm.fmi2SetTime = loadSym<fmi2SetTimeTYPE>(fm.dll, "fmi2SetTime");
  fm.fmi2GetContinuousStates = loadSym<fmi2GetContinuousStatesTYPE>(fm.dll, "fmi2GetContinuousStates");
  fm.fmi2SetContinuousStates = loadSym<fmi2SetContinuousStatesTYPE>(fm.dll, "fmi2SetContinuousStates");
  fm.fmi2GetDerivatives = loadSym<fmi2GetDerivativesTYPE>(fm.dll, "fmi2GetDerivatives");
  fm.fmi2DoStep = loadSym<fmi2DoStepTYPE>(fm.dll, "fmi2DoStep");

  std::cout << "FMI version: " << fmi2GetVersion() << "\n";

  fmi2CallbackFunctions callbacks{};
  callbacks.logger = logger;
  callbacks.allocateMemory = callocMem;
  callbacks.freeMemory = freeMem;
  callbacks.stepFinished = nullptr;
  callbacks.componentEnvironment = nullptr;

  fm.c = fmi2Instantiate(kInstanceName, type, kGuid, resourceUri.c_str(), &callbacks, fmi2False,
                         loggingOn);
  if (!fm.c) {
    std::cerr << "fmi2Instantiate returned NULL\n";
    std::exit(1);
  }

    bool stopDefined = (tStop != nullptr);
    fmi2Real tStopVal = stopDefined ? *tStop : 0.0;
    check(fmi2SetupExperiment(fm.c, fmi2False, 0.0, tStart,
             stopDefined ? fmi2True : fmi2False, tStopVal),
      "fmi2SetupExperiment");
  check(fmi2EnterInitializationMode(fm.c), "fmi2EnterInitializationMode");

  if (thrustVr && thrusts && nThrust > 0) {
    check(fm.fmi2SetReal(fm.c, thrustVr, nThrust, thrusts), "fmi2SetReal(thrust)");
  }

  check(fmi2ExitInitializationMode(fm.c), "fmi2ExitInitializationMode");

  return fm;
}

static void StepCoSimulation(FMU& fm, fmi2Real time, fmi2Real dt, const fmi2ValueReference* thrustVr,
                            std::size_t nThrust, const fmi2Real* thrusts,
                            const fmi2ValueReference* bodyVr, std::size_t nBody,
                            fmi2Real* posOut) {
  check(fm.fmi2SetReal(fm.c, thrustVr, nThrust, thrusts), "fmi2SetReal(thrust)");
  check(fm.fmi2DoStep(fm.c, time, dt, fmi2True), "fmi2DoStep");
  if (posOut && bodyVr && nBody > 0) {
    check(fm.fmi2GetReal(fm.c, bodyVr, nBody, posOut), "fmi2GetReal(body.r_0)");
  }
}

static void StepME(FMU& fm, fmi2Real& time, fmi2Real dt, fmi2Real states[], fmi2Real derivatives[],
                   std::size_t nx, const fmi2ValueReference* thrustVr, std::size_t nThrust,
                   const fmi2Real* thrusts, const fmi2ValueReference* bodyVr, std::size_t nBody,
                   fmi2Real* posOut) {
  check(fm.fmi2SetReal(fm.c, thrustVr, nThrust, thrusts), "fmi2SetReal(thrust)");

  check(fm.fmi2SetTime(fm.c, time), "fmi2SetTime");
  check(fm.fmi2GetDerivatives(fm.c, derivatives, nx), "fmi2GetDerivatives");
  for (std::size_t i = 0; i < nx; ++i) {
    states[i] += dt * derivatives[i];
  }
  check(fm.fmi2SetContinuousStates(fm.c, states, nx), "fmi2SetContinuousStates");

  time += dt;
  check(fm.fmi2SetTime(fm.c, time), "fmi2SetTime");

  if (posOut && bodyVr && nBody > 0) {
    check(fm.fmi2GetReal(fm.c, bodyVr, nBody, posOut), "fmi2GetReal(body.r_0)");
  }
}

static void Terminate(FMU& fm) {
  check(fm.fmi2Terminate(fm.c), "fmi2Terminate");
  fm.fmi2FreeInstance(fm.c);
  if (fm.dll) {
    unloadDynamicLibrary(fm.dll);
    fm.dll = nullptr;
  }
}
