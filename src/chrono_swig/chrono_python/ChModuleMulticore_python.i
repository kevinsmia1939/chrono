#pragma SWIG nowarn=302
#pragma SWIG nowarn=362
#pragma SWIG nowarn=389
#pragma SWIG nowarn=401
#pragma SWIG nowarn=402
#pragma SWIG nowarn=509

%module(directors="1", threads="1") multicore
// Release the GIL only for explicitly marked long-running calls. Globally
// enabling threadallow adds measurable overhead to frequent system queries.
%feature("nothreadallow");
%feature("threadallow") chrono::ChSystemMulticore::AdvanceDynamics;
%feature("threadallow") chrono::ChSystem::DoStepDynamics;

// Keep the public multicore interface and core enum interoperability in the
// shared language-neutral file.
%include "chrono_swig/interface/multicore/ChModuleMulticore.i"
