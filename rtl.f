# 1. Submodules (i.e. -f lib/keccak-fips202-sv/rtl.f)
-f lib/common-rtl/rtl.f
-f lib/hash-sampler-unit/rtl.f
-f lib/poly-arith-unit/rtl.f
-f lib/poly-mem-subsystem/rtl.f
-f lib/transcoder-unit/rtl.f
-f lib/core-control-unit/rtl.f

# 2. Local Packages (i.e., rtl/my_pkg.sv)
rtl/qrem_pkg.sv

# 3. Local RTL (i.e., rtl/transcoder_unit.sv)
rtl/qrem_core.sv
